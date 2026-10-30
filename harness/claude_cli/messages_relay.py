"""Host-only Anthropic Messages endpoint for Claude Code. The agent never holds credentials.

The host-run Claude Code talks to ANTHROPIC_BASE_URL=http://127.0.0.1:<port> (this
relay, on loopback TCP; a Unix socket path is also accepted). The relay adds the
provider credential and enforces a model allowlist, a model-call cap, local
(client-executed) tools only, inline media only, bounded bodies, and provider
throttling retried before any byte reaches the client. Token usage is appended to a
host-side JSONL for the operator.

Providers:
- ``openrouter``: https://openrouter.ai/api/v1/messages with the OpenRouter key.
- ``anthropic``: https://api.anthropic.com/v1/messages with an API key (x-api-key).
- ``openai``: https://api.openai.com/v1/responses with an OpenAI API key. The relay
  translates Anthropic Messages requests from Claude Code to OpenAI Responses and back.
- ``claude-login``: https://api.anthropic.com with the dedicated experiment account's
  long-lived ``claude setup-token`` token (harness/claude_cli/experiment_login.py),
  re-read for every request so a rotated token takes effect at once. The operator's
  personal Claude login (~/.claude) is refused.
"""
import gzip
from http.server import BaseHTTPRequestHandler
import http.client
import io
import json
import os
import re
import stat
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit

from harness.claude_cli.relay_common import CallCounter, LoopbackHTTPServer, UnixHTTPServer, retry_delay

MAX_BODY = 256*1024*1024
REQUEST_TIMEOUT_SECONDS = 120
RETRY_STATUSES = frozenset({429, 502, 503, 504, 529})
RETRY_WINDOW_SECONDS = 600
MAX_OUTPUT_TOKENS = 128000
BUDGET_EXHAUSTED = b'Relay model-call budget exhausted; the operator must raise --max-model-calls.'
OAUTH_BETA = 'oauth-2025-04-20'
# model: the agent; auditor: Claude Code's `sonnet` alias, for cheaper review-style
# subagents; fast: the `haiku` alias (background tasks). Keep first-party model
# names aligned with Claude Code's baked catalog; unknown names can make the CLI
# exit before the relay sees any model request.
PROVIDERS = {
    'openrouter': {'base_url': 'https://openrouter.ai/api', 'model': 'anthropic/claude-opus-5.5',
                   'auditor': 'anthropic/claude-sonnet-5.5', 'fast': 'anthropic/claude-haiku-4.5'},
    'anthropic': {'base_url': 'https://api.anthropic.com', 'model': 'claude-opus-5',
                  'auditor': 'claude-haiku-4-5', 'fast': 'claude-haiku-4-5'},
    'openai': {'base_url': 'https://api.openai.com', 'model': 'openai/gpt-6-astra',
               'auditor': 'openai/gpt-6-astra', 'fast': 'openai/gpt-6-astra'},
    'claude-login': {'base_url': 'https://api.anthropic.com', 'model': 'claude-opus-5',
                     'auditor': 'claude-haiku-4-5', 'fast': 'claude-haiku-4-5'},
}
ROUTES = {'/v1/messages': True, '/v1/messages/count_tokens': False}  # path -> counts against the cap
FORWARDED_HEADERS = ('anthropic-version', 'anthropic-beta')
MODEL_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9._:/@-]{0,199}')


def _private_text(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'r') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError(f'{path} must be a private, owner-readable regular file')
        return stream.read(1 << 20)


def personal_login(path):
    """True for the operator's own Claude login, which experiments must never use."""
    path = Path(path).expanduser()
    personal = (Path.home() / '.claude').resolve()
    resolved = path.resolve()
    return (path.name == '.credentials.json' or resolved == personal
            or personal in resolved.parents or resolved == (Path.home() / '.claude.json').resolve())


class Credential:
    """Provider auth headers, resolved per request so a rotated token applies at once."""

    def __init__(self, provider, *, key_file=None):
        if provider not in PROVIDERS:
            raise ValueError(f'Unknown provider {provider!r}; choose from {sorted(PROVIDERS)}')
        self.provider, self.key_file = provider, key_file
        if provider == 'claude-login':
            if not key_file:
                raise ValueError('claude-login needs the experiment setup-token file; '
                                 'run scripts/setup_claude_experiment_login.sh')
            if personal_login(key_file):
                raise ValueError('claude-login refuses the personal Claude login; '
                                 'use the dedicated experiment token')
        self.headers()  # Fail at startup, not on the first model call.

    def _token(self):
        token = _private_text(self.key_file).strip()
        if token.startswith('{') and 'claudeAiOauth' in token:
            raise ValueError('claude-login refuses a Claude login credentials file; '
                             'use the dedicated experiment token')
        if not token or len(token) > 16384 or any(c.isspace() for c in token):
            raise ValueError('Invalid provider credential')
        return token

    def headers(self):
        if self.provider == 'anthropic':
            key = _private_text(self.key_file).strip() if self.key_file else os.environ.get('ANTHROPIC_API_KEY', '')
            if not key:
                raise ValueError('The anthropic provider needs --key-file or ANTHROPIC_API_KEY')
            return {'x-api-key': key}
        if self.provider == 'openrouter':
            key = _private_text(self.key_file).strip() if self.key_file else os.environ.get('OPENROUTER_API_KEY', '')
            if not key:
                raise ValueError('The openrouter provider needs --key-file or OPENROUTER_API_KEY')
            return {'Authorization': 'Bearer ' + key}
        if self.provider == 'openai':
            key = _private_text(self.key_file).strip() if self.key_file else os.environ.get('OPENAI_API_KEY', '')
            if not key:
                raise ValueError('The openai provider needs --key-file or OPENAI_API_KEY')
            return {'Authorization': 'Bearer ' + key}
        return {'Authorization': 'Bearer ' + self._token()}


def _media_is_inline(value):
    """No URL or Files-API sources: the provider must never fetch on the agent's behalf."""
    if isinstance(value, list):
        return all(_media_is_inline(item) for item in value)
    if isinstance(value, dict):
        image_url = value.get('image_url')
        if isinstance(image_url, str) and not image_url.startswith('data:'):
            return False
        if isinstance(image_url, dict):
            url = image_url.get('url')
            if not isinstance(url, str) or not url.startswith('data:'):
                return False
        source = value.get('source')
        if isinstance(source, dict) and source.get('type') not in ('base64', 'text', 'content'):
            return False
        return all(_media_is_inline(item) for item in value.values())
    return True


def local_tools(tools):
    """Only client-executed tools; server tools (web search/fetch, code execution) are refused."""
    if not isinstance(tools, list):
        return False
    for tool in tools:
        if not isinstance(tool, dict) or tool.get('type') not in (None, 'custom'):
            return False
        if not isinstance(tool.get('name'), str):
            return False
    return True


def acceptable(payload, allowed):
    if not isinstance(payload, dict) or payload.get('model') not in allowed:
        return False
    if any(key in payload for key in ('mcp_servers', 'container')):
        return False
    if not local_tools(payload.get('tools', [])):
        return False
    return _media_is_inline(payload.get('messages', [])) and _media_is_inline(payload.get('system', []))


class UsageLog:
    """Parses usage from a Messages response (SSE or JSON) and appends one JSONL row."""

    def __init__(self, path):
        self.path, self.lock = (Path(path) if path else None), threading.Lock()

    def record(self, model, usage, *, reasoning_effort=None):
        if self.path is None or not usage:
            return
        row = {'time': time.time(), 'model': model}
        if reasoning_effort is not None:
            row['reasoning_effort'] = reasoning_effort
        for key in ('input_tokens', 'output_tokens', 'cache_creation_input_tokens', 'cache_read_input_tokens'):
            if isinstance(usage.get(key), int):
                row[key] = usage[key]
        if isinstance(usage.get('cost'), (int, float)):
            row['cost_usd'] = usage['cost']
        with self.lock, open(self.path, 'a') as stream:
            stream.write(json.dumps(row) + '\n')


class StreamUsage:
    """Incremental SSE scan: message_start carries input usage, message_delta the output."""

    def __init__(self):
        self.buffer, self.usage = b'', {}

    def feed(self, block):
        self.buffer += block
        *lines, self.buffer = self.buffer.split(b'\n')
        for line in lines:
            if not line.startswith(b'data:') or b'usage' not in line:
                continue
            try:
                event = json.loads(line[5:])
            except ValueError:
                continue
            usage = (event.get('message') or {}).get('usage') if event.get('type') == 'message_start' \
                else event.get('usage')
            if isinstance(usage, dict):
                self.usage.update({k: v for k, v in usage.items() if v is not None})


def _openai_model(model):
    return model[len('openai/'):] if isinstance(model, str) and model.startswith('openai/') else model


def _json_text(value):
    if isinstance(value, str):
        return value
    return json.dumps(value, separators=(',', ':'), ensure_ascii=False)


def _content_text(value):
    if isinstance(value, str):
        return value
    if not isinstance(value, list):
        return _json_text(value)
    pieces = []
    for block in value:
        if isinstance(block, str):
            pieces.append(block)
        elif isinstance(block, dict) and block.get('type') == 'text':
            pieces.append(str(block.get('text', '')))
        elif isinstance(block, dict) and block.get('type') == 'image':
            pieces.append('[inline image]')
        elif isinstance(block, dict):
            pieces.append(_json_text(block))
    return '\n'.join(piece for piece in pieces if piece)


def _anthropic_system(payload):
    system = payload.get('system')
    if not system:
        return None
    return _content_text(system)


def _openai_image(block):
    if block.get('mimeType') and block.get('data'):
        media = block.get('mimeType') or 'application/octet-stream'
        data = block.get('data') or ''
        return {'type': 'input_image', 'image_url': f'data:{media};base64,{data}'}
    source = block.get('source') or {}
    if source.get('type') != 'base64':
        return None
    media = source.get('media_type') or 'application/octet-stream'
    data = source.get('data') or ''
    return {'type': 'input_image', 'image_url': f'data:{media};base64,{data}'}


def _openai_message(role, parts):
    if not parts:
        return None
    text_parts = [part.get('text', '') for part in parts if part.get('type') == 'input_text']
    if len(text_parts) == len(parts):
        return {'role': role, 'content': '\n'.join(text_parts)}
    return {'role': role if role == 'user' else 'user', 'content': parts}


def _openai_tool_result_output(content):
    if isinstance(content, str):
        return content
    if not isinstance(content, list):
        return _json_text(content)
    pieces, output = [], []
    for block in content:
        if isinstance(block, str):
            pieces.append(block)
        elif isinstance(block, dict) and block.get('type') == 'text':
            pieces.append(str(block.get('text', '')))
        elif isinstance(block, dict) and block.get('type') == 'image':
            if pieces:
                output.append({'type': 'input_text', 'text': '\n'.join(piece for piece in pieces if piece)})
                pieces = []
            image = _openai_image(block)
            if image:
                output.append(image)
        else:
            pieces.append(_json_text(block))
    if pieces:
        output.append({'type': 'input_text', 'text': '\n'.join(piece for piece in pieces if piece)})
    if output and any(part.get('type') == 'input_image' for part in output):
        return output
    return '\n'.join(part.get('text', '') for part in output if part.get('type') == 'input_text')


def _openai_input(messages):
    items = []
    for message in messages or []:
        role = message.get('role', 'user')
        content = message.get('content', '')
        if isinstance(content, str):
            if content:
                items.append({'role': role, 'content': content})
            continue
        parts = []
        for block in content or []:
            if not isinstance(block, dict):
                parts.append({'type': 'input_text', 'text': str(block)})
                continue
            kind = block.get('type')
            if kind == 'text':
                parts.append({'type': 'input_text', 'text': str(block.get('text', ''))})
            elif kind == 'image':
                image = _openai_image(block)
                if image:
                    parts.append(image)
            elif kind == 'tool_use':
                if (item := _openai_message(role, parts)) is not None:
                    items.append(item)
                    parts = []
                arguments = block.get('input') if isinstance(block.get('input'), dict) else {}
                items.append({'type': 'function_call', 'call_id': block.get('id') or block.get('name'),
                              'name': block.get('name'), 'arguments': _json_text(arguments)})
            elif kind == 'tool_result':
                if (item := _openai_message(role, parts)) is not None:
                    items.append(item)
                    parts = []
                call_id = block.get('tool_use_id') or block.get('id') or 'call'
                items.append({'type': 'function_call_output',
                              'call_id': call_id,
                              'output': _openai_tool_result_output(block.get('content', ''))})
        if (item := _openai_message(role, parts)) is not None:
            items.append(item)
    return items


def _openai_tools(tools):
    converted = []
    for tool in tools or []:
        schema = tool.get('input_schema') or tool.get('parameters') or {'type': 'object', 'properties': {}}
        converted.append({'type': 'function', 'name': tool['name'],
                          'description': tool.get('description') or '',
                          'parameters': schema, 'strict': False})
    return converted


def openai_request(payload, *, reasoning_effort=None):
    body = {'model': _openai_model(payload['model']), 'input': _openai_input(payload.get('messages', [])),
            'max_output_tokens': payload.get('max_tokens')}
    if instructions := _anthropic_system(payload):
        body['instructions'] = instructions
    if tools := _openai_tools(payload.get('tools', [])):
        body['tools'] = tools
        body['parallel_tool_calls'] = False
        body['tool_choice'] = 'auto'
    if isinstance(payload.get('metadata'), dict):
        body['metadata'] = payload['metadata']
    thinking = payload.get('thinking') or {}
    # The operator's effort must survive Claude Code's adaptive/missing thinking
    # fields and must not be downgraded by the Messages-to-Responses translation.
    if reasoning_effort is not None:
        body['reasoning'] = {'effort': reasoning_effort}
    elif isinstance(thinking, dict) and thinking.get('type') == 'enabled':
        body['reasoning'] = {'effort': 'high'}
    return body


def _tool_input(arguments):
    if isinstance(arguments, dict):
        return arguments
    try:
        parsed = json.loads(arguments or '{}')
        return parsed if isinstance(parsed, dict) else {'value': parsed}
    except (TypeError, ValueError):
        return {'value': str(arguments or '')}


def anthropic_message_from_openai(data, requested_model):
    content = []
    for item in data.get('output') or []:
        if item.get('type') == 'message':
            for block in item.get('content') or []:
                kind = block.get('type')
                text = block.get('text') if kind in ('output_text', 'text') else None
                if text:
                    content.append({'type': 'text', 'text': text})
        elif item.get('type') in ('function_call', 'custom_tool_call'):
            content.append({'type': 'tool_use', 'id': item.get('call_id') or item.get('id'),
                            'name': item.get('name'), 'input': _tool_input(item.get('arguments') or item.get('input'))})
    usage = data.get('usage') or {}
    mapped_usage = {'input_tokens': usage.get('input_tokens', 0) or 0,
                    'output_tokens': usage.get('output_tokens', 0) or 0}
    details = usage.get('input_tokens_details') or {}
    if isinstance(details.get('cached_tokens'), int):
        mapped_usage['cache_read_input_tokens'] = details['cached_tokens']
    return {'id': data.get('id') or f'msg_{int(time.time()*1000)}', 'type': 'message',
            'role': 'assistant', 'model': requested_model, 'content': content,
            'stop_reason': 'tool_use' if any(block.get('type') == 'tool_use' for block in content) else 'end_turn',
            'stop_sequence': None, 'usage': mapped_usage}


def _anthropic_sse(message):
    usage = dict(message.get('usage') or {})
    start = dict(message)
    start['content'], start['stop_reason'], start['usage'] = [], None, {
        'input_tokens': usage.get('input_tokens', 0), 'output_tokens': 0}
    chunks = [('message_start', {'type': 'message_start', 'message': start})]
    for index, block in enumerate(message.get('content') or []):
        if block.get('type') == 'text':
            chunks.append(('content_block_start', {'type': 'content_block_start', 'index': index,
                                                   'content_block': {'type': 'text', 'text': ''}}))
            chunks.append(('content_block_delta', {'type': 'content_block_delta', 'index': index,
                                                   'delta': {'type': 'text_delta', 'text': block.get('text', '')}}))
        elif block.get('type') == 'tool_use':
            chunks.append(('content_block_start', {'type': 'content_block_start', 'index': index,
                                                   'content_block': {'type': 'tool_use', 'id': block.get('id'),
                                                                     'name': block.get('name'), 'input': {}}}))
            chunks.append(('content_block_delta', {'type': 'content_block_delta', 'index': index,
                                                   'delta': {'type': 'input_json_delta',
                                                             'partial_json': _json_text(block.get('input') or {})}}))
        chunks.append(('content_block_stop', {'type': 'content_block_stop', 'index': index}))
    chunks.append(('message_delta', {'type': 'message_delta',
                                     'delta': {'stop_reason': message.get('stop_reason'), 'stop_sequence': None},
                                     'usage': {'output_tokens': usage.get('output_tokens', 0)}}))
    chunks.append(('message_stop', {'type': 'message_stop'}))
    return ''.join(f'event: {event}\ndata: {json.dumps(data, separators=(",", ":"))}\n\n'
                   for event, data in chunks).encode()


def token_count(payload):
    return {'input_tokens': max(1, len(json.dumps(payload, separators=(',', ':'))) // 4)}


def usage_reset(response):
    """Epoch at which an exhausted subscription usage window reopens; None for an ordinary 429."""
    if response.status != 429 or response.getheader('anthropic-ratelimit-unified-status') != 'rejected':
        return None
    try:
        return float(response.getheader('anthropic-ratelimit-unified-reset'))
    except (TypeError, ValueError):
        return None


class MessagesRelay:
    def __init__(self, path, credential, *, base_url, models, max_calls=1000, client_write_timeout=120,
                 usage_log=None, calls_file=None, reasoning_effort=None):
        if reasoning_effort is not None and reasoning_effort not in {'low', 'medium', 'high', 'xhigh', 'max'}:
            raise ValueError('Invalid operator reasoning effort')
        self.reasoning_effort = reasoning_effort
        allowed = set()
        for model in models:
            if not isinstance(model, str) or not MODEL_ID.fullmatch(model):
                raise ValueError('Invalid operator model identifier')
            allowed.add(model)
        url = urlsplit(base_url)
        if url.scheme not in {'https', 'http'} or not url.hostname or url.username or url.password or url.query or url.fragment:
            raise ValueError('Invalid operator model endpoint')
        if url.scheme == 'http' and url.hostname not in {'127.0.0.1', 'localhost', '::1'}:
            raise ValueError('Non-TLS endpoint is only allowed for local tests')
        self.url, self.credential, self.max_calls = url, credential, max_calls
        self.counter, self.lock = CallCounter(calls_file), threading.Lock()
        self.usage = UsageLog(usage_log)
        self.limited_until = 0.0  # Subscription usage window exhausted until this epoch.
        relay = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *_):
                pass  # Never log headers, prompts, provider failures or credentials.

            def reject(self, status, body=b''):
                self.send_response(status)
                if body:
                    self.send_header('Content-Type', 'text/plain')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Connection', 'close')
                self.end_headers()
                self.wfile.write(body)
                self.close_connection = True

            def do_POST(self):
                upstream = None
                try:
                    self.connection.settimeout(REQUEST_TIMEOUT_SECONDS)
                    parts = urlsplit(self.path)
                    if parts.path not in ROUTES or parts.query not in ('', 'beta=true') \
                            or self.headers.get('Transfer-Encoding'):
                        self.reject(403)
                        return
                    length = int(self.headers.get('Content-Length', '-1'))
                    if not 0 < length <= MAX_BODY:
                        self.reject(413)
                        return
                    body = self.rfile.read(length)
                    if self.headers.get('Content-Encoding') == 'gzip':
                        with gzip.GzipFile(fileobj=io.BytesIO(body)) as stream:
                            body = stream.read(MAX_BODY+1)
                    elif self.headers.get('Content-Encoding'):
                        self.reject(415)
                        return
                    if len(body) > MAX_BODY:
                        self.reject(413)
                        return
                    payload = json.loads(body)
                    if not acceptable(payload, allowed):
                        self.reject(403)
                        return
                    counted = ROUTES[parts.path]
                    if relay.limited_until > time.time():
                        # Retrying would only wait out the window; let the launcher pause instead.
                        error = json.dumps({'type': 'error', 'error': {'type': 'rate_limit_error',
                                            'message': 'Usage limit reached; the session resumes after it resets'}}).encode()
                        self.send_response(429)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(error)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(error)
                        self.close_connection = True
                        return
                    if counted:
                        maximum = payload.get('max_tokens')
                        if type(maximum) is not int or not 1 <= maximum <= MAX_OUTPUT_TOKENS:
                            self.reject(400)
                            return
                        with relay.lock:
                            if relay.counter.value >= relay.max_calls:
                                self.reject(402, BUDGET_EXHAUSTED)
                                return
                            relay.counter.increment()
                    try:
                        auth = relay.credential.headers()
                    except (OSError, ValueError, RuntimeError):
                        self.reject(401, b'Relay provider credential unavailable')
                        return
                    if relay.credential.provider == 'openai' and parts.path == '/v1/messages/count_tokens':
                        data = json.dumps(token_count(payload), separators=(',', ':')).encode()
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(data)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(data)
                        self.close_connection = True
                        return
                    headers = {**auth, 'Content-Type': 'application/json', 'Accept-Encoding': 'identity',
                               'Accept': self.headers.get('Accept', 'application/json')}
                    for name in FORWARDED_HEADERS:
                        if self.headers.get(name):
                            headers[name] = self.headers[name]
                    if relay.credential.provider == 'claude-login':
                        betas = [b for b in headers.get('anthropic-beta', '').split(',') if b.strip()]
                        headers['anthropic-beta'] = ','.join(dict.fromkeys([*betas, OAUTH_BETA]))
                    headers.setdefault('anthropic-version', '2023-06-01')
                    connection = http.client.HTTPSConnection if url.scheme == 'https' else http.client.HTTPConnection
                    is_openai = relay.credential.provider == 'openai'
                    if is_openai:
                        headers.pop('anthropic-version', None)
                        headers.pop('anthropic-beta', None)
                        headers['Accept'] = 'application/json'
                        request = openai_request(payload, reasoning_effort=relay.reasoning_effort)
                        upstream_body = json.dumps(request, separators=(',', ':')).encode()
                        target = url.path.rstrip('/') + '/v1/responses'
                    else:
                        upstream_body = body
                        target = url.path.rstrip('/') + parts.path + (f'?{parts.query}' if parts.query else '')
                    deadline, attempt = time.monotonic()+RETRY_WINDOW_SECONDS, 0
                    while True:
                        upstream = connection(url.hostname, url.port, timeout=600)
                        upstream.request('POST', target, upstream_body, headers)
                        response = upstream.getresponse()
                        if response.status not in RETRY_STATUSES:
                            break
                        if (reset := usage_reset(response)) is not None:
                            relay.limited_until = max(relay.limited_until, reset)
                            break
                        delay = retry_delay(attempt, response.getheader('Retry-After'))
                        if time.monotonic()+delay >= deadline:
                            break
                        upstream.close()
                        time.sleep(delay)
                        attempt += 1
                    if response.status != 200:
                        # Claude Code needs the error type (e.g. context too long) to react;
                        # provider error bodies are small JSON and carry no credential.
                        error = response.read(65536)
                        self.send_response(response.status if 400 <= response.status <= 599 else 502)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Content-Length', str(len(error)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(error)
                        self.close_connection = True
                        return
                    # A slow client must not truncate a valid response.
                    self.connection.settimeout(client_write_timeout)
                    if is_openai:
                        data = response.read(MAX_BODY+1)
                        if len(data) > MAX_BODY:
                            self.reject(502, b'OpenAI response too large')
                            return
                        message = anthropic_message_from_openai(json.loads(data), payload.get('model'))
                        if counted:
                            relay.usage.record(payload.get('model'), message.get('usage') or {},
                                               reasoning_effort=(request.get('reasoning') or {}).get('effort'))
                        if payload.get('stream'):
                            data = _anthropic_sse(message)
                            ctype = 'text/event-stream'
                        else:
                            data = json.dumps(message, separators=(',', ':')).encode()
                            ctype = 'application/json'
                        self.send_response(200)
                        self.send_header('Content-Type', ctype)
                        self.send_header('Content-Length', str(len(data)))
                        self.send_header('Connection', 'close')
                        self.end_headers()
                        self.wfile.write(data)
                        self.close_connection = True
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', response.getheader('Content-Type', 'text/event-stream'))
                    self.send_header('Connection', 'close')
                    self.end_headers()
                    self.close_connection = True
                    streaming = 'text/event-stream' in (response.getheader('Content-Type') or '')
                    scan, whole, total = StreamUsage(), [], 0
                    while block := response.read1(65536):
                        total += len(block)
                        if total > MAX_BODY:
                            break
                        if streaming:
                            scan.feed(block)
                        elif total <= 1 << 20:
                            whole.append(block)
                        self.wfile.write(block)
                        self.wfile.flush()
                    if counted:
                        usage = scan.usage
                        if not streaming and whole:
                            try:
                                usage = json.loads(b''.join(whole)).get('usage') or {}
                            except ValueError:
                                usage = {}
                        relay.usage.record(payload.get('model'), usage)
                except (OSError, ValueError, TypeError, AttributeError, http.client.HTTPException):
                    self.close_connection = True
                finally:
                    if upstream:
                        upstream.close()

            def do_GET(self):
                self.reject(403)
            do_CONNECT = do_GET

        if isinstance(path, tuple):
            self.server = LoopbackHTTPServer(path, Handler)
        else:
            self.server = UnixHTTPServer(str(path), Handler)
            os.chmod(path, 0o600)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    @property
    def address(self):
        """(host, port) for a loopback relay; the socket path otherwise."""
        return self.server.server_address

    @property
    def calls(self):
        return self.counter.value

    def start(self):
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
