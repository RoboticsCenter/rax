"""Blind pairwise naturalness ratings of recorded trial videos, ranked with Bradley-Terry.

A rating session directory holds:

    pairs.json      randomized, anonymized pairs: {pair_id, group, camera, a, b} (clip ids only)
    key.json        clip id -> trial, condition and source video (keep it away from raters)
    clips/<id>.mp4  links (or copies) of the source videos under opaque names
    ratings.jsonl   one rating per line: {pair_id, choice: "A"|"B"|"tie", rater, at}
    index.html      the local rating page (``serve``)

Pairs compare videos of the same task (meta.skill) and camera under different
conditions; each condition pair gets ``per_pair`` comparisons, sides are randomized and
the presentation order is shuffled with the recorded seed.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import itertools
import json
import os
from pathlib import Path
import random
import re
import shutil

from .stats import bradley_terry_ci
from .trace import find_trial_dirs

CHOICES = ('A', 'B', 'tie')


def _label(meta):
    if meta.get('condition_id'):
        return str(meta['condition_id'])
    skill, variant = meta.get('skill'), meta.get('variant')
    return f'{skill}/{variant}' if skill and variant else str(variant or skill or meta.get('controller') or 'unknown')


def discover(inputs, camera=None, group_key='skill'):
    """[{trial_dir, group, condition, camera, video}] for every trial video under inputs."""
    clips = []
    for root in inputs:
        for trial_dir in find_trial_dirs(root):
            try:
                meta = json.loads((trial_dir / 'meta.json').read_text())
            except (OSError, ValueError):
                continue
            group = str(meta.get(group_key) or meta.get('controller') or 'all')
            if group_key == 'controller':
                group = 'all'
            condition = _label(meta) if group_key != 'controller' else str(meta.get('controller') or 'unknown')
            for video in sorted(trial_dir.glob('video_*.mp4')):
                cam = video.stem[len('video_'):]
                if camera and cam != camera:
                    continue
                clips.append({'trial_dir': str(trial_dir), 'group': group, 'condition': condition,
                              'camera': cam, 'video': str(video)})
    return clips


def build_session(inputs, out_dir, *, seed=0, per_pair=3, camera=None, copy=False, group_key='skill'):
    """Create a rating session; returns its summary. Refuses to overwrite an existing session."""
    out_dir = Path(out_dir)
    if (out_dir / 'pairs.json').exists():
        raise FileExistsError(f'{out_dir} already holds a rating session')
    rng = random.Random(seed)
    clips = discover(inputs, camera, group_key)
    by_cell = {}
    for clip in clips:
        by_cell.setdefault((clip['group'], clip['camera']), {}).setdefault(clip['condition'], []).append(clip)
    key, pairs = {}, []

    def clip_id(clip):
        digest = hashlib.sha256(f'{seed}|{clip["video"]}'.encode()).hexdigest()[:12]
        key[digest] = clip
        return digest

    for (group, cam), conditions in sorted(by_cell.items()):
        for first, second in itertools.combinations(sorted(conditions), 2):
            a_pool, b_pool = list(conditions[first]), list(conditions[second])
            rng.shuffle(a_pool)
            rng.shuffle(b_pool)
            for k in range(per_pair):
                a, b = a_pool[k % len(a_pool)], b_pool[k % len(b_pool)]
                if rng.random() < 0.5:
                    a, b = b, a
                pairs.append({'group': group, 'camera': cam, 'a': clip_id(a), 'b': clip_id(b)})
    rng.shuffle(pairs)
    for i, pair in enumerate(pairs, 1):
        pair['pair_id'] = f'p{i:04d}'
    (out_dir / 'clips').mkdir(parents=True, exist_ok=True)
    for ident, clip in key.items():
        target = out_dir / 'clips' / f'{ident}.mp4'
        if target.exists() or target.is_symlink():
            continue
        if copy:
            shutil.copy2(clip['video'], target)
        else:
            os.symlink(os.path.abspath(clip['video']), target)
    (out_dir / 'pairs.json').write_text(json.dumps(pairs, indent=1) + '\n')
    (out_dir / 'key.json').write_text(json.dumps({'seed': seed, 'per_pair': per_pair, 'group_key': group_key,
                                                  'created_at': datetime.now(timezone.utc).isoformat(),
                                                  'clips': key}, indent=1) + '\n')
    (out_dir / 'index.html').write_text(PAGE)
    return {'pairs': len(pairs), 'clips': len(key), 'groups': sorted({p['group'] for p in pairs})}


def load_pairs(session):
    return json.loads((Path(session) / 'pairs.json').read_text())


def load_ratings(session):
    path = Path(session) / 'ratings.jsonl'
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def record_rating(session, pair_id, choice, rater):
    """Append one validated rating."""
    if choice not in CHOICES:
        raise ValueError(f'choice must be one of {CHOICES}')
    if not isinstance(rater, str) or not re.fullmatch(r'[A-Za-z0-9_.@ -]{1,64}', rater):
        raise ValueError('rater must be 1-64 letters, digits, spaces or _.@-')
    if pair_id not in {p['pair_id'] for p in load_pairs(session)}:
        raise ValueError(f'unknown pair {pair_id}')
    entry = {'pair_id': pair_id, 'choice': choice, 'rater': rater, 'at': datetime.now(timezone.utc).isoformat()}
    with (Path(session) / 'ratings.jsonl').open('a') as stream:
        stream.write(json.dumps(entry) + '\n')
    return entry


def pending(session, rater):
    done = {r['pair_id'] for r in load_ratings(session) if r.get('rater') == rater}
    return [p for p in load_pairs(session) if p['pair_id'] not in done]


def comparisons(session):
    """{group: [(condition_a, condition_b, outcome)]} un-blinded with key.json."""
    key = json.loads((Path(session) / 'key.json').read_text())['clips']
    pairs = {p['pair_id']: p for p in load_pairs(session)}
    out = {}
    for rating in load_ratings(session):
        pair = pairs.get(rating.get('pair_id'))
        if pair is None or rating.get('choice') not in CHOICES:
            continue
        out.setdefault(pair['group'], []).append(
            (key[pair['a']]['condition'], key[pair['b']]['condition'], rating['choice']))
    return out


def ranking(session, n_boot=500, seed=0):
    """{group: [{'condition', 'strength', 'low', 'high', 'wins', 'losses', 'ties', 'comparisons'}]} best first."""
    result = {}
    key = json.loads((Path(session) / 'key.json').read_text())['clips']
    for group, items in comparisons(session).items():
        conditions = sorted({c['condition'] for c in key.values() if c['group'] == group})
        scores = bradley_terry_ci(items, conditions, n_boot=n_boot, seed=seed)
        rows = []
        for condition, (strength, low, high) in scores.items():
            wins = sum((a == condition and o == 'A') or (b == condition and o == 'B') for a, b, o in items)
            losses = sum((a == condition and o == 'B') or (b == condition and o == 'A') for a, b, o in items)
            ties = sum(o == 'tie' and condition in (a, b) for a, b, o in items)
            rows.append({'condition': condition, 'strength': strength, 'low': low, 'high': high, 'wins': wins,
                         'losses': losses, 'ties': ties, 'comparisons': wins + losses + ties})
        result[group] = sorted(rows, key=lambda r: -r['strength'])
    return result


def ranking_table(session):
    """Markdown naturalness ranking for a rating session."""
    ratings = load_ratings(session)
    raters = sorted({r.get('rater') for r in ratings})
    lines = [f'Rating session `{session}`: {len(ratings)} ratings from {len(raters)} raters. Bradley-Terry '
             'log-strength (0 = average; +1 means about 2.7x the odds of being preferred), bootstrap 95% CI.', '']
    for group, rows in ranking(session).items():
        lines += [f'**{group}**', '', '| Rank | Condition | Strength [95% CI] | Wins | Losses | Ties |',
                  '|---|---|---|---|---|---|']
        for i, row in enumerate(rows, 1):
            ci = (f' [{row["low"]:.2f}, {row["high"]:.2f}]' if row['low'] == row['low'] else '')
            lines.append(f'| {i} | {row["condition"]} | {row["strength"]:.2f}{ci} | {row["wins"]} | '
                         f'{row["losses"]} | {row["ties"]} |')
        lines.append('')
    if len(ratings) == 0:
        lines.append('No ratings yet.')
    return '\n'.join(lines)


# --------------------------------------------------------------------------- local rating page

class _Handler(BaseHTTPRequestHandler):
    session: Path = None

    def log_message(self, *args):
        pass

    def _send(self, status, body, content_type='application/json', headers=None):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html'):
            return self._send(HTTPStatus.OK, PAGE.encode(), 'text/html; charset=utf-8')
        if path == '/pairs':
            rater = ''
            if '?' in self.path:
                query = dict(part.split('=', 1) for part in self.path.split('?', 1)[1].split('&') if '=' in part)
                from urllib.parse import unquote
                rater = unquote(query.get('rater', ''))
            todo = pending(self.session, rater) if rater else load_pairs(self.session)
            return self._send(HTTPStatus.OK, json.dumps({'pairs': todo, 'total': len(load_pairs(self.session))}).encode())
        match = re.fullmatch(r'/clips/([0-9a-f]{12})\.mp4', path)
        if match:
            return self._video(self.session / 'clips' / f'{match.group(1)}.mp4')
        self._send(HTTPStatus.NOT_FOUND, b'{"error": "not found"}')

    do_HEAD = do_GET

    def _video(self, path):
        if not path.is_file():
            return self._send(HTTPStatus.NOT_FOUND, b'{"error": "no clip"}')
        size = path.stat().st_size
        start, end = 0, size - 1
        header = self.headers.get('Range', '')
        match = re.fullmatch(r'bytes=(\d*)-(\d*)', header.strip())
        status = HTTPStatus.OK
        if match and (match.group(1) or match.group(2)):
            if match.group(1):
                start = int(match.group(1))
                end = int(match.group(2)) if match.group(2) else size - 1
            else:
                start = max(0, size - int(match.group(2)))
            end = min(end, size - 1)
            status = HTTPStatus.PARTIAL_CONTENT
        with path.open('rb') as stream:
            stream.seek(start)
            body = stream.read(end - start + 1)
        headers = {'Accept-Ranges': 'bytes'}
        if status == HTTPStatus.PARTIAL_CONTENT:
            headers['Content-Range'] = f'bytes {start}-{end}/{size}'
        self._send(status, body, 'video/mp4', headers)

    def do_POST(self):
        if self.path != '/rate':
            return self._send(HTTPStatus.NOT_FOUND, b'{"error": "not found"}')
        try:
            length = min(int(self.headers.get('Content-Length', 0)), 4096)
            data = json.loads(self.rfile.read(length))
            entry = record_rating(self.session, data.get('pair_id'), data.get('choice'), data.get('rater'))
        except (ValueError, TypeError) as exc:
            return self._send(HTTPStatus.BAD_REQUEST, json.dumps({'error': str(exc)}).encode())
        self._send(HTTPStatus.OK, json.dumps(entry).encode())


def make_server(session, port=8765, host='127.0.0.1'):
    """A local rating server (loopback only by default)."""
    handler = type('SessionHandler', (_Handler,), {'session': Path(session).resolve()})
    return ThreadingHTTPServer((host, port), handler)


PAGE = """<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><title>Blind naturalness rating</title>
<style>
  body { font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background: #f9f9f7; color: #0b0b0b;
         margin: 24px; }
  h1 { font-size: 18px; margin: 0 0 4px; } p { color: #52514e; margin: 4px 0 16px; }
  .clips { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .clip { background: #fcfcfb; border: 1px solid rgba(11,11,11,0.10); border-radius: 8px; padding: 8px; }
  .clip h2 { font-size: 14px; margin: 0 0 6px; } video { width: 100%; border-radius: 4px; background: #000; }
  .controls { margin: 16px 0; display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  button { font: inherit; padding: 8px 18px; border-radius: 6px; border: 1px solid #c3c2b7; background: #fcfcfb;
           cursor: pointer; } button.primary { background: #2a78d6; border-color: #2a78d6; color: #fff; }
  #status { color: #52514e; }
</style></head><body>
<h1>Which hand motion looks more natural and smooth?</h1>
<p>Watch both clips (they show the same task). Choose A, B, or Tie. Keys: a, b, t; space replays both.</p>
<div class="controls"><label>Rater <input id="rater" autocomplete="off"></label>
  <button id="start">Start</button><span id="status"></span></div>
<div class="clips">
  <div class="clip"><h2>A</h2><video id="va" muted playsinline></video></div>
  <div class="clip"><h2>B</h2><video id="vb" muted playsinline></video></div>
</div>
<div class="controls">
  <button id="replay">Replay both</button>
  <button class="primary" data-choice="A">A is more natural</button>
  <button data-choice="tie">Tie</button>
  <button class="primary" data-choice="B">B is more natural</button>
</div>
<script>
let pairs = [], index = 0, total = 0;
const rater = document.getElementById('rater'), status = document.getElementById('status');
const va = document.getElementById('va'), vb = document.getElementById('vb');
rater.value = localStorage.getItem('dexEvalRater') || '';
function play() { for (const v of [va, vb]) { v.currentTime = 0; v.play(); } }
function show() {
  if (index >= pairs.length) { status.textContent = 'All pairs rated. Thank you.'; va.removeAttribute('src');
    vb.removeAttribute('src'); return; }
  const p = pairs[index];
  status.textContent = `Pair ${total - pairs.length + index + 1} of ${total}`;
  va.src = `clips/${p.a}.mp4`; vb.src = `clips/${p.b}.mp4`;
  Promise.all([va, vb].map(v => new Promise(r => v.addEventListener('loadeddata', r, {once: true})))).then(play);
}
async function start() {
  if (!rater.value.trim()) { status.textContent = 'Enter a rater name first.'; return; }
  localStorage.setItem('dexEvalRater', rater.value.trim());
  const data = await (await fetch(`pairs?rater=${encodeURIComponent(rater.value.trim())}`)).json();
  pairs = data.pairs; total = data.total; index = 0; show();
}
async function rate(choice) {
  if (index >= pairs.length) return;
  const response = await fetch('rate', {method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({pair_id: pairs[index].pair_id, choice, rater: rater.value.trim()})});
  if (!response.ok) { status.textContent = (await response.json()).error; return; }
  index += 1; show();
}
document.getElementById('start').onclick = start;
document.getElementById('replay').onclick = play;
for (const b of document.querySelectorAll('[data-choice]')) b.onclick = () => rate(b.dataset.choice);
document.addEventListener('keydown', e => {
  if (e.target === rater) return;
  if (e.key === 'a') rate('A'); else if (e.key === 'b') rate('B'); else if (e.key === 't') rate('tie');
  else if (e.key === ' ') { e.preventDefault(); play(); }
});
</script></body></html>
"""
