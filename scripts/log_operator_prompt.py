"""Trusted Claude Code UserPromptSubmit hook; records text without changing it."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.operator_audit import record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', type=Path, required=True)
    args = parser.parse_args()
    try:
        payload = json.load(sys.stdin)
        prompt = payload.get('prompt')
        if not isinstance(prompt, str):
            raise ValueError('UserPromptSubmit payload has no text prompt')
        # Keep the submission origin honest: this hook can also fire on
        # CLI-generated prompts. The launcher records its own prompts separately.
        record(args.session, 'prompt_submitted', source='claude_code_hook', text=prompt,
               actor='prompt_origin_not_asserted', hook_payload=payload)
    except Exception as exc:
        print(f'Operator prompt logging failed: {type(exc).__name__}: {exc}', file=sys.stderr)
    # No stdout/context injection and no veto or alteration of the submitted prompt.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
