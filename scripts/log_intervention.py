"""Save a physical intervention or spoken-instruction note without messaging the agent."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.operator_audit import record


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--text-file', type=Path, help='UTF-8 text; otherwise read stdin verbatim')
    args = parser.parse_args()
    text = args.text_file.read_bytes().decode('utf-8') if args.text_file else sys.stdin.read()
    if not text.strip():
        parser.error('provide a nonempty intervention note')
    event = record(args.session, 'operator_note', source='operator_cli', text=text, actor='operator')
    if event is None:
        raise SystemExit(1)
    print(event['id'])
