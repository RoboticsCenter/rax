"""Export a RAX session's exact operator timeline and existing recording index."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.session_report import export_report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    print(export_report(args.session, args.output)/'report.md')
