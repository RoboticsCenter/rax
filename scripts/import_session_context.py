"""Archive a demonstration file and make it available in the RAX agent workspace."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from services.context_archive import import_context


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('file', type=Path)
    args = parser.parse_args()
    print(import_context(args.session, args.file))
