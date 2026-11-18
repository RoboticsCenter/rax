"""Blind pairwise naturalness ratings of trial videos, ranked with Bradley-Terry.

    python scripts/dex_eval_rate.py build RUN_DIR [MORE ...] --session RATE_DIR [--camera front]
                                    [--per-pair 3] [--seed 0] [--copy] [--group-by skill|controller]
    python scripts/dex_eval_rate.py serve --session RATE_DIR [--port 8765]   # local page, http://127.0.0.1:8765
    python scripts/dex_eval_rate.py rate  --session RATE_DIR --rater NAME [--player mpv]   # terminal
    python scripts/dex_eval_rate.py rank  --session RATE_DIR

build pairs videos (trial video_<camera>.mp4) of the same task under different conditions,
randomizes sides and order, and hides condition names behind opaque clip ids (the
mapping is in key.json: do not show it to raters). Ratings append to ratings.jsonl.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from services.dex_eval import rating  # noqa: E402


def rate_cli(session, rater, player=None, input_fn=input, print_fn=print):
    todo = rating.pending(session, rater)
    total = len(rating.load_pairs(session))
    print_fn(f'{len(todo)} of {total} pairs left for {rater}. Answer a, b, t (tie), s (skip) or q (quit).')
    for k, pair in enumerate(todo, 1):
        a = Path(session) / 'clips' / f'{pair["a"]}.mp4'
        b = Path(session) / 'clips' / f'{pair["b"]}.mp4'
        print_fn(f'\nPair {pair["pair_id"]} ({k}/{len(todo)}): A = {a}  B = {b}')
        if player:
            for clip in (a, b):
                subprocess.run([*shlex.split(player), str(clip)], check=False)
        while True:
            answer = input_fn('Which looks more natural and smooth? [a/b/t/s/q]: ').strip().lower()
            if answer in ('a', 'b', 't', 's', 'q'):
                break
        if answer == 'q':
            break
        if answer == 's':
            continue
        rating.record_rating(session, pair['pair_id'], {'a': 'A', 'b': 'B', 't': 'tie'}[answer], rater)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    build = sub.add_parser('build', help='create a randomized, anonymized pair set')
    build.add_argument('inputs', nargs='+', type=Path)
    build.add_argument('--session', type=Path, required=True)
    build.add_argument('--camera', help='only this camera (video_<camera>.mp4)')
    build.add_argument('--per-pair', type=int, default=3, help='comparisons per condition pair')
    build.add_argument('--seed', type=int, default=0)
    build.add_argument('--copy', action='store_true', help='copy videos instead of symlinking')
    build.add_argument('--group-by', choices=('skill', 'controller'), default='skill')
    serve = sub.add_parser('serve', help='local rating page')
    serve.add_argument('--session', type=Path, required=True)
    serve.add_argument('--port', type=int, default=8765)
    rate = sub.add_parser('rate', help='rate in the terminal')
    rate.add_argument('--session', type=Path, required=True)
    rate.add_argument('--rater', required=True)
    rate.add_argument('--player', help='command to play a clip, e.g. "mpv --loop=2"')
    rank = sub.add_parser('rank', help='Bradley-Terry ranking')
    rank.add_argument('--session', type=Path, required=True)
    rank.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)

    if args.command == 'build':
        summary = rating.build_session(args.inputs, args.session, seed=args.seed, per_pair=args.per_pair,
                                       camera=args.camera, copy=args.copy, group_key=args.group_by)
        print(json.dumps(summary))
        if not summary['pairs']:
            print('No pairs: each group needs videos from at least two conditions.', file=sys.stderr)
            return 1
    elif args.command == 'serve':
        server = rating.make_server(args.session, args.port)
        print(f'Rating page: http://127.0.0.1:{args.port}/  (Ctrl-C to stop)')
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
    elif args.command == 'rate':
        rate_cli(args.session, args.rater, args.player)
    else:
        if args.json:
            print(json.dumps(rating.ranking(args.session), indent=1))
        else:
            print(rating.ranking_table(args.session))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
