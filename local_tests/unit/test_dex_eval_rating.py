"""Blind pairwise rating sessions: anonymization, rating collection, server and Bradley-Terry ranking."""
import importlib.util
import json
from pathlib import Path
import threading
import urllib.request

import pytest

from services.dex_eval import rating
from services.dex_eval.synthetic import write_trial

PROJECT = Path(__file__).resolve().parents[2]


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, PROJECT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def videos(tmp_path):
    import numpy as np
    from services.dex_eval.synthetic import hand_arrays
    q = np.zeros((10, 20))
    for variant in ('alpha', 'beta', 'gamma'):
        for k in range(2):
            directory = write_trial(tmp_path / 'trials' / f'{variant}{k}', {'right': hand_arrays(q, q, 100.0)},
                                    meta={'skill': 'rotate', 'variant': variant})
            for camera in ('front', 'side_right'):
                (directory / f'video_{camera}.mp4').write_bytes(f'{variant}-{k}-{camera}'.encode() * 50)
    return tmp_path / 'trials'


def test_pairs_are_anonymized_balanced_and_seeded(videos, tmp_path):
    session = tmp_path / 'session'
    summary = rating.build_session([videos], session, seed=3, per_pair=2, camera='front')
    assert summary == {'pairs': 6, 'clips': 6, 'groups': ['rotate']}
    pairs = rating.load_pairs(session)
    text = (session / 'pairs.json').read_text() + ' '.join(p.name for p in (session / 'clips').iterdir())
    for secret in ('alpha', 'beta', 'gamma', 'trials'):
        assert secret not in text
    key = json.loads((session / 'key.json').read_text())['clips']
    combos = [tuple(sorted((key[p['a']]['condition'], key[p['b']]['condition']))) for p in pairs]
    assert sorted(set(combos)) == [('rotate/alpha', 'rotate/beta'), ('rotate/alpha', 'rotate/gamma'),
                                   ('rotate/beta', 'rotate/gamma')]
    assert all(combos.count(c) == 2 for c in set(combos))
    assert all(key[p['a']]['condition'] != key[p['b']]['condition'] for p in pairs)
    assert (session / 'clips' / f'{pairs[0]["a"]}.mp4').read_bytes().startswith(b'')
    again = tmp_path / 'again'
    rating.build_session([videos], again, seed=3, per_pair=2, camera='front')
    assert rating.load_pairs(again) == pairs
    with pytest.raises(FileExistsError):
        rating.build_session([videos], session)


def test_ratings_validation_pending_and_bradley_terry_ranking(videos, tmp_path):
    session = tmp_path / 'session'
    rating.build_session([videos], session, seed=0, per_pair=4)
    key = json.loads((session / 'key.json').read_text())['clips']
    preference = {'rotate/alpha': 3, 'rotate/beta': 2, 'rotate/gamma': 1}
    with pytest.raises(ValueError):
        rating.record_rating(session, 'p0001', 'maybe', 'r1')
    with pytest.raises(ValueError):
        rating.record_rating(session, 'nope', 'A', 'r1')
    for rater in ('r1', 'r2'):
        for pair in rating.pending(session, rater):
            a, b = preference[key[pair['a']]['condition']], preference[key[pair['b']]['condition']]
            rating.record_rating(session, pair['pair_id'], 'A' if a > b else 'B', rater)
        assert rating.pending(session, rater) == []
    ranks = rating.ranking(session, n_boot=100)
    order = [row['condition'] for row in ranks['rotate']]
    assert order == ['rotate/alpha', 'rotate/beta', 'rotate/gamma']
    assert ranks['rotate'][0]['wins'] == ranks['rotate'][0]['comparisons']
    table = rating.ranking_table(session)
    assert '2 raters' in table and '| 1 | rotate/alpha |' in table


def test_local_rating_server_serves_pairs_ranges_and_records(videos, tmp_path):
    session = tmp_path / 'session'
    rating.build_session([videos], session, seed=0, per_pair=1, camera='front', copy=True)
    server = rating.make_server(session, port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_address[1]}'
    try:
        page = urllib.request.urlopen(base + '/').read().decode()
        assert 'Which hand motion looks more natural' in page
        data = json.loads(urllib.request.urlopen(base + '/pairs?rater=ann').read())
        assert data['total'] == 3 and len(data['pairs']) == 3
        clip = data['pairs'][0]['a']
        request = urllib.request.Request(f'{base}/clips/{clip}.mp4', headers={'Range': 'bytes=0-9'})
        response = urllib.request.urlopen(request)
        assert response.status == 206 and len(response.read()) == 10
        post = urllib.request.Request(base + '/rate', method='POST', headers={'Content-Type': 'application/json'},
                                      data=json.dumps({'pair_id': data['pairs'][0]['pair_id'], 'choice': 'tie',
                                                       'rater': 'ann'}).encode())
        assert json.loads(urllib.request.urlopen(post).read())['choice'] == 'tie'
        left = json.loads(urllib.request.urlopen(base + '/pairs?rater=ann').read())
        assert len(left['pairs']) == 2
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(base + '/clips/../key.json')
    finally:
        server.shutdown()
        server.server_close()


def test_terminal_rating_cli(videos, tmp_path, capsys):
    cli = load_script('dex_eval_rate')
    session = tmp_path / 'session'
    assert cli.main(['build', str(videos), '--session', str(session), '--camera', 'front', '--per-pair', '1']) == 0
    answers = iter(['x', 'a', 's', 'q'])
    cli.rate_cli(session, 'bob', input_fn=lambda prompt: next(answers), print_fn=lambda *a: None)
    ratings = rating.load_ratings(session)
    assert [r['choice'] for r in ratings] == ['A'] and ratings[0]['rater'] == 'bob'
    assert cli.main(['rank', '--session', str(session)]) == 0
    assert 'Bradley-Terry' in capsys.readouterr().out
