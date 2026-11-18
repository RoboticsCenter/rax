"""Small statistics helpers: Wilson intervals, bootstrap CIs, Holm, Mann-Whitney, Bradley-Terry."""
from __future__ import annotations

import math

import numpy as np

NAN = float('nan')


def wilson(successes, n, z=1.959963984540054):
    """(rate, low, high) Wilson score interval for a binomial proportion."""
    if n <= 0:
        return NAN, NAN, NAN
    p = successes / n
    denominator = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denominator
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denominator
    return p, max(0.0, centre - half), min(1.0, centre + half)


def finite(values):
    a = np.asarray([v for v in values if v is not None], dtype=float)
    return a[np.isfinite(a)]


def bootstrap_ci(values, stat=np.median, n_boot=2000, alpha=0.05, seed=0):
    """(statistic, low, high) percentile bootstrap interval, ignoring NaN/None."""
    x = finite(values)
    if x.size == 0:
        return NAN, NAN, NAN
    point = float(stat(x))
    if x.size < 2:
        return point, NAN, NAN
    rng = np.random.default_rng(seed)
    samples = rng.choice(x, size=(n_boot, x.size), replace=True)
    if stat is np.median or stat is np.mean:
        stats = stat(samples, axis=1)
    else:
        stats = np.apply_along_axis(stat, 1, samples)
    return point, float(np.percentile(stats, 100 * alpha / 2)), float(np.percentile(stats, 100 * (1 - alpha / 2)))


def holm(pvalues):
    """Holm-Bonferroni adjusted p-values (same order as given; NaN stays NaN)."""
    p = np.asarray(pvalues, dtype=float)
    out = np.full(p.shape, NAN)
    valid = np.flatnonzero(np.isfinite(p))
    order = valid[np.argsort(p[valid])]
    m = order.size
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (m - rank) * p[i]))
        out[i] = running
    return out.tolist()


def mann_whitney(a, b):
    """Two-sided Mann-Whitney U p-value, NaN when either sample has fewer than 2 values."""
    a, b = finite(a), finite(b)
    if a.size < 2 or b.size < 2:
        return NAN
    from scipy.stats import mannwhitneyu
    try:
        return float(mannwhitneyu(a, b, alternative='two-sided').pvalue)
    except ValueError:
        return NAN


def bradley_terry(comparisons, items=None, *, prior=0.5, iterations=1000, tol=1e-10):
    """Bradley-Terry strengths from pairwise outcomes (Hunter 2004 MM algorithm).

    comparisons: iterable of (a, b, outcome) with outcome 'A' (a preferred), 'B'
    (b preferred) or 'tie' (half a win each). prior adds that many pseudo-wins to each
    side of every compared pair, which keeps strengths finite when an item never loses.
    Returns {item: log-strength} centred on 0 (higher is preferred).
    """
    comparisons = list(comparisons)
    items = sorted(set(items or []) | {x for a, b, _ in comparisons for x in (a, b)})
    index = {item: i for i, item in enumerate(items)}
    n = len(items)
    if n == 0:
        return {}
    wins = np.zeros((n, n))
    for a, b, result in comparisons:
        i, j = index[a], index[b]
        if result == 'A':
            wins[i, j] += 1
        elif result == 'B':
            wins[j, i] += 1
        elif result == 'tie':
            wins[i, j] += 0.5
            wins[j, i] += 0.5
        else:
            raise ValueError(f'unknown comparison outcome {result!r}')
    games = wins + wins.T
    compared = games > 0
    wins = wins + prior * compared
    games = wins + wins.T
    total_wins = wins.sum(axis=1)
    strength = np.ones(n)
    for _ in range(iterations):
        denominator = np.where(games > 0, games / (strength[:, None] + strength[None, :]), 0.0).sum(axis=1)
        updated = np.where(denominator > 0, total_wins / np.where(denominator > 0, denominator, 1), strength)
        updated = updated / np.exp(np.mean(np.log(np.maximum(updated, 1e-300))))
        if np.max(np.abs(np.log(np.maximum(updated, 1e-300)) - np.log(strength))) < tol:
            strength = updated
            break
        strength = updated
    logs = np.log(np.maximum(strength, 1e-300))
    logs -= logs.mean()
    return {item: float(logs[index[item]]) for item in items}


def bradley_terry_ci(comparisons, items=None, n_boot=500, alpha=0.05, seed=0, **kwargs):
    """{item: (log-strength, low, high)} with a bootstrap over comparisons."""
    comparisons = list(comparisons)
    point = bradley_terry(comparisons, items, **kwargs)
    if len(comparisons) < 2:
        return {k: (v, NAN, NAN) for k, v in point.items()}
    rng = np.random.default_rng(seed)
    draws = {k: [] for k in point}
    for _ in range(n_boot):
        sample = [comparisons[i] for i in rng.integers(0, len(comparisons), len(comparisons))]
        values = bradley_terry(sample, list(point), **kwargs)
        for k in point:
            draws[k].append(values[k])
    return {k: (point[k], float(np.percentile(draws[k], 100 * alpha / 2)),
                float(np.percentile(draws[k], 100 * (1 - alpha / 2)))) for k in point}
