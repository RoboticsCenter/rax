"""Evaluation and comparison harness for dexterous skill variants on xArm 6 + Wuji Hand 2.

Pure analysis modules (``trace``, ``signals``, ``metrics``, ``audio``, ``stats``,
``thresholds``, ``evaluate``) need only numpy and scipy. ``report`` adds matplotlib.
``runner`` drives skills through the RAX controller runtime; it never arms a robot.

See docs/dex-eval.md for the trace contract, metric definitions and CLI usage.
"""

METRICS_VERSION = 1
