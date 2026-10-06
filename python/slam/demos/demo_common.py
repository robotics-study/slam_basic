"""Shared demo scaffolding: arg parsing + run flow for single-robot estimators.

Demos are assembly only — they wire params + scenario + estimator and emit the two
events the estimator cannot: `run_started` (scenario path / seed / sensor snapshot,
known only here) and `run_finished` (the metrics computed from the result against
ground truth). Everything between them is emitted by the estimator through the base
Estimator.run template. A stochastic algorithm's `seed` param receives the scenario
seed verbatim — sim noise and algorithm-internal streams then share the seed value
but stay independent streams (spec/data_formats.md).
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Callable

from ..core.estimator import Capability, Estimator
from ..core.metrics import evaluate
from ..core.params import ParamSet
from ..core.sim import build_episode
from ..core.trace import open_trace
from ..maps.loader import load_scenario


def _parse_args(name: str) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=f"slam {name} demo")
    parser.add_argument("--scenario", required=True, help="scenario yaml path")
    parser.add_argument("--params", required=True, help="algorithm config yaml path")
    parser.add_argument("--trace", required=True, help="output trace jsonl path")
    return parser.parse_args()


def run(name: str, factory: Callable[[ParamSet], Estimator]) -> None:
    args = _parse_args(name)
    params = ParamSet.from_yaml(args.params)
    scenario = load_scenario(args.scenario)
    # A stochastic algorithm declares its own `seed` param; the demo injects the
    # scenario seed verbatim (contract — see module docstring).
    if params.has("seed"):
        params.set("seed", scenario.seed)
    episode = build_episode(
        scenario.grid,
        list(scenario.waypoints),
        scenario.step_meters,
        scenario.sensor,
        scenario.landmarks,
        scenario.sigma_xy,
        scenario.sigma_theta,
        scenario.seed,
    )
    estimator = factory(params)
    assert Capability(scenario.sensor.type) in estimator.required_capabilities(), (
        f"{name} does not consume the {scenario.sensor.type!r} sensor of {args.scenario}"
    )
    with open_trace(args.trace) as recorder:
        recorder.run_started(
            estimator.name, args.scenario, params.values(), scenario.seed,
            scenario.sensor, scenario.landmarks,
        )
        result = estimator.run(episode, recorder)
        metrics = evaluate(result, episode)
        recorder.run_finished(metrics)
    # One-line JSON on stdout (bench + web export read it): algorithm first, then
    # metric keys SORTED — the same byte order C++'s std::map iteration emits.
    print(json.dumps({"algorithm": name, **dict(sorted(metrics.items()))}))
