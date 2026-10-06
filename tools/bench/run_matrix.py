#!/usr/bin/env python3
"""Benchmark matrix runner: (algorithm x its declared scenarios) -> metrics -> report.

Runs each demo as a subprocess and collects the one-line JSON the demo prints on
stdout ({"algorithm": ..., ate_rmse, rpe_rmse, map_iou?, landmark_rmse?} — keys are
sorted by contract). An algorithm is runnable exactly when `configs/<section>/<algo>.yaml`
and a demo exist for it; each config declares the scenario slugs IT runs on
(`scenarios:`), so the matrix routes every algorithm onto its own scenario list.

The C++ demos take the SAME CLI args and print the same JSON, so cross-language
comparison is one flag away: --runner cpp switches the subprocess to
cpp/build/demos/demo_<algo> (rows carry the runner in the language column).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import yaml

_REPO_ROOT = Path(__file__).resolve().parents[2]

# Metric columns in report order (absent metric renders as "—").
_METRICS = ("ate_rmse", "rpe_rmse", "map_iou", "landmark_rmse")


@dataclass
class Row:
    scenario: str
    algorithm: str
    runner: str  # "py" | "cpp"
    status: str  # "ok" | "error"
    metrics: dict[str, float]


def _run_one(runner: str, config: Path, scenario_path: Path, algo: str) -> Row:
    if runner == "py":
        demo = _REPO_ROOT / "python" / "demos" / f"demo_{algo}.py"
        cmd = [sys.executable, str(demo)]
    else:
        demo = _REPO_ROOT / "cpp" / "build" / "demos" / f"demo_{algo}"
        cmd = [str(demo)]
    if not demo.exists():
        return Row(scenario_path.stem, algo, runner, "error", {})
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as tmp:
        trace_path = Path(tmp.name)
    cmd += ["--scenario", str(scenario_path), "--params", str(config), "--trace", str(trace_path)]
    try:
        # check=False on purpose: a failing demo is a matrix row ("error"), not
        # an exception in the runner.
        proc = subprocess.run(
            cmd, capture_output=True, text=True, check=False,
            cwd=_REPO_ROOT, env={"PYTHONPATH": str(_REPO_ROOT / "python")},
        )
        if proc.returncode != 0:
            return Row(scenario_path.stem, algo, runner, "error", {})
        metrics = json.loads(proc.stdout.strip().splitlines()[-1])
    finally:
        trace_path.unlink(missing_ok=True)
    metrics.pop("algorithm", None)
    return Row(scenario_path.stem, algo, runner, "ok", {k: float(v) for k, v in metrics.items()})


def _render(rows: list[Row]) -> str:
    header = (
        "| algorithm | scenario | runner | status | "
        + " | ".join(_METRICS)
        + " |\n|" + "---|" * (4 + len(_METRICS)) + "\n"
    )
    lines = [header]
    for r in rows:
        if r.status == "error":
            lines.append(
                f"| {r.algorithm} | {r.scenario} | {r.runner} | {r.status} | "
                + " | ".join("-" for _ in _METRICS) + " |\n"
            )
            continue
        cells = [
            "-" if m not in r.metrics else f"{r.metrics[m]:.4f}" for m in _METRICS
        ]
        lines.append(
            f"| {r.algorithm} | {r.scenario} | {r.runner} | {r.status} | " + " | ".join(cells) + " |\n"
        )
    return "".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="slam benchmark matrix runner")
    parser.add_argument("--configs", default=str(_REPO_ROOT / "configs"),
                        help="algorithm-config root (algo id -> <section-dir>/<algo>.yaml under it)")
    parser.add_argument("--scenarios", default=str(_REPO_ROOT / "maps" / "scenarios"),
                        help="scenario directory; each config's `scenarios:` list selects from it")
    parser.add_argument("--out", default=str(_REPO_ROOT / "out" / "report.md"))
    parser.add_argument(
        "--runner", choices=("py", "cpp"), default="py",
        help="which language's demo to run (the C++ demo takes identical args)",
    )
    parser.add_argument(
        "--algos", nargs="*", default=None,
        help="algorithm ids to run (default: every config under --configs, sorted)",
    )
    args = parser.parse_args()

    configs_dir = Path(args.configs)
    scenarios_dir = Path(args.scenarios)
    algos = args.algos or sorted(p.stem for p in configs_dir.rglob("*.yaml"))

    rows: list[Row] = []
    for algo in algos:
        # Slugs are globally unique across sections — two matches is a repo bug,
        # not a tie to break silently. A missing config still runs the demo
        # subprocess with a nonexistent --params path so an explicit --algos
        # request fails loudly ("error"), never silently skipped.
        found = sorted(configs_dir.rglob(f"{algo}.yaml"))
        if len(found) > 1:
            raise SystemExit(
                f"expected exactly one configs/<section>/{algo}.yaml, found {len(found)}"
            )
        config_path = found[0] if found else configs_dir / f"{algo}.yaml"
        # Route the algorithm onto ITS declared scenarios (raw read — the runner
        # stays free of slam.core; the demo subprocess validates the config itself).
        try:
            declared: list[str] = yaml.safe_load(config_path.read_text(encoding="utf-8")).get("scenarios", [])
        except FileNotFoundError:
            declared = []
        for scenario_name in declared:
            rows.append(_run_one(args.runner, config_path,
                                 scenarios_dir / f"{scenario_name}.yaml", algo))
            print(f"ran {algo} x {scenario_name} ({args.runner}) -> {rows[-1].status}", file=sys.stderr)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("# slam_basic benchmark matrix\n\n" + _render(rows), encoding="utf-8")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
