#!/usr/bin/env python3
"""Export web-site data assets: grid maps + scenarios as JSON, demo traces gzipped.

The docs SPA runs every estimator live in the browser; the exported traces are not
replayed by pages — they remain the ground truth the TypeScript engines are checked
against. The C++ and Python demos emit field-for-field identical event streams, so
web assets are generated from the PYTHON demo alone and stored gzipped for static
serving (gzip mtime pinned to 0 so identical inputs produce byte-identical files —
stable git diffs).

Traces are keyed by SCENARIO slug (one map can host several scenarios); each
algorithm's config declares the scenarios it runs on (`scenarios:`), and each
scenario's `map:` field names which map JSON to export. Scenarios themselves are
exported verbatim (yaml -> json) so the browser engines read the SAME inputs.

Usage:
    python tools/web_export/export_web_assets.py --algos histogram_filter,grid_mapping
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
DATA_DIR = REPO / "document" / "public" / "data"
# Web replay cap — traces larger than this are never committed (unplayable size).
MAX_EVENTS = 200_000


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    """Minimal PGM reader (P2/P5)."""
    data = path.read_bytes()
    if data[:2] == b"P2":
        tokens: list[str] = []
        for line in data.decode("ascii").splitlines():
            line = line.split("#", 1)[0]
            tokens.extend(line.split())
        width, height, _maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
        pixels = [int(t) for t in tokens[4:4 + width * height]]
        return width, height, pixels
    if data[:2] == b"P5":
        idx = 2
        values: list[int] = []
        while len(values) < 3:
            while idx < len(data) and data[idx:idx + 1].isspace():
                idx += 1
            if data[idx:idx + 1] == b"#":
                while idx < len(data) and data[idx:idx + 1] != b"\n":
                    idx += 1
                continue
            start = idx
            while idx < len(data) and not data[idx:idx + 1].isspace():
                idx += 1
            values.append(int(data[start:idx]))
        idx += 1  # single whitespace byte terminating the header
        width, height, _maxval = values
        return width, height, list(data[idx:idx + width * height])
    raise ValueError(f"unsupported PGM format: {path}")


def export_map(name: str) -> None:
    map_yaml = REPO / "maps" / "grid" / f"{name}.yaml"
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image = map_yaml.parent / meta["image"]
    width, height, pixels = read_pgm(image)
    # ROS style: 0 (black) = occupied, 255 (white) = free. No ambiguous values —
    # the spec allows only 0/255 — so a midpoint split is exact.
    rows = [
        "".join("#" if pixels[r * width + c] < 128 else "." for c in range(width))
        for r in range(height)
    ]
    out = DATA_DIR / "maps" / f"{name}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    origin = meta["origin"]
    payload = {
        "name": name,
        "width": width,
        "height": height,
        "resolution": float(meta["resolution"]),
        "free_thresh": float(meta.get("free_thresh", 0.65)),
        "origin": [float(origin[0]), float(origin[1])],
        "rows": rows,
    }
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"map: {out.relative_to(REPO)} ({width}x{height})")


def export_scenario(name: str) -> None:
    """Scenario yaml -> json verbatim (json can't hold tuples/comments — the yaml is
    authored so plain scalars/lists are all it contains)."""
    src = REPO / "maps" / "scenarios" / f"{name}.yaml"
    raw = yaml.safe_load(src.read_text(encoding="utf-8"))
    out = DATA_DIR / "scenarios" / f"{name}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(raw, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"scenario: {out.relative_to(REPO)}")


def params_file(algo: str, overrides: dict[str, str], tmp: Path) -> Path:
    """Config yaml path — with overrides, a copy whose defaults are replaced.

    Demo default budgets can be too large for web replay files, so web assets are
    re-run with smaller budgets; parameters are recorded in the trace's run_started
    event, so replay and parity stay exact either way."""
    matches = sorted((REPO / "configs").rglob(f"{algo}.yaml"))
    if len(matches) != 1:
        raise SystemExit(
            f"expected exactly one configs/<section>/{algo}.yaml, found {len(matches)}"
        )
    src = matches[0]
    if not overrides:
        return src
    doc = yaml.safe_load(src.read_text(encoding="utf-8"))
    for p in doc["params"]:
        if p["name"] in overrides:
            raw = overrides[p["name"]]
            p["default"] = int(raw) if p["type"] == "int" else float(raw)
    out = tmp / f"{algo}.yaml"
    out.write_text(yaml.safe_dump(doc, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return out


def export_trace(algo: str, scenario: Path, params_path: Path) -> None:
    with tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False) as tmp:
        trace = Path(tmp.name)
    try:
        # The demo is a package module (relative imports) — run it as a module.
        cmd = [
            sys.executable, "-m", f"slam.demos.demo_{algo}",
            "--scenario", str(scenario),
            "--params", str(params_path),
            "--trace", str(trace),
        ]
        subprocess.run(cmd, check=True, cwd=REPO,
                       env={**os.environ, "PYTHONPATH": str(REPO / "python")},
                       stdout=subprocess.DEVNULL)
        events = sum(1 for _ in trace.open())
        if events > MAX_EVENTS:
            raise SystemExit(
                f"{algo}/{scenario.stem}/py: {events} events > {MAX_EVENTS} — "
                "reduce the demo budget before exporting for the web"
            )
        out = DATA_DIR / "traces" / algo / f"{scenario.stem}.py.jsonl.gz"
        out.parent.mkdir(parents=True, exist_ok=True)
        # mtime=0 keeps identical inputs byte-identical (stable git diffs).
        with trace.open("rb") as src, gzip.GzipFile(out, "wb", mtime=0) as dst:
            shutil.copyfileobj(src, dst)
        print(f"trace: {out.relative_to(REPO)} ({events} events)")
    finally:
        trace.unlink(missing_ok=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="export web data assets for document/")
    parser.add_argument(
        "--algos", default="", help="comma-separated algorithm slugs (empty: maps/scenarios only)"
    )
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="param default override for the demo run (repeatable)")
    args = parser.parse_args()
    algos = [a for a in args.algos.split(",") if a]
    overrides = dict(kv.split("=", 1) for kv in args.set)

    # Maps and scenarios are static assets — every one is exported. Traces exist only
    # for --algos algorithms, each routed onto the scenarios ITS config declares.
    # Raw reads only — this tool stays free of slam.core.
    scenario_map: dict[str, str] = {}   # scenario slug -> grid map name
    per_algo: list[tuple[str, list[str]]] = []
    for scenario_path in sorted((REPO / "maps" / "scenarios").glob("*.yaml")):
        raw = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
        scenario_map[scenario_path.stem] = Path(str(raw["map"])).stem
    for algo in algos:
        matches = sorted((REPO / "configs").rglob(f"{algo}.yaml"))
        if len(matches) != 1:
            raise SystemExit(
                f"expected exactly one configs/<section>/{algo}.yaml, found {len(matches)}"
            )
        declared = yaml.safe_load(matches[0].read_text(encoding="utf-8")).get("scenarios", [])
        per_algo.append((algo, list(declared)))

    for name in sorted(scenario_map):
        export_scenario(name)
    for map_name in sorted(set(scenario_map.values())):
        export_map(map_name)
    with tempfile.TemporaryDirectory() as tmp:
        for algo, names in per_algo:
            for name in names:
                export_trace(algo, REPO / "maps" / "scenarios" / f"{name}.yaml",
                             params_file(algo, overrides, Path(tmp)))


if __name__ == "__main__":
    main()
