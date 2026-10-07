"""grid_mapping tests — the log-odds update and the cells_along walk contract.

The unit tests pin the mechanics: a hit cell takes exactly +logit(p_hit), every
cell the segment passes through takes exactly −logit(p_hit) per pass, untouched
cells keep their prior 0.0 bit-for-bit (unknown ≠ occupied), and the DDA walk
follows the same fixed rules as core/sim.raycast (tie takes Y; an axis whose delta
is 0 never steps; a degenerate segment is just its own cell). The scenario run
pins the branch premise end-to-end: with noiseless odometry anchored to the
declared start pose, every estimated pose equals ground truth bit-for-bit (ATE and
RPE read ~0), while the map still misses exactly the cells no beam can ever reach —
the sealed interiors of the thick blocks and the enclosed corners.
"""

import io
import json
import math

from conftest import REPO_ROOT, grid_from

from slam.core.params import ParamDecl, ParamError, ParamSet
from slam.core.sim import build_episode
from slam.core.trace import TraceRecorder
from slam.core.types import Episode, Pose, Step
from slam.filtering.grid_mapping import GridMapping, cells_along
from slam.maps.loader import load_scenario


def _params(p_hit: float = 0.9) -> ParamSet:
    """A tiny config for unit tests (anchor sits inside the MINI raster)."""
    return ParamSet(
        "grid_mapping",
        "filtering",
        {
            "p_hit": ParamDecl("p_hit", "float", p_hit),
            "x0": ParamDecl("x0", "float", 0.5),
            "y0": ParamDecl("y0", "float", 0.5),
            "theta_deg": ParamDecl("theta_deg", "float", 0.0),
        },
    )


def test_config_defaults_and_range_validation() -> None:
    params = ParamSet.from_yaml(REPO_ROOT / "configs" / "filtering" / "grid_mapping.yaml")
    assert params.algorithm == "grid_mapping" and params.section == "filtering"
    assert params.get_float("p_hit") == 0.9
    # The anchor defaults are the tour's start cell center, exact binary floats.
    assert params.get_float("x0") == 5.875 and params.get_float("y0") == 4.375
    assert params.get_float("theta_deg") == 180.0
    # p_hit <= 0.5 makes the inverse model uninformative — out of range by contract.
    try:
        params.set("p_hit", 0.4)
        raise AssertionError("expected ParamError for p_hit below min")
    except ParamError:
        pass


def test_cells_along_walk_rules() -> None:
    """The walk mirrors raycast's DDA on the raw segment vector: an axis with zero
    delta never steps, a tMax tie takes Y, and a degenerate segment is just its own
    cell (the start IS the hit)."""
    grid = grid_from(["..", ".."])  # all free — this pins pure walk geometry

    # Due east from (0.5, 0.5): dy == 0 → y never steps; stops ON the target cell.
    assert cells_along(grid, (0.5, 0.5), (1.5, 0.5)) == [(1, 0), (1, 1)]
    # Due north: dx == 0 → x never steps.
    assert cells_along(grid, (0.5, 0.5), (0.5, 1.5)) == [(1, 0), (0, 0)]
    # Exact diagonal from the cell center: tMaxX == tMaxY at the shared corner and
    # the tie takes Y — the middle cell is (0, 0) (row index grows downward), NOT
    # (1, 1); a wrong tie rule would pin [(1,0),(1,1),(0,1)] here.
    assert cells_along(grid, (0.5, 0.5), (1.5, 1.5)) == [(1, 0), (0, 0), (0, 1)]
    # Degenerate: the endpoint sits in the start cell — that cell IS the hit.
    assert cells_along(grid, (0.5, 0.5), (0.75, 0.25)) == [(1, 0)]
    # A segment starting off-raster updates nothing (impossible with an exact pose,
    # but the walk's contract is explicit about it).
    assert cells_along(grid, (5.0, 5.0), (0.5, 0.5)) == []


def test_update_hits_passes_and_unknowns() -> None:
    """One hit +logit(p) on the endpoint cell; every passed-through cell −logit(p)
    per pass (exact floats — 0.0 − L − L is exactly −2L); a never-touched cell
    keeps its prior 0.0 bit-for-bit."""
    grid = grid_from([".#", ".#"])  # col 1 occupied, col 0 free
    est = GridMapping(_params())
    est.episode = Episode(steps=(), landmarks=None, grid=grid)

    L = math.log(9.0)  # logit(0.9)
    step = Step(
        t=0, gt=Pose(0.5, 0.5, 0.0), odom=None,
        scan=((1.0, 0.0), (0.0, 1.0)), obs=None,
    )
    est.update(step, None)

    field = est._l
    assert field is not None
    # Beam 1 due east ends inside the occupied cell (1, 1); beam 2 due north ends
    # inside the occupied cell (0, 0). Both pass through the robot's own cell.
    assert field[1][1] == L and field[0][0] == L
    assert field[1][0] == -2.0 * L  # passed through twice — exactly −2L
    assert field[0][1] == 0.0  # never touched: the prior stays, bit-for-bit

    # A point inside the robot's own cell makes that cell itself the hit.
    est.update(
        Step(t=1, gt=Pose(0.5, 0.5, 0.0), odom=None, scan=((0.25, 0.0),), obs=None), None
    )
    assert field[1][0] == -L  # −2L + L


def test_update_events_are_row_major_sorted() -> None:
    """pose_estimated precedes map_updated at the same t, and the diff lists cells
    row-major sorted (the byte order C++'s std::set walk emits)."""
    grid = grid_from([".#", ".#"])
    est = GridMapping(_params())
    episode = Episode(steps=(), landmarks=None, grid=grid)
    est.episode = episode

    buf = io.StringIO()
    recorder = TraceRecorder(buf)
    step = Step(
        t=0, gt=Pose(0.5, 0.5, 0.0), odom=None,
        scan=((1.0, 0.0), (0.0, 1.0)), obs=None,
    )
    est.update(step, recorder)
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert [e["event"] for e in events] == ["pose_estimated", "map_updated"]
    assert events[0]["t"] == 0 and events[1]["t"] == 0
    assert events[0]["pose"] == [0.5, 0.5, 0.0]
    cells = events[1]["cells"]
    # Only TOUCHED cells are emitted, row-major sorted: row 0 before row 1, col
    # ascending inside a row — the byte order C++'s std::set walk emits.
    assert [[r, c] for r, c, _ in cells] == [[0, 0], [1, 0], [1, 1]]
    L = math.log(9.0)
    assert cells[0][2] == L and cells[1][2] == -2.0 * L and cells[2][2] == L


def test_run_on_scenario_is_exact_and_honest() -> None:
    """The branch premise end-to-end: noiseless odometry anchored to the declared
    start pose reproduces ground truth bit-for-bit (pose[0] is exact, so ATE/RPE
    read float-noise ~0), while the map keeps its honest ceiling — the sealed block
    interiors keep prior 0.0 and IoU sits near but below 1."""
    sc = load_scenario(REPO_ROOT / "maps" / "scenarios" / "office01_tour.yaml")
    episode = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        sc.sigma_xy, sc.sigma_theta, sc.seed,
    )
    params = ParamSet.from_yaml(REPO_ROOT / "configs" / "filtering" / "grid_mapping.yaml")
    est = GridMapping(params)
    result = est.run(episode, None)

    # Pose is GIVEN: the anchor equals ground truth bit-for-bit and noiseless odometry
    # keeps it there (ATE/RPE are float noise, not drift).
    assert result.poses[0] == episode.steps[0].gt
    from slam.core.metrics import evaluate

    metrics = evaluate(result, episode)
    assert metrics["ate_rmse"] < 1e-9 and metrics["rpe_rmse"] < 1e-9

    field = result.grid.log_odds
    # Sealed interiors (never reachable by any beam): prior stays EXACTLY 0.0.
    assert field[26][37] == 0.0 and field[9][7] == 0.0 and field[0][47] == 0.0
    # Everything observable got occupied evidence: IoU lands in the honest band
    # (structural blind spots + a handful of noise-flipped cells keep it below 1).
    assert 0.85 < metrics["map_iou"] < 0.95
