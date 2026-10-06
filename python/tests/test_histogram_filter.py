"""histogram_filter tests — the discrete-pose Bayes filter contract.

Two scenario runs pin the two halves of the story. corridor01 (lattice-matched,
uneven pillars) pins exactness: the belief collapses to the TRUE state (a single
cell × heading bin) and the estimated pose equals ground truth bit-for-bit.
corridor02 (periodic baffles, range-limited signature) pins honest multi-hypothesis
behaviour: the first scan leaves exactly three equally-massed position hypotheses,
the estimate starts at their mean — NOT at ground truth — and only asymmetric
structure entering the sensor range collapses the belief back onto the true cell.
The unit tests pin the mechanics both runs rest on: child-of-movement with slip
mix + slide semantics, lattice-exact heading rotation, and param validation.

The mini-grid is deliberately ASYMMETRIC (three free cells, no mirror partner): a
single scan must exclude every wrong state. The earlier draft ["#..", ".#."] failed
exactly here — cell (1,0)@θ=0 and cell (0,1)@θ=−π fire the same two 0.5 m rays on
rotated beams, so their likelihoods were identical and the belief never collapsed.
"""

import io
import json
import math

import numpy as np
from conftest import REPO_ROOT, grid_from

from slam.core.params import ParamDecl, ParamError, ParamSet
from slam.core.sim import build_episode
from slam.core.trace import TraceRecorder
from slam.core.types import Episode, Pose, SensorConfig, Twist
from slam.filtering.histogram_filter import HistogramFilter
from slam.maps.loader import load_scenario

# Free cells row-major: (0,1) → center (1.5, 1.5), (0,2) → (2.5, 1.5), (1,2) → (2.5, 0.5).
MINI = ["#..", "##."]


def _params(p_slip: float = 0.1, beams: int = 5, fov_deg: float = 360.0,
            range_max: float = 4.0, sigma_range: float = 0.05) -> ParamSet:
    """A tiny-lattice config for unit tests (5 beams over 360° → lattice step π/2,
    B = 4 heading bins, off = 2)."""
    return ParamSet(
        "histogram_filter",
        "filtering",
        {
            "p_slip": ParamDecl("p_slip", "float", p_slip),
            "beams": ParamDecl("beams", "int", beams),
            "fov_deg": ParamDecl("fov_deg", "float", fov_deg),
            "range_max": ParamDecl("range_max", "float", range_max),
            "sigma_range": ParamDecl("sigma_range", "float", sigma_range),
        },
    )


def test_config_defaults_and_range_validation() -> None:
    params = ParamSet.from_yaml(REPO_ROOT / "configs" / "filtering" / "histogram_filter.yaml")
    assert params.algorithm == "histogram_filter" and params.section == "filtering"
    assert params.get_float("p_slip") == 0.1
    assert params.get_int("beams") == 361 and params.get_float("fov_deg") == 360.0
    assert params.get_float("range_max") == 6.0 and params.get_float("sigma_range") == 0.05
    # Out-of-range sets raise (the demo injects the scenario sensor through set()).
    try:
        params.set("beams", 362)
        raise AssertionError("expected ParamError for beams > max")
    except ParamError:
        pass


def test_uniform_prior_collapses_to_true_state() -> None:
    """One noise-free scan collapses the uniform prior onto exactly one state and
    the readout pose equals the ground-truth pose bit-for-bit (the lattice makes the
    true pose exactly representable)."""
    grid = grid_from(MINI)
    gt = Pose(1.5, 1.5, 0.0)  # center of cell (0,1), the first free cell
    sensor = SensorConfig(type="beam", beams=5, fov_deg=360.0, range_max=4.0, sigma_range=0.05)
    episode = build_episode(grid, [(gt.x, gt.y)], 0.5, sensor, None, 0.0, 0.0, seed=7)
    assert len(episode.steps) == 1

    est = HistogramFilter(_params())
    result = est.run(episode, None)
    # Position exact to the bit; heading is the lattice bin for θ = 0 (exact 0.0).
    assert result.poses[0].x == gt.x and result.poses[0].y == gt.y
    assert result.poses[0].theta == 0.0


def test_trace_events_and_cell_marginals() -> None:
    """run() echoes step_observed before the estimate events; belief_updated carries
    EVERY cell row-major (occupied cells emit exactly 0.0 — they are not states)."""
    grid = grid_from(MINI)
    sensor = SensorConfig(type="beam", beams=5, fov_deg=360.0, range_max=4.0, sigma_range=0.05)
    episode = build_episode(grid, [(1.5, 1.5)], 0.5, sensor, None, 0.0, 0.0, seed=7)

    buf = io.StringIO()
    est = HistogramFilter(_params())
    est.run(episode, TraceRecorder(buf))
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    assert [e["event"] for e in events] == ["step_observed", "pose_estimated", "belief_updated"]
    cells = events[2]["cells"]
    # Every cell of the 2x3 raster, row-major.
    assert [[r, c] for r, c, _ in cells] == [[r, c] for r in range(2) for c in range(3)]
    # Occupied cells (0,0), (1,0), (1,1) are not states: exactly 0.0.
    assert cells[0][2] == 0.0 and cells[3][2] == 0.0 and cells[4][2] == 0.0
    # The free cell the robot sits on carries the whole mass after one scan.
    assert abs(cells[1][2] - 1.0) < 1e-12


def test_predict_slips_and_rotates_on_the_lattice() -> None:
    """Child-of-movement: (1−p_slip) rides the twist onto the child state, p_slip
    stays put; a move into an occupied cell does not move (slide); a dtheta of one
    lattice step rotates the heading bin by exactly one (self-heal)."""
    # free row-major: (0,1),(0,2),(1,2) → centers (1.5,1.5), (2.5,1.5), (2.5,0.5)
    grid = grid_from(MINI)
    est = HistogramFilter(_params(p_slip=0.1))
    est.episode = Episode(steps=(), landmarks=None, grid=grid)
    est._init_state()  # private on purpose — this unit-tests the predict step
    bins, off = est._bins, est._off
    assert bins == 4 and off == 2
    w_bin = est._w_bin
    fi_01, fi_02 = 0, 1  # free cells (0,1) → (1.5,1.5) and (0,2) → (2.5,1.5)

    # δ at (cell (0,1), heading bin 2 ↔ θ = 0). Move +x into free cell (0,2):
    # mass splits 0.9 onto the child state, 0.1 stays — exact floats.
    est._belief = np.zeros(est._n_states)
    est._belief[fi_01 * bins + 2] = 1.0
    est._predict(Twist(dx=1.0, dy=0.0, dtheta=0.0))
    assert est._belief[fi_02 * bins + 2] == 0.9
    assert est._belief[fi_01 * bins + 2] == 0.1

    # Slide: from (0,2) a further +x move leaves the grid — the hypothesis does not
    # move; both terms land on the SAME state and sum back to exactly 1.
    est._belief = np.zeros(est._n_states)
    est._belief[fi_02 * bins + 2] = 1.0
    est._predict(Twist(dx=1.0, dy=0.0, dtheta=0.0))
    assert est._belief[fi_02 * bins + 2] == 1.0

    # Heading rotation: a twist of exactly one lattice step rotates bin 2 → 3
    # (0.9 child / 0.1 stay — the heading axis self-heals by construction).
    est._predict(Twist(dx=0.0, dy=0.0, dtheta=w_bin))
    assert est._belief[fi_02 * bins + 3] == 0.9
    assert est._belief[fi_02 * bins + 2] == 0.1


def test_run_on_scenario_is_exact() -> None:
    """The showcase run: lattice-matched corridor, the belief collapses onto the true
    state at t=0 and rides it — ATE/RPE are zero to float rounding."""
    sc = load_scenario(REPO_ROOT / "maps" / "scenarios" / "corridor01_back_and_forth.yaml")
    episode = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        sc.sigma_xy, sc.sigma_theta, sc.seed,
    )
    params = ParamSet.from_yaml(REPO_ROOT / "configs" / "filtering" / "histogram_filter.yaml")
    est = HistogramFilter(params)
    result = est.run(episode, None)
    assert len(result.poses) == len(episode.steps)

    total = 0.0
    for est_pose, step in zip(result.poses, episode.steps, strict=True):
        dx, dy = est_pose.x - step.gt.x, est_pose.y - step.gt.y
        total += dx * dx + dy * dy
    ate = math.sqrt(total / len(result.poses))
    assert ate < 1e-9  # exact to float rounding (the demo reports ~1.3e-16)


def test_run_on_ambiguous_scenario_resolves_hypotheses() -> None:
    """The ambiguity showcase: periodic baffles spaced 4 cells apart and a 2.5 m
    range that never reaches the second-next face make the first scan's signature
    an exact translate at c8/c12/c16 — so the belief keeps EXACTLY three nonzero
    cells with bit-identical mass, and the readout is their mean (not ground
    truth). Only when asymmetric structure enters the range does the belief
    collapse onto the true cell again; from that step on the estimate is exact to
    the bit again. The scenario sensor must be injected like the demo does — the
    config default range_max 6.0 would dissolve the ambiguity by design."""
    sc = load_scenario(REPO_ROOT / "maps" / "scenarios" / "corridor02_ambiguous.yaml")
    episode = build_episode(
        sc.grid, list(sc.waypoints), sc.step_meters, sc.sensor, None,
        sc.sigma_xy, sc.sigma_theta, sc.seed,
    )
    params = ParamSet.from_yaml(REPO_ROOT / "configs" / "filtering" / "histogram_filter.yaml")
    # The demo's injection contract — the scenario sensor overrides every default.
    params.set("beams", int(sc.sensor.beams))
    params.set("fov_deg", float(sc.sensor.fov_deg))
    params.set("range_max", float(sc.sensor.range_max))
    params.set("sigma_range", float(sc.sensor.sigma_range))

    buf = io.StringIO()
    est = HistogramFilter(params)
    result = est.run(episode, TraceRecorder(buf))
    assert len(result.poses) == len(episode.steps)

    # t=0: exactly three nonzero cells (the translation triple), bit-identical mass,
    # everything else exactly 0.0 — the ambiguity is exact, not smeared.
    events = [json.loads(line) for line in buf.getvalue().splitlines()]
    first_belief = next(e for e in events if e["event"] == "belief_updated")
    nonzero = [(r, c, p) for r, c, p in first_belief["cells"] if p > 0.0]
    assert [(r, c) for r, c, _ in nonzero] == [(3, 8), (3, 12), (3, 16)]
    masses = [p for _, _, p in nonzero]
    assert max(masses) - min(masses) == 0.0
    # The readout is the mean of the three hypotheses — visibly NOT ground truth.
    est0, gt0 = result.poses[0], episode.steps[0].gt
    assert abs(est0.x - 6.25) < 1e-9 and abs(est0.y - 1.75) < 1e-9
    assert est0.x != gt0.x

    # Once the true position's second-next baffle face enters range_max, the wrong
    # hypotheses eat sentinel penalties and die; from step 25 on the belief is a
    # single state again and the estimate equals ground truth bit-for-bit.
    for t in range(25, len(result.poses)):
        assert result.poses[t].x == episode.steps[t].gt.x
        assert result.poses[t].y == episode.steps[t].gt.y
