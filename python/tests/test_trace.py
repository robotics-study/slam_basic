"""Trace wire contract: field order, seq monotonicity, sorted maps, int/float bytes."""

import io
import json

from slam.core.trace import TraceRecorder, open_trace
from slam.core.types import LandmarkEstimate, LandmarkObs, Pose, SensorConfig, Step, Twist


def test_event_field_order_and_types() -> None:
    buf = io.StringIO()
    rec = TraceRecorder(buf)
    sensor = SensorConfig(type="beam", range_max=6.0, sigma_range=0.05, beams=90, fov_deg=180.0)
    rec.run_started("algo_x", "maps/scenarios/x.yaml", {"zeta": 1, "alpha": 2.5}, 42, sensor, None)
    rec.step_observed(Step(t=0, gt=Pose(1.0, 2.0, 0.5), odom=None, scan=[(0.5, 0.0)], obs=None))
    rec.pose_estimated(0, Pose(1.1, 2.1, 0.4), cov=[0.1, 0.1, 0.02])
    rec.run_finished({"rpe_rmse": 0.2, "ate_rmse": 0.1})

    lines = buf.getvalue().splitlines()
    assert len(lines) == 4
    events = [json.loads(line, object_pairs_hook=list) for line in lines]

    started = dict(events[0])
    keys = [k for k, _ in events[0]]
    # Fixed field order (the C++ recorder emits the identical order).
    assert keys == ["seq", "event", "algorithm", "scenario", "params", "seed", "sensor"]
    assert started["seq"] == 0 and started["event"] == "run_started"
    # params serialized sorted by key; int stays int on the wire.
    assert [k for k, _ in started["params"]] == ["alpha", "zeta"]
    assert started["seed"] == 42
    assert [k for k, _ in started["sensor"]] == [
        "type", "beams", "fov_deg", "range_max", "sigma_range"
    ]

    keys = [k for k, _ in events[1]]
    assert keys == ["seq", "event", "t", "gt", "scan"]  # odom omitted at t=0; seq increments
    step = dict(events[1])
    assert step["t"] == 0 and step["gt"] == [1.0, 2.0, 0.5] and step["scan"] == [[0.5, 0.0]]

    keys = [k for k, _ in events[2]]
    assert keys == ["seq", "event", "t", "pose", "cov"]
    pose_ev = dict(events[2])
    assert pose_ev["pose"] == [1.1, 2.1, 0.4] and pose_ev["cov"] == [0.1, 0.1, 0.02]

    keys = [k for k, _ in events[3]]
    assert keys == ["seq", "event", "metrics"]
    finished = dict(events[3])
    assert [k for k, _ in finished["metrics"]] == ["ate_rmse", "rpe_rmse"]  # sorted


def test_step_observed_landmark_fields(tmp_path) -> None:
    path = tmp_path / "t.jsonl"
    with open_trace(str(path)) as rec:
        rec.step_observed(
            Step(
                t=3,
                gt=Pose(0.0, 0.0, 0.0),
                odom=Twist(0.5, 0.0, 0.1),
                scan=None,
                obs=(LandmarkObs(id=2, bearing=0.3, range=2.5),),
            )
        )
        rec.landmarks_updated(3, [LandmarkEstimate(id=2, x=1.0, y=2.0)])
    lines = path.read_text(encoding="utf-8").splitlines()
    ev = json.loads(lines[0], object_pairs_hook=list)
    d = dict(ev)
    assert [k for k, _ in ev] == ["seq", "event", "t", "gt", "odom", "obs"]
    obs0 = dict(d["obs"][0])
    assert [k for k, _ in d["obs"][0]] == ["id", "bearing", "range"]
    assert obs0["id"] == 2 and obs0["bearing"] == 0.3 and obs0["range"] == 2.5
    # sx/sy omitted when absent; present order fixed.
    ev2 = json.loads(lines[1], object_pairs_hook=list)
    d2 = dict(ev2)
    assert [k for k, _ in ev2] == ["seq", "event", "t", "estimated"]
    assert [k for k, _ in d2["estimated"][0]] == ["id", "x", "y"]
