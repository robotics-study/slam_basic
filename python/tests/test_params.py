"""ParamSet: declaration validation (type / range / choices) and set() overrides."""

import pytest

from slam.core.params import ParamError, ParamSet


def _yaml(tmp_path, section="filtering") -> str:
    path = tmp_path / "algo.yaml"
    path.write_text(
        f"""
algorithm: algo
section: {section}
# Block-style scenarios list (a scalar block-sequence item — the real configs use
# this style, so the C++ mini-parser must handle it; flow lists stay covered by
# choices/waypoints elsewhere.)
scenarios:
  - corridor01_back_and_forth
params:
  - name: particles
    type: int
    default: 100
    min: 8
    max: 4096
    description: particle count
  - name: sigma
    type: float
    default: 0.1
    description: noise
  - name: adaptive
    type: bool
    default: false
    description: KLD flag
  - name: mode
    type: enum
    default: a
    choices: [a, b]
    description: enum param
""",
        encoding="utf-8",
    )
    return str(path)


def test_load_and_read(tmp_path) -> None:
    ps = ParamSet.from_yaml(_yaml(tmp_path))
    assert ps.algorithm == "algo" and ps.section == "filtering"
    assert ps.scenarios == ["corridor01_back_and_forth"]
    assert ps.get_int("particles") == 100
    assert ps.get_float("sigma") == 0.1
    assert ps.get_bool("adaptive") is False
    assert ps.get_string("mode") == "a"
    assert not ps.has("missing")


def test_unknown_section_rejected(tmp_path) -> None:
    with pytest.raises(ParamError):
        ParamSet.from_yaml(_yaml(tmp_path, section="nonsense"))


def test_type_mismatch_and_range(tmp_path) -> None:
    ps = ParamSet.from_yaml(_yaml(tmp_path))
    with pytest.raises(ParamError):  # int param read as float
        ps.get_float("particles")
    with pytest.raises(ParamError):
        ps.set("particles", "100")  # wrong type
    with pytest.raises(ParamError):
        ps.set("particles", 4)  # below min
    ps.set("particles", 256)
    assert ps.get_int("particles") == 256
    with pytest.raises(ParamError):
        ps.set("mode", "c")  # not in choices


def test_values_snapshot_is_sorted_by_trace_recorder(tmp_path) -> None:
    ps = ParamSet.from_yaml(_yaml(tmp_path))
    values = ps.values()
    assert set(values) == {"particles", "sigma", "adaptive", "mode"}
