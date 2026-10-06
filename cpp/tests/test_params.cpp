// ParamSet: declaration validation (type / range / choices) and set() overrides —
// the C++ mirror of python/tests/test_params.py.

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "slam/core/params.hpp"
#include "test_util.hpp"

using slam::core::ParamSet;

namespace {

std::string algo_yaml(const std::string& section = "filtering") {
  return slam::test::write_temp(
      // Block-style scenarios list (a scalar block-sequence item — the real configs
      // use this style, so the parser must handle it; flow lists stay covered by
      // choices/waypoints elsewhere).
      "algo.yaml", "algorithm: algo\nsection: " + section +
                       "\nscenarios:\n  - corridor01_back_and_forth\nparams:\n"
                       "  - name: particles\n    type: int\n    default: 100\n    min: 8\n    max: "
                       "4096\n    description: particle count\n"
                       "  - name: sigma\n    type: float\n    default: 0.1\n    description: "
                       "noise\n"
                       "  - name: adaptive\n    type: bool\n    default: false\n    description: "
                       "KLD flag\n"
                       "  - name: mode\n    type: enum\n    default: a\n    choices: [a, b]\n    "
                       "description: enum param\n");
}

}  // namespace

TEST(Params, LoadsDeclaredDefaults) {
  ParamSet ps = ParamSet::from_yaml(algo_yaml());
  EXPECT_EQ(ps.algorithm(), "algo");
  EXPECT_EQ(ps.section(), "filtering");
  ASSERT_EQ(ps.scenarios().size(), 1u);
  EXPECT_EQ(ps.scenarios()[0], "corridor01_back_and_forth");
  EXPECT_EQ(ps.get_int("particles"), 100);
  EXPECT_DOUBLE_EQ(ps.get_float("sigma"), 0.1);
  ASSERT_FALSE(ps.get_bool("adaptive"));  // default is false
  EXPECT_EQ(ps.get_string("mode"), "a");
  EXPECT_FALSE(ps.has("missing"));
}

TEST(Params, UnknownSectionRejected) {
  EXPECT_THROW(ParamSet::from_yaml(algo_yaml("nonsense")), std::runtime_error);
}

TEST(Params, TypeMismatchAndRange) {
  ParamSet ps = ParamSet::from_yaml(algo_yaml());
  EXPECT_THROW(ps.get_float("particles"), std::runtime_error);  // int param read as float
  EXPECT_THROW(ps.set("particles", std::string("100")), std::runtime_error);  // wrong type
  EXPECT_THROW(ps.set("particles", 4), std::runtime_error);                   // below min
  ps.set("particles", 256);
  EXPECT_EQ(ps.get_int("particles"), 256);
  EXPECT_THROW(ps.set("mode", std::string("c")), std::runtime_error);  // not in choices
}

TEST(Params, ValuesSnapshotCarriesAllDecls) {
  ParamSet ps = ParamSet::from_yaml(algo_yaml());
  const auto& values = ps.values();
  ASSERT_EQ(values.size(), 4u);
  EXPECT_TRUE(values.count("particles") && values.count("sigma") && values.count("adaptive") &&
              values.count("mode"));
}
