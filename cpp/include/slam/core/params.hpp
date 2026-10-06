#pragma once

#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace slam::core {

using ParamValue = std::variant<int, double, bool, std::string>;

// Loads and validates an algorithm's parameter set from its configs yaml
// (spec/param_schema.json). Validation (type match, [min,max] range, enum choices)
// runs at load time and throws std::runtime_error ("param error: ...") on failure —
// this is the "param validation failure" contract. A config also declares the
// scenario slugs its algorithm runs on (`scenarios:`); the matrix runner and web
// exporter route per config instead of running every algorithm against every scenario.
//
// Unlike a planning repo, an estimator here may itself be stochastic (particle
// resampling): such algorithms declare their own `seed` int parameter and the demo
// passes the scenario seed verbatim through set() — validated exactly like a
// default (type mismatch / range failure throws).
class ParamSet {
 public:
  static ParamSet from_yaml(const std::string& path);

  int get_int(const std::string& name) const;
  double get_float(const std::string& name) const;
  bool get_bool(const std::string& name) const;
  std::string get_string(const std::string& name) const;
  bool has(const std::string& name) const;

  // Override one declared value (the demo injecting the scenario seed into a
  // stochastic algorithm's `seed` param); validated like a default.
  void set(const std::string& name, const ParamValue& value);

  const std::map<std::string, ParamValue>& values() const { return values_; }
  const std::string& algorithm() const { return algorithm_; }
  const std::string& section() const { return section_; }
  const std::vector<std::string>& scenarios() const { return scenarios_; }

 private:
  struct Decl {
    std::string type;
    std::optional<double> min, max;
    std::vector<std::string> choices;
  };
  std::string algorithm_;
  std::string section_;
  std::vector<std::string> scenarios_;
  std::map<std::string, ParamValue> values_;
  std::map<std::string, Decl> decls_;
};

}  // namespace slam::core
