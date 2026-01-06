// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_EXECUTOR__EFFECTFAILURE_HPP_
#define PLANSYS2_EXECUTOR__EFFECTFAILURE_HPP_

#include <string>
#include <vector>
#include <sstream>
#include <optional>

#include <rclcpp/rclcpp.hpp>

namespace plansys2
{

/**
 * @struct EffectFailure
 *
 * @brief Describes a failed effect check (predicate or function) during action execution.
 *
 * This structure is used by the Behavior Tree nodes
 * `CheckAtStartEffect` and `CheckAtEndEffect` to report to the executor
 * which expected action effects did not hold and why.
 */
struct EffectFailure
{
  enum class EffectType { PREDICATE, FUNCTION };

  EffectType type;
  std::string symbol_name;                      // predicate or function name
  std::vector<std::string> parameters;

  // Predicate-specific
  std::optional<bool> negated;

  // Function-specific
  std::optional<double> actual_value;    // sensed value
  std::optional<double> expected_value;  // expected value

  enum class Reason
  {
    FAILED,             // predicate/function does not hold
    SENSING_ERROR,      // sensing plugin exists but returned an error
    SENSING_MISSING,    // no sensing plugin available
    PARSE_ERROR         // unsupported or invalid PDDL effect structure
  };

  Reason reason;

  // Factory for functions
  static EffectFailure FunctionFailure(
    const std::string & symbol_name,
    const std::vector<std::string> & params,
    double actual,
    double expected,
    Reason reason = Reason::FAILED)
  {
    return EffectFailure{
      EffectType::FUNCTION,
      symbol_name,
      params,
      std::nullopt,             // negated irrelevant for functions
      actual,
      expected,
      reason
    };
  }

  // Factory for predicates
  static EffectFailure PredicateFailure(
    const std::string & symbol_name,
    const std::vector<std::string> & params,
    bool negated,
    Reason reason = Reason::FAILED)
  {
    return EffectFailure{
      EffectType::PREDICATE,
      symbol_name,
      params,
      negated,
      std::nullopt,
      std::nullopt,
      reason
    };
  }

  // Factory for parsing error
  static EffectFailure ParseError()
  {
    return SensingError("", {}, EffectType::PREDICATE, Reason::PARSE_ERROR);
  }

  // Factory for errors
  static EffectFailure SensingError(
    const std::string & symbol_name,
    const std::vector<std::string> & params,
    EffectType type,
    Reason reason)
  {
    return EffectFailure{
      type,
      symbol_name,
      params,
      std::nullopt,
      std::nullopt,
      std::nullopt,
      reason
    };
  }

  // Deleted default constructor: use factory methods instead
  EffectFailure() = delete;
};

inline std::string toString(const EffectFailure& f)
{
  std::stringstream ss;

  if (f.reason == EffectFailure::Reason::PARSE_ERROR) {
    ss << "Effect parsing failed: unsupported or invalid PDDL effect structure";
    return ss.str();
  }

  ss << (f.type == EffectFailure::EffectType::PREDICATE ? "Predicate: " : "Function: ");
  ss << f.symbol_name << "(";
  for (size_t i = 0; i < f.parameters.size(); ++i) {
    ss << f.parameters[i];
    if (i + 1 < f.parameters.size()) ss << ", ";
  }
  ss << ")";

  if (f.reason == EffectFailure::Reason::FAILED) {
    if (f.type == EffectFailure::EffectType::PREDICATE) {
      if (f.negated.has_value()){
        ss << " | Expected: " << (f.negated.value() ? "FALSE | Got: TRUE" : "TRUE | Got: FALSE");
      }
    } else if (f.type == EffectFailure::EffectType::FUNCTION) {
      if (f.actual_value.has_value() && f.expected_value.has_value()) {
        ss << " | Actual: " << f.actual_value.value()
           << " | Expected: " << f.expected_value.value();
      }
    }
  }

  ss << " | Reason: ";
  switch (f.reason)
  {
    case EffectFailure::Reason::FAILED:
      ss << "FAILED (predicate/function does not hold)";
      break;

    case EffectFailure::Reason::SENSING_ERROR:
      ss << "SENSING_ERROR (sensing plugin returned an error)";
      break;

    case EffectFailure::Reason::SENSING_MISSING:
      ss << "SENSING_MISSING (no sensing plugin available)";
      break;
  }

  return ss.str();
}

inline void logEffectFailures(
  const rclcpp::Logger& logger, 
  const std::string& where, // start or end effects
  const std::vector<EffectFailure>& failures)
{
  for (const auto& f : failures) {
    RCLCPP_ERROR(logger, "%s %s", where.c_str(), toString(f).c_str());
  }
}

} // namespace plansys2

#endif // PLANSYS2_EXECUTOR__EFFECTFAILURE_HPP_
