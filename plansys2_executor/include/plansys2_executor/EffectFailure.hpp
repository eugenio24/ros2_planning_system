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

#ifndef PLANSYS2_EXECUTOR__EFFECTFAILURE_HPP
#define PLANSYS2_EXECUTOR__EFFECTFAILURE_HPP

#include <string>
#include <vector>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

namespace plansys2
{

/**
 * @struct EffectFailure
 *
 * @brief Describes a failed effect check during action execution.
 *
 * This structure is used by the Behavior Tree nodes
 * `CheckAtStart` and `CheckAtEnd` to report to the executor
 * which expected action effects did not hold and why.
 */
struct EffectFailure
{
  std::string predicate;
  std::vector<std::string> parameters;
  bool negated{false};  // predicate was expected to be negated

  enum class Reason
  {
    FAILED,             // predicate does not hold
    SENSING_ERROR,      // sensing plugin exists but returned an error
    SENSING_MISSING     // no sensing plugin available
  };

  Reason reason{Reason::FAILED};
};

inline std::string toString(const EffectFailure& f)
{
  std::stringstream ss;

  ss << "Predicate: " << f.predicate << "(";
  for (size_t i = 0; i < f.parameters.size(); ++i)
  {
    ss << f.parameters[i];
    if (i + 1 < f.parameters.size()) ss << ", ";
  }
  ss << ")";

  ss << " | Expected: " << (f.negated ? "NOT true" : "true");

  ss << " | Reason: ";
  switch (f.reason)
  {
    case EffectFailure::Reason::FAILED:
      ss << "FAILED (predicate does not hold)";
      break;

    case EffectFailure::Reason::SENSING_ERROR:
      ss << "SENSOR_ERROR (sensing plugin returned an error)";
      break;

    case EffectFailure::Reason::SENSING_MISSING:
      ss << "SENSOR_MISSING (no sensing plugin available)";
      break;
  }

  return ss.str();
}

inline void logEffectFailures(
  const rclcpp::Logger& logger, 
  const std::string& where, // start or end
  const std::vector<EffectFailure>& failures)
{
  for (const auto& f : failures)
  {
    RCLCPP_ERROR(logger, "%s %s", where.c_str(), toString(f).c_str());
  }
}

} // namespace plansys2

#endif // PLANSYS2_EXECUTOR__EFFECTFAILURE_HPP
