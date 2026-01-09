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

#ifndef PLANSYS2_EXECUTOR__UTILS_HPP_
#define PLANSYS2_EXECUTOR__UTILS_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_pddl_parser/Utils.hpp"

namespace plansys2
{

// Numeric comparison tolerance (epsilon) constant
inline constexpr double kNumericToleranceEPS = 1e-6;

struct ParsedEffect
{
  std::string type;      // "predicate" | "function"
  std::string name;      // predicate / function name
  std::vector<std::string> parameters;

  // Predicate-only
  bool negate = false;

  // Function-only
  uint8_t modifier;     // ASSIGN / INCREASE / DECREASE / SCALE_UP / SCALE_DOWN
  double value = 0.0; 
};

/**
 * @brief Parses the root of a PDDL effect tree into a vector of ParsedEffect.
 * 
 * @param tree The PDDL effect tree.
 * @param out Vector where parsed effects will be stored.
 * @param logger ROS2 logger for error reporting.
 * @param problem_client Pointer to ProblemExpertClient used to query current function values to evaluate expressions.
 *
 * @return true if the extraction succeeded, false if an error occurred.
 */
bool parse_effects_tree(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<ParsedEffect> & out,
  const rclcpp::Logger & logger,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client);

/**
 * @brief Utility to create a string representation of a PDDL symbol (function or predicate)
 * 
 * Used for logging and to create problem expert query expression
 * e.g., fuel_level(car1), car_at(car1, waypoint1)
 *
 * @param name predicate or function name.
 * @param args ordered list of parameters.
 * 
 * @return A string representing the symbol with its parameters.
 */
inline std::string makeSymbolString(
  const std::string & name,
  const std::vector<std::string> & args)
{
  std::string key = name + "(";
  for (size_t i = 0; i < args.size(); ++i) {
    key += args[i];
    if (i + 1 < args.size()) key += ",";
  }
  key += ")";
  return key;
}

} // namespace plansys2

#endif // PLANSYS2_EXECUTOR__UTILS_HPP_
