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

#ifndef PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__PREDICATESENSINGBASE_HPP_
#define PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__PREDICATESENSINGBASE_HPP_

#include <string>
#include <vector>

#include "plansys2_executor/effect_monitoring/plugin_interfaces/SensingBase.hpp"

namespace plansys2
{

/**
 * @brief Result of a predicate sensing operation.
 */
struct PredicateSensingResult
{
  enum class Status
  {
    TRUE,
    FALSE,
    ERROR
  };

  Status status;
  std::string message;
};

/**
 * @brief Base interface for predicate sensing plugins.
 *
 * Implementations of this interface allow PlanSys2 to evaluate
 * predicates at runtime using custom sensing logic.
 *
 * Example PDDL predicate:
 *   (robot_at ?r - robot ?room - room)
 *
 * Example plugin:
 *   - symbol_name_ = "robot_at"
 *   - sense({"robot1", "kitchen"})
 */
class PredicateSensingBase : public SensingBase
{
public:
  virtual ~PredicateSensingBase() = default;

  /**
   * @brief Senses the predicate.
   *
   * @param args Predicate arguments in the same order as defined in PDDL.
   *
   * @return PredicateSensingResult
   * with:
   *   - Status::TRUE  if the predicate holds
   *   - Status::FALSE if the predicate does not hold
   *   - Status::ERROR if sensing failed or result is unknown
   */
  virtual PredicateSensingResult sense(const std::vector<std::string> & args) = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__PREDICATESENSINGBASE_HPP_
