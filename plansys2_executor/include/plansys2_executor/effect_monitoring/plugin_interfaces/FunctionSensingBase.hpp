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

#ifndef PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__FUNCTIONSENSINGBASE_HPP_
#define PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__FUNCTIONSENSINGBASE_HPP_

#include <string>
#include <vector>

#include "plansys2_executor/effect_monitoring/plugin_interfaces/SensingBase.hpp"

namespace plansys2
{

/**
 * @brief Result of a function (numeric fluent) sensing operation.
 *
 * This struct contains the numeric value returned by a sensing plugin
 * and a status indicating success or failure.
 */
struct FunctionSensingResult
{
  enum class Status
  {
    OK,     // Sensing succeeded
    ERROR   // Sensing failed or value unknown
  };

  Status status;
  std::optional<double> value;  // Numeric value of the function/fluent (meaningful if status == OK)
  std::string message;          // Optional message for errors or diagnostics
};

/**
 * @brief Base interface for function sensing plugins.
 *
 * Plugins implementing this interface allow PlanSys2 to query function values
 * at runtime.
 *
 * Example PDDL function:
 *   (fuel_level ?s - car)
 *
 * Example plugin usage:
 *   - symbol_name_ = "fuel_level"
 *   - sense({"car1"}) -> FunctionSensingResult { status=OK, value=90.5 }
 */
class FunctionSensingBase : public SensingBase
{
public:
  virtual ~FunctionSensingBase() = default;

  /**
   * @brief Query the current value of a function.
   *
   * @param args Function arguments, in the same order as in the PDDL domain.
   *
   * @return FunctionSensingResult containing the numeric value or an error status.
   */
  virtual FunctionSensingResult sense(const std::vector<std::string> & args) = 0;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__FUNCTIONSENSINGBASE_HPP_
