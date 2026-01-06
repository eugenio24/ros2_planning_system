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

#ifndef PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__SENSINGBASE_HPP_
#define PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__SENSINGBASE_HPP_

#include <string>

namespace plansys2
{

/**
 * @brief Base class for all sensing plugins used for monitoring effects.
 *
 * This class defines the minimal common interface shared by the sensing plugins
 * (predicate sensing, numeric fluent sensing).
 *
 * Plugins implementing these interfaces are expected to be loaded via pluginlib.
 */
class SensingBase
{
public:
  virtual ~SensingBase() = default;

  /**
   * @brief Returns the PDDL symbol name (predicate or function) of this plugin.
   */
  const std::string & get_symbol_name() const {return symbol_name_;}

protected:
  std::string symbol_name_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EFFECT_MONITORING__PLUGIN_INTERFACES__SENSINGBASE_HPP_
