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

#ifndef PLANSYS2_EXECUTOR__PREDICATESENSINGBASE_HPP_
#define PLANSYS2_EXECUTOR__PREDICATESENSINGBASE_HPP_

#include <string>
#include <vector>

namespace plansys2
{

struct SensingResult
{
  enum Status { TRUE, FALSE, ERROR };
  Status status;
  std::string message;
};

class PredicateSensingBase
{
public:
  virtual ~PredicateSensingBase() = default;

  /**
   * @brief Base interface for predicate sensing plugins.
   * 
   * This interface is intended to be implemented by users and exported as a
   * plugin using `pluginlib`. Implementations allow PlanSys2 to evaluate
   * predicates at runtime through custom sensing logic.
   *
   * @param args Vector of predicate arguments, in the same order as defined
   *             in the PDDL domain.
   *
   * @return SensingResult with:
   *   - Status::TRUE  if the predicate holds
   *   - Status::FALSE if the predicate does not hold
   *   - Status::ERROR if the sensing process failed or the result is unknown
   * 
   */
  virtual SensingResult sense(const std::vector<std::string> & args) = 0;

  const std::string & get_predicate_name() const {return predicate_name_;}

protected:
  std::string predicate_name_;

};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__PREDICATESENSINGBASE_HPP_
