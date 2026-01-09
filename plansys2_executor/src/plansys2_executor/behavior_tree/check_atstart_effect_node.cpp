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

#include <string>
#include <map>
#include <memory>
#include <cmath>

#include "plansys2_executor/behavior_tree/check_atstart_effect_node.hpp"

namespace plansys2
{

CheckAtStartEffect::CheckAtStartEffect(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");

  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");

  predicate_sensing_registry_ =
    config().blackboard->get<std::shared_ptr<PredicateSensingRegistry>>(
    "predicate_sensing_registry");

  function_sensing_registry_ =
    config().blackboard->get<std::shared_ptr<FunctionSensingRegistry>>(
    "function_sensing_registry");

  node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
  start_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
}

BT::NodeStatus
CheckAtStartEffect::tick()
{
  std::string action;
  getInput("action", action);

  log_prefix_ = "[" + action + "] [CheckAtStartEffect]";

  if (start_.nanoseconds() == 0) {
    start_ = node_->now();  // first tick
  }

  double delay_sec = 0.0;
  getInput("delay", delay_sec);

  auto elapsed = (node_->now() - start_).seconds();
  if (elapsed < delay_sec) {
    return BT::NodeStatus::RUNNING;
  }

  auto it = action_map_->find(action);
  if (it == action_map_->end()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      log_prefix_ << " Action not found in action_map");
    return BT::NodeStatus::FAILURE;
  }
  auto effects_tree = it->second.action_info.get_at_start_effects();

  std::vector<ParsedEffect> parsed_effects;
  if (!parse_effects_tree(effects_tree, parsed_effects, node_->get_logger(), problem_client_)) {
    EffectFailure failure = EffectFailure::ParseError();
    setOutput("start_effect_failures", std::vector<EffectFailure>{failure});

    RCLCPP_ERROR_STREAM(node_->get_logger(), log_prefix_ << " Failed to parse at_start effects");
    return BT::NodeStatus::FAILURE;
  }

  std::vector<EffectFailure> failed;

  for (const auto & effect : parsed_effects) {
    std::optional<EffectFailure> failure;

    if (effect.type == "predicate") {
      failure = check_predicate_effect(effect);
    } else if (effect.type == "function") {
      failure = check_function_effect(effect);
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
        log_prefix_ << " Unknown effect type '" << effect.type << "'"); 
    }

    if (failure.has_value()) {
      failed.push_back(failure.value());
    }
  }

  if (!failed.empty()) {
    setOutput("start_effect_failures", failed);
    return BT::NodeStatus::FAILURE;
  }else{
    return BT::NodeStatus::SUCCESS;
  }
}

std::optional<EffectFailure>
CheckAtStartEffect::check_predicate_effect(const ParsedEffect & eff)
{
  const auto & predicate = eff.name;
  const auto & parameters = eff.parameters;
  const bool negated = eff.negate;

  auto it = predicate_sensing_registry_->find(predicate);
  if (it == predicate_sensing_registry_->end()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      log_prefix_ << " No sensing plugin for predicate '" << predicate << "'");
    return EffectFailure::SensingError(
      predicate, parameters,
      EffectFailure::EffectType::PREDICATE,
      EffectFailure::Reason::SENSING_MISSING
    );
  }

  auto result = it->second->sense(parameters);

  if (result.status == PredicateSensingResult::Status::ERROR) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      log_prefix_ << " Error sensing predicate '" << predicate << "': " << result.message);
    return EffectFailure::SensingError(
      predicate, parameters,
      EffectFailure::EffectType::PREDICATE,
      EffectFailure::Reason::SENSING_ERROR
    );
  }

  const bool expected = !negated;
  const bool actual = (result.status == PredicateSensingResult::Status::TRUE);

  auto log_predicate = makeSymbolString(predicate, parameters);
  RCLCPP_INFO_STREAM(node_->get_logger(),
    log_prefix_ << " Predicate '" << log_predicate 
    << "' expected: " << (expected ? "TRUE" : "FALSE") 
    << ", sensed: " << (actual ? "TRUE" : "FALSE"));

  if (actual != expected) {
    return EffectFailure::PredicateFailure(
      predicate, parameters, negated,
      EffectFailure::Reason::FAILED
    );
  }else{
    return std::nullopt;
  }
}

std::optional<EffectFailure>
CheckAtStartEffect::check_function_effect(const ParsedEffect & eff)
{
  const auto & function = eff.name;
  const auto & parameters = eff.parameters;

  auto it = function_sensing_registry_->find(function);
  if (it == function_sensing_registry_->end()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      log_prefix_ << " No sensing plugin for function '" << function << "'");
    return EffectFailure::SensingError(
      function, parameters,
      EffectFailure::EffectType::FUNCTION,
      EffectFailure::Reason::SENSING_MISSING
    );
  }

  auto result = it->second->sense(parameters);

  if (result.status != FunctionSensingResult::Status::OK || !result.value.has_value()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      log_prefix_ << " Error sensing function '" << function << "': " << result.message);
    return EffectFailure::SensingError(
      function, parameters,
      EffectFailure::EffectType::FUNCTION,
      EffectFailure::Reason::SENSING_ERROR
    );
  }

  double sensed = result.value.value();

  auto function_str = makeSymbolString(function, parameters);
  
  auto fval = problem_client_->getFunction(function_str);
  if (!fval.has_value()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      log_prefix_ << " Function '" << function_str << "' not found in problem expert");
    return EffectFailure::SensingError(
      function, parameters,
      EffectFailure::EffectType::FUNCTION,
      EffectFailure::Reason::PARSE_ERROR
    );
  }

  double current = fval.value().value;

  // NOTE:
  // For at-start effects, the BT has already applied the effects
  // to the Problem Expert *before* this CheckAtStartEffect node is executed.
  // Therefore, the value currently stored in the Problem Expert already
  // represents the EXPECTED value after the effect.
  // For this reason, this function simply compare the sensed value
  // against the current value in the Problem Expert.
  //
  // This is different for at-end effects, where the check is performed
  // before applying the effects. In that case, the expected value
  // must be computed explicitly as:
  //   expected = current (+,-,*,/) effect.value
  auto expected = current;

  RCLCPP_INFO_STREAM(node_->get_logger(),
    log_prefix_ << " Function '" << function_str
    << "' expected: " << expected
    << ", sensed: " << sensed);

  if (std::fabs(sensed - expected) > kNumericToleranceEPS) {
    return EffectFailure::FunctionFailure(
      function, parameters, sensed, expected,
      EffectFailure::Reason::FAILED
    );
  }else{
    return std::nullopt;
  }
}

}  // namespace plansys2
