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

#include "plansys2_executor/behavior_tree/check_atend_effect_node.hpp"

namespace plansys2
{

CheckAtEndEffect::CheckAtEndEffect(
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
}

BT::NodeStatus
CheckAtEndEffect::tick()
{
  std::string action;
  getInput("action", action);

  log_prefix_ = "[" + action + "] [CheckAtEndEffect]";

  auto it = action_map_->find(action);
  if (it == action_map_->end()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
      log_prefix_ << " Action not found in action_map");
    return BT::NodeStatus::FAILURE;
  }
  auto effects_tree = it->second.action_info.get_at_end_effects();

  std::vector<ParsedEffect> parsed_effects;
  if (!parse_effects_tree(effects_tree, parsed_effects, node_->get_logger(), problem_client_)) {
    EffectFailure failure = EffectFailure::ParseError();
    setOutput("end_effect_failures", std::vector<EffectFailure>{failure});

    RCLCPP_ERROR_STREAM(node_->get_logger(), log_prefix_ << " Failed to parse at_end effects");
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
    setOutput("end_effect_failures", failed);
    return BT::NodeStatus::FAILURE;
  }else{
    return BT::NodeStatus::SUCCESS;
  }
}

std::optional<EffectFailure>
CheckAtEndEffect::check_predicate_effect(const ParsedEffect & eff)
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
CheckAtEndEffect::check_function_effect(const ParsedEffect & eff)
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
  // For at-end effects, the BT checks the effects before they are applied
  // to the Problem Expert. At this point, the Problem Expert still contains
  // the value before the numeric effect.
  // So, the expected value after the effect must be computed
  // explicitly according to the PDDL modifier (assign, increase,
  // decrease, scale up/down) and the effect value.
  double expected;
  using Node = plansys2_msgs::msg::Node;
  switch (eff.modifier) {
    case Node::ASSIGN:
      expected = eff.value;
      break;
    case Node::INCREASE:
      expected = current + eff.value;
      break;
    case Node::DECREASE:
      expected = current - eff.value;
      break;
    case Node::SCALE_UP:
      expected = current * eff.value;
      break;
    case Node::SCALE_DOWN:
      expected = current / eff.value;
      break;
    default:
      RCLCPP_ERROR_STREAM(node_->get_logger(),
        log_prefix_ << " Unknown function modifier for '" << function << "'");
      return EffectFailure::SensingError(
        function, parameters, 
        EffectFailure::EffectType::FUNCTION,
        EffectFailure::Reason::PARSE_ERROR
      );
  }

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
