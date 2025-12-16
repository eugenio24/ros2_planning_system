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

  predicate_sensing_registry_ =
    config().blackboard->get<std::shared_ptr<PredicateSensingRegistry>>(
    "predicate_sensing_registry");
}

BT::NodeStatus
CheckAtEndEffect::tick()
{
  std::string action;
  getInput("action", action);

  auto node = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

  auto effects = (*action_map_)[action].action_info.get_at_end_effects();

  std::vector<EffectFailure> failed;

  auto next_is_negated = false;
  for(const auto& effect_node: effects.nodes){
    auto node_type = effect_node.node_type;
    
    if(node_type == plansys2_msgs::msg::Node::PREDICATE){

      auto predicate = effect_node.name;
      auto negated = next_is_negated;
      std::vector<std::string> parameters;
      parameters.reserve(effect_node.parameters.size());
      for (const auto& p : effect_node.parameters) {
        parameters.push_back(p.name);
      }

      auto it = predicate_sensing_registry_->find(predicate);

      if (it == predicate_sensing_registry_->end()) {
        // No sensor available
        RCLCPP_ERROR_STREAM(node->get_logger(), "[" << action << "]" 
          << " [CheckAtEndEffect] No sensing plugin found for" << predicate);
        failed.push_back({predicate, parameters, negated, EffectFailure::Reason::SENSING_MISSING});
        continue;
      }

      auto sensor = it->second;
      auto sense_result = sensor->sense(parameters);


      if (sense_result.status == SensingResult::Status::ERROR) {
        // Sensing failed; can't confirm

        RCLCPP_ERROR_STREAM(node->get_logger(), "[" << action << "]" 
          << " [CheckAtEndEffect] Error sensing effect" << predicate << ": " << sense_result.message);
        failed.push_back({predicate, parameters, negated, EffectFailure::Reason::SENSING_ERROR});
        continue;
      }

      // Expected value: TRUE for normal predicate, FALSE for negated predicate
      const bool expected = !negated;
      const bool actual = (sense_result.status == SensingResult::Status::TRUE);

      // Convert parameters to a string for printing
      std::string param_str;
      for (size_t i = 0; i < parameters.size(); ++i) {
        param_str += parameters[i];
        if (i < parameters.size() - 1)
          param_str += ", ";
      }

      // Print predicate, parameters, expected, and sensed value
      RCLCPP_INFO_STREAM(node->get_logger(),
          "[" << action << "] [CheckAtEndEffect] Predicate '" << predicate
          << "(" << param_str << ")'"
          << " expected: " << (expected ? "TRUE" : "FALSE")
          << ", sensed: " << (actual ? "TRUE" : "FALSE"));

      
      if (actual != expected) {
        failed.push_back({predicate, parameters, negated, EffectFailure::Reason::FAILED});
      }

    }

    // in msg::Node there is a negate field that should keep track of this but is not working
    next_is_negated = node_type == plansys2_msgs::msg::Node::NOT;
  }

  if(!failed.empty()){
    setOutput("end_effect_failures", failed);
    return BT::NodeStatus::FAILURE;
  }else{
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2
