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


#include "plansys2_executor/effect_monitoring/Utils.hpp"

namespace plansys2
{


/**
 * @brief Recursively evaluates the RHS of a function modifier expression
 * 
 * Supports NUMBER, EXPRESSION (+, -, *, /), and FUNCTION nodes.
 * Returns a tuple of (success, value).
 * 
 * @param tree The PDDL effect tree.
 * @param node_id Index of the node to evaluate.
 * @param problem_client ProblemExpertClient pointer to query function values and evaluate the expression
 * 
 * @return std::tuple<bool,double> (success, evaluated value)
 */
std::tuple<bool,double> evaluate_numeric_node(
  const plansys2_msgs::msg::Tree & tree,
  uint8_t node_id,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  using Node = plansys2_msgs::msg::Node;
  const auto & n = tree.nodes[node_id];

  switch(n.node_type)
  {
    case Node::NUMBER:
      return { true, n.value };

    case Node::EXPRESSION:
    {
      if(n.children.size() != 2) {
        return { false, 0.0 };
      }

      auto [ok_l,left] = evaluate_numeric_node(tree, n.children[0], problem_client);
      auto [ok_r,right] = evaluate_numeric_node(tree, n.children[1], problem_client);

      if(!ok_l || !ok_r) {
        return { false, 0.0 };
      }

      switch(n.expression_type){
        case Node::ARITH_ADD:  
          return { true, left + right };
        case Node::ARITH_SUB:  
          return { true, left - right };
        case Node::ARITH_MULT: 
          return { true, left * right };
        case Node::ARITH_DIV:  
          if(std::abs(right) > kNumericToleranceEPS) {
            return { true, left / right };
          } else {
            return { false, 0.0 }; // division by zero
          }
        default:
          return { false, 0.0 };
      }
    }

    case Node::FUNCTION:
    {
      auto fname = parser::pddl::toString(tree, node_id);
      auto fval = problem_client->getFunction(fname);
      if(!fval.has_value()) {
        return { false, 0.0 };
      }
      return { true, fval.value().value };
    }

    default:
      return { false, 0.0 };
  }
}

/**
 * @brief Recursively extracts effects from a PDDL effect tree.
 * 
 * Converts the nodes into a vector of ParsedEffect objects. It handles:
 *   - Predicate effects (may be negated)
 *   - Numeric effects (functions) with modifiers (assign, increase, decrease, ...)
 * 
 * Supported node types:
 *   - AND: extracts effects from all children
 *   - NOT: negates the effect of the child node
 *   - PREDICATE: creates a ParsedEffect of type "predicate"
 *   - FUNCTION_MODIFIER: evaluates RHS expression and creates a ParsedEffect of type "function"
 * 
 * @param tree The PDDL effect tree.
 * @param node_id Index of the current node to process.
 * @param out Vector where parsed effects are appended.
 * @param negate Whether the current node’s effect should be negated.
 * @param logger ROS2 logger for error reporting.
 * @param problem_client Pointer to ProblemExpertClient used to query current function values to evaluare expressions.
 * 
 * @return true if the extraction succeeded, false if an error occurred.
 */
bool extract_effects(
  const plansys2_msgs::msg::Tree & tree,
  uint8_t node_id,
  std::vector<plansys2::ParsedEffect> & out,
  bool negate,
  const rclcpp::Logger & logger,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  using Node = plansys2_msgs::msg::Node;
  const auto & node = tree.nodes[node_id];

  switch(node.node_type)
  {
    case Node::AND: {
      for (auto child : node.children) {
        if (!extract_effects(tree, child, out, negate, logger, problem_client)) {
          return false;
        }
      }

      return true;
    }

    case Node::NOT: {
      if (node.children.empty()) { 
        RCLCPP_ERROR(logger,"NOT node has no children"); 
        return false; 
      }
      return extract_effects(tree, node.children[0], out, !negate, logger, problem_client);
    }

    case Node::PREDICATE: {
      plansys2::ParsedEffect eff;
      eff.type = "predicate";
      eff.name = node.name;
      eff.negate = negate;
      for (auto &p : node.parameters) eff.parameters.push_back(p.name);
      
      out.push_back(std::move(eff));
      return true;
    }

    case Node::FUNCTION_MODIFIER: {
      if (negate) { 
        RCLCPP_ERROR(logger,"Negated numeric effect not allowed"); 
        return false; 
      }

      if (node.children.size() != 2) { 
        RCLCPP_ERROR(logger,"FUNCTION_MODIFIER must have 2 children"); 
        return false; 
      }

      const auto & lhs = tree.nodes[node.children[0]];
      uint8_t rhs_id = node.children[1];

      if (lhs.node_type != Node::FUNCTION) { 
        RCLCPP_ERROR(logger,"LHS not a function"); 
        return false; 
      }

      // Evaluate RHS recursively
      auto [ok,value] = evaluate_numeric_node(tree, rhs_id, problem_client);
      if(!ok) { 
        RCLCPP_ERROR(logger, "Failed to evaluate RHS"); 
        return false; 
      }

      plansys2::ParsedEffect eff;
      eff.type = "function";
      eff.name = lhs.name;
      eff.modifier = node.modifier_type;
      eff.value = value;
      for (auto &p : lhs.parameters) eff.parameters.push_back(p.name);

      out.push_back(std::move(eff));
      return true;
    }

    default: {
      RCLCPP_ERROR(logger,"Unsupported node_type %u", node.node_type);
      return false;
    }
  }
}

bool parse_effects_tree(
  const plansys2_msgs::msg::Tree & tree,
  std::vector<ParsedEffect> & out,
  const rclcpp::Logger & logger,
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client)
{
  out.clear();

  if (tree.nodes.empty()) {
    return true;
  }

  return extract_effects(tree, 0, out, false, logger, problem_client);
}


}  // namespace plansys2
