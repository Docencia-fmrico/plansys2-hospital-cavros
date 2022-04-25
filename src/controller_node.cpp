// Copyright 2019 Intelligent Robotics Lab
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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class PatrollingController : public rclcpp::Node
{
public:
  PatrollingController()
  : rclcpp::Node("controller_node"), state_(STARTING)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
    std::cout << "finished init" << std::endl;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"tiago", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"hand", "gripper"});
    problem_expert_->addInstance(plansys2::Instance{"object", "object"});
    problem_expert_->addInstance(plansys2::Instance{"corr1", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"corr2", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"corr3", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"corr4", "corridor"});
    problem_expert_->addInstance(plansys2::Instance{"main", "room"});
    problem_expert_->addInstance(plansys2::Instance{"s1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"s2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"z1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"z2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"z3", "room"});
    problem_expert_->addInstance(plansys2::Instance{"z4", "room"});
    problem_expert_->addInstance(plansys2::Instance{"g1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"g2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"g3", "room"});
    problem_expert_->addInstance(plansys2::Instance{"g4", "room"});
    problem_expert_->addInstance(plansys2::Instance{"g5", "room"});
    problem_expert_->addInstance(plansys2::Instance{"o1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"o2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"o3", "room"});
    problem_expert_->addInstance(plansys2::Instance{"o4", "room"});
    problem_expert_->addInstance(plansys2::Instance{"b1", "room"});
    problem_expert_->addInstance(plansys2::Instance{"b2", "room"});
    problem_expert_->addInstance(plansys2::Instance{"w1", "room"});
    std::cout << "1" << std::endl;


    problem_expert_->addPredicate(plansys2::Predicate("(robot_at tiago main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(gripper_at hand tiago)"));
    problem_expert_->addPredicate(plansys2::Predicate("(gripper_free hand)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at object w1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main corr1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corr1 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main corr2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corr2 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main corr3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corr3 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main corr4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corr4 main)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected main s1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s1 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main s2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected s2 main)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected main z1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z1 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main z2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z2 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main z3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z3 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main z4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected z4 main)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected main g1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected g1 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main g4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected g4 main)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected main o1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected o1 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main o2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected o2 main)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected main b1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected b1 main)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected main b2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected b2 main)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected corr3 g2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected g2 corr3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corr3 g3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected g3 corr3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corr3 o3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected o3 corr3)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected corr4 g5)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected g5 corr4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corr4 o4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected o4 corr4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corr4 w1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected w1 corr4)"));

    std::cout << "finished initialization of instances" << std::endl;
  }

  void step()
  {
    switch (state_) {
      case STARTING:
        {
          // Set the goal for next state
          problem_expert_->setGoal(plansys2::Goal("(and(object_at object main))"));
          std::cout << "goal setteled" << std::endl;

          // Compute the plan
          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);
          std::cout << "plan computed" << std::endl;

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
            break;
          }
          std::cout << "we have a plan" << std::endl;

          // Execute the plan
          if (executor_client_->start_plan_execution(plan.value())) {
            std::cout << "started executing" << std::endl;
            state_ = PATROL_WP1;
          }
          
        }
        break;
      case PATROL_WP1:
        {
          auto feedback = executor_client_->getFeedBack();

          for (const auto & action_feedback : feedback.action_execution_status) {
            std::cout << "[" << action_feedback.action << " " <<
              action_feedback.completion * 100.0 << "%]";
          }
          std::cout << std::endl;

          if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
            if (executor_client_->getResult().value().success) {
              std::cout << "Successful finished " << std::endl;

              // Cleanning up
              problem_expert_->removePredicate(plansys2::Predicate("(and(object_at object w1))"));

              // // Set the goal for next state
              // problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp2))"));

              // // Compute the plan
              // auto domain = domain_expert_->getDomain();
              // auto problem = problem_expert_->getProblem();
              // auto plan = planner_client_->getPlan(domain, problem);

              // if (!plan.has_value()) {
              //   std::cout << "Could not find plan to reach goal " <<
              //     parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              //   break;
              // }

              // // Execute the plan
              // if (executor_client_->start_plan_execution(plan.value())) {
              //   state_ = PATROL_WP2;
              // }

              std::cout<< "FINISHED PLAANNNN!!" << std::endl;

            } else {
              for (const auto & action_feedback : feedback.action_execution_status) {
                if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                  std::cout << "[" << action_feedback.action << "] finished with error: " <<
                    action_feedback.message_status << std::endl;
                }
              }

              // Replan
              auto domain = domain_expert_->getDomain();
              auto problem = problem_expert_->getProblem();
              auto plan = planner_client_->getPlan(domain, problem);

              if (!plan.has_value()) {
                std::cout << "Unsuccessful replan attempt to reach goal " <<
                  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
                break;
              }

              // Execute the plan
              executor_client_->start_plan_execution(plan.value());
            }
          }
        }
        break;
      
      default:
        break;
    }
  }

private:
  typedef enum {STARTING, PATROL_WP1} StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrollingController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
