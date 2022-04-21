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

#include <memory>

#include "geometry_msgs/msg/twist.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"

using namespace std::chrono_literals;

class Pick : public plansys2::ActionExecutorClient
{
public:
  Pick()
  : plansys2::ActionExecutorClient("pick", 1s)
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    auto cmd_led_pub_ = this->create_publisher<kobuki_ros_interfaces::msg::Led>("/commands/led1", 10);
    cmd_led_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    cmd_led_pub_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:
  void do_work()
  {
    kobuki_ros_interfaces::msg::Led led;
    led.value = 2; // green
    cmd_led_pub_->publish(led);
    finish(true, 1.0, "object picked");
  }

  rclcpp_lifecycle::LifecyclePublisher<kobuki_ros_interfaces::msg::Led>::SharedPtr cmd_led_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pick>();

  node->set_parameter(rclcpp::Parameter("action_name", "pick"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
