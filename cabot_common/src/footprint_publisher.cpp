// Copyright (c) 2023  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// lookup transform service
// Author: Daisuke Sato <daisukes@cmu.edu>


#include <math.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

enum Mode { UNKNOWN = -1, NORMAL = 0, SMALLEST = 1, DYNAMIC = 2, SMALL = 3 };

class FootprintPublisher : public rclcpp::Node
{
private:
  void check_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    RCLCPP_INFO(this->get_logger(), "check_status");

    if (current_mode_ == Mode::UNKNOWN) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Mode is not specified");
      return;
    }
    if (current_mode_ < 0 || 3 < current_mode_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown mode = " + std::to_string(current_mode_));
      return;
    }
    if (footprint_ == nullptr) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "No footprint (mode=" + std::to_string(current_mode_) + ")");
      return;
    }
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "working (mode=" + std::to_string(current_mode_) + ")");
  }

public:
  FootprintPublisher()
  : Node("footprint_publisher"), current_mode_(Mode::UNKNOWN)
  {
    this->declare_parameter("footprint_mode", static_cast<int>(Mode::NORMAL));
    this->declare_parameter(
      "footprint_topics",
      std::vector<std::string>{"/global_costmap/footprint", "/local_costmap/footprint"});
    this->declare_parameter("footprint_normal", 0.45);
    this->declare_parameter("footprint_smallest", 0.35);
    this->declare_parameter("footprint_small", 0.40);
    this->declare_parameter("footprint_links", std::vector<std::string>{"base_footprint"});
    this->declare_parameter("offset_links", std::vector<std::string>{"base_control_shift"});
    this->declare_parameter("offset_normal", 0.25);
    this->declare_parameter("offset_smallest", 0.15);
    this->declare_parameter("offset_small", 0.20);

    std::vector<std::string> footprint_topics;
    this->get_parameter("footprint_topics", footprint_topics);

    for (const auto & topic : footprint_topics) {
      auto publisher = this->create_publisher<geometry_msgs::msg::Polygon>(topic, 10);
      publishers_.push_back(publisher);
    }

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FootprintPublisher::timer_callback, this));

    updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    updater_->setHardwareID("Footprint Publisher");
    updater_->add("ROS2 Footprint Publisher", this, &FootprintPublisher::check_status);

    callback_handler_ =
      this->add_on_set_parameters_callback(std::bind(&FootprintPublisher::param_set_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "constructor completed");
  }

private:
  void timer_callback()
  {
    int new_mode;
    this->get_parameter("footprint_mode", new_mode);
    count++;
    if (static_cast<Mode>(new_mode) != current_mode_ || count > 10) {
      count = 0;
      RCLCPP_INFO(this->get_logger(), "mode updated, %d -> %d", current_mode_, new_mode);
      current_mode_ = static_cast<Mode>(new_mode);
      footprint_ = get_footprint(current_mode_);
      if (footprint_ == nullptr) {
        return;
      }
      for (const auto & publisher : publishers_) {
        publisher->publish(*footprint_);
      }
    }

    auto joint_state = get_offset_joint_state(current_mode_);
    joint_state_pub_->publish(joint_state);
  }

  geometry_msgs::msg::Polygon::SharedPtr get_footprint(Mode mode)
  {
    sensor_msgs::msg::JointState state;

    double footprint = 0;
    if (mode == Mode::NORMAL) {
      footprint = this->get_parameter("footprint_normal").get_value<double>();
    } else if (mode == Mode::SMALLEST) {
      footprint = this->get_parameter("footprint_smallest").get_value<double>();
    } else if (mode == Mode::SMALL) {
      footprint = this->get_parameter("footprint_small").get_value<double>();
    }
    RCLCPP_INFO(this->get_logger(), "Footprint size=%.2f", footprint);

    if (footprint == 0) {
      return nullptr;
    }

    return circle_footprint(footprint);
  }

  geometry_msgs::msg::Polygon::SharedPtr circle_footprint(double radius)
  {
    int N = 16;
    auto polygon = std::make_shared<geometry_msgs::msg::Polygon>();
    for (int w = 0; w < N; w++) {
      double rad = 2 * M_PI * w / N;
      geometry_msgs::msg::Point32 p;
      p.x = cos(rad) * radius;
      p.y = sin(rad) * radius;
      polygon->points.push_back(p);
    }
    return polygon;
  }

  sensor_msgs::msg::JointState get_offset_joint_state(Mode mode)
  {
    sensor_msgs::msg::JointState t;
    double offset = 0;
    if (mode == Mode::NORMAL) {
      offset = this->get_parameter("offset_normal").get_value<double>();
    } else if (mode == Mode::SMALLEST) {
      offset = this->get_parameter("offset_smallest").get_value<double>();
    } else if (mode == Mode::SMALL) {
      offset = this->get_parameter("offset_small").get_value<double>();
    }

    t.header.stamp = this->get_clock()->now();
    t.name.push_back("base_joint");
    t.position.push_back(offset);
    return t;
  }

  rcl_interfaces::msg::SetParametersResult param_set_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  int count = 0;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handler_;
  geometry_msgs::msg::Polygon::SharedPtr footprint_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr> publishers_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  Mode current_mode_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootprintPublisher>());
  rclcpp::shutdown();
  return 0;
}
