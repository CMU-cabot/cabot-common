// Copyright (c) 2025  IBM Corporation
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

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "topic_tools/tool_base_node.hpp"
#include "topic_tools/visibility_control.h"

namespace topic_tools
{
class GridThrottleNode final : public ToolBaseNode
{
  using Sent = std::pair<double, size_t>;

public:
  TOPIC_TOOLS_PUBLIC
  explicit GridThrottleNode(const rclcpp::NodeOptions & options)
  : ToolBaseNode("grid_throttle", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic");
    output_topic_ = declare_parameter<std::string>("output_topic", input_topic_ + "_throttle");
    lazy_ = declare_parameter<bool>("lazy", false);
    throttle_hz_ = declare_parameter<double>("throttle_hz", 1.0);
    period_ = rclcpp::Rate(throttle_hz_).period();
    interval_sec_ = 1.0/throttle_hz_;

    last_time_ = this->now();

    discovery_timer_ = this->create_wall_timer(
      discovery_period_,
      std::bind(&GridThrottleNode::make_subscribe_unsubscribe_decisions, this));

    make_subscribe_unsubscribe_decisions();
  }


private:
  void process_message(std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    std::scoped_lock lock(pub_mutex_);
    if (!pub_) {
      return;
    }

    const auto & now = this->now();
    if (last_time_ > now) {
      RCLCPP_WARN(
        get_logger(), "Detected jump back in time, resetting throttle period to now for.");
      last_time_ = now;
    }

    double now_s = static_cast<double>(now.nanoseconds()) * 1e-9;
    double grid_now = std::floor(now_s / interval_sec_) * interval_sec_;

    double last_s = static_cast<double>(last_time_.nanoseconds()) * 1e-9;
    double grid_last = std::floor(last_s / interval_sec_) * interval_sec_;

    if (grid_now != grid_last){
      pub_->publish(*msg);
      last_time_ = now;
    }
  }

  double throttle_hz_;
  double interval_sec_;
  std::chrono::nanoseconds period_;
  rclcpp::Time last_time_;
};
}  // namespace topic_tools

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<topic_tools::GridThrottleNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
