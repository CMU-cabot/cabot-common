// Copyright (c) 2024  Carnegie Mellon University and IBM Corporation
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

#include "rclcpp/rclcpp.hpp"
#include <rcl_interfaces/msg/log.hpp>

using std::placeholders::_1;

namespace CaBot
{
class LogRedirectorNode : public rclcpp::Node
{
public:
  explicit LogRedirectorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("log_redirector",
                 rclcpp::NodeOptions(options).enable_rosout(false)  // disable publishing to /rosout topic
                 )
  {
    RCLCPP_INFO(get_logger(), "log_redirector constructor");
    target_node_ = declare_parameter("target_node", "");
    log_sub_ = create_subscription<rcl_interfaces::msg::Log>("/rosout", 10, std::bind(&LogRedirectorNode::rosoutCallback, this, _1));
  }

  void rosoutCallback(const rcl_interfaces::msg::Log::SharedPtr input)
  {
    if (input->name == target_node_){
      if (input->level == rcl_interfaces::msg::Log::DEBUG){
        RCLCPP_DEBUG(get_logger(), "[%s]: %s", input->name.c_str(), input->msg.c_str());
      } else if (input->level == rcl_interfaces::msg::Log::INFO){
        RCLCPP_INFO(get_logger(), "[%s]: %s", input->name.c_str(), input->msg.c_str());
      } else if (input->level == rcl_interfaces::msg::Log::WARN){
        RCLCPP_WARN(get_logger(), "[%s]: %s", input->name.c_str(), input->msg.c_str());
      } else if (input->level == rcl_interfaces::msg::Log::ERROR){
        RCLCPP_ERROR(get_logger(), "[%s]: %s", input->name.c_str(), input->msg.c_str());
      } else if (input->level == rcl_interfaces::msg::Log::FATAL){
        RCLCPP_FATAL(get_logger(), "[%s]: %s", input->name.c_str(), input->msg.c_str());
      }
    }

  }

private:
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
  std::string target_node_;
};  // class LogRedirectorNode

}  // namespace CaBot
//#include <rclcpp_components/register_node_macro.hpp>
//RCLCPP_COMPONENTS_REGISTER_NODE(CaBot::LogRedirectorNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaBot::LogRedirectorNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
