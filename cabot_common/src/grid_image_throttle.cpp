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

#include <any>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;

class GridImageThrottleNode : public rclcpp::Node
{
public:
  explicit GridImageThrottleNode(const rclcpp::NodeOptions & options)
  : Node("grid_image_throttle", options)
  {
    compressed_ = declare_parameter<bool>("compressed", false);
    input_topic_ = declare_parameter<std::string>("input_topic");
    output_topic_ = declare_parameter<std::string>("output_topic", input_topic_ + "_throttle");
    max_sync_interval_ = declare_parameter<double>("max_sync_interval", 0.2);
    throttle_hz_ = declare_parameter<double>("throttle_hz", 1.0);
    log_interval_ms_ = declare_parameter<int>("log_interval_ms", 10000);  // default: 10000 ms = 10 s
    interval_sec_ = 1.0 / throttle_hz_;

    last_time_ = this->now();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    if (compressed_) {
      // CompressedImage
      sub_ = create_subscription<CompressedImage>(
        input_topic_, qos,
        [this](const CompressedImage::SharedPtr msg) {
          this->process_message(msg);
        });
      pub_ = create_publisher<CompressedImage>(output_topic_, qos);
    } else {
      // Image
      sub_ = create_subscription<Image>(
        input_topic_, qos,
        [this](const Image::SharedPtr msg) {
          this->process_message(msg);
        });
      pub_ = create_publisher<Image>(output_topic_, qos);
    }
  }

private:
  void process_message(const std::any & msg)
  {
    const auto & now = this->now();
    if (last_time_ > now) {
      RCLCPP_WARN(
        get_logger(), "Detected jump back in time, resetting throttle period to now for.");
      last_time_ = now;
      // reset counter
      total_count_ = 0;
      grid_count_ = 0;
      pub_count_ = 0;
    }

    bool published = false;
    total_count_++;
    double last_s = static_cast<double>(last_time_.seconds());
    double grid_last = std::floor(last_s / interval_sec_) * interval_sec_;

    if (msg.type() == typeid(CompressedImage::SharedPtr)) {
      CompressedImage::SharedPtr image = std::any_cast<CompressedImage::SharedPtr>(msg);
      rclcpp::Time t_stamp(image->header.stamp);
      double stamp_s = static_cast<double>(t_stamp.seconds());
      double grid_stamp = std::floor(stamp_s / interval_sec_) * interval_sec_;
      if (grid_stamp != grid_last) {
        grid_count_++;
        if (stamp_s - grid_stamp < max_sync_interval_) {
          pub_count_++;
          std::static_pointer_cast<rclcpp::Publisher<CompressedImage>>(pub_)->publish(*image);
          published = true;
        }
        last_time_ = t_stamp;
      }
    } else if (msg.type() == typeid(Image::SharedPtr)) {
      Image::SharedPtr image = std::any_cast<Image::SharedPtr>(msg);
      rclcpp::Time t_stamp(image->header.stamp);
      double stamp_s = static_cast<double>(t_stamp.seconds());
      double grid_stamp = std::floor(stamp_s / interval_sec_) * interval_sec_;
      if (grid_stamp != grid_last) {
        grid_count_++;
        if (stamp_s - grid_stamp < max_sync_interval_) {
          pub_count_++;
          std::static_pointer_cast<rclcpp::Publisher<Image>>(pub_)->publish(*image);
          published = true;
        }
        last_time_ = t_stamp;
      }
    }
    latest_any_ = msg;

    if (published) {
      double pub_rate = static_cast<double>(pub_count_) / total_count_;
      double sync_rate = static_cast<double>(pub_count_) / grid_count_;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_interval_ms_, " publish rate = %f, sync rate = %f.", pub_rate, sync_rate);
    }
  }

  rclcpp::PublisherBase::SharedPtr pub_;
  rclcpp::SubscriptionBase::SharedPtr sub_;

  bool compressed_;
  double max_sync_interval_;
  double throttle_hz_;
  double interval_sec_;
  int log_interval_ms_;
  std::string input_topic_;
  std::string output_topic_;
  int total_count_ = 0;
  int grid_count_ = 0;
  int pub_count_ = 0;

  rclcpp::Time last_time_;
  std::any latest_any_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridImageThrottleNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
