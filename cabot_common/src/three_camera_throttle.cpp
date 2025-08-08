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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rmw/qos_profiles.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::CompressedImage;

class ThreeCameraThrottle : public rclcpp::Node
{
public:
  ThreeCameraThrottle()
  : rclcpp::Node("three_camera_throttle")
  {
    // parameters
    queue_size_ = declare_parameter<int>("queue_size", 10);
    max_sync_interval_ = declare_parameter<double>("max_sync_interval", 0.1);
    throttle_hz_ = declare_parameter<double>("throttle_hz", 1.0);
    info_best_effort_ = declare_parameter<bool>("info_best_effort", false);

    in_img_[0] = declare_parameter<std::string>("in1.image", "/camera1/image_raw/compressed");
    in_info_[0] = declare_parameter<std::string>("in1.info", "/camera1/camera_info");
    in_img_[1] = declare_parameter<std::string>("in2.image", "/camera2/image_raw/compressed");
    in_info_[1] = declare_parameter<std::string>("in2.info", "/camera2/camera_info");
    in_img_[2] = declare_parameter<std::string>("in3.image", "/camera3/image_raw/compressed");
    in_info_[2] = declare_parameter<std::string>("in3.info", "/camera3/camera_info");

    out_img_[0] = declare_parameter<std::string>("out1.image", "/camera1/throttled/image_raw/compressed");
    out_info_[0] = declare_parameter<std::string>("out1.info", "/camera1/throttled/camera_info");
    out_img_[1] = declare_parameter<std::string>("out2.image", "/camera2/throttled/image_raw/compressed");
    out_info_[1] = declare_parameter<std::string>("out2.info", "/camera2/throttled/camera_info");
    out_img_[2] = declare_parameter<std::string>("out3.image", "/camera3/throttled/image_raw/compressed");
    out_info_[2] = declare_parameter<std::string>("out3.info", "/camera3/throttled/camera_info");

    // subscribers (message_filters)
    const rmw_qos_profile_t img_qos_sub = rmw_qos_profile_sensor_data;
    rmw_qos_profile_t info_qos_sub = info_best_effort_ ?
      rmw_qos_profile_sensor_data :
      rmw_qos_profile_default;

    sub_img_[0] = std::make_shared<message_filters::Subscriber<CompressedImage>>(this, in_img_[0], img_qos_sub);
    sub_inf_[0] = std::make_shared<message_filters::Subscriber<CameraInfo>>(this, in_info_[0], info_qos_sub);
    sub_img_[1] = std::make_shared<message_filters::Subscriber<CompressedImage>>(this, in_img_[1], img_qos_sub);
    sub_inf_[1] = std::make_shared<message_filters::Subscriber<CameraInfo>>(this, in_info_[1], info_qos_sub);
    sub_img_[2] = std::make_shared<message_filters::Subscriber<CompressedImage>>(this, in_img_[2], img_qos_sub);
    sub_inf_[2] = std::make_shared<message_filters::Subscriber<CameraInfo>>(this, in_info_[2], info_qos_sub);

    using Policy = message_filters::sync_policies::ApproximateTime<
      CompressedImage, CameraInfo, CompressedImage, CameraInfo, CompressedImage, CameraInfo>;
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(queue_size_));
    sync_->connectInput(
      *sub_img_[0], *sub_inf_[0],
      *sub_img_[1], *sub_inf_[1],
      *sub_img_[2], *sub_inf_[2]);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_interval_));
    sync_->registerCallback(
      std::bind(
        &ThreeCameraThrottle::on_sync, this,
        std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4,
        std::placeholders::_5, std::placeholders::_6));

    // publishers
    auto img_qos_pub = rclcpp::QoS(10).reliable();
    auto info_qos_pub = rclcpp::QoS(10).reliable();
    pub_img_[0] = create_publisher<CompressedImage>(out_img_[0], img_qos_pub);
    pub_info_[0] = create_publisher<CameraInfo>(out_info_[0], info_qos_pub);
    pub_img_[1] = create_publisher<CompressedImage>(out_img_[1], img_qos_pub);
    pub_info_[1] = create_publisher<CameraInfo>(out_info_[1], info_qos_pub);
    pub_img_[2] = create_publisher<CompressedImage>(out_img_[2], img_qos_pub);
    pub_info_[2] = create_publisher<CameraInfo>(out_info_[2], info_qos_pub);

    if (throttle_hz_ > 0.0) {
      min_period_ = rclcpp::Duration::from_seconds(1.0 / throttle_hz_);
    }

    RCLCPP_INFO(
      get_logger(), "ThreeCameraThrottle started. max_sync_interval=%.3f [s], throttle_hz=%.2f [H]",
      max_sync_interval_, throttle_hz_);
  }

private:
  void on_sync(
    CompressedImage::ConstSharedPtr i1, CameraInfo::ConstSharedPtr c1,
    CompressedImage::ConstSharedPtr i2, CameraInfo::ConstSharedPtr c2,
    CompressedImage::ConstSharedPtr i3, CameraInfo::ConstSharedPtr c3)
  {
    if (min_period_.nanoseconds() > 0) {
      rclcpp::Time now = this->now();
      if (last_pub_time_.nanoseconds() != 0 &&
        (now - last_pub_time_) < min_period_)
      {
        return;
      }
      last_pub_time_ = now;
    }

    pub_img_[0]->publish(*i1);
    pub_info_[0]->publish(*c1);
    pub_img_[1]->publish(*i2);
    pub_info_[1]->publish(*c2);
    pub_img_[2]->publish(*i3);
    pub_info_[2]->publish(*c3);
  }

private:
  int queue_size_;
  double max_sync_interval_;
  double throttle_hz_;
  bool info_best_effort_;

  std::string in_img_[3];
  std::string in_info_[3];
  std::string out_img_[3];
  std::string out_info_[3];

  std::shared_ptr<message_filters::Subscriber<CompressedImage>> sub_img_[3];
  std::shared_ptr<message_filters::Subscriber<CameraInfo>> sub_inf_[3];

  rclcpp::Publisher<CompressedImage>::SharedPtr pub_img_[3];
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_info_[3];

  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<
        CompressedImage, CameraInfo, CompressedImage, CameraInfo, CompressedImage, CameraInfo>>> sync_;

  rclcpp::Duration min_period_{0, 0};
  rclcpp::Time last_pub_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThreeCameraThrottle>());
  rclcpp::shutdown();
  return 0;
}
