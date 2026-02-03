#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tenaxis_msg/msg/tenaxis_imu.hpp"

using std::placeholders::_1;

class ImuOrientationNode : public rclcpp::Node
{
public:
  ImuOrientationNode() : Node("imu_orientation_node")
  {
    // ---------------- Parameters ----------------
    input_topic_   = this->declare_parameter<std::string>("input_topic", "/imu/jy901b");
    imu_topic_     = this->declare_parameter<std::string>("imu_topic", "/imu/data");
    pose_topic_    = this->declare_parameter<std::string>("pose_topic", "/imu/pose");
    marker_topic_  = this->declare_parameter<std::string>("marker_topic", "/imu/axes_markers");

    // RViz Fixed Frame / message frame
    frame_id_      = this->declare_parameter<std::string>("frame_id", "imu_link");

    // Marker styling
    axis_length_   = this->declare_parameter<double>("axis_length", 0.6);
    axis_radius_   = this->declare_parameter<double>("axis_radius", 0.03);
    text_scale_    = this->declare_parameter<double>("text_scale", 0.12);
    publish_markers_ = this->declare_parameter<bool>("publish_markers", true);

    // Whether to use msg.orientation instead of msg.euler_rpy (你当前需求是欧拉角，所以默认 false)
    use_msg_quat_  = this->declare_parameter<bool>("use_msg_quat", false);

    // ---------------- Publishers ----------------
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    // ---------------- Subscriber (QoS fixed) ----------------
    // 传感器话题常用 QoS：BestEffort，避免与硬件驱动常见设置不匹配
    auto qos = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<tenaxis_msg::msg::TenaxisImu>(
      input_topic_, qos, std::bind(&ImuOrientationNode::cb, this, _1));

    RCLCPP_INFO(this->get_logger(),
      "Listening: %s (SensorDataQoS), publishing Imu: %s, Pose: %s, markers: %s, frame_id: %s",
      input_topic_.c_str(), imu_topic_.c_str(), pose_topic_.c_str(), marker_topic_.c_str(), frame_id_.c_str());
  }

private:
  void cb(const tenaxis_msg::msg::TenaxisImu & msg)
  {
    // 使用消息时间戳
    const auto stamp = msg.header.stamp;

    tf2::Quaternion q;

    if (use_msg_quat_) {
      // 使用消息自带四元数（如果你的设备/融合算法输出可靠）
      q.setX(msg.orientation.x);
      q.setY(msg.orientation.y);
      q.setZ(msg.orientation.z);
      q.setW(msg.orientation.w);
      if (q.length2() < 1e-12) {
        // 防止零四元数导致 NaN
        q.setValue(0.0, 0.0, 0.0, 1.0);
      } else {
        q.normalize();
      }
    } else {
      // 使用欧拉角（东北天 ENU）：roll/pitch/yaw 绕 X/Y/Z（单位 rad）
      const double roll  = msg.euler_rpy.x;
      const double pitch = msg.euler_rpy.y;
      const double yaw   = msg.euler_rpy.z;

      q.setRPY(roll, pitch, yaw);
      q.normalize();
    }

    // ---------------- Publish sensor_msgs/Imu ----------------
    sensor_msgs::msg::Imu imu;
    imu.header = msg.header;
    imu.header.frame_id = frame_id_;

    imu.orientation = tf2::toMsg(q);

    // 透传传感器原始值（如果你不想透传也可以置零）
    imu.angular_velocity = msg.angular_velocity;
    imu.linear_acceleration = msg.linear_acceleration;

    // Covariance unknown
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;

    imu_pub_->publish(imu);

    // ---------------- Publish PoseStamped for RViz Pose display ----------------
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf2::toMsg(q);

    pose_pub_->publish(pose);

    // ---------------- Publish Markers (axes + text labels) ----------------
    if (publish_markers_) {
      marker_pub_->publish(make_markers(stamp));
    }
  }

  visualization_msgs::msg::Marker make_arrow(
    int id,
    const rclcpp::Time & t,
    double x, double y, double z,
    double r, double g, double b,
    const std::string & ns)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = t;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p0, p1;
    p0.x = 0.0; p0.y = 0.0; p0.z = 0.0;
    p1.x = x;   p1.y = y;   p1.z = z;
    m.points.push_back(p0);
    m.points.push_back(p1);

    // scale: x=shaft diameter, y=head diameter, z=head length
    m.scale.x = axis_radius_;
    m.scale.y = axis_radius_ * 2.0;
    m.scale.z = axis_length_ * 0.25;

    m.color.a = 1.0;
    m.color.r = r; m.color.g = g; m.color.b = b;

    // 让 marker 持续存在，避免 RViz 因为不刷新消失（可选）
    m.lifetime = rclcpp::Duration(0, 0);

    return m;
  }

  visualization_msgs::msg::Marker make_text(
    int id,
    const rclcpp::Time & t,
    double x, double y, double z,
    const std::string & text,
    const std::string & ns)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = t;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;

    m.scale.z = text_scale_;

    // 白字
    m.color.a = 1.0;
    m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0;

    m.text = text;

    m.lifetime = rclcpp::Duration(0, 0);
    return m;
  }

  visualization_msgs::msg::MarkerArray make_markers(const builtin_interfaces::msg::Time & stamp_msg)
  {
    rclcpp::Time t(stamp_msg);

    visualization_msgs::msg::MarkerArray arr;

    // ENU: X=E, Y=N, Z=U
    const double L = axis_length_;

    // X (E) - red
    arr.markers.push_back(make_arrow(0, t, L, 0.0, 0.0, 1.0, 0.0, 0.0, "axes"));
    // Y (N) - green
    arr.markers.push_back(make_arrow(1, t, 0.0, L, 0.0, 0.0, 1.0, 0.0, "axes"));
    // Z (U) - blue
    arr.markers.push_back(make_arrow(2, t, 0.0, 0.0, L, 0.0, 0.0, 1.0, "axes"));

    // Labels
    arr.markers.push_back(make_text(10, t, L, 0.0, 0.0, "X (E)", "labels"));
    arr.markers.push_back(make_text(11, t, 0.0, L, 0.0, "Y (N)", "labels"));
    arr.markers.push_back(make_text(12, t, 0.0, 0.0, L, "Z (U)", "labels"));
    arr.markers.push_back(make_text(13, t, 0.0, 0.0, L * 1.15, "ENU (East-North-Up)", "labels"));

    return arr;
  }

private:
  std::string input_topic_;
  std::string imu_topic_;
  std::string pose_topic_;
  std::string marker_topic_;
  std::string frame_id_;

  double axis_length_;
  double axis_radius_;
  double text_scale_;
  bool publish_markers_;
  bool use_msg_quat_;

  rclcpp::Subscription<tenaxis_msg::msg::TenaxisImu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOrientationNode>());
  rclcpp::shutdown();
  return 0;
}
