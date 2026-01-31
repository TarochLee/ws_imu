#include <rclcpp/rclcpp.hpp>
#include <glog/logging.h>

#include "ws_imu_jy901b/jy901b_driver.hpp"
#include "ws_imu_jy901b/log.hpp"

class JY901BNode : public rclcpp::Node {
public:
  JY901BNode() : Node("jy901b_node") {
    driver_ = std::make_unique<ws_imu_jy901b::JY901BDriver>(this);
    driver_->start();
    AINFO << "Node started. Publishing topic: /imu/jy901b";
  }

  ~JY901BNode() override {
    if (driver_) driver_->stop();
  }

private:
  std::unique_ptr<ws_imu_jy901b::JY901BDriver> driver_;
};

int main(int argc, char** argv) {
  // glog 初始化（Apollo 风格）
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = 1;     // 同时输出到终端
  // FLAGS_log_dir = "/tmp/ws_imu_logs"; // 如需落盘，取消注释并确保目录存在
  // FLAGS_minloglevel = 0;       // 0=INFO,1=WARNING,2=ERROR,3=FATAL

  rclcpp::init(argc, argv);
  auto node = std::make_shared<JY901BNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  google::ShutdownGoogleLogging();
  return 0;
}
