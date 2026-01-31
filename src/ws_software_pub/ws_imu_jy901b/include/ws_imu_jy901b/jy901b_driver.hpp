#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tenaxis_msg/msg/tenaxis_imu.hpp>

#include "ws_imu_jy901b/ring_buffer.hpp"

namespace ws_imu_jy901b {

class JY901BDriver {
public:
  explicit JY901BDriver(rclcpp::Node* node);
  ~JY901BDriver();

  void start();
  void stop();

private:
  enum class State {
    INIT = 0,
    RUNNING,
    ERROR,
    RECONNECT_WAIT,
    STOPPED
  };

  // 线程入口
  void serial_thread_();
  void parse_thread_();

  // 状态机动作
  bool try_open_serial_();
  void close_serial_();
  void enter_error_(const std::string& why);
  void enter_reconnect_wait_();
  void maybe_reconnect_();

  // 解析：ring buffer + 帧状态机
  void parse_from_ring_();
  void handle_frame_(const uint8_t* f);

  // 发布（ROS timer 回调，100Hz）
  void publish_timer_cb_();

  // 工具函数
  static bool verify_checksum_(const uint8_t* f);
  static int16_t to_i16_(uint8_t l, uint8_t h);

private:
  rclcpp::Node* node_{nullptr};

  // 参数
  std::string port_;
  int baud_{115200};
  std::string frame_id_{"imu_link"};
  double publish_rate_hz_{100.0};

  // ROS publisher/timer
  rclcpp::Publisher<tenaxis_msg::msg::TenaxisImu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // 串口
  boost::asio::io_context io_;
  boost::asio::serial_port serial_{io_};

  std::thread th_serial_;
  std::thread th_parse_;

  std::atomic<bool> running_{false};
  std::atomic<State> state_{State::INIT};

  // 5s 重连控制
  std::chrono::steady_clock::time_point next_reconnect_tp_{};

  // ring buffer（串口线程写入，解析线程读出）
  RingBuffer<16384> rb_;
  std::mutex rb_mtx_; // 简单互斥保护 push/pop（足够稳；后续可换 lock-free）

  // 帧解析状态机
  enum class ParseState { WAIT_HEAD, COLLECT };
  ParseState pst_{ParseState::WAIT_HEAD};
  uint8_t frame_[11]{};
  int frame_idx_{0};

  // 数据缓存（解析线程更新，发布线程读取）
  std::mutex data_mtx_;
  tenaxis_msg::msg::TenaxisImu cached_;

  // 统计
  std::atomic<uint64_t> chk_fail_{0};
  std::atomic<uint64_t> rb_overflow_{0};
  std::atomic<uint64_t> frames_ok_{0};

  // “info 用于状态” 的节流输出
  std::chrono::steady_clock::time_point last_state_info_tp_{};
};

} // namespace ws_imu_jy901b
