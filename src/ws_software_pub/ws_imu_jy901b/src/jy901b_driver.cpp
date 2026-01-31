#include "ws_imu_jy901b/jy901b_driver.hpp"
#include "ws_imu_jy901b/log.hpp"

#include <cmath>
#include <limits>

namespace ws_imu_jy901b {

static constexpr double G0 = 9.8;

JY901BDriver::JY901BDriver(rclcpp::Node* node)
: node_(node) {
  port_ = node_->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_ = node_->declare_parameter<int>("baud", 115200);
  frame_id_ = node_->declare_parameter<std::string>("frame_id", "imu_link");
  publish_rate_hz_ = node_->declare_parameter<double>("publish_rate_hz", 100.0);

  pub_ = node_->create_publisher<tenaxis_msg::msg::TenaxisImu>("/imu/jy901b", rclcpp::SensorDataQoS());

  // 初始 NaN，便于判断是否更新过
  cached_.pressure_pa = std::numeric_limits<float>::quiet_NaN();
  cached_.altitude_m  = std::numeric_limits<float>::quiet_NaN();

  last_state_info_tp_ = std::chrono::steady_clock::now();

  AINFO << "Driver constructed. port=" << port_ << " baud=" << baud_ << " frame_id=" << frame_id_
        << " publish_rate_hz=" << publish_rate_hz_;
}

JY901BDriver::~JY901BDriver() {
  stop();
}

void JY901BDriver::start() {
  if (running_.exchange(true)) return;

  state_.store(State::INIT);

  // 100Hz 发布（与串口/解析解耦）
  auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
  pub_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&JY901BDriver::publish_timer_cb_, this)
  );

  th_serial_ = std::thread(&JY901BDriver::serial_thread_, this);
  th_parse_  = std::thread(&JY901BDriver::parse_thread_, this);

  AINFO << "Driver start(): threads launched.";
}

void JY901BDriver::stop() {
  if (!running_.exchange(false)) return;
  state_.store(State::STOPPED);

  close_serial_();

  if (th_serial_.joinable()) th_serial_.join();
  if (th_parse_.joinable()) th_parse_.join();

  AINFO << "Driver stopped.";
}

bool JY901BDriver::try_open_serial_() {
  try {
    if (serial_.is_open()) return true;
    serial_.open(port_);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    AINFO << "INIT: serial opened OK: " << port_ << " @" << baud_;
    return true;
  } catch (const boost::system::system_error& e) {
    AERROR << "INIT: open serial failed: " << e.what()
           << " (check dialout permission / device busy / correct tty)";
    return false;
  }
}

void JY901BDriver::close_serial_() {
  try {
    if (serial_.is_open()) {
      boost::system::error_code ec;
      serial_.cancel(ec);
      serial_.close(ec);
      AINFO << "Serial closed.";
    }
  } catch (...) {
    // 忽略异常，避免析构时再崩
  }
}

// void JY901BDriver::enter_error_(const std::string& why) {
//   state_.store(State::ERROR);
//   AERROR << "ERROR: " << why;
//   close_serial_();
//   enter_reconnect_wait_();
// }

void JY901BDriver::enter_error_(const std::string& why) {
  if (!running_.load() || state_.load() == State::STOPPED) {
    // 正在停止，不做重连
    AINFO << "Ignore error during shutdown: " << why;
    return;
  }
  state_.store(State::ERROR);
  AERROR << "ERROR: " << why;
  close_serial_();
  enter_reconnect_wait_();
}


void JY901BDriver::enter_reconnect_wait_() {
  state_.store(State::RECONNECT_WAIT);
  next_reconnect_tp_ = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  AINFO << "RECONNECT_WAIT: will retry in 5 seconds.";
}

void JY901BDriver::maybe_reconnect_() {
  if (std::chrono::steady_clock::now() < next_reconnect_tp_) return;
  AINFO << "RECONNECT_WAIT: trying reconnect...";
  if (try_open_serial_()) {
    // 清空解析状态，避免残留错位
    {
      std::lock_guard<std::mutex> lk(rb_mtx_);
      rb_.clear();
    }
    pst_ = ParseState::WAIT_HEAD;
    frame_idx_ = 0;
    state_.store(State::RUNNING);
    AINFO << "RUNNING: reconnect success.";
  } else {
    enter_reconnect_wait_();
  }
}

void JY901BDriver::serial_thread_() {
  AINFO << "Serial thread started.";
  while (running_) {
    State s = state_.load();

    // 状态驱动
    if (s == State::INIT) {
      AINFO << "INIT: checking serial accessibility...";
      if (try_open_serial_()) {
        state_.store(State::RUNNING);
        AINFO << "RUNNING: start reading.";
      } else {
        enter_reconnect_wait_();
      }
      continue;
    }

    if (s == State::RECONNECT_WAIT) {
      maybe_reconnect_();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    if (s != State::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // RUNNING：读块 -> push ring buffer
    std::array<uint8_t, 512> chunk{};
    boost::system::error_code ec;
    // size_t n = serial_.read_some(boost::asio::buffer(chunk), ec);
    // if (ec) {
    //   enter_error_(std::string("serial read error: ") + ec.message());
    //   continue;
    // }
    size_t n = serial_.read_some(boost::asio::buffer(chunk), ec);
    if (ec) {
    // 如果正在退出，不当作错误，不进入重连
    if (!running_.load() || state_.load() == State::STOPPED) {
        AINFO << "Serial read aborted due to shutdown: " << ec.message();
        break;
    }
    enter_error_(std::string("serial read error: ") + ec.message());
    continue;
    }

    if (n == 0) continue;

    {
      std::lock_guard<std::mutex> lk(rb_mtx_);
      for (size_t i = 0; i < n; ++i) {
        if (!rb_.push(chunk[i])) {
          rb_overflow_++;
          // overflow 属于严重问题：会造成丢字节，建议立即重同步并记录
          AERROR << "ring buffer overflow: drop byte. overflow_count=" << rb_overflow_.load();
          rb_.clear();
          pst_ = ParseState::WAIT_HEAD;
          frame_idx_ = 0;
          break;
        }
      }
    }

    // info：状态监控（节流）
    auto now = std::chrono::steady_clock::now();
    if (now - last_state_info_tp_ > std::chrono::seconds(2)) {
      last_state_info_tp_ = now;
      ADEBUG << "RUNNING: reading serial... rb_overflow=" << rb_overflow_.load()
            << " chk_fail=" << chk_fail_.load()
            << " frames_ok=" << frames_ok_.load();
    }
  }
  AINFO << "Serial thread exit.";
}

void JY901BDriver::parse_thread_() {
  AINFO << "Parse thread started.";
  while (running_) {
    State s = state_.load();
    if (s != State::RUNNING) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    parse_from_ring_();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  AINFO << "Parse thread exit.";
}

void JY901BDriver::parse_from_ring_() {
  uint8_t b = 0;
  for (;;) {
    {
      std::lock_guard<std::mutex> lk(rb_mtx_);
      if (!rb_.pop(b)) break;
    }

    switch (pst_) {
      case ParseState::WAIT_HEAD:
        if (b == 0x55) {
          frame_[0] = 0x55;
          frame_idx_ = 1;
          pst_ = ParseState::COLLECT;
        }
        break;

      case ParseState::COLLECT:
        frame_[frame_idx_++] = b;
        if (frame_idx_ == 11) {
          if (verify_checksum_(frame_)) {
            frames_ok_++;
            handle_frame_(frame_);
          } else {
            chk_fail_++;
            AWARN_EVERY_N(500) << "checksum fail count=" << chk_fail_.load();
          }
          pst_ = ParseState::WAIT_HEAD;
          frame_idx_ = 0;
        }
        break;
    }
  }
}

bool JY901BDriver::verify_checksum_(const uint8_t* f) {
  uint32_t sum = 0;
  for (int i = 0; i < 10; ++i) sum += f[i];
  return static_cast<uint8_t>(sum & 0xFF) == f[10];
}

int16_t JY901BDriver::to_i16_(uint8_t l, uint8_t h) {
  const int16_t hh = static_cast<int16_t>(static_cast<int8_t>(h));
  return static_cast<int16_t>((hh << 8) | l);
}

void JY901BDriver::handle_frame_(const uint8_t* f) {
  const uint8_t type = f[1];

  // 原始输出：每种 type 一行（debug）
  // 这里把 raw 的 int16/uint32 都打印出来，方便你对照上位机
  if (type == 0x51) {
    int16_t ax = to_i16_(f[2], f[3]);
    int16_t ay = to_i16_(f[4], f[5]);
    int16_t az = to_i16_(f[6], f[7]);
    ADEBUG << "RAW[0x51 accel] ax=" << ax << " ay=" << ay << " az=" << az
           << " tl=" << int(f[8]) << " th=" << int(f[9]);
    std::lock_guard<std::mutex> lk(data_mtx_);
    cached_.linear_acceleration.x = (double(ax) / 32768.0) * 16.0 * G0;
    cached_.linear_acceleration.y = (double(ay) / 32768.0) * 16.0 * G0;
    cached_.linear_acceleration.z = (double(az) / 32768.0) * 16.0 * G0;
    return;
  }

  if (type == 0x52) {
    int16_t gx = to_i16_(f[2], f[3]);
    int16_t gy = to_i16_(f[4], f[5]);
    int16_t gz = to_i16_(f[6], f[7]);
    ADEBUG << "RAW[0x52 gyro] gx=" << gx << " gy=" << gy << " gz=" << gz
           << " volL=" << int(f[8]) << " volH=" << int(f[9]);
    const double k = M_PI / 180.0;
    std::lock_guard<std::mutex> lk(data_mtx_);
    cached_.angular_velocity.x = (double(gx) / 32768.0) * 2000.0 * k;
    cached_.angular_velocity.y = (double(gy) / 32768.0) * 2000.0 * k;
    cached_.angular_velocity.z = (double(gz) / 32768.0) * 2000.0 * k;
    return;
  }

  if (type == 0x53) {
    int16_t roll  = to_i16_(f[2], f[3]);
    int16_t pitch = to_i16_(f[4], f[5]);
    int16_t yaw   = to_i16_(f[6], f[7]);
    ADEBUG << "RAW[0x53 euler] roll=" << roll << " pitch=" << pitch << " yaw=" << yaw
           << " verL=" << int(f[8]) << " verH=" << int(f[9]);
    std::lock_guard<std::mutex> lk(data_mtx_);
    cached_.euler_rpy.x = (double(roll)  / 32768.0) * M_PI;
    cached_.euler_rpy.y = (double(pitch) / 32768.0) * M_PI;
    cached_.euler_rpy.z = (double(yaw)   / 32768.0) * M_PI;
    return;
  }

  if (type == 0x54) {
    int16_t mx = to_i16_(f[2], f[3]);
    int16_t my = to_i16_(f[4], f[5]);
    int16_t mz = to_i16_(f[6], f[7]);
    ADEBUG << "RAW[0x54 mag] mx=" << mx << " my=" << my << " mz=" << mz
           << " tl=" << int(f[8]) << " th=" << int(f[9]);
    // 注意：磁场单位与你机型可能不一致，这里保持你之前假设：raw 为 mG -> Tesla=raw*1e-7
    std::lock_guard<std::mutex> lk(data_mtx_);
    cached_.magnetic_field.x = double(mx) * 1e-7;
    cached_.magnetic_field.y = double(my) * 1e-7;
    cached_.magnetic_field.z = double(mz) * 1e-7;
    return;
  }

  if (type == 0x56) {
    uint32_t p = (uint32_t(f[5]) << 24) | (uint32_t(f[4]) << 16) | (uint32_t(f[3]) << 8) | uint32_t(f[2]);
    int32_t h_cm = (int32_t(int8_t(f[9])) << 24) | (int32_t(f[8]) << 16) | (int32_t(f[7]) << 8) | int32_t(f[6]);
    ADEBUG << "RAW[0x56 baro] pressure_pa=" << p << " height_cm=" << h_cm;
    std::lock_guard<std::mutex> lk(data_mtx_);
    cached_.pressure_pa = float(p);
    cached_.altitude_m  = float(double(h_cm) / 100.0);
    return;
  }

  if (type == 0x59) {
    int16_t q0 = to_i16_(f[2], f[3]);
    int16_t q1 = to_i16_(f[4], f[5]);
    int16_t q2 = to_i16_(f[6], f[7]);
    int16_t q3 = to_i16_(f[8], f[9]);
    ADEBUG << "RAW[0x59 quat] q0=" << q0 << " q1=" << q1 << " q2=" << q2 << " q3=" << q3;

    std::lock_guard<std::mutex> lk(data_mtx_);
    cached_.orientation.w = double(q0) / 32768.0;
    cached_.orientation.x = double(q1) / 32768.0;
    cached_.orientation.y = double(q2) / 32768.0;
    cached_.orientation.z = double(q3) / 32768.0;
    return;
  }

  // 其他类型：debug 打一行方便你确认第7种帧是什么
  ADEBUG << "RAW[0x" << std::hex << int(type) << std::dec << "] "
         << "d0=" << int(f[2]) << " d1=" << int(f[3]) << " d2=" << int(f[4]) << " d3=" << int(f[5])
         << " d4=" << int(f[6]) << " d5=" << int(f[7]) << " d6=" << int(f[8]) << " d7=" << int(f[9]);
}

void JY901BDriver::publish_timer_cb_() {
  // 发布线程（ROS timer）：100Hz 固定发布，完全与串口/解析解耦
  if (state_.load() != State::RUNNING) return;

  tenaxis_msg::msg::TenaxisImu out;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    out = cached_;
  }
  out.header.stamp = node_->now();
  out.header.frame_id = frame_id_;
  pub_->publish(out);
}

} // namespace ws_imu_jy901b
