#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace ws_imu_jy901b {

template <size_t N>
class RingBuffer {
public:
  bool push(uint8_t b) {
    if (size_ >= N) return false;
    buf_[head_] = b;
    head_ = (head_ + 1) % N;
    ++size_;
    return true;
  }

  bool pop(uint8_t &b) {
    if (size_ == 0) return false;
    b = buf_[tail_];
    tail_ = (tail_ + 1) % N;
    --size_;
    return true;
  }

  size_t size() const { return size_; }
  constexpr size_t capacity() const { return N; }
  void clear() { head_ = tail_ = size_ = 0; }

private:
  std::array<uint8_t, N> buf_{};
  size_t head_{0}, tail_{0}, size_{0};
};

} // namespace ws_imu_jy901b
