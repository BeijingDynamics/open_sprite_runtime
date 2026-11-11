#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

class FixedRmsWindow {
 public:
  void prepare(std::size_t size) {
    if (!size) throw std::runtime_error("empty RMS window");
    samples_.assign(size, 0.0);
    reset();
  }
  void reset() {
    std::fill(samples_.begin(), samples_.end(), 0.0);
    head_ = count_ = 0;
    sum_ = 0;
  }
  double update(double squared) {
    if (samples_.empty()) throw std::runtime_error("RMS window not prepared");
    // Preserve the original add-then-subtract summation order.
    sum_ += squared;
    if (count_ == samples_.size()) sum_ -= samples_[head_];
    else ++count_;
    samples_[head_] = squared;
    if (++head_ == samples_.size()) head_ = 0;
    return std::sqrt(std::max(0.0, sum_) / static_cast<double>(count_));
  }
  bool full() const { return count_ == samples_.size() && count_ != 0; }
 private:
  std::vector<double> samples_;
  std::size_t head_ = 0, count_ = 0;
  double sum_ = 0;
};
