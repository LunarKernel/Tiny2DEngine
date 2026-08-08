#ifndef TINY2DENGINE_SANDBOX_FIXED_STEP_CLOCK_H_
#define TINY2DENGINE_SANDBOX_FIXED_STEP_CLOCK_H_

#include <cassert>
#include <cmath>
#include <cstdint>

namespace tiny2d::sandbox {

class FixedStepClock {
 public:
  // counter_frequency_hz must be positive and finite.
  FixedStepClock(double counter_frequency_hz,
                 std::uint64_t initial_counter) noexcept
      : counter_frequency_hz_(counter_frequency_hz),
        previous_counter_(initial_counter) {
    assert(std::isfinite(counter_frequency_hz_) && counter_frequency_hz_ > 0.0);
  }

  void Reset(std::uint64_t current_counter) noexcept {
    previous_counter_ = current_counter;
    accumulated_time_seconds_ = 0.0;
  }

  // maximum_frame_time_seconds must be positive and finite.
  void Accumulate(std::uint64_t current_counter,
                  double maximum_frame_time_seconds) noexcept {
    assert(std::isfinite(maximum_frame_time_seconds) &&
           maximum_frame_time_seconds > 0.0);
    if (current_counter >= previous_counter_) {
      const double elapsed_seconds =
          static_cast<double>(current_counter - previous_counter_) /
          counter_frequency_hz_;
      accumulated_time_seconds_ += elapsed_seconds < maximum_frame_time_seconds
                                       ? elapsed_seconds
                                       : maximum_frame_time_seconds;
    }
    previous_counter_ = current_counter;
  }

  // step_seconds must be positive and finite.
  [[nodiscard]] bool HasStep(double step_seconds) const noexcept {
    assert(std::isfinite(step_seconds) && step_seconds > 0.0);
    return accumulated_time_seconds_ >= step_seconds;
  }

  // step_seconds must be positive and finite, and HasStep must be true.
  void ConsumeStep(double step_seconds) noexcept {
    assert(std::isfinite(step_seconds) && step_seconds > 0.0);
    assert(HasStep(step_seconds));
    accumulated_time_seconds_ -= step_seconds;
  }

  void DiscardPendingSteps() noexcept { accumulated_time_seconds_ = 0.0; }

 private:
  double counter_frequency_hz_;
  std::uint64_t previous_counter_;
  double accumulated_time_seconds_ = 0.0;
};

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_FIXED_STEP_CLOCK_H_
