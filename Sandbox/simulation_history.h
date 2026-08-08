#ifndef TINY2DENGINE_SANDBOX_SIMULATION_HISTORY_H_
#define TINY2DENGINE_SANDBOX_SIMULATION_HISTORY_H_

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace tiny2d::sandbox {

inline constexpr std::size_t kMaxSimulationHistorySamples = 16384;

// Appends a finite sample whose time in simulation seconds is strictly newer
// than the recorded tail. Null pointers and invalid times are rejected without
// changing history. At the limit, older samples are decimated before append.
template <typename Sample>
bool AppendHistorySample(std::vector<Sample>* history, Sample sample,
                         double Sample::* time_member) {
  if (history == nullptr || time_member == nullptr ||
      !std::isfinite(sample.*time_member) ||
      (!history->empty() &&
       (!std::isfinite(history->back().*time_member) ||
        sample.*time_member <= history->back().*time_member))) {
    return false;
  }

  if (history->size() >= kMaxSimulationHistorySamples) {
    std::size_t write_index = 0;
    for (std::size_t read_index = 0; read_index + 1 < history->size();
         read_index += 2) {
      if (write_index != read_index) {
        (*history)[write_index] = std::move((*history)[read_index]);
      }
      ++write_index;
    }
    (*history)[write_index++] = std::move(history->back());
    history->resize(write_index);
  }
  history->push_back(std::move(sample));
  return true;
}

}  // namespace tiny2d::sandbox

#endif  // TINY2DENGINE_SANDBOX_SIMULATION_HISTORY_H_
