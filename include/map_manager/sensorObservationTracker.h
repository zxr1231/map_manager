#ifndef MAPMANAGER_SENSOR_OBSERVATION_TRACKER_H
#define MAPMANAGER_SENSOR_OBSERVATION_TRACKER_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace mapManager {

class SensorObservationTracker {
 public:
  void reset(std::size_t voxelCount) {
    observed_.assign(voxelCount, 0);
    pending_.clear();
    observedTotal_ = 0;
  }

  bool mark(int address) {
    if (address < 0 || static_cast<std::size_t>(address) >= observed_.size()) return false;
    if (observed_[address] != 0) return false;
    observed_[address] = 1;
    pending_.push_back(static_cast<std::uint32_t>(address));
    ++observedTotal_;
    return true;
  }

  const std::vector<std::uint32_t>& pending() const { return pending_; }
  void clearPending() { pending_.clear(); }
  std::uint64_t observedTotal() const { return observedTotal_; }
  std::size_t voxelCount() const { return observed_.size(); }

 private:
  std::vector<std::uint8_t> observed_;
  std::vector<std::uint32_t> pending_;
  std::uint64_t observedTotal_ = 0;
};

}  // namespace mapManager

#endif
