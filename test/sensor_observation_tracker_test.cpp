#include <gtest/gtest.h>

#include <map_manager/occupancyMap.h>
#include <map_manager/sensorObservationTracker.h>

namespace {
class InstrumentedOccMap : public mapManager::occMap {
 public:
  void initializeSyntheticMap() {
    mapRes_ = 1.0;
    mapSizeMin_ = Eigen::Vector3d::Zero();
    mapSizeMax_ = Eigen::Vector3d(3.0, 3.0, 3.0);
    mapVoxelMin_ = Eigen::Vector3i::Zero();
    mapVoxelMax_ = Eigen::Vector3i(3, 3, 3);
    robotSize_ = Eigen::Vector3d::Zero();
    pMinLog_ = -2.0;
    pOccLog_ = 2.0;
    const int count = 27;
    countHitMiss_.assign(count, 0);
    countHit_.assign(count, 0);
    occupancy_.assign(count, 0.0);
    occupancyInflated_.assign(count, true);
    sensorObservationTracker_.reset(count);
  }

  std::uint64_t observedTotal() const {
    return sensorObservationTracker_.observedTotal();
  }
  std::size_t pendingCount() const { return sensorObservationTracker_.pending().size(); }
  void simulateSensorUpdate(const Eigen::Vector3i& index) {
    Eigen::Vector3d position;
    indexToPos(index, position);
    updateOccupancyInfo(position, false);
  }
};
}  // namespace

TEST(SensorObservationTracker, CountsEachValidAddressOnce) {
  mapManager::SensorObservationTracker tracker;
  tracker.reset(5);

  EXPECT_FALSE(tracker.mark(-1));
  EXPECT_FALSE(tracker.mark(5));
  EXPECT_TRUE(tracker.mark(2));
  EXPECT_FALSE(tracker.mark(2));
  ASSERT_EQ(tracker.pending().size(), 1u);
  EXPECT_EQ(tracker.pending().front(), 2u);
  EXPECT_EQ(tracker.observedTotal(), 1u);

  tracker.clearPending();
  EXPECT_TRUE(tracker.pending().empty());
  EXPECT_FALSE(tracker.mark(2));
  EXPECT_TRUE(tracker.mark(3));
  ASSERT_EQ(tracker.pending().size(), 1u);
  EXPECT_EQ(tracker.pending().front(), 3u);
  EXPECT_EQ(tracker.observedTotal(), 2u);
}

TEST(SensorObservationTracker, ResetClearsUnionAndPendingDelta) {
  mapManager::SensorObservationTracker tracker;
  tracker.reset(3);
  ASSERT_TRUE(tracker.mark(1));
  tracker.reset(2);
  EXPECT_EQ(tracker.voxelCount(), 2u);
  EXPECT_EQ(tracker.observedTotal(), 0u);
  EXPECT_TRUE(tracker.pending().empty());
  EXPECT_TRUE(tracker.mark(1));
}

TEST(SensorObservationTracker, ArtificialMapClearingDoesNotCreateSensorProvenance) {
  InstrumentedOccMap map;
  map.initializeSyntheticMap();

  map.setFree(Eigen::Vector3i(1, 1, 1));
  map.freeRegion(Eigen::Vector3d(0.0, 0.0, 0.0),
                 Eigen::Vector3d(2.9, 2.9, 2.9));
  EXPECT_EQ(map.observedTotal(), 0u);
  EXPECT_EQ(map.pendingCount(), 0u);

  map.simulateSensorUpdate(Eigen::Vector3i(1, 1, 1));
  EXPECT_EQ(map.observedTotal(), 1u);
  EXPECT_EQ(map.pendingCount(), 1u);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_observation_tracker_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
