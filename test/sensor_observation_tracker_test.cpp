#include <gtest/gtest.h>
#include <limits>

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
    validSensorObservationTracker_.reset(count);
    validHitFrameCount_.assign(count, 0);
    validHitLastRaycastId_.assign(count, -1);
    trackValidDepthObservations_ = true;
  }

  std::uint64_t observedTotal() const {
    return sensorObservationTracker_.observedTotal();
  }
  std::uint64_t validObservedTotal() const {
    return validSensorObservationTracker_.observedTotal();
  }
  std::size_t pendingCount() const { return sensorObservationTracker_.pending().size(); }
  void simulateSensorUpdate(const Eigen::Vector3i& index, bool valid = false) {
    Eigen::Vector3d position;
    indexToPos(index, position);
    currentRayValid_ = valid;
    updateOccupancyInfo(position, false);
    currentRayValid_ = false;
  }
  void simulateValidHitFrame(const Eigen::Vector3i& index, int frameId) {
    Eigen::Vector3d position;
    indexToPos(index, position);
    raycastNum_ = frameId;
    currentRayValid_ = true;
    updateOccupancyInfo(position, true);
    currentRayValid_ = false;
  }
  std::vector<double> projectSyntheticDepth(bool strict) {
    filterInvalidDepth_ = strict;
    depthMinValue_ = 0.5;
    depthMaxValue_ = 5.0;
    raycastMaxLength_ = 5.0;
    depthScale_ = 10.0;
    depthFilterMargin_ = 0;
    skipPixel_ = 1;
    fx_ = fy_ = 1.0;
    cx_ = cy_ = 0.0;
    position_ = Eigen::Vector3d::Zero();
    orientation_ = Eigen::Matrix3d::Identity();
    depthImage_ = (cv::Mat_<std::uint16_t>(1, 4) << 10, 0, 2, 20);
    depthRawValidity_ = (cv::Mat_<std::uint8_t>(1, 4) << 1, 0, 0, 1);
    projPoints_.resize(4);
    projPointsValid_.resize(4);
    projectDepthImage();
    std::vector<double> depths;
    for (int i = 0; i < projPointsNum_; ++i) depths.push_back(projPoints_[i].z());
    return depths;
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
  EXPECT_TRUE(tracker.hasObserved(2));
  EXPECT_TRUE(tracker.hasObserved(3));
  EXPECT_FALSE(tracker.hasObserved(1));
  EXPECT_FALSE(tracker.hasObserved(-1));
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

TEST(DepthEvidence, RejectsNonFiniteAndOutOfRangeReadings) {
  using mapManager::validDepthHit;
  EXPECT_FALSE(validDepthHit(std::numeric_limits<float>::quiet_NaN(), 0.5, 5.0));
  EXPECT_FALSE(validDepthHit(std::numeric_limits<float>::infinity(), 0.5, 5.0));
  EXPECT_FALSE(validDepthHit(0.0f, 0.5, 5.0));
  EXPECT_FALSE(validDepthHit(0.49f, 0.5, 5.0));
  EXPECT_TRUE(validDepthHit(0.5f, 0.5, 5.0));
  EXPECT_TRUE(validDepthHit(4.99f, 0.5, 5.0));
  EXPECT_FALSE(validDepthHit(5.0f, 0.5, 5.0));
}

TEST(DepthEvidence, StrictProjectionSkipsInvalidWithoutChangingLegacyProjection) {
  InstrumentedOccMap map;
  const auto strict = map.projectSyntheticDepth(true);
  ASSERT_EQ(strict.size(), 2u);
  EXPECT_DOUBLE_EQ(strict[0], 1.0);
  EXPECT_DOUBLE_EQ(strict[1], 2.0);
  const auto legacy = map.projectSyntheticDepth(false);
  ASSERT_EQ(legacy.size(), 2u);
  EXPECT_DOUBLE_EQ(legacy[0], 1.0);
  EXPECT_DOUBLE_EQ(legacy[1], 5.1);
}

TEST(DepthEvidence, ValidOnlyProvenanceExcludesInvalidRayMapUpdates) {
  InstrumentedOccMap map;
  map.initializeSyntheticMap();
  map.simulateSensorUpdate(Eigen::Vector3i(0, 0, 0), true);
  map.simulateSensorUpdate(Eigen::Vector3i(1, 0, 0), false);
  EXPECT_EQ(map.observedTotal(), 2u);
  EXPECT_EQ(map.validObservedTotal(), 1u);
  EXPECT_TRUE(map.hasValidDepthEvidence(Eigen::Vector3d(0.5, 0.5, 0.5)));
  EXPECT_FALSE(map.hasValidDepthEvidence(Eigen::Vector3d(1.5, 0.5, 0.5)));
  EXPECT_FALSE(map.hasValidDepthEvidence(Eigen::Vector3d(3.0, 0.5, 0.5)));
}

TEST(DepthEvidence, StableSurfaceSupportCountsDistinctDepthFrames) {
  InstrumentedOccMap map;
  map.initializeSyntheticMap();
  const Eigen::Vector3i address(1, 1, 1);
  map.simulateValidHitFrame(address, 1);
  map.simulateValidHitFrame(address, 1);
  EXPECT_EQ(map.captureSnapshot().validHitFrameCount[13], 1u);
  map.simulateValidHitFrame(address, 2);
  map.simulateValidHitFrame(address, 3);
  EXPECT_EQ(map.captureSnapshot().validHitFrameCount[13], 3u);
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
