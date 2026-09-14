#include <gtest/gtest.h>

#include <map_manager/sensorObservationTracker.h>

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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
