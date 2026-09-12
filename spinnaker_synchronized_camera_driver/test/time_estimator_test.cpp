// Copyright 2026 TableTop contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstdint>

#include <spinnaker_synchronized_camera_driver/time_estimator.hpp>

namespace
{
constexpr uint64_t SECOND = 1000000000ULL;
constexpr uint64_t FRAME_PERIOD = SECOND / 120;

TEST(TimeEstimator, ResetsAndRecoversAfterHugeForwardTimestampJump)
{
  spinnaker_synchronized_camera_driver::TimeEstimator estimator;
  uint64_t frameTime = 0;
  estimator.initialize(SECOND, 1.0 / 120.0);

  EXPECT_TRUE(estimator.update(0, SECOND, &frameTime));

  const uint64_t jumpedTime = SECOND + 24ULL * 60ULL * 60ULL * SECOND;
  EXPECT_FALSE(estimator.update(0, jumpedTime, &frameTime));
  EXPECT_EQ(frameTime, jumpedTime);
  EXPECT_TRUE(estimator.update(0, jumpedTime + FRAME_PERIOD, &frameTime));
}

TEST(TimeEstimator, ResetsAndRecoversAfterHostClockRewind)
{
  spinnaker_synchronized_camera_driver::TimeEstimator estimator;
  uint64_t frameTime = 0;
  estimator.initialize(10ULL * SECOND, 1.0 / 120.0);

  EXPECT_TRUE(estimator.update(0, 10ULL * SECOND, &frameTime));
  EXPECT_FALSE(estimator.update(0, SECOND, &frameTime));
  EXPECT_EQ(frameTime, SECOND);
  EXPECT_TRUE(estimator.update(0, SECOND + FRAME_PERIOD, &frameTime));
}

TEST(TimeEstimator, InvalidInitialPeriodCannotCreateANonAdvancingPredictor)
{
  spinnaker_synchronized_camera_driver::TimeEstimator estimator;
  uint64_t frameTime = 0;
  estimator.initialize(SECOND, -1.0);

  EXPECT_TRUE(estimator.update(0, SECOND, &frameTime));
  EXPECT_TRUE(estimator.update(0, SECOND + FRAME_PERIOD, &frameTime));
}

TEST(TimeEstimator, RetainsHistoryForTenFrameCameraFifoBacklog)
{
  spinnaker_synchronized_camera_driver::TimeEstimator estimator;
  uint64_t frameTime = 0;
  estimator.initialize(SECOND, 1.0 / 120.0);

  for (uint64_t frame = 0; frame <= 20; ++frame) {
    EXPECT_TRUE(estimator.update(0, SECOND + frame * FRAME_PERIOD, &frameTime));
  }

  // One camera can still deliver this valid older frame after the other
  // cameras have advanced the shared estimator by the full FIFO depth.
  EXPECT_TRUE(estimator.update(1, SECOND + 10 * FRAME_PERIOD, &frameTime));
}

TEST(TimeEstimator, RetainsThirtyTwoTimestampRecordsDuringSchedulingStall)
{
  spinnaker_synchronized_camera_driver::TimeEstimator estimator;
  uint64_t frameTime = 0;
  estimator.initialize(SECOND, 1.0 / 120.0);

  for (uint64_t frame = 0; frame <= 48; ++frame) {
    EXPECT_TRUE(estimator.update(0, SECOND + frame * FRAME_PERIOD, &frameTime));
  }

  // The active list contains 32 timestamp slots (frames 17 through 48), so a
  // camera delayed by 31 exposure periods still obtains canonical timing.
  EXPECT_TRUE(estimator.update(1, SECOND + 17 * FRAME_PERIOD, &frameTime));
}
}  // namespace
