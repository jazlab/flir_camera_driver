// Copyright 2026 The TableTop contributors
// Licensed under the Apache License, Version 2.0.
#include <gtest/gtest.h>
#include <spinnaker_camera_driver/pipeline_counters.hpp>

using spinnaker_camera_driver::PipelineCounters;

TEST(PipelineCounters, SeparatesSdkAndDriverDrops)
{
  PipelineCounters c;
  c.sdk_received = 13;
  c.sdk_skipped = 2;
  for (size_t depth = 1; depth <= 10; ++depth) {c.ingress(true, depth);}
  for (int i = 0; i < 3; ++i) {c.ingress(false, 10);}
  EXPECT_EQ(c.callbacks, 13u);
  EXPECT_EQ(c.enqueued, 10u);
  EXPECT_EQ(c.queue_dropped, 3u);
  EXPECT_EQ(c.sdk_skipped, 2u);
  EXPECT_EQ(c.queue_peak, 10u);
}

TEST(PipelineCounters, ArchivesAreNotLossesAndTimingSeparatesCpuFromWait)
{
  PipelineCounters c;
  c.dequeue(95000);
  c.published(PipelineCounters::Outcome::MAIN, 10000, 3000);
  c.published(PipelineCounters::Outcome::UNMATCHED, 20000, 4000);
  c.published(PipelineCounters::Outcome::FAILED, 100, 50);
  EXPECT_EQ(c.published_main, 1u);
  EXPECT_EQ(c.published_unmatched, 1u);
  EXPECT_EQ(c.publish_failed, 1u);
  EXPECT_EQ(c.publish_calls, 3u);
  EXPECT_EQ(c.publish_wall_us, 30100u);
  EXPECT_EQ(c.publish_cpu_us, 7050u);
  EXPECT_EQ(c.publish_wall_us_max, 20000u);
  EXPECT_EQ(c.callback_age_us_max, 95000u);
}

TEST(PipelineCounters, SnapshotsResetOnlyIntervalPeaks)
{
  PipelineCounters c;
  c.ingress(true, 10);
  c.dequeue(90000);
  c.published(PipelineCounters::Outcome::MAIN, 10000, 3000);
  const auto first = c.snapshot(2);
  EXPECT_EQ(first.interval_queue_peak, 10u);
  EXPECT_EQ(first.interval_wall_us_max, 10000u);
  const auto second = c.snapshot(0);
  EXPECT_EQ(second.interval_queue_peak, 2u);
  EXPECT_EQ(second.interval_wall_us_max, 0u);
  EXPECT_EQ(second.interval_age_us_max, 0u);
  EXPECT_EQ(second.publish_calls, 1u);
  EXPECT_EQ(second.queue_peak, 10u);
  EXPECT_EQ(second.publish_wall_us_max, 10000u);
}
