// Copyright 2026 The TableTop contributors
// Licensed under the Apache License, Version 2.0.
#ifndef SPINNAKER_CAMERA_DRIVER__PIPELINE_COUNTERS_HPP_
#define SPINNAKER_CAMERA_DRIVER__PIPELINE_COUNTERS_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace spinnaker_camera_driver
{
// Caller holds Camera::mutex_. No allocation, logging, SDK calls or new locks
// in these per-frame operations. Totals live until the camera is recreated.
struct PipelineCounters
{
  enum class Outcome { MAIN, UNMATCHED, FAILED };
  uint64_t sdk_received{0}, sdk_skipped{0}, sdk_incomplete{0}, sdk_timeouts{0};
  uint64_t callbacks{0}, enqueued{0}, queue_dropped{0}, dequeued{0};
  uint64_t published_main{0}, published_unmatched{0}, publish_failed{0};
  uint64_t publish_calls{0}, publish_wall_us{0}, publish_cpu_us{0};
  uint64_t publish_wall_us_max{0}, publish_cpu_us_max{0}, callback_age_us_max{0};
  size_t queue_peak{0};
  uint64_t interval_wall_us_max{0}, interval_cpu_us_max{0}, interval_age_us_max{0};
  size_t interval_queue_peak{0};

  void ingress(bool accepted, size_t depth)
  {
    ++callbacks;
    if (accepted) {++enqueued;} else {++queue_dropped;}
    queue_peak = std::max(queue_peak, depth);
    interval_queue_peak = std::max(interval_queue_peak, depth);
  }
  void dequeue(uint64_t age_us)
  {
    ++dequeued;
    callback_age_us_max = std::max(callback_age_us_max, age_us);
    interval_age_us_max = std::max(interval_age_us_max, age_us);
  }
  void published(Outcome outcome, uint64_t wall_us, uint64_t cpu_us)
  {
    ++publish_calls;
    if (outcome == Outcome::MAIN) {++published_main;}
    else if (outcome == Outcome::UNMATCHED) {++published_unmatched;}
    else {++publish_failed;}
    publish_wall_us += wall_us;
    publish_cpu_us += cpu_us;
    publish_wall_us_max = std::max(publish_wall_us_max, wall_us);
    publish_cpu_us_max = std::max(publish_cpu_us_max, cpu_us);
    interval_wall_us_max = std::max(interval_wall_us_max, wall_us);
    interval_cpu_us_max = std::max(interval_cpu_us_max, cpu_us);
  }
  PipelineCounters snapshot(size_t current_depth)
  {
    const auto result = *this;
    interval_wall_us_max = interval_cpu_us_max = interval_age_us_max = 0;
    interval_queue_peak = current_depth;
    return result;
  }
};
}  // namespace spinnaker_camera_driver
#endif  // SPINNAKER_CAMERA_DRIVER__PIPELINE_COUNTERS_HPP_
