// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <spinnaker_synchronized_camera_driver/logging.hpp>
#include <spinnaker_synchronized_camera_driver/time_estimator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

// #define DEBUG

static rclcpp::Logger get_logger() { return (rclcpp::get_logger("cam_sync")); }

namespace spinnaker_synchronized_camera_driver
{
void TimeEstimator::initialize(uint64_t t0, double dt)
{
  if (!std::isfinite(dt) || dt < 1e-6 || dt > 1.0) {
    LOG_WARN(
      "invalid frame interval " << dt << " seconds; using "
                                << DEFAULT_FRAME_INTERVAL_SECONDS << " seconds");
    dt = DEFAULT_FRAME_INTERVAL_SECONDS;
  }
  nominalFrameInterval_ = dt;
  T0_ = t0;
  x_[0] = 0;
  x_[1] = dt;
  const double processSigma = dt * 0.01;     // process noise
  const double measurementSigma = dt * 0.1;  // measurement noise
  Q_ = processSigma * processSigma;
  // it is important to initialize the off-diagonal elements as well
  // or else the
  P_[0][0] = P_[1][1] = P_[0][1] = P_[1][0] = Q_;
  R_ = measurementSigma * measurementSigma;
  frameTimes_.clear();
  LOG_INFO_FMT("frequency estimator initialized with %.3f Hz", 1.0 / dt);
  frameTimes_.push_back(FrameTime(0));
}

void TimeEstimator::resetAt(uint64_t t)
{
  initialize(t, nominalFrameInterval_);
}

void TimeEstimator::updateKalman(double z)
{
  /*
  The next synchronized frame time is the result of a
  Kalman filter prediction step. When a new frame arrives that
  no longer fits within the bounds of the current frame,
  then the average (z) of the current frames' arrival times is
  used as a measurement, which will ultimately and indirectly update
  the estimated inter-frame time difference.
  The state of the filter has thus two components:
     x = [current_frame_arrival_time]
         [frame_interval time       ]
  where only x[0] is directly measured, but x[1] is not observed.

  Filter equations:
     x = [current_frame_arrival_time]
         [frame_interval            ]
     H = [1 0]
     F = [1 1]
         [0 1]
     y = z - H * x = z - H * x[0]
     S = H P H^T + R = P[0][0] + R
     K = P H^T S^-1
       = S^-1 * [P[0][0]]
                [P[1][0]]
     x = x + K y
     P = (I - K H) * P
    */
  const double y = z - x_[0];      // residual
  const double S = P_[0][0] + R_;  // measurement noise
  const double S_inv = 1.0 / S;
  x_[0] += S_inv * P_[0][0] * y;
  x_[1] += S_inv * P_[1][0] * y;

  const double f = 1 - S_inv * P_[0][0];
  P_[0][0] *= f;  // this is what the filter equations reduce to
  P_[0][1] *= f;  // due to H and K being very simple
  P_[1][0] *= f;
  P_[1][1] -= S_inv * P_[0][1] * P_[1][0];
}

int64_t TimeEstimator::predict()
{
  x_[0] += x_[1];
  // x_[1] = x_[1];
  P_[0][0] += Q_;
  P_[1][1] += Q_;
  P_[0][1] += Q_;
  P_[1][0] += Q_;
  return (static_cast<int64_t>(x_[0] * 1e9));
}

bool TimeEstimator::getTimeFromList(uint64_t t_a, uint64_t * T)
{
#if 0
  LOG_INFO_FMT("checking list for time: %8ld", t_a);
  for (const auto & tl : frameTimes_) {
    LOG_INFO_FMT("   %8ld", tl.getFrameTime());
  }
#endif
  // A camera callback may legitimately trail the other five cameras by the
  // configured 10-frame FIFO depth. Keep substantial headroom for brief
  // six-camera scheduling stalls while retaining the independent bound on
  // forward estimator work. These are tiny timestamp records, not images.
  constexpr size_t MAX_NUM_FRAMES_TO_KEEP = 32;
  if (frameTimes_.empty()) {
    resetAt(t_a);
    *T = t_a;
    return (false);
  }

  // All cameras share this estimator. A timestamp a fraction of a frame older
  // than the current epoch can therefore be a normal late arrival. A larger
  // rewind means the source clock reset and the old estimator is no longer
  // usable.
  const auto lateArrivalTolerance = static_cast<uint64_t>(
    std::max(1e-3, nominalFrameInterval_ * 2.0) * 1e9);
  if (t_a < T0_ && T0_ - t_a > lateArrivalTolerance) {
    LOG_WARN("camera host timestamp moved backwards; resetting time estimator");
    resetAt(t_a);
    *T = t_a;
    return (false);
  }

  const uint64_t relativeTime = t_a >= T0_ ? t_a - T0_ : 0;
  if (relativeTime > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())) {
    LOG_WARN("camera host timestamp exceeded estimator range; resetting time estimator");
    resetAt(t_a);
    *T = t_a;
    return (false);
  }
  if (!std::isfinite(x_[0]) || !std::isfinite(x_[1]) || x_[1] < 1e-6 || x_[1] > 1.0) {
    LOG_WARN("camera frame-period estimate became invalid; resetting time estimator");
    resetAt(t_a);
    *T = t_a;
    return (false);
  }

  const auto t = static_cast<int64_t>(relativeTime);
  const auto dT = static_cast<int64_t>(x_[1] * 1e9);
  const auto latestFrameTime = (*frameTimes_.rbegin()).getFrameTime();
  if (
    t > latestFrameTime &&
    static_cast<uint64_t>(t - latestFrameTime) / static_cast<uint64_t>(dT) >
      MAX_ESTIMATOR_ADVANCE_STEPS)
  {
    LOG_WARN("camera timestamp discontinuity exceeded safe advance; resetting time estimator");
    resetAt(t_a);
    *T = t_a;
    return (false);
  }
  const auto T_min = (*frameTimes_.begin()).getFrameTime();
  const int64_t dT_min = static_cast<int64_t>(T_min) - static_cast<int64_t>(t);

  if (dT_min * 2 > dT) {
    // far away from the oldest frame time
    oldFrameRejectionCount_++;
    if (oldFrameRejectionCount_ == 1 || oldFrameRejectionCount_ % 120 == 0) {
      LOG_WARN(
        "rejecting frame older than timestamp history (total: "
        << oldFrameRejectionCount_ << ")");
    }
    return (false);
  }

  if (dT_min >= 0 && dT_min * 2 <= dT) {
    // this frame is older than the oldest frame time but still within range
    (*frameTimes_.begin()).addTime(t);
    *T = static_cast<uint64_t>(T_min) + T0_;
#ifdef DEBUG
    LOG_INFO("frame old but within range");
#endif
    return (true);
  }

  // make sure to add new frames if needed
  size_t advanceSteps = 0;
  for (auto ft = *(frameTimes_.rbegin());
       t > ft.getFrameTime() && t - ft.getFrameTime() > dT / 2;
       ft = *(frameTimes_.rbegin())) {
    // The list is trimmed only after this loop. Without this bound, a bad
    // timestamp or Kalman period can make one callback allocate until the host
    // is OOM-killed. This is a computation/memory bound, not a reorder window.
    if (advanceSteps++ >= MAX_ESTIMATOR_ADVANCE_STEPS) {
      LOG_WARN("camera timestamp jumped ahead; resetting time estimator");
      resetAt(t_a);
      *T = t_a;
      return (false);
    }
    // if a new frame is found, perform a measurement update
    // based on the average arrival time of the last frames that
    // have arrived before the new frame
    if (ft.isValid()) {
      updateKalman(ft.getAverageFrameTime());
    }
    const auto previousTime = ft.getFrameTime();
    const auto nextTime = predict();
    if (nextTime <= previousTime) {
      LOG_WARN("camera frame-period prediction stopped advancing; resetting time estimator");
      resetAt(t_a);
      *T = t_a;
      return (false);
    }
    frameTimes_.push_back(FrameTime(nextTime));
  }

  while (frameTimes_.size() > MAX_NUM_FRAMES_TO_KEEP) {
    frameTimes_.pop_front();
  }

  const int64_t T_max = (*frameTimes_.rbegin()).getFrameTime();
  const auto dT_max = static_cast<int64_t>(t) - T_max;

  if (dT_max >= 0 && dT_max * 2 <= dT) {
    // this frame is younger than the latest frame time but still in range
    (*frameTimes_.rbegin()).addTime(t);
    *T = static_cast<uint64_t>(T_max) + T0_;
#ifdef DEBUG
    LOG_INFO("frame young but within range");
#endif
    return (true);
  }

  // the following search must succeed
  for (auto it = frameTimes_.begin();; ++it) {
    auto itp1 = it;
    itp1++;
    if (itp1 == frameTimes_.end()) {
      break;
    }
    const auto t1 = it->getFrameTime();
    const auto t2 = itp1->getFrameTime();
    const auto dTi = t2 - t1;
    if (t >= t1 && t < t2) {
      if ((t - t1) * 2 < dTi) {
        // closer to beginning of interval
        (*it).addTime(t);
        *T = static_cast<uint64_t>(t1) + T0_;
#ifdef DEBUG
        LOG_INFO("frame closer to begin of interval");
#endif
        return (true);
      } else {
        // closer to end of interval
        (*itp1).addTime(t);
        *T = static_cast<uint64_t>(t2) + T0_;
#ifdef DEBUG
        LOG_INFO("frame closer to end of interval");
#endif
        return (true);
      }
    }
  }
  // since frames are added to frameTimes_ until the current
  // time is included in the active frame list, the frame time
  // should always be found in the list, and this point should
  // never be reached.
  LOG_ERROR("INTERNAL BUG, should never reach this point!!");
  *T = static_cast<uint64_t>(T_max) + T0_;
  LOG_INFO_FMT("newly added time: %8ld", t_a);
  for (const auto & tl : frameTimes_) {
    LOG_INFO_FMT("   %8ld", tl.getFrameTime());
  }
  return (true);
}

bool TimeEstimator::update(size_t idx, uint64_t t_a, uint64_t * frameTime)
{
  (void)idx;
  const bool gotValidTime = getTimeFromList(t_a, frameTime);
  return (gotValidTime);
}
}  // namespace spinnaker_synchronized_camera_driver
