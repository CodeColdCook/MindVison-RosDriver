#pragma once

#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <chrono>

using namespace std::chrono;

/*!
 * Get nanoseconds abs
 */
inline int64_t GetNsAbs()
{
  system_clock::time_point tp = system_clock::now();
  system_clock::duration dtn = tp.time_since_epoch();
  return dtn.count();
}

/*!
 * Get seconds abs
 */
inline double GetSecondsAbs() { return (double)GetNsAbs() / 1.e9; }

/*!
 * Timer for measuring time elapsed with clock_monotonic
 */
class Timer
{
public:
  /*!
   * Construct and start timer
   */
  explicit Timer() { Start(); }

  /*!
   * Start the timer
   */
  void Start() { clock_gettime(CLOCK_MONOTONIC, &_startTime); }

  /*!
   * Get milliseconds elapsed
   */
  double GetMs() { return (double)GetNs() / 1.e6; }

  /*!
   * Get nanoseconds elapsed
   */
  int64_t GetNs()
  {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)(now.tv_nsec - _startTime.tv_nsec) +
           1000000000 * (now.tv_sec - _startTime.tv_sec);
  }

  /*!
   * Get seconds elapsed
   */
  double GetSeconds() { return (double)GetNs() / 1.e9; }

  double GetUs() { return (double)GetNs() / 1.e3; }

  struct timespec _startTime;
};
