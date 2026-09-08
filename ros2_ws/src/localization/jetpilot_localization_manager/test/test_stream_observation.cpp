#include <cassert>
#include <chrono>
#include "jetpilot_localization_manager/stream_observation.hpp"

int main()
{
  using jetpilot_localization_manager::StreamObservation;
  using namespace std::chrono_literals;
  StreamObservation stream;
  const auto t = StreamObservation::Clock::time_point{};
  assert(!stream.fresh(1000000000, t, 1.5));
  stream.observe(1000000000, t);
  assert(stream.fresh(1000000000, t, 1.5));
  // A frozen ROS clock, or retransmitted old TF, cannot hide a stopped stream.
  stream.observe(1000000000, t + 2s);
  assert(!stream.fresh(1000000000, t + 2s, 1.5));
  // Newly received old and future-dated transforms are not healthy.
  stream.observe(2000000000, t + 3s);
  assert(!stream.fresh(5000000000, t + 3s, 1.5));
  assert(!stream.fresh(1000000000, t + 3s, 1.5));
  stream.observe(5000000000, t + 4s);
  assert(stream.fresh(5100000000, t + 4s, 1.5));
  // A replay clock rewind can recover once the stamps agree again.
  stream.observe(100000000, t + 5s);
  assert(stream.fresh(100000000, t + 5s, 1.5));
}
