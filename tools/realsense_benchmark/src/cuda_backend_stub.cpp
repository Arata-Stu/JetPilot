#include "realsense_benchmark/benchmark.hpp"

#include <stdexcept>

namespace realsense_benchmark
{
bool cuda_available() {return false;}
std::string cuda_device_description() {return "unavailable";}
Result run_cuda(const Dataset &, const Config &, std::size_t)
{
  throw std::runtime_error("CUDA backend was not built");
}
}  // namespace realsense_benchmark
