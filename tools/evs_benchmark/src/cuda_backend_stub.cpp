#include "evs_benchmark/benchmark.hpp"

#include <stdexcept>

namespace evs_benchmark
{

bool cuda_available() {return false;}
std::string cuda_device_description() {return "CUDA backend was not built";}

Result run_cuda_rolling(const Dataset &, const Config &, std::size_t)
{
  throw std::runtime_error("CUDA backend was not built");
}

}  // namespace evs_benchmark

