#include <sstream>
#include <iomanip>
#include <cctype>
#include <filesystem>
#include <algorithm>

#include "shared.h"


std::unique_ptr<Shared> Shared::instance = nullptr;
std::once_flag Shared::flag;

void Shared::init(std::random_device::result_type rand_seed, bool is_show_rand_seed) {

  // cout setting
  std::cout << std::boolalpha;

  // random seek
  std::call_once(Shared::flag, [&]() {
    Shared::instance.reset(new Shared);
    Shared::instance->setRandSeed(rand_seed);
    if (is_show_rand_seed) {
      std::cout << "Random seed = " << Shared::instance->rand_seed << std::endl;
    }
  });
}

void Shared::setRandSeed(std::random_device::result_type new_rand_seed) {
  if (new_rand_seed == 0) {
    std::random_device dev;
    new_rand_seed = dev();
  }
  rand_seed = new_rand_seed;

  rng = std::mt19937{rand_seed};
}


void __line__(const std::string& message, const std::source_location& location) {
  std::cout << "--- " << location.line();
  std::cout << " in " << std::filesystem::path(location.file_name()).filename();
  if (!message.empty()) {
    std::cout << " : " << message;
  }
  std::cout << " ---" << std::endl;
}


void __pp__() {
  std::cout << std::endl;
}

void __pp_impl__(const std::vector<bool>& vs) {
  std::cout << "[";
  bool isFirst = true;
  for(auto v : vs) {
    if (isFirst) {
      isFirst = false;
    } else {
      std::cout << ",";
    }
    __pp_impl__(v);
  }
  std::cout << "]";
}
