#include "biped_control/policy_isaacgym/torch_policy.hpp"

#include <iostream>
#include <stdexcept>

namespace locomotion_isaacgym {

Policy::Policy(const std::string& model_path) {
  try {
    module_ = torch::jit::load(model_path);
  } catch (const c10::Error& e) {
    throw std::runtime_error("Failed to load policy: " + model_path
                             + "\n  " + e.what());
  }
  module_.eval();

  // Use a single thread. For a tiny policy, multiple threads add jitter
  // without any speed benefit.
  //   torch::set_num_threads(1);

  std::cout << "IsaacGym policy loaded: " << model_path << "\n";
}

std::vector<float> Policy::forward(const std::vector<float>& obs) {
  // Create a tensor that views the input data. clone() so torch owns the memory.
  auto tensor = torch::from_blob(
      const_cast<float*>(obs.data()),
      {1, static_cast<long>(obs.size())},
      torch::kFloat32
  ).clone();

  torch::NoGradGuard no_grad;
  auto output = module_.forward({tensor}).toTensor().contiguous();

  // Copy output into a std::vector
  float* ptr = output.data_ptr<float>();
  return std::vector<float>(ptr, ptr + output.numel());
}

}  // namespace locomotion_isaacgym
