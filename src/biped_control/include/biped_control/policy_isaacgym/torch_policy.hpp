#pragma once

#include <string>
#include <vector>

#include <torch/script.h>

// TorchScript runner for the IsaacGym `biped_walk_flat` actor
// (logs/biped_walk_flat/exported/policies/policy_1.pt): a plain MLP
// [512, 256, 128] with ELU that maps a 189-d observation to a 6-d action.
// Functionally identical to locomotion::Policy; kept in its own namespace.

namespace locomotion_isaacgym {

class Policy {
public:
  explicit Policy(const std::string& model_path);

  // Takes the flat stacked observation (189), returns the action vector (6).
  std::vector<float> forward(const std::vector<float>& obs);

private:
  torch::jit::script::Module module_;
};

}  // namespace locomotion_isaacgym
