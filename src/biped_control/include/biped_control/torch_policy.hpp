#pragma once

#include <string>
#include <vector>

#include <torch/script.h>

namespace locomotion {

class Policy {
public:
  explicit Policy(const std::string& model_path);

  // Takes the flat stacked observation, returns action vector (6 values).
  std::vector<float> forward(const std::vector<float>& obs);

private:
  torch::jit::script::Module module_;
};

}  // namespace locomotion