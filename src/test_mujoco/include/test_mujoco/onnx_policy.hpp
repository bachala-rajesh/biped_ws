#pragma once

#include <string>
#include <vector>
#include <memory>

#include <onnxruntime_cxx_api.h>

namespace test_mujoco {

class Policy {
public:
  explicit Policy(const std::string& model_path, bool use_gpu = false);

  // Takes flat stacked observation, returns action vector (6 values).
  std::vector<float> forward(const std::vector<float>& obs);

private:
  Ort::Env env_;
  Ort::Session session_;
  Ort::AllocatorWithDefaultOptions allocator_;

  // Cached input/output metadata
  std::vector<std::string> input_names_;
  std::vector<std::string> output_names_;
  std::vector<int64_t> input_shape_;
};

}  // namespace test_mujoco