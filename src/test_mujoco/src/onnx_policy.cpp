#include "test_mujoco/onnx_policy.hpp"

#include <iostream>
#include <stdexcept>

namespace test_mujoco {

Policy::Policy(const std::string& model_path, bool use_gpu)
    : env_(ORT_LOGGING_LEVEL_WARNING, "policy"),
      session_(nullptr) {
  
  // session options
  Ort::SessionOptions session_options;

  if (use_gpu) {
    try {
      OrtCUDAProviderOptions cuda_opts;
      cuda_opts.device_id = 0;
      cuda_opts.arena_extend_strategy = 0;
      cuda_opts.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchDefault;
      cuda_opts.do_copy_in_default_stream = 1;

      session_options.AppendExecutionProvider_CUDA(cuda_opts);
      std::cout << "Using GPU (CUDA) for inference\n";
    } catch (const Ort::Exception& e) {
      std::cout << "GPU requested but CUDA not available: " << e.what() << "\n";
      std::cout << "Falling back to CPU\n";
    }
  } else {
    std::cout << "Using CPU for inference\n";
  }
  

  // load the model
  session_ = Ort::Session(env_, model_path.c_str(), session_options);

  // ---- Query input info ----
  size_t num_inputs = session_.GetInputCount();
  if (num_inputs != 1) {
    throw std::runtime_error("Expected 1 input, got " + std::to_string(num_inputs));
  }

  // Get input name
  auto input_name = session_.GetInputNameAllocated(0, allocator_);
  input_names_.push_back(input_name.get());

  // Get input shape
  auto input_info = session_.GetInputTypeInfo(0);
  auto tensor_info = input_info.GetTensorTypeAndShapeInfo();
  input_shape_ = tensor_info.GetShape();

  // Shape is typically [1, 160] or [-1, 160]. Fix batch dim to 1.
  if (input_shape_[0] < 0) input_shape_[0] = 1;

  std::cout << "Policy loaded: " << model_path << "\n";
  std::cout << "  input: \"" << input_names_[0]
            << "\"  shape=[" << input_shape_[0] << ", " << input_shape_[1] << "]\n";

  // ---- Query output info ----
  size_t num_outputs = session_.GetOutputCount();
  if (num_outputs != 1) {
    throw std::runtime_error("Expected 1 output, got " + std::to_string(num_outputs));
  }
  auto output_name = session_.GetOutputNameAllocated(0, allocator_);
  output_names_.push_back(output_name.get());

  std::cout << "  output: \"" << output_names_[0] << "\"\n";
}

std::vector<float> Policy::forward(const std::vector<float>& obs) {
  // ---- Build input tensor ----
  // ONNX Runtime doesn't copy the data — it reads directly from our vector.
  auto memory_info = Ort::MemoryInfo::CreateCpu(
      OrtArenaAllocator, OrtMemTypeDefault);

  std::vector<int64_t> shape = {1, static_cast<int64_t>(obs.size())};

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info,
      const_cast<float*>(obs.data()),
      obs.size(),
      shape.data(),
      shape.size());

  // ---- Build name arrays (ONNX Runtime wants const char*) ----
  std::vector<const char*> input_name_ptrs;
  for (const auto& n : input_names_) input_name_ptrs.push_back(n.c_str());

  std::vector<const char*> output_name_ptrs;
  for (const auto& n : output_names_) output_name_ptrs.push_back(n.c_str());

  // ---- Run inference ----
  auto outputs = session_.Run(
      Ort::RunOptions{nullptr},
      input_name_ptrs.data(),
      &input_tensor,
      1,                          // num inputs
      output_name_ptrs.data(),
      1);                         // num outputs

  // ---- Extract output ----
  float* out_ptr = outputs[0].GetTensorMutableData<float>();
  size_t out_size = outputs[0].GetTensorTypeAndShapeInfo().GetElementCount();

  return std::vector<float>(out_ptr, out_ptr + out_size);
}

}  // namespace test_mujoco