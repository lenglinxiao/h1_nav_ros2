#include "h1_2_policy_bridge/core.hpp"
#include <onnxruntime_cxx_api.h>
#include <stdexcept>
#include <cmath>
namespace h1_policy {
struct OrtPolicy::Impl {
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "h1_policy"};
  Ort::SessionOptions opts;
  Ort::Session session;
  Impl(
    const std::string&p
    ) : session(env, p.c_str(), opts){
    auto in_type = session.GetInputTypeInfo(0);
    auto out_type = session.GetOutputTypeInfo(0);
    auto in = in_type.GetTensorTypeAndShapeInfo();
    auto out = out_type.GetTensorTypeAndShapeInfo();
    Ort::AllocatorWithDefaultOptions alloc;
    auto a = session.GetInputNameAllocated(0, alloc), b = session.GetOutputNameAllocated(0, alloc);
    if(session.GetInputCount() != 1||session.GetOutputCount() != 1||std::string(a.get()) != "obs"||std::string(b.get()) != "actions"||in.GetShape() != std::vector<int64_t>{1, 92}||out.GetShape() != std::vector<int64_t>{1, 27}||in.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT||out.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) throw std::runtime_error("unexpected ONNX interface");
  }
};
OrtPolicy::OrtPolicy(
  const std::string& p
  ) : impl_(new Impl(p)){
}
OrtPolicy::~OrtPolicy(){
  delete impl_;
}
Joints OrtPolicy::infer(
  const Observation&o
  ){
  std::array<int64_t, 2> shape{1, 92};
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto input = Ort::Value::CreateTensor<float>(mem, const_cast<float*>(o.data()), o.size(), shape.data(), 2);
  const char* in[] = {"obs"};
  const char* out[] = {"actions"};
  auto result = impl_->session.Run(Ort::RunOptions{nullptr}, in, &input, 1, out, 1);
  Joints a{};
  auto p = result[0].GetTensorData<float>();
  for(int i = 0; i < 27; i++) {
    a[i] = p[i];
    if(!std::isfinite(a[i])) throw std::runtime_error("non-finite policy output");
  } return a;
}
}
