#include "h1_2_policy_bridge/core.hpp"
#include <cmath>
#include <iostream>
int main(
  int argc, char**argv
  ){
  try{
    h1_policy::OrtPolicy policy(argc > 1?argv[1]:"/home/llx/policy.onnx");
    h1_policy::Observation o{};
    o[3] = .04;
    o[5] = -1;
    auto a = policy.infer(o);
    const h1_policy::Joints expected = {.033, -.218, -.213, -1.021, .222, -.153, -.136, .187, -.307, -.805, .333, .126, .175, .220, -.004, -.055, -.893, -.039, -.289, -.094, -.559, .479, -.288, -1.187, .102, -.350, -.068};
    float max = 0;
    for(int i = 0; i < 27; i++) max = std::max(max, std::abs(a[i] - expected[i]));
    std::cout << "maximum absolute error " << max << '\n';
    return max <= .01?0:1;
  }catch(const std::exception&e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
