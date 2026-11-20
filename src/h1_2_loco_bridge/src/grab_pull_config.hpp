#ifndef H1_2_LOCO_BRIDGE_GRAB_PULL_CONFIG_HPP_
#define H1_2_LOCO_BRIDGE_GRAB_PULL_CONFIG_HPP_

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "arm_trajectory.hpp"

namespace h1_2_grab_pull {

using namespace h1_2_arm_trajectory;

// 简单的YAML解析器（仅支持本配置文件格式）
class SimpleYamlParser {
public:
  explicit SimpleYamlParser(const std::string& content) {
    parse(content);
  }

  bool has(const std::string& key) const {
    return values_.find(key) != values_.end();
  }

  std::string get_string(const std::string& key, const std::string& default_value = "") const {
    auto it = values_.find(key);
    return (it != values_.end()) ? it->second : default_value;
  }

  double get_double(const std::string& key, double default_value = 0.0) const {
    auto it = values_.find(key);
    if (it == values_.end()) return default_value;
    try {
      return std::stod(it->second);
    } catch (...) {
      return default_value;
    }
  }

  std::vector<double> get_double_array(const std::string& key) const {
    auto it = values_.find(key);
    if (it == values_.end()) return {};
    return parse_double_array(it->second);
  }

private:
  void parse(const std::string& content) {
    std::istringstream stream(content);
    std::string line;
    std::vector<std::string> section_stack;  // 支持多层嵌套

    while (std::getline(stream, line)) {
      // 移除注释
      size_t comment_pos = line.find('#');
      if (comment_pos != std::string::npos) {
        line = line.substr(0, comment_pos);
      }

      // 保存原始行用于计算缩进
      std::string original_line = line;

      // 去除前后空格
      line = trim(line);
      if (line.empty()) continue;

      // 计算缩进级别（每2个空格为一级）
      size_t indent = 0;
      for (size_t i = 0; i < original_line.size() && original_line[i] == ' '; ++i) {
        indent++;
      }
      size_t indent_level = indent / 2;

      // 检查是否是section（以冒号结尾）
      if (line.back() == ':') {
        std::string section_name = line.substr(0, line.size() - 1);

        // 调整section栈到当前缩进级别
        while (section_stack.size() > indent_level) {
          section_stack.pop_back();
        }

        // 添加新section
        section_stack.push_back(section_name);
        continue;
      }

      // 解析键值对
      size_t colon_pos = line.find(':');
      if (colon_pos != std::string::npos) {
        std::string key = trim(line.substr(0, colon_pos));
        std::string value = trim(line.substr(colon_pos + 1));

        // 去除引号
        if (!value.empty() && value.front() == '"' && value.back() == '"') {
          value = value.substr(1, value.size() - 2);
        }

        // 构建完整路径
        std::string full_key = key;
        if (!section_stack.empty()) {
          full_key = "";
          for (size_t i = 0; i < section_stack.size(); ++i) {
            if (i > 0) full_key += ".";
            full_key += section_stack[i];
          }
          full_key += "." + key;
        }

        values_[full_key] = value;
      }
    }
  }

  std::string trim(const std::string& str) const {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
  }

  std::vector<double> parse_double_array(const std::string& str) const {
    std::vector<double> result;
    std::string cleaned = str;

    // 移除方括号
    if (!cleaned.empty() && cleaned.front() == '[') cleaned = cleaned.substr(1);
    if (!cleaned.empty() && cleaned.back() == ']') cleaned.pop_back();

    std::istringstream stream(cleaned);
    std::string token;
    while (std::getline(stream, token, ',')) {
      token = trim(token);
      if (!token.empty()) {
        try {
          result.push_back(std::stod(token));
        } catch (...) {
          // 忽略无效值
        }
      }
    }
    return result;
  }

  std::map<std::string, std::string> values_;
};

// 配置加载器
struct GrabPullConfig {
  TrajectoryConfig trajectory_config;
  std::map<std::string, ArmPose> poses;
  std::vector<MotionSegment> motion_sequence;

  // 加载配置文件
  static bool load_from_file(const std::string& file_path, GrabPullConfig& config, std::string& error) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      error = "Failed to open config file: " + file_path;
      return false;
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    SimpleYamlParser parser(content);

    // 加载时间参数
    config.trajectory_config.control_period = parser.get_double("timing.control_period", 0.02);
    config.trajectory_config.takeover_duration = parser.get_double("timing.takeover_duration", 1.0);
    config.trajectory_config.handback_duration = parser.get_double("timing.handback_duration", 2.0);
    config.trajectory_config.abort_handback_duration = parser.get_double("timing.abort_handback_duration", 1.0);

    // 加载速度限制
    config.trajectory_config.max_joint_velocity = parser.get_double("limits.max_joint_velocity", 0.5);

    // 加载增益
    auto right_kp = parser.get_double_array("right_arm_gains.kp");
    auto right_kd = parser.get_double_array("right_arm_gains.kd");
    auto right_tau = parser.get_double_array("right_arm_gains.tau_ff");

    if (right_kp.size() == kArmJoints && right_kd.size() == kArmJoints) {
      for (int i = 0; i < kArmJoints; ++i) {
        config.trajectory_config.right_arm_gains[i].kp = right_kp[i];
        config.trajectory_config.right_arm_gains[i].kd = right_kd[i];
        config.trajectory_config.right_arm_gains[i].tau_ff = (right_tau.size() == kArmJoints) ? right_tau[i] : 0.0;
      }
    }

    auto left_kp = parser.get_double_array("left_arm_gains.kp");
    auto left_kd = parser.get_double_array("left_arm_gains.kd");
    auto left_tau = parser.get_double_array("left_arm_gains.tau_ff");

    if (left_kp.size() == kArmJoints && left_kd.size() == kArmJoints) {
      for (int i = 0; i < kArmJoints; ++i) {
        config.trajectory_config.left_arm_gains[i].kp = left_kp[i];
        config.trajectory_config.left_arm_gains[i].kd = left_kd[i];
        config.trajectory_config.left_arm_gains[i].tau_ff = (left_tau.size() == kArmJoints) ? left_tau[i] : 0.0;
      }
    }

    config.trajectory_config.waist_gains.kp = parser.get_double("waist_gains.kp", 200.0);
    config.trajectory_config.waist_gains.kd = parser.get_double("waist_gains.kd", 2.0);
    config.trajectory_config.waist_gains.tau_ff = parser.get_double("waist_gains.tau_ff", 0.0);

    // 加载姿势
    for (const std::string& pose_name : {"chest", "ready", "pull_back"}) {
      std::string key = "poses." + pose_name + ".joints";
      auto joints = parser.get_double_array(key);
      if (joints.size() == kArmJoints) {
        ArmPose pose;
        std::copy(joints.begin(), joints.end(), pose.begin());
        config.poses[pose_name] = pose;
      }
    }

    // 验证必需的姿势
    if (config.poses.find("chest") == config.poses.end() ||
        config.poses.find("ready") == config.poses.end() ||
        config.poses.find("pull_back") == config.poses.end()) {
      error = "Missing required poses (chest, ready, pull_back)";
      return false;
    }

    // 加载动作序列（这里需要手动解析，因为是数组）
    // 简化实现：硬编码默认序列
    config.motion_sequence = create_default_sequence(config.poses);

    return true;
  }

  static std::vector<MotionSegment> create_default_sequence(const std::map<std::string, ArmPose>& poses) {
    std::vector<MotionSegment> sequence;

    MotionSegment seg1;
    seg1.target_pose = poses.at("chest");
    seg1.nominal_duration = 1.0;
    seg1.hand_shape = HandShape::Open;
    sequence.push_back(seg1);

    MotionSegment seg2;
    seg2.target_pose = poses.at("ready");
    seg2.nominal_duration = 1.0;
    seg2.hand_shape = HandShape::Open;
    sequence.push_back(seg2);

    MotionSegment seg3;
    seg3.target_pose = poses.at("ready");
    seg3.nominal_duration = 0.3;
    seg3.hand_shape = HandShape::Open;
    sequence.push_back(seg3);

    MotionSegment seg4;
    seg4.target_pose = poses.at("ready");
    seg4.nominal_duration = 0.4;
    seg4.hand_shape = HandShape::Fist;
    sequence.push_back(seg4);

    MotionSegment seg5;
    seg5.target_pose = poses.at("pull_back");
    seg5.nominal_duration = 0.8;
    seg5.hand_shape = HandShape::Fist;
    sequence.push_back(seg5);

    MotionSegment seg6;
    seg6.target_pose = poses.at("pull_back");
    seg6.nominal_duration = 0.2;
    seg6.hand_shape = HandShape::Open;
    sequence.push_back(seg6);

    // 收回到起始姿势（这里需要在运行时获取）
    // seg7 将在播放时动态设置为initial pose

    return sequence;
  }
};

}  // namespace h1_2_grab_pull

#endif  // H1_2_LOCO_BRIDGE_GRAB_PULL_CONFIG_HPP_
