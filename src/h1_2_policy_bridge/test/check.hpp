#pragma once
#include <iostream>
#include <stdexcept>
#include <string>

inline void check_condition(bool condition, const char *expression, const char *file, int line)
{
  if (!condition) {
    throw std::runtime_error(std::string(file) + ":" + std::to_string(line) +
                             ": check failed: " + expression);
  }
}
#define CHECK(expression) check_condition(static_cast<bool>(expression), #expression, __FILE__, __LINE__)
