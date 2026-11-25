#ifndef TOOLS__YAML_HPP
#define TOOLS__YAML_HPP

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace tools
{
inline YAML::Node load(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    logger()->error("[YAML] Failed to load file: {}", e.what());
    exit(1);
  } catch (const YAML::ParserException & e) {
    logger()->error("[YAML] Parser error: {}", e.what());
    exit(1);
  }
}

template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key)
{
  return yaml[key].as<T>();
}

template <typename T>
inline T read(const YAML::Node & yaml, const std::string & key, T default_value)
{
  if (!yaml || yaml.IsNull()) return default_value;
  const auto value = yaml[key];
  if (value && !value.IsNull()) return value.as<T>();
  return default_value;
}

}  // namespace tools

#endif  // TOOLS__YAML_HPP
