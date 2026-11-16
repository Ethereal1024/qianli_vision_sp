#ifndef AUTO_AIM__CLASSIFIER_HPP
#define AUTO_AIM__CLASSIFIER_HPP

#include <memory>
#include <string>

#include "backend/detector.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace auto_aim
{
class Classifier
{
public:
  explicit Classifier(const std::string & config_path);
  void classify(Armor & armor);
  void backend_classify(Armor & armor);

private:
  std::string config_path_;
  std::unique_ptr<backend::Detector> detector_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__CLASSIFIER_HPP
