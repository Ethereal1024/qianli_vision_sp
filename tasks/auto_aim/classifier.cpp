#include "classifier.hpp"

#include "tools/yaml.hpp"

namespace auto_aim
{
Classifier::Classifier(const std::string & config_path) : config_path_(config_path) {}

void Classifier::classify(Armor & armor)
{
  armor.name = ArmorName::not_armor;
  armor.confidence = 0;
  if (armor.pattern.empty()) return;
  if (!detector_) {
    const auto yaml = tools::load(config_path_);
    detector_ = std::make_unique<backend::Detector>(yaml["pipeline"]);
  }
  const auto results = detector_->detect(armor.pattern, 0);
  const auto detections = std::get_if<std::vector<backend::types::YOLOResult>>(&results);
  if (!detections) return;
  for (const auto & result : *detections) {
    if (result.confidence_ > armor.confidence) {
      armor.confidence = result.confidence_;
      armor.name = armor_name_from_model(static_cast<int>(result.type_));
    }
  }
}

void Classifier::backend_classify(Armor & armor) { classify(armor); }
}  // namespace auto_aim
