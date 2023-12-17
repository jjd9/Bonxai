/**
 *
 * Rviz plugin for visualizing Bonxai messages.
 *
 * This plugin takes a lot of inspiration from the Octomap rviz plugin
 * https://github.com/OctoMap/octomap_rviz_plugins/tree/ros2
 *
 * Author: John D'Angelo
 */

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <sstream>

#include "bonxai_rviz_plugins/probabilistic_grid_display.hpp"

namespace bonxai_rviz_plugins
{

ProbabilisticBonxaiGridDisplay::ProbabilisticBonxaiGridDisplay()
  : TemplatedBonxaiGridDisplay<ProbabilisticCell>()
{
  probability_threshold_property_ =
      new rviz_common::properties::FloatProperty("Threshold",
                                                 0.5,
                                                 "Set probability (not log-odds) threshold. "
                                                 "Voxels with lower probabilities are "
                                                 "ignored.",
                                                 this,
                                                 SLOT(updateProbabilityThreshold()));
  probability_threshold_property_->setMin(0.0);
  probability_threshold_property_->setMax(1.0);

}

void ProbabilisticBonxaiGridDisplay::updateProbabilityThreshold()
{
  MFDClass::updateTopic();

  std::scoped_lock<std::mutex> lock(mutex_);
  const float prob_threshold =
      std::min(1.0f, std::max(0.0f, probability_threshold_property_->getFloat()));
  log_odds_threshold_ = Bonxai::ProbabilisticMap::logods(prob_threshold);
}

bool ProbabilisticBonxaiGridDisplay::checkType(const std::string& type_id)
{
  return (type_id == "Bonxai::ProbabilisticMap::CellT");
}

bool ProbabilisticBonxaiGridDisplay::shouldShowCell(const ProbabilisticCell& cell)
{
  return (cell.probability_log > log_odds_threshold_);
}

void ProbabilisticBonxaiGridDisplay::setVoxelColor(
    rviz_rendering::PointCloud::Point& new_point,
    ProbabilisticCell& cell)
{
  BonxaiVoxelColorMode bonxai_color_mode =
      static_cast<BonxaiVoxelColorMode>(bonxai_coloring_property_->getOptionInt());
  float cell_probability{}, z_scaled{};
  switch (bonxai_color_mode)
  {
    case BONXAI_CELL_COLOR:
      setStatusStd(StatusProperty::Error,
                   "Messages",
                   "Cannot extract color from Scalar grid");
      break;
    case BONXAI_Z_AXIS_COLOR:
      z_scaled = (max_z_ - new_point.position.z) / (max_z_ - min_z_);
      new_point.setColor((1.0f - z_scaled), z_scaled, 0.0);
      break;
    case BONXAI_PROBABILITY_COLOR:
      cell_probability = Bonxai::ProbabilisticMap::prob(cell.probability_log);
      cell_probability = std::max(0.0f, std::min(cell_probability, 1.0f));
      new_point.setColor((1.0f - cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
}

}  // namespace bonxai_rviz_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bonxai_rviz_plugins::ProbabilisticBonxaiGridDisplay,
                       rviz_common::Display)