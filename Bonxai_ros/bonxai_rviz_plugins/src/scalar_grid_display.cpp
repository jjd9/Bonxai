/**
 *
 * Rviz plugin for visualizing Bonxai messages.
 *
 * This plugin takes a lot of inspiration from the Octomap rviz plugin
 * https://github.com/OctoMap/octomap_rviz_plugins/tree/ros2
 *
 * Author: John D'Angelo
 */

#include "bonxai_rviz_plugins/scalar_grid_display.hpp"

namespace bonxai_rviz_plugins
{

template <>
ScalarBonxaiGridDisplay<float>::ScalarBonxaiGridDisplay()
  : TemplatedBonxaiGridDisplay<float>()
{
  scalar_threshold_property_ =
      new rviz_common::properties::FloatProperty("Threshold",
                                                 0.5,
                                                 "Set scalar threshold. "
                                                 "Voxels with lower values are "
                                                 "ignored.",
                                                 this,
                                                 SLOT(updateScalarThreshold()));

}

template <>
void ScalarBonxaiGridDisplay<float>::updateScalarThreshold()
{
  MFDClass::updateTopic();
}

template <>
bool ScalarBonxaiGridDisplay<float>::checkType(const std::string& type_id)
{
  return (type_id == "float");
}

template <>
bool ScalarBonxaiGridDisplay<float>::shouldShowCell(const float& cell)
{
  return (cell > scalar_threshold_property_->getFloat());
}

template <>
void ScalarBonxaiGridDisplay<float>::setVoxelColor(
    rviz_rendering::PointCloud::Point& new_point,
    float& cell)
{
  BonxaiVoxelColorMode bonxai_color_mode =
      static_cast<BonxaiVoxelColorMode>(bonxai_coloring_property_->getOptionInt());
  float cell_value{}, z_scaled{};
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
      // TODO (jjd9): Add min/max parameter for scaling between 0 and 1.
      cell_value = std::max(std::min(1.0f, cell), 0.0f);
      // high probability -> green, low probability -> red
      new_point.setColor((1.0f - cell_value), cell_value, 0.0);
      break;
    default:
      break;
  }
}

using FloatBonxaiGridDisplay = ScalarBonxaiGridDisplay<float>;
}  // namespace bonxai_rviz_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bonxai_rviz_plugins::FloatBonxaiGridDisplay,
                       rviz_common::Display)
