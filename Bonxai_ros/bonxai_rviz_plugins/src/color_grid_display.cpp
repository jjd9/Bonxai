/**
 *
 * Rviz plugin for visualizing Bonxai messages.
 *
 * This plugin takes a lot of inspiration from the Octomap rviz plugin
 * https://github.com/OctoMap/octomap_rviz_plugins/tree/ros2
 *
 * Author: John D'Angelo
 */

#include <string>

#include "bonxai_rviz_plugins/color_grid_display.hpp"

namespace bonxai_rviz_plugins
{

bool ColorBonxaiGridDisplay::checkType(const std::string& type_id)
{
  // == does not work b/c ros type generation adds some endings to
  // the datatype
  return (type_id.find("std_msgs::msg::ColorRGBA") != std::string::npos);
}

bool ColorBonxaiGridDisplay::shouldShowCell(const std_msgs::msg::ColorRGBA& cell)
{
  (void)cell;  // silence compiler warning

  return true;
};

void ColorBonxaiGridDisplay::setVoxelColor(
    rviz_rendering::PointCloud::Point& new_point,
    std_msgs::msg::ColorRGBA& cell)
{
  BonxaiVoxelColorMode bonxai_color_mode =
      static_cast<BonxaiVoxelColorMode>(bonxai_coloring_property_->getOptionInt());

  float z_scaled{};
  switch (bonxai_color_mode)
  {
    case BONXAI_CELL_COLOR:
      new_point.setColor(cell.r, cell.g, cell.b);
      break;
    case BONXAI_Z_AXIS_COLOR:
      z_scaled = (max_z_ - new_point.position.z) / (max_z_ - min_z_);
      new_point.setColor((1.0f - z_scaled), z_scaled, 0.0);
      break;
    case BONXAI_PROBABILITY_COLOR:
      setStatusStd(StatusProperty::Error,
                   "Messages",
                   "Cannot extract probability from Color grid.");
      break;
    default:
      break;
  }
}

}  // namespace bonxai_rviz_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bonxai_rviz_plugins::ColorBonxaiGridDisplay,
                       rviz_common::Display)
