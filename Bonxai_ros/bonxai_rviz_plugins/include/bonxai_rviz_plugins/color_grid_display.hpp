/**
 *
 * Rviz plugin for visualizing Bonxai messages.
 *
 * This plugin takes a lot of inspiration from the Octomap rviz plugin
 * https://github.com/OctoMap/octomap_rviz_plugins/tree/ros2
 *
 * Author: John D'Angelo
 */

#ifndef BONXAI_RVIZ_PLUGINS__COLOR_GRID_DISPLAY_HPP_
#define BONXAI_RVIZ_PLUGINS__COLOR_GRID_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include <std_msgs/msg/color_rgba.hpp>

#include "bonxai_rviz_plugins/bonxai_grid_display.hpp"

#endif

namespace bonxai_rviz_plugins
{

/**
 * @brief Specialization of BonxaiGrid template for color values (struct's with r,g,b
 * fields)
 */
class ColorBonxaiGridDisplay
  : public TemplatedBonxaiGridDisplay<std_msgs::msg::ColorRGBA>
{
protected:
  void setVoxelColor(rviz_rendering::PointCloud::Point& new_point,
                     std_msgs::msg::ColorRGBA& cell) override;

  bool shouldShowCell(const std_msgs::msg::ColorRGBA& cell) override;

  bool checkType(const std::string& type_id) override;
};

}  // namespace bonxai_rviz_plugins

#endif  // BONXAI_RVIZ_PLUGINS__COLOR_GRID_DISPLAY_HPP_