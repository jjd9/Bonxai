/**
 *
 * Rviz plugin for visualizing Bonxai messages.
 *
 * This plugin takes a lot of inspiration from the Octomap rviz plugin
 * https://github.com/OctoMap/octomap_rviz_plugins/tree/ros2
 *
 * Author: John D'Angelo
 */

#ifndef BONXAI_RVIZ_PLUGINS__SCALAR_GRID_DISPLAY_HPP_
#define BONXAI_RVIZ_PLUGINS__SCALAR_GRID_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include "bonxai_rviz_plugins/bonxai_grid_display.hpp"

#endif

namespace bonxai_rviz_plugins
{

/**
 * @brief Specialization of BonxaiGrid template for scalar values (float, int,
 * etc...)
 */
template <typename CellT>
class ScalarBonxaiGridDisplay : public TemplatedBonxaiGridDisplay<CellT>
{
public:
  ScalarBonxaiGridDisplay();

private Q_SLOTS:
  void updateScalarThreshold();

protected:
  void setVoxelColor(rviz_rendering::PointCloud::Point& new_point,
                     CellT& cell) override;

  bool shouldShowCell(const CellT& cell) override;

  bool checkType(const std::string& type_id) override;

  rviz_common::properties::FloatProperty *scalar_threshold_property_;
};

}  // namespace bonxai_rviz_plugins

#endif  // BONXAI_RVIZ_PLUGINS__SCALAR_GRID_DISPLAY_HPP_