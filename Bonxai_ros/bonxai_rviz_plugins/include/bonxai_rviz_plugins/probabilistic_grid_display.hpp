/**
 *
 * Rviz plugin for visualizing Bonxai messages.
 *
 * This plugin takes a lot of inspiration from the Octomap rviz plugin
 * https://github.com/OctoMap/octomap_rviz_plugins/tree/ros2
 *
 * Author: John D'Angelo
 */

#ifndef BONXAI_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_
#define BONXAI_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include "bonxai_map/probabilistic_map.hpp"
#include "bonxai_rviz_plugins/bonxai_grid_display.hpp"

#endif

namespace bonxai_rviz_plugins
{

using ProbabilisticCell = Bonxai::ProbabilisticMap::CellT;

/**
 * @brief Specialization of BonxaiGrid template for ProbabilisticMap
 */
class ProbabilisticBonxaiGridDisplay
  : public TemplatedBonxaiGridDisplay<ProbabilisticCell>
{
public:
  ProbabilisticBonxaiGridDisplay();

private Q_SLOTS:
  void updateProbabilityThreshold();

protected:
  void setVoxelColor(rviz_rendering::PointCloud::Point& new_point,
                     ProbabilisticCell& cell) override;

  bool shouldShowCell(const ProbabilisticCell& cell) override;

  bool checkType(const std::string& type_id) override;

private:
  // for probability grids
  int32_t log_odds_threshold_{ 0 };

  rviz_common::properties::FloatProperty *probability_threshold_property_;
};

}  // namespace bonxai_rviz_plugins

#endif  // BONXAI_RVIZ_PLUGINS__OCCUPANCY_GRID_DISPLAY_HPP_