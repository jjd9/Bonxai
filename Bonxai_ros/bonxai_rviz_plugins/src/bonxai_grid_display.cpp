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

#include <QObject>  // NOLINT

#include "rviz_common/visualization_manager.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/status_property.hpp"

#include "bonxai/bonxai.hpp"
#include "bonxai_msgs/msg/bonxai.hpp"
#include "bonxai_msgs/conversion.hpp"
#include "bonxai_rviz_plugins/bonxai_grid_display.hpp"

namespace bonxai_rviz_plugins
{
using rviz_common::properties::StatusProperty;

enum BonxaiVoxelColorMode
{
  BONXAI_CELL_COLOR,
  BONXAI_Z_AXIS_COLOR,
  BONXAI_PROBABILITY_COLOR,
};

BonxaiGridDisplay::BonxaiGridDisplay()
{
  RCLCPP_DEBUG(rclcpp::get_logger("rviz2"), "Constructing bonxai grid display!");

  bonxai_coloring_property_ =
      std::make_unique<rviz_common::properties::EnumProperty>("Voxel Coloring",
                                                "Z-Axis",
                                                "Select voxel coloring mode",
                                                this,
                                                SLOT(updateBonxaiColorMode()));
  bonxai_coloring_property_->addOption("Cell Color", BONXAI_CELL_COLOR);
  bonxai_coloring_property_->addOption("Z-Axis", BONXAI_Z_AXIS_COLOR);
  bonxai_coloring_property_->addOption("Cell Probability", BONXAI_PROBABILITY_COLOR);

  alpha_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
      "Voxel Alpha", 1.0, "Set voxel transparency alpha", this, SLOT(updateAlpha()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  scalar_threshold_property_ =
      std::make_unique<rviz_common::properties::FloatProperty>("Threshold",
                                                 0.5,
                                                 "Set scalar/probability threshold. "
                                                 "Voxels with lower values are "
                                                 "ignored.",
                                                 this,
                                                 SLOT(updateScalarThreshold()));

  // credit to: https://github.com/OctoMap/octomap_rviz_plugins/pull/20
  style_property_ = std::make_unique<rviz_common::properties::EnumProperty>("Style",
                                                              "Boxes",
                                                              "Rendering mode to "
                                                              "use, in order of "
                                                              "computational "
                                                              "complexity.",
                                                              this,
                                                              SLOT(updateStyle()));
  style_property_->addOption("Points", rviz_rendering::PointCloud::RM_POINTS);
  style_property_->addOption("Squares", rviz_rendering::PointCloud::RM_SQUARES);
  style_property_->addOption("Flat Squares",
                             rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  style_property_->addOption("Spheres", rviz_rendering::PointCloud::RM_SPHERES);
  style_property_->addOption("Boxes", rviz_rendering::PointCloud::RM_BOXES);

  check_occlusions_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
    "Check Occlusions",
    true,
    "If True, perform occlusion check. Useful for very dense grids, but if you know your "
    "grid is not dense, you can disable this to improve efficiency.",
    this,
    SLOT(updateCheckOcclusions()),
    this);

  RCLCPP_DEBUG(rclcpp::get_logger("rviz2"),
               "Done constructing bonxai grid display!");
}

void BonxaiGridDisplay::onInitialize()
{
  MFDClass::onInitialize();
  std::scoped_lock<std::mutex> lock(mutex_);
  cloud_ = std::make_shared<rviz_rendering::PointCloud>();
  std::string name{ "Bonxai PointCloud" };
  cloud_->setName(name);
  cloud_->setRenderMode(static_cast<rviz_rendering::PointCloud::RenderMode>(
      style_property_->getOptionInt()));
  scene_node_->attachObject(cloud_.get());
}

void BonxaiGridDisplay::unsubscribe()
{
  clear();  // locks mutex_
  MFDClass::unsubscribe();
}

void BonxaiGridDisplay::setColorFromZAxis(rviz_rendering::PointCloud::Point& point)
{
  float z_scaled = (max_z_ - point.position.z) / (max_z_ - min_z_);
  z_scaled = std::max(std::min(z_scaled, 1.0f), 0.0f);
  point.setColor((1.0f - z_scaled), z_scaled, 0.0);
}

void BonxaiGridDisplay::updateBonxaiColorMode()
{
  MFDClass::updateTopic();
  reprocessLastMessage();
}

void BonxaiGridDisplay::updateAlpha()
{
  MFDClass::updateTopic();
  reprocessLastMessage();
}

void BonxaiGridDisplay::updateScalarThreshold()
{
  MFDClass::updateTopic();

  {
    std::scoped_lock<std::mutex> lock(mutex_);
    const float prob_threshold =
        std::min(1.0f, std::max(0.0f, scalar_threshold_property_->getFloat()));
    log_odds_threshold_ = Bonxai::ProbabilisticMap::logods(prob_threshold);
  }
  reprocessLastMessage();
}

void BonxaiGridDisplay::updateStyle()
{
  MFDClass::updateTopic();
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    cloud_->setRenderMode(static_cast<rviz_rendering::PointCloud::RenderMode>(
        style_property_->getOptionInt()));
  }
  reprocessLastMessage();
}

void BonxaiGridDisplay::updateCheckOcclusions(){
  MFDClass::updateTopic();
  reprocessLastMessage();
}

void BonxaiGridDisplay::clear()
{
  std::scoped_lock<std::mutex> lock(mutex_);

  // reset rviz pointcloud boxes
  cloud_->clear();
}

void BonxaiGridDisplay::update([[maybe_unused]] float wall_dt,
                               [[maybe_unused]] float ros_dt)
{
  if (new_points_received_)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    cloud_->clearAndRemoveAllPoints();
    // Set the dimensions of the billboards used to render each point.
    // Width/height are only applicable to billboards and boxes, depth
    // is only applicable to boxes. Points are also treated differently.
    if (static_cast<rviz_rendering::PointCloud::RenderMode>(
            style_property_->getOptionInt()) ==
        rviz_rendering::PointCloud::RM_POINTS)
    {
      cloud_->setDimensions(1, 1, 1);
    }
    else
    {
      cloud_->setDimensions(box_size_, box_size_, box_size_);
    }

    cloud_->addPoints(new_points_.begin(), new_points_.end());
    new_points_.clear();
    cloud_->setAlpha(alpha_property_->getFloat());
    new_points_received_ = false;
  }
  updateFromTF();
}

void BonxaiGridDisplay::reset()
{
  MFDClass::reset();
  clear();
}

bool BonxaiGridDisplay::updateFromTF()
{
  // Check if frame exists to avoid spamming the RViz console output
  std::string error{};
  if (context_->getFrameManager()->transformHasProblems(header_.frame_id, error))
  {
    return false;
  }

  // get tf transform
  Ogre::Vector3 pos;
  Ogre::Quaternion orient;
  if (!context_->getFrameManager()->getTransform(header_, pos, orient))
  {
    return false;
  }

  scene_node_->setOrientation(orient);
  scene_node_->setPosition(pos);
  return true;
}

template <>
bool ScalarBonxaiGridDisplay<float>::checkType(const std::string& type_id)
{
  return (type_id == "float");
}

bool ColorBonxaiGridDisplay::checkType(const std::string& type_id)
{
  // == does not work b/c ros type generation adds some fancy endings to
  // the datatype
  return (type_id.find("std_msgs::msg::ColorRGBA") != std::string::npos);
}

bool ProbabilisticBonxaiGridDisplay::checkType(const std::string& type_id)
{
  return (type_id == "Bonxai::ProbabilisticMap::CellT");
}

template <>
bool ScalarBonxaiGridDisplay<float>::shouldShowCell(const float& cell)
{
  return (cell > scalar_threshold_property_->getFloat());
}

bool ProbabilisticBonxaiGridDisplay::shouldShowCell(const ProbabilisticCell& cell)
{
  return (cell.probability_log > log_odds_threshold_);
}

template <>
void ScalarBonxaiGridDisplay<float>::setVoxelColor(
    rviz_rendering::PointCloud::Point& new_point,
    float& cell)
{
  BonxaiVoxelColorMode bonxai_color_mode =
      static_cast<BonxaiVoxelColorMode>(bonxai_coloring_property_->getOptionInt());
  float cell_probability{};
  switch (bonxai_color_mode)
  {
    case BONXAI_CELL_COLOR:
      setStatusStd(StatusProperty::Error,
                   "Messages",
                   "Cannot extract color from Scalar grid");
      break;
    case BONXAI_Z_AXIS_COLOR:
      setColorFromZAxis(new_point);
      break;
    case BONXAI_PROBABILITY_COLOR:
      // Should we support values outside the 0 to 1 range for the scalar bonxai grid
      // display?
      cell_probability = std::max(std::min(1.0f, cell), 0.0f);
      // high probability -> green, low probability -> red
      new_point.setColor((1.0f - cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
}

void ProbabilisticBonxaiGridDisplay::setVoxelColor(
    rviz_rendering::PointCloud::Point& new_point,
    ProbabilisticCell& cell)
{
  BonxaiVoxelColorMode bonxai_color_mode =
      static_cast<BonxaiVoxelColorMode>(bonxai_coloring_property_->getOptionInt());
  float cell_probability{};
  switch (bonxai_color_mode)
  {
    case BONXAI_CELL_COLOR:
      setStatusStd(StatusProperty::Error,
                   "Messages",
                   "Cannot extract color from Scalar grid");
      break;
    case BONXAI_Z_AXIS_COLOR:
      setColorFromZAxis(new_point);
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

void ColorBonxaiGridDisplay::setVoxelColor(
    rviz_rendering::PointCloud::Point& new_point,
    std_msgs::msg::ColorRGBA& cell)
{
  BonxaiVoxelColorMode bonxai_color_mode =
      static_cast<BonxaiVoxelColorMode>(bonxai_coloring_property_->getOptionInt());

  switch (bonxai_color_mode)
  {
    case BONXAI_CELL_COLOR:
      new_point.setColor(cell.r, cell.g, cell.b);
      break;
    case BONXAI_Z_AXIS_COLOR:
      setColorFromZAxis(new_point);
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

template <typename CellT>
void TemplatedBonxaiGridDisplay<CellT>::processMessage(
    const bonxai_msgs::msg::Bonxai::ConstSharedPtr msg)
{
  if(!msg){
    return;
  }

  std::scoped_lock<std::mutex> lock(mutex_);

  RCLCPP_DEBUG(rclcpp::get_logger("rviz2"),
               "Received Serialized Bonxai message (size: %zu bytes)",
               msg->raw_data.size());

  header_ = msg->header;
  if (!updateFromTF())
  {
    std::stringstream ss;
    ss << "Failed to transform from frame [" << header_.frame_id << "] to frame ["
       << context_->getFrameManager()->getFixedFrame() << "]";
    setStatusStd(StatusProperty::Error, "Message", ss.str());
    return;
  }

  try
  {
    // creating bonxai grid
    std::string grid_type;  // this will be CellT as a string
    Bonxai::VoxelGrid<CellT> grid = bonxai_msgs::fromRosMsg<CellT>(*msg, grid_type);
    box_size_ = grid.resolution;

    setStatusStd(StatusProperty::Ok, "Type", grid_type.c_str());
    if (!checkType(grid_type))
    {
      setStatusStd(StatusProperty::Error,
                   "Message",
                   "Wrong cell data type found! Maybe you have the wrong display "
                   "type?");
      return;
    }

    // reset the z-bounds (needed for z-axis coloring)
    min_z_ = std::numeric_limits<double>::max();
    max_z_ = std::numeric_limits<double>::lowest();

    const bool check_occlusions = check_occlusions_property_->getBool();

    // do this for every cell in the grid
    auto accessor = grid.createAccessor();
    auto visitor_func = [&](CellT& cell, const Bonxai::CoordT& coord) {
      if (!shouldShowCell(cell))
      {
        return;
      }

      if(check_occlusions){
        // if the cell has at least one unoccupied neighbor, it should
        // be added, since it may be visible. otherwise, it can be ignored
        // without affecting the grid visualization.
        Bonxai::CoordT neighbor_coord;
        bool occluded = true;
        for (const auto& offset : CUBE_FACES)
        {
          neighbor_coord.x = coord.x + offset[0];
          neighbor_coord.y = coord.y + offset[1];
          neighbor_coord.z = coord.z + offset[2];
          if (accessor.value(neighbor_coord, false) == nullptr)
          {
            occluded = false;
            break;
          }
        }

        if (occluded)
        {
          return;
        }
      }

      const auto point = CoordToPos(coord, box_size_);
      rviz_rendering::PointCloud::Point new_point;
      new_point.position.x = point.x;
      new_point.position.y = point.y;
      new_point.position.z = point.z;

      setVoxelColor(new_point, cell);
      min_z_ = std::min(min_z_, point.z);
      max_z_ = std::max(max_z_, point.z);
      point_buf_.push_back(new_point);
    };

    grid.forEachCell(visitor_func);

    if (!point_buf_.empty())
    {
      new_points_received_ = true;

      new_points_.swap(point_buf_);

      last_msg_ = msg;
    }
    else
    {
      min_z_ = -1;
      max_z_ = 1;
    }
  }
  catch (const std::runtime_error& ex)
  {
    std::stringstream ss;
    ss << "Parsing the message failed: " << ex.what();
    setStatusStd(StatusProperty::Error, "Message", ss.str());
  }
}

using FloatBonxaiGridDisplay = ScalarBonxaiGridDisplay<float>;
}  // namespace bonxai_rviz_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(bonxai_rviz_plugins::FloatBonxaiGridDisplay,
                       rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(bonxai_rviz_plugins::ColorBonxaiGridDisplay,
                       rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(bonxai_rviz_plugins::ProbabilisticBonxaiGridDisplay,
                       rviz_common::Display)