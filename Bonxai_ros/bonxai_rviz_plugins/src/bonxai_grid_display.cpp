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

#include "bonxai/bonxai.hpp"
#include "bonxai_msgs/conversion.hpp"
#include "bonxai_rviz_plugins/bonxai_grid_display.hpp"

namespace bonxai_rviz_plugins
{

BonxaiGridDisplay::BonxaiGridDisplay()
{
  RCLCPP_DEBUG(rclcpp::get_logger("rviz2"), "Constructing bonxai grid display!");

  bonxai_coloring_property_ =
      new rviz_common::properties::EnumProperty("Voxel Coloring",
                                                "Z-Axis",
                                                "Select voxel coloring mode",
                                                this,
                                                SLOT(updateBonxaiColorMode()));
  bonxai_coloring_property_->addOption("Cell Color", BONXAI_CELL_COLOR);
  bonxai_coloring_property_->addOption("Z-Axis", BONXAI_Z_AXIS_COLOR);
  bonxai_coloring_property_->addOption("Cell Probability", BONXAI_PROBABILITY_COLOR);

  alpha_property_ = new rviz_common::properties::FloatProperty(
      "Voxel Alpha", 1.0, "Set voxel transparency alpha", this, SLOT(updateAlpha()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  // credit to: https://github.com/OctoMap/octomap_rviz_plugins/pull/20
  style_property_ = new rviz_common::properties::EnumProperty("Style",
                                                              "Boxes",
                                                              "Rendering mode to "
                                                              "use, in order of "
                                                              "computational "
                                                              "complexity. Note that the 'Points' "
                                                              "option is not scaled to voxel size.",
                                                              this,
                                                              SLOT(updateStyle()));
  style_property_->addOption("Points", rviz_rendering::PointCloud::RM_POINTS);
  style_property_->addOption("Squares", rviz_rendering::PointCloud::RM_SQUARES);
  style_property_->addOption("Flat Squares",
                             rviz_rendering::PointCloud::RM_FLAT_SQUARES);
  style_property_->addOption("Spheres", rviz_rendering::PointCloud::RM_SPHERES);
  style_property_->addOption("Boxes", rviz_rendering::PointCloud::RM_BOXES);

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

void BonxaiGridDisplay::updateBonxaiColorMode()
{
  MFDClass::updateTopic();
}

void BonxaiGridDisplay::updateAlpha()
{
  MFDClass::updateTopic();
}

void BonxaiGridDisplay::updateStyle()
{
  MFDClass::updateTopic();

  std::scoped_lock<std::mutex> lock(mutex_);
  cloud_->setRenderMode(static_cast<rviz_rendering::PointCloud::RenderMode>(
      style_property_->getOptionInt()));
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
      cloud_->setDimensions(voxel_size_, voxel_size_, voxel_size_);
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

template <typename CellT>
void TemplatedBonxaiGridDisplay<CellT>::processMessage(
    const bonxai_msgs::msg::Bonxai::ConstSharedPtr msg)
{
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
    voxel_size_ = grid.resolution;

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

    // do this for every cell in the grid
    auto accessor = grid.createAccessor();
    auto visitor_func = [&](CellT& cell, const Bonxai::CoordT& coord) {
      if (!shouldShowCell(cell))
      {
        return;
      }

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
        if (accessor.value(neighbor_coord) == nullptr)
        {
          occluded = false;
          break;
        }
      }

      if (occluded)
      {
        return;
      }

      const auto point = CoordToPos(coord, voxel_size_);
      rviz_rendering::PointCloud::Point new_point;
      new_point.position.x = point.x;
      new_point.position.y = point.y;
      new_point.position.z = point.z;

      RCLCPP_DEBUG(rclcpp::get_logger("rviz2"), "Processing cell");

      setVoxelColor(new_point, cell);
      min_z_ = std::min(min_z_, point.z);
      max_z_ = std::max(max_z_, point.z);
      point_buf_.push_back(new_point);
    };

    grid.forEachCell(visitor_func);

    if (!point_buf_.empty())
    {
      std::scoped_lock<std::mutex> lock(mutex_);

      new_points_received_ = true;

      new_points_.swap(point_buf_);
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

}  // namespace bonxai_rviz_plugins

#include "pluginlib/class_list_macros.hpp"
