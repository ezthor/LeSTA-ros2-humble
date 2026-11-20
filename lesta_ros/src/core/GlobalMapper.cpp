/*
 * GlobalMapper.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 *  Modified for LeSTA ROS 2 Humble: record measured cells and handle empty scans.
 */

#include "lesta/core/GlobalMapper.h"

namespace lesta {

GlobalMapper::GlobalMapper(const Config &cfg) : HeightMapper(cfg) {

  const auto &map = getHeightMap();
  measured_indices_.reserve(map.getSize().prod());
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr GlobalMapper::heightMapping(
    const typename pcl::PointCloud<PointT>::Ptr &cloud) {

  auto cloud_rasterized = HeightMapper::heightMapping<PointT>(cloud);

  // Guard empty rasterization before dereferencing the output cloud.
  if (!cloud_rasterized || cloud_rasterized->empty()) return cloud_rasterized;

  // Save measured indices for efficiency
  recordMeasuredCells(getHeightMap(), *cloud_rasterized);

  return cloud_rasterized;
}

// Save measured indices for efficiency
template <typename PointT>
void GlobalMapper::recordMeasuredCells(const HeightMap &map,
                                       const pcl::PointCloud<PointT> &cloud) {

  grid_map::Index measured_cell_index;
  grid_map::Position measured_cell_position;

  for (const auto &point : cloud.points) {
    measured_cell_position.x() = point.x;
    measured_cell_position.y() = point.y;

    // Skip if the point is out of the map
    if (!map.getIndex(measured_cell_position, measured_cell_index))
      continue;

    if (map.isEmptyAt(measured_cell_index))
      continue;

    measured_indices_.insert(measured_cell_index);
  }
}

//////////////////////////////////////////////////
// Explicit instantiation of template functions //
//////////////////////////////////////////////////
// Laser
template typename pcl::PointCloud<Laser>::Ptr
GlobalMapper::heightMapping<Laser>(
    const typename pcl::PointCloud<Laser>::Ptr &cloud);
template void GlobalMapper::recordMeasuredCells(const HeightMap &map,
                                                const pcl::PointCloud<Laser> &cloud);

// Color
template typename pcl::PointCloud<Color>::Ptr
GlobalMapper::heightMapping<Color>(
    const typename pcl::PointCloud<Color>::Ptr &cloud);
template void GlobalMapper::recordMeasuredCells(const HeightMap &map,
                                                const pcl::PointCloud<Color> &cloud);

} // namespace lesta
