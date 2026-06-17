#ifndef H1_2_ICP_LOCALIZATION_CLOUD_ACCUMULATOR_HPP_
#define H1_2_ICP_LOCALIZATION_CLOUD_ACCUMULATOR_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <pcl/point_cloud.h>

namespace h1_2_icp_localization
{

template<typename PointT>
void keepNewestPoints(pcl::PointCloud<PointT> & cloud, std::size_t max_points)
{
  if (max_points == 0 || cloud.size() <= max_points) {
    return;
  }

  const auto erase_end =
    cloud.points.end() - static_cast<typename pcl::PointCloud<PointT>::VectorType::difference_type>(max_points);
  cloud.points.erase(cloud.points.begin(), erase_end);
  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1;
}

}  // namespace h1_2_icp_localization

#endif  // H1_2_ICP_LOCALIZATION_CLOUD_ACCUMULATOR_HPP_
