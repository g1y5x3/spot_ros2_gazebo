#ifndef SPOT_GAZEBO__THERMAL_COLORMAP_HPP_
#define SPOT_GAZEBO__THERMAL_COLORMAP_HPP_

#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace spot_gazebo
{

inline cv::Mat thermalColormap(const cv::Mat & thermal)
{
  if (thermal.empty() || thermal.type() != CV_16UC1) {
    throw std::invalid_argument("thermal image must be non-empty mono16");
  }

  cv::Mat normalized;
  cv::normalize(thermal, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

  cv::Mat colorized;
  cv::applyColorMap(normalized, colorized, cv::COLORMAP_INFERNO);
  return colorized;
}

}  // namespace spot_gazebo

#endif  // SPOT_GAZEBO__THERMAL_COLORMAP_HPP_
