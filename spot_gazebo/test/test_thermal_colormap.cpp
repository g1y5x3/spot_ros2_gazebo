#include <cstdint>
#include <stdexcept>

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "spot_gazebo/thermal_colormap.hpp"

TEST(ThermalColormap, MatchesOpenCvInfernoMapping)
{
  cv::Mat thermal(1, 3, CV_16UC1);
  thermal.at<uint16_t>(0, 0) = 0;
  thermal.at<uint16_t>(0, 1) = 32768;
  thermal.at<uint16_t>(0, 2) = 65535;

  cv::Mat normalized;
  cv::normalize(thermal, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::Mat expected;
  cv::applyColorMap(normalized, expected, cv::COLORMAP_INFERNO);

  const cv::Mat actual = spot_gazebo::thermalColormap(thermal);

  EXPECT_EQ(actual.type(), CV_8UC3);
  EXPECT_EQ(actual.size(), thermal.size());
  EXPECT_EQ(cv::norm(actual, expected, cv::NORM_INF), 0.0);
}

TEST(ThermalColormap, RejectsNonMonochrome16Input)
{
  const cv::Mat rgb = cv::Mat::zeros(2, 2, CV_8UC3);
  EXPECT_THROW(spot_gazebo::thermalColormap(rgb), std::invalid_argument);
}
