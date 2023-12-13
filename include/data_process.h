/*
 * @Author: zjj
 * @Date: 2023-12-05 18:21:17
 * @LastEditors: zjj
 * @LastEditTime: 2023-12-13 13:31:33
 * @FilePath: /DataProcess/include/data_process.h
 * @Description:
 *
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */
#pragma once

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace ParkingPerception
{
namespace DataProcess
{
struct Size
{
  int w_;
  int h_;
  Size() = default;
  Size(int w, int h) : w_(w), h_(h){};
  Size operator=(const Size& other)
  {
    this->w_ = other.w_;
    this->h_ = other.h_;
    return *this;
  }
};

class ImageProcess
{
public:
  ImageProcess(std::string config_path);
  ~ImageProcess(){};
  int init();
  int imgWarpAffine(const cv::Mat& src_img, cv::Mat& dst_img);
  int imgWarpAffineInv(const cv::Mat& src_img, cv::Mat& dst_img);
  int llseg_filter(const cv::Mat& src_img, cv::Mat& dst_img);
  int daseg_filter(const cv::Mat& src_img, cv::Mat& dst_img);

private:
  int load_config();
  int initwarpaffine();

private:
  std::string config_path_;
  Size size_src_;
  Size size_dst_;
  cv::Mat m2x3_i2d = cv::Mat::zeros(2, 3, CV_32F);  // src to dst
  cv::Mat m2x3_d2i = cv::Mat::zeros(2, 3, CV_32F);  // dst to src
  bool if_use_warpaffine = true;
  int kernel_size_ll_;
  int kernel_size_da_;
  double DP_epsilon_;
};
}  // namespace DataProcess
}  // namespace ParkingPerception