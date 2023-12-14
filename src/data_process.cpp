/*
 * @Author: zjj
 * @Date: 2023-12-05 18:21:29
 * @LastEditors: zjj
 * @LastEditTime: 2023-12-14 16:42:17
 * @FilePath: /DataProcess/src/data_process.cpp
 * @Description:
 *
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */
#include "data_process.h"

namespace ParkingPerception
{
namespace DataProcess
{
ImageProcess::ImageProcess(std::string config_path) : config_path_(config_path)
{
}

int ImageProcess::init()
{
  if (0 != load_config())
  {
    std::cout << "[ImageProcess]->[init] Failed to load config file." << std::endl;
    return -1;
  }

  if (size_src_.h_ == size_dst_.h_ && size_src_.w_ == size_dst_.w_)
  {
    if_use_warpaffine = false;
    std::cout << "[ImageProcess]->[init] Init success." << std::endl;
    return 0;
  }

  initwarpaffine();

  std::cout << "[ImageProcess]->[init] Init success." << std::endl;
  return 0;
}

int ImageProcess::load_config()
{
  //导入yaml文件
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path_);
  }
  catch (const std::exception& e)
  {
    std::cout << "[ImageProcess]->[load_config] No config file: " << config_path_ << std::endl;
    return -1;
  }

  //导入配置参数
  size_src_.h_ = config["src_img_size"]["h"].as<int>();
  size_src_.w_ = config["src_img_size"]["w"].as<int>();
  size_dst_.h_ = config["dst_img_size"]["h"].as<int>();
  size_dst_.w_ = config["dst_img_size"]["w"].as<int>();

  if (size_src_.h_ < 1 || size_src_.h_ > 4096 || size_src_.w_ < 1 || size_src_.w_ > 4096)
  {
    std::cout << "[ImageProcess]->[load_config] Input src size error !!!" << std::endl;
    return -1;
  }

  if (size_dst_.h_ < 1 || size_dst_.h_ > 4096 || size_dst_.w_ < 1 || size_dst_.w_ > 4096)
  {
    std::cout << "[ImageProcess]->[load_config] Input dst size error !!!" << std::endl;
    return -1;
  }

  kernel_size_ll_ = config["ll_filter"]["kernel_size"].as<int>();
  if (kernel_size_ll_ < 1)
  {
    std::cout << "[ImageProcess]->[load_config] Input ll_filter kernel_size error !!!" << std::endl;
    return -1;
  }

  kernel_size_da_ = config["da_filter"]["kernel_size"].as<int>();
  if (kernel_size_da_ < 1)
  {
    std::cout << "[ImageProcess]->[load_config] Input da_filter kernel_size error !!!" << std::endl;
    return -1;
  }

  DP_epsilon_ = config["da_filter"]["DP_epsilon"].as<double>();
  if (DP_epsilon_ < 0)
  {
    std::cout << "[ImageProcess]->[load_config] Input da_filter DP_epsilon error !!!" << std::endl;
    return -1;
  }

  return 0;
}

int ImageProcess::initwarpaffine()
{
  // 计算图像缩放
  float scale_x = size_dst_.w_ / float(size_src_.w_);
  float scale_y = size_dst_.h_ / float(size_src_.h_);
  float scale = std::min(scale_x, scale_y);

  if (scale < 0)
  {
    std::cout << "[ImageProcess]->[initwarpaffine] Scale must > 0 !!!" << std::endl;
    return -1;
  }
  //存放原图到目标图的变换矩阵，源图像和目标图像几何中心的对齐
  float i2d[6];
  i2d[0] = scale;
  i2d[1] = 0;
  i2d[2] = (-scale * size_src_.w_ + size_dst_.w_ + scale - 1) * 0.5;
  i2d[3] = 0;
  i2d[4] = scale;
  i2d[5] = (-scale * size_src_.h_ + size_dst_.h_ + scale - 1) * 0.5;

  //转为3×3矩阵，方便求逆
  cv::Mat m3x3_i2d = cv::Mat::eye(3, 3, CV_32F);
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      m3x3_i2d.at<float>(i, j) = i2d[i * 3 + j];
    }
  }

  //求逆
  cv::Mat m3x3_d2i = m3x3_i2d.inv();

  //保存2*3矩阵
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      m2x3_i2d.at<float>(i, j) = m3x3_i2d.at<float>(i, j);
      // std::cout << i << "," << j << ":" << m2x3_i2d.at<float>(i, j) << std::endl;
    }
  }
  //保存2*3矩阵
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      m2x3_d2i.at<float>(i, j) = m3x3_d2i.at<float>(i, j);
      // std::cout << i << "," << j << ":" << m2x3_d2i.at<float>(i, j) << std::endl;
    }
  }

  return 0;
}

int ImageProcess::imgWarpAffine(const cv::Mat& src_img, cv::Mat& dst_img)
{
  if (src_img.empty())
  {
    std::cout << "[ImageProcess]->[imgWarpAffine] Input src_img is empty !!!" << std::endl;
    return -1;
  }
  if (src_img.rows != size_src_.h_ || src_img.cols != size_src_.w_)
  {
    std::cout << "[ImageProcess]->[imgWarpAffine] Input src_img is not equal with config !!!" << std::endl;
    return -1;
  }

  if (!if_use_warpaffine)
  {
    dst_img = src_img.clone();
    return 0;
  }
  cv::warpAffine(src_img, dst_img, m2x3_i2d, cv::Size(size_dst_.w_, size_dst_.h_), cv::INTER_LINEAR,
                 cv::BORDER_CONSTANT, cv::Scalar::all(114));
  return 0;
}

int ImageProcess::imgWarpAffineInv(const cv::Mat& src_img, cv::Mat& dst_img)
{
  if (src_img.empty())
  {
    std::cout << "[ImageProcess]->[imgWarpAffineInv] Input src_img is empty !!!" << std::endl;
    return -1;
  }
  if (src_img.rows != size_dst_.h_ || src_img.cols != size_dst_.w_)
  {
    std::cout << "[ImageProcess]->[imgWarpAffineInv] Input src_img is not equal with config !!!" << std::endl;
    return -1;
  }

  if (!if_use_warpaffine)
  {
    dst_img = src_img.clone();
    return 0;
  }
  cv::warpAffine(src_img, dst_img, m2x3_d2i, cv::Size(size_src_.w_, size_src_.h_), cv::INTER_NEAREST,
                 cv::BORDER_CONSTANT, cv::Scalar::all(114));
  return 0;
}

int ImageProcess::llseg_filter(const cv::Mat& src_img, cv::Mat& dst_img)
{
  if (src_img.empty())
  {
    std::cout << "[ImageProcess]->[llseg_filter] Input src_img is empty !!!" << std::endl;
    return -1;
  }
  if (src_img.type() != CV_8UC1)
  {
    std::cout << "[ImageProcess]->[llseg_filter] Input src_img must be CV_8UC1 !!!" << std::endl;
    return -1;
  }

  //腐蚀膨胀去除噪点
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_ll_, kernel_size_ll_));
  cv::morphologyEx(src_img, dst_img, cv::MORPH_OPEN, element);
  // dst_img=src_img.clone();

  //逆仿射变换恢复原图
  if (0 != imgWarpAffineInv(dst_img, dst_img))
  {
    return -1;
  }

  return 0;
}

int ImageProcess::daseg_filter(const cv::Mat& src_img, cv::Mat& dst_img)
{
  if (src_img.empty())
  {
    std::cout << "[ImageProcess]->[daseg_filter] Input src_img is empty !!!" << std::endl;
    return -1;
  }

  if (src_img.type() != CV_8UC1)
  {
    std::cout << "[ImageProcess]->[daseg_filter] Input src_img must be CV_8UC1 !!!" << std::endl;
    return -1;
  }

  //腐蚀膨胀去除噪点
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_da_, kernel_size_da_));
  cv::morphologyEx(src_img, dst_img, cv::MORPH_OPEN, element);

  //获得可行驶区域连通域
  std::vector<std::vector<cv::Point>> contours;  //连通域点集
  std::vector<cv::Vec4i> hierarchy;  //分别表示第i个轮廓的后一个轮廓、前一个轮廓、第一条子轮廓、父轮廓的索引编号
  // cv::RETR_EXTERNAL：只检测最外围轮廓；
  // cv::RETR_LIST：检测所有的轮廓，但是不建立等级关系；
  // cv::RETR_CCOMP：检测所有的轮廓，但所有轮廓只建立两种等级关系，外围为顶层
  // cv::RETR_TREE：检测所有的轮廓，所有轮廓建立一个等级树结构
  cv::findContours(dst_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  //优化连通域
  std::vector<std::vector<cv::Point>> filter_contours;  //优化后的连通域点集
  for (auto& contour : contours)
  {
    //剔除面积小的凸包
    if (cv::contourArea(contour) < size_dst_.h_ * size_dst_.w_ * 0.01)
    {
      continue;
    }
    //拟合凸包并减少凸包点数
    std::vector<cv::Point> filter_contour;
    cv::approxPolyDP(contour, filter_contour, DP_epsilon_,
                     true);  //第三个参数为拟合精度，原始曲线与近似曲线之间的最大距离。
    filter_contours.push_back(filter_contour);
  }

  //重新生成mask图
  dst_img = cv::Scalar::all(0);
  cv::drawContours(dst_img, filter_contours, -1, cv::Scalar::all(1), -1);  //绘制da连通域

  //逆仿射变换恢复原图
  if (0 != imgWarpAffineInv(dst_img, dst_img))
  {
    return -1;
  }

  return 0;
}

int ImageProcess::calculate_parking_slot(const cv::Point2f& pt0, const cv::Point2f& pt1, const float depth,
                                         const float theta, std::vector<cv::Point2f>& outpts)
{
  float x0 = pt0.x;
  float y0 = pt0.y;
  float x1 = pt1.x;
  float y1 = pt1.y;
  if (if_use_warpaffine)
  {
    //关键点x,y
    x0 = x0 * m2x3_d2i.at<float>(0, 0) + m2x3_d2i.at<float>(0, 2);
    y0 = y0 * m2x3_d2i.at<float>(0, 0) + m2x3_d2i.at<float>(1, 2);
    x1 = x1 * m2x3_d2i.at<float>(0, 0) + m2x3_d2i.at<float>(0, 2);
    y1 = y1 * m2x3_d2i.at<float>(0, 0) + m2x3_d2i.at<float>(1, 2);
  }
  float delta_x = depth * std::cos(theta);
  float delta_y = depth * std::sin(theta);
  cv::Point2f pt3 = { x0 - delta_x, y0 - delta_y };
  cv::Point2f pt2 = { x1 - delta_x, y1 - delta_y };

  //依次存入车位0，1，2，3点及中心点
  outpts.clear();
  outpts.push_back(pt0);
  outpts.push_back(pt1);
  outpts.push_back(pt2);
  outpts.push_back(pt3);
}

}  // namespace DataProcess
}  // namespace ParkingPerception
