/*
 * @Author: zjj
 * @Date: 2023-12-06 09:47:24
 * @LastEditors: zjj
 * @LastEditTime: 2023-12-13 14:27:31
 * @FilePath: /DataProcess/test/test.cpp
 * @Description:
 *
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */
#include <opencv2/opencv.hpp>

#include "dataprocess_api.h"

using namespace ParkingPerception::DataProcess;

static std::vector<std::vector<int>> _color = { { 0, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 0, 255, 255 } };

void render(cv::Mat& image, const cv::Mat& iclass, const std::string flag)
{
  auto pimage = image.ptr<cv::Vec3b>(0);
  // auto pprob = prob.ptr<float>(0);
  auto pclass = iclass.ptr<uint8_t>(0);

  // for (int i = 0; i < image.cols * image.rows; ++i, ++pimage, ++pprob, ++pclass)
  for (int i = 0; i < image.cols * image.rows; ++i, ++pimage, ++pclass)
  {
    int iclass = *pclass;
    // float probability = *pprob;
    auto& pixel = *pimage;
    float foreground;
    if (iclass == 0)
    {
      foreground = 0.0;
    }
    else
    {
      foreground = 0.8;
    }
    // float foreground = std::min(0.6f + probability * 0.2f, 0.5f);
    float background = 1 - foreground;
    for (int c = 0; c < 3; ++c)
    {
      float value;
      if (flag == "da")
      {
        value = pixel[c] * background + foreground * _color[iclass][c];
      }
      else if (flag == "ll")
      {
        value = pixel[c] * background + foreground * _color[iclass + 1][c];
      }
      pixel[c] = std::min((int)value, 255);
    }
  }
}

int main()
{
  // prepare input
  cv::Mat img_raw = cv::imread("/hostdata/projects/parking_perception/modules/DataProcess/test/test.jpg");
  cv::resize(img_raw,img_raw,cv::Size(800,800));

  //实例化ImageWarpaffine
  std::string config_path = "/hostdata/projects/parking_perception/modules/DataProcess/config/DataProcess.yaml";
  std::shared_ptr<ImageProcess> img_process = CreateImageProcess(config_path);

  //初始化
  if (0 != img_process->init())
  {
    std::cout << "init failed" << std::endl;
    return 0;
  }

  //图像仿射变换测试
  cv::Mat output;
  if (0 != img_process->imgWarpAffine(img_raw, output))
  {
    std::cout << "imgWarpAffine failed" << std::endl;
    return 0;
  }

  cv::imwrite("/hostdata/projects/parking_perception/modules/DataProcess/test/After_WarpAffine.jpg", output);
  std::cout << "Save After_WarpAffine.jpg." << std::endl;

  // ll_seg_filter测试
  cv::Mat ll_seg_mask = cv::imread("/hostdata/projects/parking_perception/modules/DataProcess/test/ll_mask.bmp",
                                   0);  // bmp文件不会损失精度，这里需要读入灰度图
  cv::Mat ll_seg_filter_mask;
  img_process->llseg_filter(ll_seg_mask, ll_seg_filter_mask);
  cv::Mat draw_ll = img_raw.clone();
  render(draw_ll, ll_seg_filter_mask, "ll");
  cv::imwrite("/hostdata/projects/parking_perception/modules/DataProcess/test/ll_seg_filter.jpg", draw_ll);
  std::cout << "Save ll_seg_filter.jpg." << std::endl;

  // da_seg_filter测试
  cv::Mat da_seg_mask = cv::imread("/hostdata/projects/parking_perception/modules/DataProcess/test/da_mask.bmp",
                                   0);  // bmp文件不会损失精度，这里需要读入灰度图
  cv::Mat da_seg_filter_mask;
  img_process->daseg_filter(da_seg_mask, da_seg_filter_mask);
  cv::Mat draw_da = img_raw.clone();
  render(draw_da, da_seg_filter_mask, "da");
  cv::imwrite("/hostdata/projects/parking_perception/modules/DataProcess/test/da_seg_filter.jpg", draw_da);
  std::cout << "Save da_seg_filter.jpg." << std::endl;

  return 0;
}
