#pragma once

#include <string.h>

// 相机分辨率
enum MVResolutionIndex
{
  R1920_1200 = 0,
  R1600_1200 = 1,
  R1920_1080HD = 2,
  R960_600ROICenter = 7,
  CustomSize = 8,
};

// 图像采样区域选择参数，中心区域
struct ResolutionCustomSize
{
  int offset_cols = 360;
  int offset_rows = 0;
  int witdh = 1200;
  int height = 1200;
};

// 相机基本参数设置
struct MVCameraConfig
{
  bool debug = false;
  std::string ros_topic = "/mv_camera";
  bool auto_exposure = true;
  int exposure_time = 30; // ms
  int exposure_time_min = 5;
  int exposure_time_max = 500;
  int ae_target = 75;
  int max_frame_frate = 20;
  int set_wb_every_n_frame = 1200; // 每1200贞设置一次自动白平衡
  int sharpness = 2;               // 锐度
  int gama = 117;                  // not use
  int conrast = 100;               // not use
  MVResolutionIndex resolution_index = MVResolutionIndex::R1600_1200;
  ResolutionCustomSize resolution_size;
};