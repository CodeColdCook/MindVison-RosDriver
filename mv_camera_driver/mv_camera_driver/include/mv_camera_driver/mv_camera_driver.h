#include <CameraApi.h> //相机SDK的API头文件

#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "mv_camera_driver/utils_param.h"
#include "mv_camera_config.h"
#include "utils_timer.h"
/**
 * @brief 用于迈德威视相机的图像采样
 * 
 */
class MVCameraDriver
{
private:
  unsigned char *g_pRgbBuffer_; //处理后数据缓存区
  bool capturing_; // 是否正在采样
  ros::NodeHandle *nh_; 
  ros::Publisher ros_pub_; // 图像发布节点 sensor_msgs/Image.h
  MVCameraConfig mv_config_; // 相机参数

public:
/**
 * @brief Construct a new MVCameraDriver object
 *  初始化，读取参数
 * @param nh 
 */
  MVCameraDriver(ros::NodeHandle *nh)
      : capturing_(false),
        nh_(nh)
  {
    mv_camera::GPARAM(*nh_, "/mv_camera/debug", mv_config_.debug);
    mv_camera::GPARAM(*nh_, "/mv_camera/ros_topic", mv_config_.ros_topic);
    mv_camera::GPARAM(*nh_, "/mv_camera/auto_exposure", mv_config_.auto_exposure);
    mv_camera::GPARAM(*nh_, "/mv_camera/exposure_time", mv_config_.exposure_time);
    mv_camera::GPARAM(*nh_, "/mv_camera/exposure_time_min", mv_config_.exposure_time_min);
    mv_camera::GPARAM(*nh_, "/mv_camera/exposure_time_max", mv_config_.exposure_time_max);
    mv_camera::GPARAM(*nh_, "/mv_camera/ae_target", mv_config_.ae_target);
    mv_camera::GPARAM(*nh_, "/mv_camera/max_frame_frate", mv_config_.max_frame_frate);
    mv_camera::GPARAM(*nh_, "/mv_camera/set_wb_every_n_frame", mv_config_.set_wb_every_n_frame);
    mv_camera::GPARAM(*nh_, "/mv_camera/sharpness", mv_config_.sharpness);
    mv_camera::GPARAM(*nh_, "/mv_camera/gama", mv_config_.gama);
    mv_camera::GPARAM(*nh_, "/mv_camera/conrast", mv_config_.conrast);

    int r_index;
    mv_camera::GPARAM(*nh_, "/mv_camera/resolution_index", r_index);
    mv_config_.resolution_index = MVResolutionIndex(r_index);
    mv_camera::GPARAM(*nh_, "/mv_camera/resolution_size/offset_cols",
                      mv_config_.resolution_size.offset_cols);
    mv_camera::GPARAM(*nh_, "/mv_camera/resolution_size/offset_rows",
                      mv_config_.resolution_size.offset_rows);
    mv_camera::GPARAM(*nh_, "/mv_camera/resolution_size/witdh",
                      mv_config_.resolution_size.witdh);
    mv_camera::GPARAM(*nh_, "/mv_camera/resolution_size/height",
                      mv_config_.resolution_size.height);

    ros_pub_ = nh_->advertise<sensor_msgs::Image>(mv_config_.ros_topic, 1);
  }
  /**
   * @brief 初始化相机，并在设置参数后进入持续采样模式
   *  
   * @return true 相机正常运行
   * @return false 相机采样失败 
   */
  bool Start();

  /**
   * @brief 结束持续采样
   * 
   */
  inline void Stop() { capturing_ = false; }

  /**
   * @brief oepncv show
   * 
   * @param img 
   */
  void DefaultCallback(cv::Mat &img);

  private:
  /**
   * @brief 设置相机的基本参数，mv_config_
   * 
   * @param hCamera 相机句柄
   * @return true 
   * @return false 
   */
  bool MVSetCameraConfig(const CameraHandle &hCamera);

  /**
   * @brief 设置一次自动白平衡
   * 
   * @param hCamera 相机句柄
   */
  void MVSetCameraWB(const CameraHandle &hCamera);
};