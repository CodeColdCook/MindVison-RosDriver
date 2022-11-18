#pragma once

#include <sys/stat.h>
#include <fstream>
#include <string>

#include <ros/ros.h>

namespace mv_camera
{

template <class T>
void GPARAM(const ros::NodeHandle &n, std::string param_path, T &param)
{
  if (!n.getParam(param_path, param))
    ROS_ERROR_STREAM("Load param from " << param_path << " failed...");
}

} // namespace odape
