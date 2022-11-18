#include "mv_camera_driver/mv_camera_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mv_camera");
  ros::NodeHandle nh;

  MVCameraDriver *cam;
  cam = new MVCameraDriver(&nh);
  return cam->Start();
}