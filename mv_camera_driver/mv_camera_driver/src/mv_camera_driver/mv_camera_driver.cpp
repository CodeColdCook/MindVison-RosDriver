#include "mv_camera_driver/mv_camera_driver.h"

using namespace cv;

bool MVCameraDriver::Start()
{
  int iCameraCounts = 1;
  int iStatus = -1;
  tSdkCameraDevInfo tCameraEnumList;
  int hCamera;
  tSdkCameraCapbility tCapability; //设备描述信息
  tSdkFrameHead sFrameInfo;
  BYTE *pbyBuffer;
  // IplImage *iplImage = NULL;
  // int channel = 3;

  CameraSdkInit(1);
  //枚举设备，并建立设备列表
  iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
  printf("state = %d\n", iStatus);

  printf("count = %d\n", iCameraCounts);
  //没有连接设备
  if (iCameraCounts == 0)
    return -1;

  //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
  iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

  //初始化失败
  printf("state = %d\n", iStatus);
  if (iStatus != CAMERA_STATUS_SUCCESS)
  {
    return -1;
  }

  //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
  CameraGetCapability(hCamera, &tCapability);

  //
  g_pRgbBuffer_ = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
  // g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

  /*让SDK进入工作模式，开始接收来自相机发送的图像
  数据。如果当前相机是触发模式，则需要接收到
  触发帧以后才会更新图像。    */
  CameraPlay(hCamera);
  capturing_ = true;

  if (!MVSetCameraConfig(hCamera))
    return false;

  if(mv_config_.max_frame_frate < 1)
    mv_config_.max_frame_frate = 1;
  int64_t min_frame_dur = 1000000 / mv_config_.max_frame_frate; // us
  Timer timer;
  int counter = 0;
  //循环显示1000帧图像
  while (ros::ok() && capturing_)
  {
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
    {
      CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer_, &sFrameInfo);
      cv::Mat matImage(
          cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
          sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
          g_pRgbBuffer_);
      CameraReleaseImageBuffer(hCamera, pbyBuffer);

      DefaultCallback(matImage);
      sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", matImage).toImageMsg();
      image_msg->header.stamp = ros::Time::now();
      ros_pub_.publish(image_msg);

      // set white balance
      counter++;
      if(counter == mv_config_.set_wb_every_n_frame)
      {
        MVSetCameraWB(hCamera);
        counter = 0;
      }

      // ensure frame rate
      int64_t dur_cur = timer.GetUs();
      if(dur_cur < min_frame_dur)
        usleep(min_frame_dur - dur_cur);
      timer.Start();
    }
  }

  CameraUnInit(hCamera);
  free(g_pRgbBuffer_);

  return 0;
}

void MVCameraDriver::DefaultCallback(cv::Mat &img)
{
  imshow("Opencv Demo", img);

  waitKey(1);
}

bool MVCameraDriver::MVSetCameraConfig(const CameraHandle &hCamera)
{
  int iStatus = 0;
  tSdkCameraCapbility tCapability;
  CameraGetCapability(hCamera, &tCapability);

  tSdkImageResolution *pImageSizeDesc = tCapability.pImageSizeDesc;
  if (mv_config_.resolution_index == MVResolutionIndex::CustomSize)
  {
    pImageSizeDesc->iIndex = 255;
    pImageSizeDesc->iHOffsetFOV = mv_config_.resolution_size.offset_cols;
    pImageSizeDesc->iVOffsetFOV = mv_config_.resolution_size.offset_rows;
    pImageSizeDesc->iWidthFOV = mv_config_.resolution_size.witdh;
    pImageSizeDesc->iHeightFOV = mv_config_.resolution_size.height;
    pImageSizeDesc->iWidth = mv_config_.resolution_size.witdh;
    pImageSizeDesc->iHeight = mv_config_.resolution_size.height;

    iStatus = CameraSetImageResolution(hCamera, pImageSizeDesc);
    if (mv_config_.debug)
    {

      std::cout << "ImageResolution iIndex: " << pImageSizeDesc->iIndex << std::endl;
      std::cout << "  iHOffsetFOV: " << pImageSizeDesc->iHOffsetFOV << std::endl;
      std::cout << "  iVOffsetFOV: " << pImageSizeDesc->iVOffsetFOV << std::endl;
      std::cout << "  iWidthFOV: " << pImageSizeDesc->iWidthFOV << std::endl;
      std::cout << "  iHeightFOV: " << pImageSizeDesc->iHeightFOV << std::endl;
      std::cout << "  iWidth: " << pImageSizeDesc->iWidth << std::endl;
      std::cout << "  iHeight: " << pImageSizeDesc->iHeight << std::endl;
    }
    printf("SetImageResolution state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
      return false;
  }
  else
  {
    iStatus = CameraSetImageResolution(hCamera, &(pImageSizeDesc[1]));
    printf("SetImageResolution state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
      return false;
  }

  // 重新获取参数
  CameraGetCapability(hCamera, &tCapability);

  // 曝光设置
  if (mv_config_.auto_exposure)
  {
    iStatus = CameraSetAeExposureRange(hCamera,
                                       mv_config_.exposure_time_min * 1000,
                                       mv_config_.exposure_time_max * 1000);
    printf("SetAeExposureRange state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
      return false;
    if (mv_config_.debug)
    {
      double e_min, e_max;
      CameraGetAeExposureRange(hCamera, &e_min, &e_max);
      std::cout << "e_min: " << e_min / 1000 << "ms" << std::endl;
      std::cout << "e_max: " << e_max / 1000 << "ms" << std::endl;
    }
  }
  else // manual
  {
    iStatus = CameraSetAeState(hCamera, 0);
    printf("SetAeState state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
      return false;

    iStatus = CameraSetExposureTime(hCamera, mv_config_.exposure_time * 1000);
    if (iStatus != CAMERA_STATUS_SUCCESS)
      return false;
    if (mv_config_.debug)
    {
      double e;
      CameraGetExposureTime(hCamera, &e);
      std::cout << "ExposureTime: " << e / 1000 << "ms" << std::endl;
    }
  }

  // 亮度目标
  CameraSetAeTarget(hCamera, mv_config_.ae_target);
  std::cout << "SetAeTarget: " << mv_config_.ae_target << std::endl;

  // init whit balance
  MVSetCameraWB(hCamera);

  if (tCapability.sIspCapacity.bMonoSensor)
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
  else
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
  return true;
}

void MVCameraDriver::MVSetCameraWB(const CameraHandle &hCamera)
{
  // set whit balance
  int RPos, GPos, BPos;
  CameraSetOnceWB(hCamera);
  if (mv_config_.debug)
  {
    CameraGetGain(hCamera, &RPos, &GPos, &BPos);
    std::cout << "RPos: " << RPos << std::endl;
    std::cout << "GPos: " << GPos << std::endl;
    std::cout << "BPos: " << BPos << std::endl;
  }
}