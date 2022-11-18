# MindVison-RosDriver
MindVison camera driver for ros

## 依赖

- ros-melodic
- opencv3.2+
- MVSDK

## 使用

```shell
# compile
cd [path to your catkin workspace]/src
git clone https://github.com/CodeColdCook/MindVison-RosDriver.git
cd ..
catkin_make

# run
source devel/setup.bash
roslaunch mv_camera_driver mv_camera.launch
```

## 参数设置

`mv_camera_driver)/config/config.yaml`

```yaml
mv_camera: 
  debug: true
  ros_topic: "/mv_camera/image"
  auto_exposure: true
  exposure_time: 30
  exposure_time_min: 1
  exposure_time_max: 500
  ae_target: 100
  max_frame_frate: 20
  set_wb_every_n_frame: 2000
  sharpness: 2
  gama: 117
  conrast: 100
  resolution_index: 8
  resolution_size: 
    offset_cols: 400
    offset_rows: 200
    witdh: 1120 # 1920 - 2*offset_cols
    height: 800 # 1200 - 2* offset_rows
```



