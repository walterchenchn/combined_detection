## 相机与激光雷达融合
### 1. 相机与激光雷达联合标定
#### 1.1 相机内参标定
##### 1.1.1 相机驱动
使用[usb_cam](https://github.com/ros-drivers/usb_cam)驱动usb相机即可
```
$ cd
$ mkdir -p combine_detect/src
$ cd combine_detect/src
$ git clone https://github.com/ros-drivers/usb_cam.git
$ cd ..
$ catkin_make
```
然后就可以运行驱动程序打开usb相机
```
$ source devel/setup.bash
$ roslaunch usb_cam usb_cam-test.launch 
```
##### 1.1.2 内参标定
准备一个已知尺寸的标定板，本实验使用的是8X6的棋盘标定板。由于标定过程使用的是棋盘内部的角点进行，所以实际上我们使用的是9X7的[棋盘标定板](https://i.loli.net/2019/11/05/nswLf1Cvd9Qrx8a.png)。
安装相机标定依赖,并查看usb相机发布的话题
```
$ rosdep install camera_calibration
$ rostopic list
```
不出意外,usb相机发布的话题应该如下
```
/usb_cam/camera_info
/usb_cam/image_raw
```
那接下来就运行标定节点
```
$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/usb_cam/image_raw camera:=/camera
```
1. --size:为当前标定板的大小;  
2. --square 0.108为每个棋盘格的边长(单位为米);  
3. image:=/usb_cam/image_raw标定当前订阅图像来源自名为/usb_cam/image_raw的topic;  
4. camera:=/camera为摄像机名;  
之后出现的界面如下
![biadoing.png](https://i.loli.net/2019/11/05/Q95bBaSjuXrhRyn.png)  
为了达到良好的标定效果，需要在摄像机周围移动标定板，并完成以下基本需求：  
（1）移动标定板到画面的最左、右，最上、下方  
（2）移动标定板到视野的最近和最远处  
（3）移动标定板使其充满整个画面  
（4）保持标定板倾斜状态并使其移动到画面的最左、右，最上、下方  
当标定板移动到画面的最左、右方时，此时，窗口的x会达到最小或满值。同理，y指示标定板的在画面的上下位置，size表示标定板在视野中的距离。每次移动之后，请保持标定板不动直到窗口出现高亮提示。当calibration按钮亮起时，代表已经有足够的数据进行摄像头的标定，此时请按下calibration并等待一分钟左右.在标定完成后，终端会输出校正结果。如下所示,camera matrix即为相机内参
```
[image]

width
640

height
480

[narrow_stereo]

camera matrix
350.510957 0.000000 312.359597
0.000000 349.926394 228.725090
0.000000 0.000000 1.000000

distortion
-0.309584 0.084831 -0.000042 -0.000825 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
260.644318 0.000000 311.737900 0.000000
0.000000 288.382446 222.988164 0.000000
0.000000 0.000000 1.000000 0.000000
```
#### 1.2 相机与激光雷达外参标定
##### 1.2.1 安装Autoware
联合标定需要使用Autoware,请按照[教程](https://www.jianshu.com/p/daa91bc28108)自行安装
##### 1.2.2 外参标定
1. 打开usb相机
```
$ source devel/setup.bash
$ roslaunch usb_cam usb_cam-test.launch 
```
2. 在Autoware工作空间下进行标定
```
$ cd
$ cd autoware/ros
$ source devel/setup.bash
$ roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:=/PATH/TO/camera_calibration.yaml image_src:=/usb_cam/image_raw camera_info_src:=/usb_cam/camera_info
```
3. 将会显示图像查看器  
4. 打开激光雷达驱动,并打开RVIZ显示雷达点云  
5. 在图像中找到一个可以匹配点云中对应点的点,单击图像中点的像素。  
6. 使用RVIZ中Publish Point单击Rviz中的相应3D点。  
7. 用至少9个不同点重复此操作。  
8. 完成后，文件将保存在home目录中，名称为YYYYmmdd_HHMM_autoware_lidar_camera_calibration.yaml
具体如下
```
%YAML:1.0
---
CameraExtrinsicMat: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ -6.1559133916108677e-03, 7.3873260083392678e-02,
       9.9724863809130693e-01, 1.0245305299758911e+00,
       -9.9956827686914074e-01, -2.9105779245669927e-02,
       -4.0141613519019126e-03, -3.8873579353094101e-02,
       2.8729159527764891e-02, -9.9684281361664728e-01,
       7.4020539944494368e-02, -9.3657863140106201e-01, 0., 0., 0., 1. ]
CameraMat: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 3.5051095700000002e+02, 0., 3.1235959700000001e+02, 0.,
       3.4992639400000002e+02, 2.2872508999999999e+02, 0., 0., 1. ]
DistCoeff: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ -3.0958400000000003e-01, 8.4831000000000004e-02,
       -4.1999999999999998e-05, -8.2500000000000000e-04, 0. ]
ImageSize: [ 640, 480 ]
ReprojectionError: 0
DistModel: plumb_bob
```
CameraExtrinsicMat即为相机与激光雷达外参矩阵,最后一行是补齐行,前3行4列才是有用的数据.前3行前3列表示外参旋转矩阵R,前3行第4列表示外参平移矩阵t
#### 1.2 相机与激光雷达联合联合检测
基本思想是使用YOLO v3检测相机图像上的车辆目标,然后通过外参矩阵将目标边界框投影至激光雷达坐标系下,如果有激光点云了进入目标边界框的范围,那么可以认为这些点云是车辆上的点,可以在RVIZ上显示出来.此程序需要下载[数据包](https://stuhiteducn-my.sharepoint.com/:u:/g/personal/chenhao2017_stu_hit_edu_cn/Ee99PM4AJaxPvItE-KA6iq4BJUKiKOMPneXT8UyFu3kN_g?e=C41X9x)
##### 1.2.1 修改程序外参矩阵
```
$ cd combine_detect/src
$ git clone https://github.com/walterchenchn/combined_detection.git
```
修改程序中相机与激光雷达外参
```
$ cd /combined_detection/combine/src
$ gedit combine.cpp 
```
1. 修改47行"外参,旋转矩阵R",将标定得到的外参矩阵CameraExtrinsicMat的旋转矩阵(即前三行三列)求逆矩阵,并写入;  
2. 修改51行,将CameraExtrinsicMat的平移矩阵(即前三行第四列)求逆矩阵,并写入;  
3. 修改49行,将CameraMat求逆矩阵,并写入.然后进行程序编译
```
$ cd ..
$ catkin_make
```
##### 1.2.2 联合检测
在打开相机和雷达的情况下,进行联合检测.首先需要下载深度学习网络[权重包](https://stuhiteducn-my.sharepoint.com/:u:/g/personal/chenhao2017_stu_hit_edu_cn/ETlas-Z4zvhCv-oJyoYkTWYBjjL1VKUefxGWUpXOc56ZhQ?e=7vKdXL),并放到/combined_detection/darknet_ros/darknet_ros/yolo_network_config/weights文件夹下
```
$ source devel/setup.bash
$ roslaunch darknet_ros darknet_ros.launch
```
此时再打开一个终端界面,运行数据包,会出现图像检测画面
```
$ rosbag play -r 0.1 -s 28 airs_car.bag
```
然后再打开雷达检测程序
```
$ rosrun combine combine
```
此时打开RVIZ,并使用文件包中的RVIZ配置(具体在rviz_config文件夹下),即可看到绿色框出现在车辆点云处.








