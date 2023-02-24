# ORADAR ROS package V1.0.6
## 使用方法： 

1. 在系统中安装ros环境，具体安装方法参考下面连接：

   安装链接：http://wiki.ros.org/kinetic/Installation/Ubuntu

   搭建ros工程链接：http://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment  

2. 将oradar_ros源码复制到ros工作目录下的src目录

   ```shell
   mkdir -p ~/lidar_ros_ws/src
   cp -ar oradar_ros ~/lidar_ros_ws/src/
   ```

3. 编译工程

   ```shell
   cd ~/lidar_ros_ws
   catkin_make
   ```

4. 设置环境变量

   ```shell
   source devel/setup.sh
   ```

5. 配置上位机串口

   配置串口port_name和波特率： 默认配置port_name为/dev/ttyUSB0, 波特率为230400

6. 配置雷达参数

   打开oradar_ros/launch/ms200_scan.launch 进行参数配置或者oradar_ros/launch/ms200_scan_view.launch 进行参数配置

   参数说明如下：

   | 参数名      | 数据类型 | 描述                                                         |
   | ----------- | -------- | ------------------------------------------------------------ |
   | frame_id    | string   | 激光雷达坐标系名称。 默认为laser_frame                       |
   | scan_topic  | string   | LaserScan主题名。 默认为scan                                 |
   | port_name   | string   | 激光雷达串口名称。 默认值为/dev/ttyUSB0                      |
   | baudrate    | int      | 雷达串口波特率.。 默认值为230400                             |
   | angle_min   | double   | 最小角度，单位度，取值范围[0, 360]。 默认值为0 |
   | angle_max   | double   | 最大角度，单位度，取值范围[0, 360]。 默认值为360 |
   | range_min   | double   | 最小距离，单位米，默认值为0.05                               |
   | range_max   | double   | 最大距离，单位米，默认值为20.0                               |
   | clockwise    | bool     | 配置点云方向，true为顺时针， false为逆时针。默认为false |
   | motor_speed | int      | 雷达转速，单位Hz，取值范围为5~15Hz。默认值为10Hz             |

   

7. 启动Oradar ros节点

   ```shell
   roslaunch oradar_lidar ms200_scan.launch
   或者
   roslaunch oradar_lidar ms200_scan_view.launch (使用rviz显示) 
   ```
