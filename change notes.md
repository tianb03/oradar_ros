# Oradar_ROS

## version
oradar_ROS_1.0.2 

## modify
1. 解决对串口重复打开问题
2. 阻塞式获取点云数据，采用信号方式进行阻塞，降低CPU占用率

## version
oradar_ROS_1.0.3 

## modify
1. 针对阻塞式接口功能：初始化pthread相关变量，给pthread条件变量加锁，解决了获取点云数据时间不均匀问题

## version
oradar_ROS_1.0.4

## modify
1. 修改串口重复打开问题，兼容嵌入式linux平台

## version
oradar_ROS_1.0.5

## modify
1. 添加 C API复用C++接口类， 并添加C API测试示例
2. 添加设置雷达电机转速、进入待机状态、测距状态接口

## version
oradar_ROS_1.0.6

## modify
1. 解决设置电机转速时工作状态不同步问题