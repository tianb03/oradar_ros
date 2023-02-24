#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include "ord_lidar_driver.h"

#include <sys/time.h>

#define Degree2Rad(X) ((X)*M_PI / 180.)
using namespace ord_sdk;

void publish_msg(ros::Publisher *pub, full_scan_data_st *scan_frame, ros::Time start,
                 double scan_time, std::string frame_id, bool clockwise,
                 double angle_min, double angle_max, double min_range, double max_range)
{
  sensor_msgs::LaserScan scanMsg;
  int node_count = scan_frame->vailtidy_point_num;
  int counts = node_count * ((angle_max - angle_min) / 360.0f);
  int angle_start = angle_min;
  int node_start = node_count * (angle_start / 360.0f);

  // ROS_INFO("get lidar frame count = %d, %d, %d, %d ", node_count, counts, angle_start, node_start);

  scanMsg.ranges.resize(counts);
  scanMsg.intensities.resize(counts);

  float range = 0.0;
  float intensity = 0.0;

  for (int i = 0; i < counts; i++)
  {
    range = scan_frame->data[node_start].distance * 0.001;
    intensity = scan_frame->data[node_start].intensity;
    // printf("range %f, distance:%d, i %d \n", range, scan_frame->data[node_start].distance, i);
    if ((range > max_range) || (range < min_range))
    {
      range = 0.0;
      intensity = 0.0;
    }

    if (!clockwise)
    {
      scanMsg.ranges[counts - 1 - i] = range;
      scanMsg.intensities[counts - 1 - i] = intensity;
    }
    else
    {
      scanMsg.ranges[i] = range;
      scanMsg.intensities[i] = intensity;
    }

    node_start = node_start + 1;
  }
  if (counts > 0)
  {
    scanMsg.header.stamp = start;
    scanMsg.header.frame_id = frame_id;
    scanMsg.angle_min = Degree2Rad(angle_min);
    scanMsg.angle_max = Degree2Rad(angle_max);
    scanMsg.angle_increment = (scanMsg.angle_max - scanMsg.angle_min) / (double)counts;
    scanMsg.scan_time = scan_time;
    scanMsg.time_increment = scan_time / (double)node_count;
    scanMsg.range_min = min_range;
    scanMsg.range_max = max_range;
    pub->publish(scanMsg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oradar_ros");
  std::string frame_id, scan_topic;
  std::string port;
  std::string device_model;
  int baudrate;
  int motor_speed;
  double angle_min, angle_max;
  double min_range, max_range;
  bool clockwise = false;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  nh_private.param<std::string>("port_name", port, "/dev/ttyUSB0");
  nh_private.param<int>("baudrate", baudrate, 230400);
  nh_private.param<double>("angle_max", angle_max, 180.00);
  nh_private.param<double>("angle_min", angle_min, -180.00);
  nh_private.param<double>("range_max", max_range, 20.0);
  nh_private.param<double>("range_min", min_range, 0.05);
  nh_private.param<bool>("clockwise", clockwise, false);
  nh_private.param<int>("motor_speed", motor_speed, 10);
  nh_private.param<std::string>("device_model", device_model, "ms200");
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<std::string>("scan_topic", scan_topic, "scan");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic, 1000);

  OrdlidarDriver device;
  bool ret = false;

  if (port.empty())
  {
    printf("can't find lidar ms200 !\n");
  }
  else
  {
    device.SetSerialPort(port, baudrate);
    ROS_INFO("get lidar type: %s", device_model.c_str());
    ROS_INFO("get serial port: %s, baudrate: %d", port.c_str(), baudrate);

    while (ros::ok())
    {
      if (device.isConnected() == true)
      {
        device.Disconnect();
        ROS_INFO("Disconnect lidar device \n");
      }

      if (device.Connect())
      {
        ROS_INFO("lidar device connect succuss.");
        break;
      }
      else
      {
        ROS_INFO("lidar device connecting...");
        sleep(1);
      }
    }

    full_scan_data_st scan_data;
    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;

    ROS_INFO("get lidar scan data");
    ROS_INFO("ROS topic:%s", scan_topic.c_str());

    device.SetRotationSpeed(motor_speed);

    while (ros::ok())
    {
      start_scan_time = ros::Time::now();
      ret = device.GrabFullScanBlocking(scan_data, 1000);
      end_scan_time = ros::Time::now();

      scan_duration = (end_scan_time - start_scan_time).toSec();
      if (ret)
      {
        publish_msg(&scan_pub, &scan_data, start_scan_time, scan_duration, frame_id,
                    clockwise, angle_min, angle_max, min_range, max_range);
      }
    }

    device.Disconnect();
  }

  return 0;
}
