/**
 *******************************************************************
 * @file    ord_lidar_driver.h
 * @author  jiasiting
 * @version V0.0.1
 * @date    2021-09-07
 * @brief   This header file contains oradar lidar interface.
 *******************************************************************
 */

#ifndef ORD_LIDAR_DRIVER_H
#define ORD_LIDAR_DRIVER_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <atomic>
#include <fcntl.h>
#include <thread>
#include <sys/types.h>
#include <time.h>
#include "serial.h"
#include "parse.h"

#if !defined(__cplusplus)
#ifndef __cplusplus
#error "The Oradar LIDAR SDK requires a C++ compiler to be built"
#endif
#endif

using namespace serial;

namespace ord_sdk
{

	class OrdlidarDriver
	{
	public:
		OrdlidarDriver();
		OrdlidarDriver(const std::string &port_name, const int &baudrate);
		~OrdlidarDriver();

		// 设置串口属性
		bool SetSerialPort(const std::string &port_name, const int &baudrate);
		// 检查激光雷达串口并打开，创建激光雷达串口读写线程
		bool Connect();
		// 关闭激光雷达串口读写线程， 关闭串口
		void Disconnect();
		// 判断激光雷达串口是否打开
		bool isConnected() const;
		// 激光雷达从待机状态进入测距状态
		bool Activate();
		// 激光雷达从测距状态进入待机状态
		bool Deactive();
		// 获取最新一包点云数据，非阻塞式。 点云数据包含所有点的角度、距离和强度信息
		bool GrabOneScan(one_scan_data_st &scan_data);
		// 获取最新一包点云数据，阻塞式。 点云数据包含所有点的角度、距离和强度信息
		bool GrabOneScanBlocking(one_scan_data_st &scan_data, int timeout_ms);
		// 获取最新一圈点云数据，非阻塞式。 点云数据包含所有点的角度、距离和强度信息
		bool GrabFullScan(full_scan_data_st &scan_data);
		// 获取最新一圈点云数据，阻塞式。 点云数据包含所有点的角度、距离和强度信息
		bool GrabFullScanBlocking(full_scan_data_st &scan_data, int timeout_ms);
		// 获取最新包的时间戳, 单位为毫秒
		uint16_t GetTimestamp() const;
		// 获取最新包的电机转速, 单位为Hz
		double GetRotationSpeed() const;
		// 设置电机转速
		bool SetRotationSpeed(uint16_t speed);
		// 获取上下部组固件版本号(暂未实现此功能)
		bool GetFirmwareVersion(std::string &firmware_version) const;

	private:
		static void mRxThreadProc(void *arg);
		int read_data(unsigned char *data, int length);
		int write_data(unsigned char *data, int length);
		bool uart_data_handle(unsigned char *data, int len);
		bool IsFullScanReady(void) { return full_scan_ready_; }
		void ResetFullScanReady(void) { full_scan_ready_ = false; }
		bool IsOneScanReady(void) { return one_scan_ready_; }
		void ResetOneScanReady(void) { one_scan_ready_ = false; }

	private:
		serial::Serial *serial_port_;
		std::string port_name_;
		int baudrate_;
		int serial_fd_;
		bool is_connected_;
		bool full_scan_ready_;
		bool one_scan_ready_;
		int valid_data_;
		std::vector<uint8_t> bin_buf_;
		parsed_data_st parsed_data_;
		full_scan_data_st full_scan_data_;
		full_scan_data_st temp_data;
		one_scan_data_st one_scan_data_;
		std::thread *rx_thread_;
		std::atomic<bool> rx_thread_exit_flag_;
		pthread_mutex_t one_scan_mtx_;
		pthread_cond_t one_scan_cond_;
		pthread_mutex_t full_scan_mtx_;
		pthread_cond_t full_scan_cond_;
		pthread_condattr_t cond_cattr_;
	};

}

#endif
