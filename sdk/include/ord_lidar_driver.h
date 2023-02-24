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

		// ���ô�������
		bool SetSerialPort(const std::string &port_name, const int &baudrate);
		// ��鼤���״ﴮ�ڲ��򿪣����������״ﴮ�ڶ�д�߳�
		bool Connect();
		// �رռ����״ﴮ�ڶ�д�̣߳� �رմ���
		void Disconnect();
		// �жϼ����״ﴮ���Ƿ��
		bool isConnected() const;
		// �����״�Ӵ���״̬������״̬
		bool Activate();
		// �����״�Ӳ��״̬�������״̬
		bool Deactive();
		// ��ȡ����һ���������ݣ�������ʽ�� �������ݰ������е�ĽǶȡ������ǿ����Ϣ
		bool GrabOneScan(one_scan_data_st &scan_data);
		// ��ȡ����һ���������ݣ�����ʽ�� �������ݰ������е�ĽǶȡ������ǿ����Ϣ
		bool GrabOneScanBlocking(one_scan_data_st &scan_data, int timeout_ms);
		// ��ȡ����һȦ�������ݣ�������ʽ�� �������ݰ������е�ĽǶȡ������ǿ����Ϣ
		bool GrabFullScan(full_scan_data_st &scan_data);
		// ��ȡ����һȦ�������ݣ�����ʽ�� �������ݰ������е�ĽǶȡ������ǿ����Ϣ
		bool GrabFullScanBlocking(full_scan_data_st &scan_data, int timeout_ms);
		// ��ȡ���°���ʱ���, ��λΪ����
		uint16_t GetTimestamp() const;
		// ��ȡ���°��ĵ��ת��, ��λΪHz
		double GetRotationSpeed() const;
		// ���õ��ת��
		bool SetRotationSpeed(uint16_t speed);
		// ��ȡ���²���̼��汾��(��δʵ�ִ˹���)
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
