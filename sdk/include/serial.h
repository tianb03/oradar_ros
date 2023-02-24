/**
 *******************************************************************
 * @file    serial.h
 * @author  jiasiting
 * @version V0.0.1
 * @date    2021-05-17
 * @brief   This header file contains serial uart interface.
 *******************************************************************
 */

#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdio.h>
#include <string.h>
#include <unistd.h>

namespace serial
{
	class Serial
	{
	public:
		Serial();
		~Serial();
		bool uart_open(const char *dev, int baudrate, int *serialfd);
		bool uart_close(int serialfd);
		int uart_write(int serialfd, unsigned char *buffer, int len);
		int uart_read(int serialfd, unsigned char *buffer, int len);
	private:
		pthread_mutex_t read_mutex;
		pthread_mutex_t write_mutex;
		int baudrate;
		int uart_fd;
	};
}

#endif
