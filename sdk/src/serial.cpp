/**
 *******************************************************************
 * @file    serial.c
 * @author  jiasiting
 * @version V0.0.1
 * @date    2021-05-17
 * @brief   This C file contains hardware driver serial interface init.
 *******************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/epoll.h>
#include <sys/file.h>
#include "serial.h"

namespace serial
{

    Serial::Serial()
    {
        baudrate = 0;
        uart_fd = 0;
    }

    Serial::~Serial()
    {
        if (uart_fd > 0)
        {
            uart_fd = -1;
        }
    }

    bool Serial::uart_open(const char *dev, int baudrate, int *serialfd)
    {
        struct termios stermios;
        int fd = -1;
        int err;

        pthread_mutex_init(&read_mutex, NULL);
        pthread_mutex_init(&write_mutex, NULL);
        speed_t rate = 0;

        if (baudrate == 9600)
            rate = B9600;
        else if (baudrate == 19200)
            rate = B19200;
        else if (baudrate == 38400)
            rate = B38400;
        else if (baudrate == 57600)
            rate = B57600;
        else if (baudrate == 115200)
            rate = B115200;
        else if (baudrate == 230400)
            rate = B230400;
        else if (baudrate == 460800)
            rate = B460800;
        else if (baudrate == 500000)
            rate = B500000;

        if (rate == 0)
        {
            printf("serial baudrate (%d) not support, open fail!\n", baudrate);
            goto Fail;
        }

        fd = open(dev, O_RDWR | O_NONBLOCK | O_NOCTTY);
        if (fd < 0)
        {
            printf("open serial dev (%s) error, fd = %d\n", dev, fd);
            goto Fail;
        }
        else
        {
            if(flock(fd,LOCK_EX|LOCK_NB) == 0)
            {
               // printf("the file was not locked.\n");
            }
            else
            {
                printf("the (%s) dev was locked.\n", dev);
                goto Fail;
            }
            // printf("open serial dev (%s) baudrate (%d) succceed! fd = %d \n", dev, baudrate, fd);
            tcflush(fd, TCIOFLUSH);
            if ((err = tcgetattr(fd, &stermios)) != 0)
            {
                printf("tcgetattr (%d) = %d\n", fd, err);
                goto Fail;
            }

            stermios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
            stermios.c_oflag &= ~OPOST;
            stermios.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            stermios.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
            stermios.c_cflag |= CS8;
            stermios.c_cflag &= ~CRTSCTS;

            tcflush(fd, TCIOFLUSH);
            if (cfsetispeed(&stermios, rate))
            {
                printf("cfsetispeed.. errno..\n");
                goto Fail;
            }

            tcsetattr(fd, TCSANOW, &stermios);
            // printf("serial dev setup finished..\n");
        }

        *serialfd = fd;
        uart_fd = fd;

        return true;

    Fail:
        if (fd > 0)
        {
            close(fd);
            fd = -1;
        }

        *serialfd = -1;
        return false;
    }

    bool Serial::uart_close(int serialfd)
    {
        if (serialfd > 0)
        {
            close(serialfd);
            serialfd = -1;
            return true;
        }
        pthread_mutex_destroy(&read_mutex);
        pthread_mutex_destroy(&write_mutex);

        return false;
    }

    int Serial::uart_write(int serialfd, unsigned char *buffer, int len)
    {
        int ret = -1;

        // PRINT_BUF(buffer,len);
        pthread_mutex_lock(&write_mutex);
        if (serialfd > 0)
        {
            ret = 0;
            if (buffer != NULL)
            {
                if (-1 == (ret = write(serialfd, buffer, len)))
                {
                    printf("UART write error");
                }
            }
        }
        pthread_mutex_unlock(&write_mutex);

        return ret;
    }

    int Serial::uart_read(int serialfd, unsigned char *buffer, int len)
    {
        int ret = -1;

        pthread_mutex_lock(&read_mutex);
        if (serialfd > 0)
        {
            ret = 0;
            if (buffer != NULL)
            {
                ret = read(serialfd, buffer, len);
            }
        }
        pthread_mutex_unlock(&read_mutex);

        return ret;
    }

}
