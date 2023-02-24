#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>
#include <iostream>
#include "ord_lidar_driver.h"

using namespace ord_sdk;
int running = 1;

static void sig_handle(int signo)
{
    printf("program exit, [%s,%s] Receive SIGNAL %d ====== \r\n", __FILE__, __func__, signo);
    running = 0;
    sleep(1);
    exit(1);
}

long long get_localtime_us(void)
{
    struct timeval tv;
	long long us = 0;

    gettimeofday(&tv, NULL);

	us = (long long)tv.tv_sec * (long long)1000*1000 + (long long)tv.tv_usec;
    return us;
}

int main(int argc, char *argv[])
{
    signal(SIGABRT, sig_handle);
    signal(SIGQUIT, sig_handle);
    signal(SIGINT, sig_handle);
    signal(SIGHUP, sig_handle);
    signal(SIGKILL, sig_handle);
    signal(SIGTERM, sig_handle);
    signal(SIGUSR1, sig_handle);
    signal(SIGSEGV, sig_handle);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGCHLD, SIG_IGN);

    OrdlidarDriver device;
    full_scan_data_st scan_data;
    std::string port_name("/dev/ttyUSB0");
    int serialBaudrate = 230400;
    bool is_logging = false;
    bool ret = false;
    long long count = 0;
    device.SetSerialPort(port_name, serialBaudrate);

    while (running)
    {
        if (device.Connect())
        {
            printf("scan_frame_data lidar device connect succuss..\n");
            break;
        }
        else
        {
            printf("lidar device connect fail..\n");
            sleep(1);
        }
    }
    
    while (running)
    {
        ret = device.GrabFullScanBlocking(scan_data, 1000);
        if (ret)
        {
            printf("[%lld] count = %lld, point_num: %d\n", get_localtime_us(), ++count, scan_data.vailtidy_point_num);
            if (is_logging)
            {
                for (int i = 0; i < scan_data.vailtidy_point_num; i++)
                {
                    printf("[%d: %f, %f] \n", i, (scan_data.data[i].distance * 0.001), scan_data.data[i].angle);
                }
            }

        }
        else
        {
            printf("error: fail get full scan data\n");
        }
    }

    device.Disconnect();
exit:
    return 0;
}
