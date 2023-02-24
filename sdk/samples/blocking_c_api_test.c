#include <stdio.h>
#include <signal.h>
#include "ord_lidar_sdk.h"

int running = 1;

static void sig_handle(int signo)
{
    printf("program exit, [%s,%s] Receive SIGNAL %d ====== \r\n", __FILE__, __func__, signo);
    running = 0;
    sleep(1);
    exit(1);
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
    signal(SIGPIPE, sig_handle);
    signal(SIGCHLD, sig_handle);

    full_scan_data_st scan_data;
    char port[50] = "/dev/ttyUSB0";
    int baudrate = 230400;
    bool ret = false;
    bool is_logging = false; 
    ORDLidar *laser = oradar_lidar_create();
    ret = oradar_set_serial_port(laser, port, baudrate);
    if (!ret)
    {
        printf("serial port set fail\n");
        goto exit;
    }
    ret = oradar_connect(laser);
    if(!ret)
    {
        printf("serial connect fail\n");
        goto exit;
    }

    while (running)
    {
        if (oradar_get_grabfullscan_blocking(laser, &scan_data, 1000))
        {
            printf("vailtidy_point_num: %d\n", scan_data.vailtidy_point_num);
            if(is_logging)
            {
                for (int i = 0; i < scan_data.vailtidy_point_num; i++)
                {
                    printf("[%d: %f, %f] \n", i, (scan_data.data[i].distance * 0.001), scan_data.data[i].angle);
                }
            }
        }
        else
        {
            printf("fail to get lidar data\n");
        }
    }

exit:
    oradar_lidar_destroy(&laser);
    return 0;
}
