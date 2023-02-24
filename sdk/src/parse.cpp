#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "parse.h"
#include <stdio.h>

int point_data_parse_frame_ms200(point_data_t* data, unsigned char* buf, unsigned short buf_len, float start_angle, float end_angle)
{

	if (data == NULL || buf == NULL || buf_len == 0)
	{
		return -1;
	}
	unsigned short point_num = buf_len / 3;

	float angle_ratio;
	if (point_num > 1)
	{
		start_angle < end_angle ? (angle_ratio = (end_angle-start_angle)/(point_num-1)) : 
								  (angle_ratio = (end_angle + 360 - start_angle)/(point_num-1));
	}
	else 
	{
		angle_ratio = 0;
	}
	for (int i = 0; i < point_num; i++)
	{

		data[i].distance = buf[i * 3 + 1]*256 + buf[i * 3 +0];
		// printf("i %d \n",i);
		
		data[i].intensity = buf[i * 3 + 2];

		float tmp_angle = start_angle + angle_ratio * i;
		if (tmp_angle > 360)
		{
			data[i].angle = tmp_angle - 360;
		}
		else 
		{
			data[i].angle = tmp_angle;
		}
	}
	return 0;
}
