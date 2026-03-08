#ifndef _TOF_DISTANCE_H_
#define _TOF_DISTANCE_H_
#include "string.h"
#include "main.h"
#define BUFLENGTH  128 //最大接收数据
#define DATALENGTH 16//有效数据
typedef struct
{
	    uint8_t real_receive[BUFLENGTH];    // 原始接收数据
			float distance;
	    uint16_t strength;

} TOF_data_t;

void TOF_control_init(void);

#endif
