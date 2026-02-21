#ifndef STATE_CHECK_H
#define STATE_CHECK_H
#include "robot_param_balanced_infantry.h"
#include "stdbool.h"
#include "struct_typedef.h"

// 定义起立状态
typedef enum {
	  STATE_NORMAL,
    STATE_stretchleg,        // 伸腿态
    STATE_Backleg,       // 后甩腿
    STATE_COMPLETE   // 左腿完成状态
} State;



#endif
