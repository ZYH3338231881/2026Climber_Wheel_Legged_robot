#ifndef ROBOT_TYPEDEF_H
#define ROBOT_TYPEDEF_H

// clang-format off
// 可用底盘硬件类型
#define CHASSIS_NONE            0  // 无底盘
#define CHASSIS_MECANUM_WHEEL   1  // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL      2  // 全向轮底盘
#define CHASSIS_STEERING_WHEEL  3  // 舵轮底盘
#define CHASSIS_BALANCE         4  // 平衡底盘

// 可用云台硬件类型
#define GIMBAL_NONE                0  // 无云台
#define GIMBAL_YAW_PITCH_DIRECT    1  // yaw-pitch电机直连云台

// 可用的发射机构硬件类型
#define SHOOT_NONE               0  // 无发射机构
#define SHOOT_FRIC_TRIGGER       1  // 摩擦轮+拨弹盘发射机构
#define SHOOT_PNEUMATIC_TRIGGER  2  // 气动+拨弹盘发射机构

// 可用机械臂硬件类型
#define MECHANICAL_ARM_NONE              0  // 无机械臂
#define MECHANICAL_ARM_PENGUIN_MINI_ARM  1  // 企鹅mini机械臂
#define MECHANICAL_ARM_ENGINEER_ARM      2  // 工程机械臂

// 可用自定义控制器硬件类型
#define CUSTOM_CONTROLLER_NONE         0  // 无自定义控制器
#define CUSTOM_CONTROLLER_PENGUIN_MINI 1  // 企鹅mini自定义控制器
#define CUSTOM_CONTROLLER_ENGINEER     2  // 工程用的自定义控制器

// 控制类型（板间通信时用到）
#define CHASSIS_ONLY       0  // 只控制底盘
#define GIMBAL_ONLY        1  // 只控制云台
#define CHASSIS_AND_GIMBAL 2  // 控制底盘和云台

// 可用C板ID
#define BoardC1 1  //C1板
#define BoardC2 2  //C2板
#define BoardC3 3  //C3板
#define BoardC4 4  //C4板

// 可用调参模式
#define TUNING_NONE     0
#define TUNING_CHASSIS  1
#define TUNING_GIMBAL   2
#define TUNING_SHOOT    3

// 校准数据来源
#define CALI_FROM_FLASH 1
#define CALI_FROM_USB   2
#define CALI_FROM_CODE  3

// 自定义控制器类型
#define CC_RECEIVER 0  // 接收器
#define CC_SENDER   1  // 发送器

// 遥控器类型
#define RC_DT7      0  // DT7遥控器
#define RC_AT9S_PRO 1  // AT9S PRO遥控器
#define RC_HT8A     2  // HT8A遥控器
#define RC_ET08A    3  // ET08A遥控器

// C板id
#define C_BOARD_DEFAULT                  1  // C板默认id
#define C_BOARD_BALANCE_CHASSIS          2  // 平衡底盘C板
#define C_BOARD_BALANCE_GIMBAL           3  // 平衡云台C板
#define C_BOARD_ENGINEER_CHASSIS         4  // 工程底盘C板
#define C_BOARD_ENGINEER_MECHANICAL_ARM  5  // 工程机械臂C板
#define C_BOARD_ENGINEER_GIMBAL          6  // 工程云台C板
#define C_BOARD_OMNI_INFANTRY            7  // (单板)全向轮步兵C板
#define C_BOARD_MECANUM_HERO             8  // (单板)麦克纳姆轮英雄C板
#define C_BOARD_OMNI_SENTINEIL           9  // (单板)全向轮哨兵C板

// 控制链路相关
#define CL_RC_NONE           0x100  // 无遥控器链路
#define CL_RC_DIRECT         0x101  // 直连遥控器（通过dbus口获取直接的遥控器数据）
#define CL_RC_CAN            0x102  // 通过CAN口获取遥控器数据
#define CL_RC_UART2          0x103  // 通过UART2口获取遥控器数据

#define CL_KM_NONE     0x200  // 无键鼠数据
#define CL_KM_RC       0x201  // 仅通过遥控器获取键鼠数据
#define CL_KM_VT       0x202  // 仅通过图传链路获取键鼠数据(Video Transmission)
#define CL_KM_RC_VT    0x203  // 同时使用图传链路和遥控器获取键鼠数据（互补操作，遥控器优先）
#define CL_KM_VT_RC    0x204  // 同时使用图传链路和遥控器获取键鼠数据（互补操作，图传链路优先）

#define CL_PS2_NONE     0x300  // 无PS2链路
#define CL_PS2_DIRECT   0x301  // PS2直接连接（通过 8pin 自定义io口获取直接的PS2数据）
#define CL_PS2_CAN      0x302  // 通过CAN口获取PS2数据
#define CL_PS2_UART2    0x303  // 通过UART2口获取PS2数据

// 虚拟云台数据源
#define VG_FROM_NONE      0x000 // 无数据源
#define VG_FROM_UART2     0x001 // uart2串口数据源
#define VG_FROM_CAN       0x002 // can口数据源
#define VG_FROM_YAW_MOTOR 0x003 // 直接使用yaw电机数据

// 可用电机类型
typedef enum __MotorType {
    DJI_M3508,
    DM_8009,

} MotorType_e;
// clang-format on

#endif /* ROBOT_TYPEDEF_H */
/*------------------------------ End of File ------------------------------*/
