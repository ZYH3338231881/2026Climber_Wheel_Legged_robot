#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_rc.h"
#include "IMU_task.h"
#include "motor.h"
#include "CAN_receive.h"
#include "stdbool.h"
#include "string.h"
#include "kalman_filter.h"
#include "CAN_cmd_damiao.h"
#include "bsp_delay.h"
#include "bsp_can.h"
#include "chassis_balance_extra.h"
#include "stdio.h"
#include "fdcan.h"
#include "usart.h"
#include "State_chexk_task.h"
#include "CAN_receive.h"
#include "bsp_rc.h"
#include "computer_rec.h"
#include "user_lib.h"
#define CHASSIS_TASK_INIT_TIME 300
#define CHASSIS_CONTROL_TIME_MS 3  //底盘控制周期3ms
#define MS_TO_S 0.001f
#define GIMBAL_DIRECT_YAW_MID (-2.73968983)    //云台初始化正对齐的时候使用的yaw轴正中心量
#define rc_deadband_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }
extern  Keyboard_Data keyboard_data;
extern  JudgementDataTypedef JudgementData;

CTOM_message_t ctom_message;
//机器人结构体
Chassis_s CHASSIS = {
	.mode=CHASSIS_SAFE
};
fp32 BODY_MASS_FN=(90.0f);      // 前馈抵消重量    N
Observer_t OBSERVER;

Calibrate_s CALIBRATE = {
    .cali_cnt = 0,
    .velocity = {0.0f, 0.0f, 0.0f, 0.0f},
    .stpo_time = {0, 0, 0, 0},
    .reached = {false, false, false, false},
    .calibrated = false,
};
int8_t TRANSITION_MATRIX[10] = {0};

 void ChassisInit(void);
  void ChassisHandleException(void);
 void ChassisSetMode(void);
 void ChassisObserver(void);
 void ChassisReference(void);
 void ChassisConsole(void);
 void ChassisSendCmd(void);
 void ChassisHandleException(void);


void chassis_task(void const * pvParamewwters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // 初始化底盘
    ChassisInit();
    while (1) {
        // 更新状态量
        ChassisObserver(); 
        // 处理异常
        ChassisHandleException();
        // 设置底盘模式
        ChassisSetMode();
        // 更新目标量
        ChassisReference();
        // 计算控制量
        ChassisConsole();
        // 发送控制量
      ChassisSendCmd();
			  MToC_sendControl(3,0x555,JudgementData.power_heat_data_t.shooter_17_heat1,JudgementData.shoot_data_t.initial_speed);
//       vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}


 void ChassisInit(void)
 {
	 	CHASSIS.rc = get_remote_control_point();  //获取遥控器指针 
    //CHASSIS.imu = Subscribe(IMU_NAME);    // 在陀螺仪任务里面直接赋值
	  /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.joint_motor[0], 1, JOINT_CAN, DM_8009, J0_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[1], 2, JOINT_CAN, DM_8009, J1_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[2], 3, JOINT_CAN, DM_8009, J2_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[3], 4, JOINT_CAN, DM_8009, J3_DIRECTION, 1, DM_MODE_MIT);
	  
	  MotorInit(&CHASSIS.wheel_motor[0], 1, WHEEL_CAN, DJI_M3508, W0_DIRECTION, 15.76470588235294, 0);//注意电流控制要转换成扭矩控制
    MotorInit(&CHASSIS.wheel_motor[1], 2, WHEEL_CAN, DJI_M3508, W1_DIRECTION, 15.76470588235294, 0);//反馈数据是转子的转速
	 
    // 使能达妙电机
    DmEnable(&CHASSIS.joint_motor[0]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[1]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[2]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[3]);

    /*-------------------- 初始化状态转移矩阵 --------------------*/
    TRANSITION_MATRIX[NORMAL_STEP]=NORMAL_STEP;
    TRANSITION_MATRIX[JUMP_STEP_SQUST]=JUMP_STEP_JUMP;
    TRANSITION_MATRIX[JUMP_STEP_JUMP]=JUMP_STEP_RECOVERY;
    TRANSITION_MATRIX[JUMP_STEP_RECOVERY]=NORMAL_STEP;
	  /*-------------------- 值归零 --------------------*/
    memset(&CHASSIS.fdb, 0, sizeof(CHASSIS.fdb));
    memset(&CHASSIS.ref, 0, sizeof(CHASSIS.ref));  
	 
	  CHASSIS.fdb.leg[0].is_take_off = false;
    CHASSIS.fdb.leg[1].is_take_off = false;

    /*-------------------- 初始化底盘模式 --------------------*/
    CHASSIS.mode = CHASSIS_SAFE;//默认电机关闭
		
		// yaw轴速度环pid  
    float yaw_velocity_pid[3] = {KP_CHASSIS_YAW_VELOCITY, KI_CHASSIS_YAW_VELOCITY, KD_CHASSIS_YAW_VELOCITY};
    PID_init(&CHASSIS.pid.yaw_velocity, PID_POSITION, yaw_velocity_pid, MAX_OUT_CHASSIS_YAW_VELOCITY,MAX_IOUT_CHASSIS_YAW_VELOCITY);
  
    //初始化腿部防劈叉pid
		float leg_tp_pid[3]={KP_CHASSIS_TP,KI_CHASSIS_TP,KD_CHASSIS_TP};
		PID_init(&CHASSIS.pid.leg_tp,PID_POSITION,leg_tp_pid,MAX_OUT_CHASSIS_TP,MAX_IOUT_CHASSIS_TP);
    
    //初始化腿部追逐PID
    //左腿追右腿
    float leg_chase_pid_L_to_R[3]={KP_CHASSIS_CAHSE_LEG_L_to_R,KI_CHASSIS_CAHSE_LEG_L_to_R,KD_CHASSIS_CAHSE_LEG_L_to_R};
    PID_init(&CHASSIS.pid.leg_chase_L_to_R,PID_POSITION,leg_chase_pid_L_to_R,MAX_OUT_CHASSIS_CAHSE_LEG_L_to_R,MAX_IOUT_CHASSIS_CAHSE_LEG_L_to_R);
    //右腿追左腿
    float leg_chase_pid_R_to_L[3]={KP_CHASSIS_CAHSE_LEG_R_to_L,KI_CHASSIS_CAHSE_LEG_R_to_L,KD_CHASSIS_CAHSE_LEG_R_to_L};
    PID_init(&CHASSIS.pid.leg_chase_R_to_L,PID_POSITION,leg_chase_pid_R_to_L,MAX_OUT_CHASSIS_CAHSE_LEG_R_to_L,MAX_IOUT_CHASSIS_CAHSE_LEG_R_to_L);

    //初始化腿部腿长pid
    float leg_length_length_pid[3] = {KP_CHASSIS_LEG_LENGTH_LENGTH, KI_CHASSIS_LEG_LENGTH_LENGTH, KD_CHASSIS_LEG_LENGTH_LENGTH};
		PID_init(&CHASSIS.pid.leg_length_length[0], PID_POSITION, leg_length_length_pid,MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);//单级pid控制腿长
		CHASSIS.pid.leg_length_length[0].N = N_LEG_LENGTH_LENGTH;//普通左腿长
		PID_init(&CHASSIS.pid.leg_length_length[1], PID_POSITION, leg_length_length_pid,MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    CHASSIS.pid.leg_length_length[1].N = N_LEG_LENGTH_LENGTH;//普通右腿长
		
    //站立pid->站立收腿pid
    float stand_up_pid[3] = {KP_CHASSIS_STAND_UP, KI_CHASSIS_STAND_UP, KD_CHASSIS_STAND_UP};
    PID_init(&CHASSIS.pid.stand_up, PID_POSITION, stand_up_pid, MAX_OUT_CHASSIS_STAND_UP,MAX_IOUT_CHASSIS_STAND_UP);
		
		//PITCH轴PID
    float pitch_pid[3] = {KP_CHASSIS_PITCH_ANGLE,KI_CHASSIS_PITCH_ANGLE ,KD_CHASSIS_PITCH_ANGLE };
    PID_init(&CHASSIS.pid.pitch_angle, PID_POSITION, pitch_pid, MAX_OUT_CHASSIS_PITCH_ANGLE,MAX_IOUT_CHASSIS_PITCH_ANGLE);

		//底盘跟随云台PID
    float chassis_folllow_gimbal[3] = {KP_CHASSIS_FOLLOW_GIMBAL,KI_CHASSIS_FOLLOW_GIMBAL,KD_CHASSIS_FOLLOW_GIMBAL};
    PID_init(&CHASSIS.pid.chassis_follow_gimbal, PID_POSITION, chassis_folllow_gimbal, MAX_OUT_CHASSIS_FOLLOW_GIMBAL,MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);
		
    // 初始化低通滤波器
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[0], LEG_DDL0_LPF_ALPHA);//腿长L0加速度滤波
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[1], LEG_DDL0_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[0], LEG_DDPHI0_LPF_ALPHA);//机体phi0与水平方向夹角加速度滤波
    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[1], LEG_DDPHI0_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[0], LEG_DDTHETA_LPF_ALPHA);//机体系theta加速度滤波
    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[1], LEG_DDTHETA_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[0], LEG_SUPPORT_FORCE_LPF_ALPHA);//支持力滤波
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[1], LEG_SUPPORT_FORCE_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.pitch, CHASSIS_PICTH_ALPHA);//pitch轴滤波
		    
		
	  LowPassFilterInit(&CHASSIS.lpf.L_theta, L_LEG_THETA_ALPHA);//腿theta误差值滤波
	  LowPassFilterInit(&CHASSIS.lpf.R_theta, R_LEG_THETA_ALPHA);//腿theta误差值滤波
		
	  LowPassFilterInit(&CHASSIS.lpf.VX_filter, 0.9);//键鼠速度滤波
    // 初始化加速度低通滤波器
    LowPassFilterInit(&CHASSIS.lpf.x_acc_lpf, 0.95);//可根据需要调整alpha值

    Kalman_Filter_Init(&OBSERVER.body.v_kf, 2, 1, 2);
    float F[4] = {1, 0.003, 0, 1};//状态转移矩阵
    float B[2] = {0.003*0.003*0.5, 0.003};//输入矩阵
  float Q[4] = {0.1, 0, 0, 0.5};   // 位置过程噪声0.1，速度过程噪声0.5
  float R[4] = {0.5, 0, 0, 10.0};  // 位置测量噪声0.5，速度测量噪声10.0
    float P[4] = {100000, 0, 0, 100000};//协方差
    float H[4] = {1, 0, 0, 1};//观测矩阵
    // 复制矩阵到滤波器
    memcpy(OBSERVER.body.v_kf.F_data, F, sizeof(F));
    memcpy(OBSERVER.body.v_kf.B_data, B, sizeof(B));  // 设置控制矩阵
    memcpy(OBSERVER.body.v_kf.P_data, P, sizeof(P));
    memcpy(OBSERVER.body.v_kf.Q_data, Q, sizeof(Q));
    memcpy(OBSERVER.body.v_kf.R_data, R, sizeof(R));
    memcpy(OBSERVER.body.v_kf.H_data, H, sizeof(H));
		
 }

 //大疆遥控器设置模式
 void ChassisSetMode(void)
 {
	  if(switch_is_down(CHASSIS.rc->sw2)&&switch_is_down(CHASSIS.rc->sw1))
		{
        CHASSIS.mode = CHASSIS_SAFE; //双下无力安全模式
    } 
	  else if (switch_is_mid(CHASSIS.rc->sw2)&&switch_is_down(CHASSIS.rc->sw1)) 
		{
        CHASSIS.mode = CHASSIS_STAND_UP;  // 左中右下，底盘起立，从倒地状态到站立状态的中间过程左中右下
    }
		else if (switch_is_up(CHASSIS.rc->sw2)&&switch_is_down(CHASSIS.rc->sw1)) 
		{
        CHASSIS.mode =CHASSIS_FREE;       //左上右下   底盘自由移动  
		}
		else if (switch_is_up(CHASSIS.rc->sw2)&&switch_is_up(CHASSIS.rc->sw1)) 
		{
        CHASSIS.mode =CHASSIS_FREE;       //左上右下   底盘自由移动  
		}
		
    #if OPEN_CALIBRATE
    else if (switch_is_down(CHASSIS.rc->sw2)&&switch_is_up(CHASSIS.rc->sw1)) 
		{
        CHASSIS.mode = CHASSIS_CALIBRATE;       // 左下右上   速度校准
    }
    #endif  
		#if ENBAL_keyboad
		else if(switch_is_up(CHASSIS.rc->sw2)&&switch_is_up(CHASSIS.rc->sw1))
		{
			CHASSIS.mode = CHASSIS_COMPTER;   //双上开启键鼠功能
		}
		#endif
    else if (switch_is_down(CHASSIS.rc->sw2)&&switch_is_mid(CHASSIS.rc->sw1)) 
		{
        CHASSIS.mode = CHASSIS_OFF_HOOK;       //左下右中   收腿自救脱困
    }
		
    else 
    {
        CHASSIS.mode = CHASSIS_SAFE; // 默认为安全模式
    }	

#if TAKE_OFF_DETECT
    // 离地状态切换
		if(CHASSIS.rc->ch3==-660)
		{
			    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.fdb.leg[i].is_take_off &&CHASSIS.fdb.leg[i].touch_time > TOUCH_TOGGLE_THRESHOLD) 
					{
            CHASSIS.fdb.leg[i].is_take_off = false;
          } 
						else if (!CHASSIS.fdb.leg[i].is_take_off &&CHASSIS.fdb.leg[i].take_off_time > TOUCH_TOGGLE_THRESHOLD) 
					{
            CHASSIS.fdb.leg[i].is_take_off = true;
          }
          }
		}
		else
		{
				CHASSIS.fdb.leg[0].is_take_off = false;
				CHASSIS.fdb.leg[1].is_take_off = false;

		}
    
#endif
		
 }
 
 /******************************************************************/
/* Observe                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisObserver                            */
/* auxiliary function: UpdateBodyStatus      机体状态                     */
/*                     UpdateLegStatus       腿部状态                   */
/*                     UpdateMotorStatus     电机状态                     */
/*                     BodyMotionObserve     移动观测                     */
/******************************************************************/
#define ZERO_POS_THRESHOLD 0.001f
static void UpdateBodyStatus(void);
static void UpdateLegStatus(void);
static void UpdateMotorStatus(void);
static void BodyMotionObserve(void);
 static void UpdateStepStatus(void);
static void UpdateGimbalStatus(void);//更新云台状态
static void UpdateCalibrateStatus(void);//校准状态
extern void DmFdbData(DmMeasure_s * dm_measure, uint8_t * rx_data);
extern void GetDmFdbData(Motor_s *motor,DmMeasure_s *dm_measure);
extern void GetMotorMeasure(Motor_s * p_motor);
 void ChassisObserver(void)
 {
	 CHASSIS.duration = xTaskGetTickCount() - CHASSIS.last_time;
	 
   CHASSIS.last_time = xTaskGetTickCount();                   //计算控制周期
	 CHASSIS.step_time++;
	 
	 UpdateMotorStatus(); // 更新电机状态

   UpdateBodyStatus (); // 更新机体状态

	 UpdateLegStatus  ();  // 更新腿部状态
	 
	 UpdateStepStatus();   // 状态转移更新
   
	 BodyMotionObserve();  // 机体运动状态观测器
	 
	 UpdateGimbalStatus(); //更新云台状态
	 
	 UpdateCalibrateStatus();

}

 static void UpdateGimbalStatus(void)
 {
	   CHASSIS.fdb.gimbal.gimbal_yaw_6020=ctom_message.gimbal_yaw_6020;
 }
 static void UpdateCalibrateStatus(void)
{
    if ((CHASSIS.mode == CHASSIS_CALIBRATE) &&
        fabs(CHASSIS.joint_motor[0].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[1].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[2].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[3].fdb.pos) < ZERO_POS_THRESHOLD) {
        CALIBRATE.calibrated = true;
    }
				
    // m  
    uint32_t now = HAL_GetTick();
    if (CHASSIS.mode == CHASSIS_CALIBRATE) 
    {
        for (uint8_t i = 0; i < 4; i++) 
        {  
            CALIBRATE.velocity[i] = CHASSIS.joint_motor[i].fdb.vel;
            if (CALIBRATE.velocity[i] > CALIBRATE_STOP_VELOCITY) {  // 速度大于阈值时重置计时
                CALIBRATE.reached[i] = false;
                CALIBRATE.stpo_time[i] = now;
            } 
            else
            {
                if (now - CALIBRATE.stpo_time[i] > CALIBRATE_STOP_TIME) {
                    CALIBRATE.reached[i] = true;
                }
            }
        }
    }
}
void UpdateBodyStatus()
{
 //记得检测是否和模型一致
 //陀螺仪数据-->>机体数据
  //机体的roll就是电池正方向  imu对应安装也是roll角度
	CHASSIS.fdb.body.roll=CHASSIS.imu->angle[0];
	CHASSIS.fdb.body.roll_dot=CHASSIS.imu->gyro[0];
	
  //机体系pitch就是左右倾斜的角
	CHASSIS.fdb.body.pitch = CHASSIS.imu->angle[1];
	CHASSIS.fdb.body.pitch_dot=CHASSIS.imu->gyro[1];
	
	CHASSIS.fdb.body.yaw=CHASSIS.imu->angle[2];
	CHASSIS.fdb.body.yaw_dot=CHASSIS.imu->gyro[2];
	
  // 滤波计算轴pitch lpf.pitch  因为高低差往往是突然出现的
	LowPassFilterCalc(&CHASSIS.lpf.pitch, CHASSIS.fdb.body.pitch);
	
	 // 更新加速度反馈数据，记录下来方便使用
    float ax = CHASSIS.imu->accel[0];
    float ay = CHASSIS.imu->accel[1];
    float az = CHASSIS.imu->accel[2];
	
	    // 计算几个常用的三角函数值，减少重复计算
    float cos_roll = cosf(CHASSIS.fdb.body.roll);
    float sin_roll = sinf(CHASSIS.fdb.body.roll);
    float cos_pitch = cosf(CHASSIS.fdb.body.pitch);
    float sin_pitch = sinf(CHASSIS.fdb.body.pitch);
    float cos_yaw = cosf(CHASSIS.fdb.body.yaw);
    float sin_yaw = sinf(CHASSIS.fdb.body.yaw);
		/**达妙BMI088旋转后得出的坐标系是左手系
		Rz(ψ) = [ cosψ  sinψ  0 ]
						[-sinψ  cosψ  0 ]
						[  0     0    1 ]
		Ry(θ) = [ cosθ  0  -sinθ ]
						[  0    1    0   ]
						[ sinθ  0   cosθ ]
		Rx(φ) = [ 1    0     0   ]
            [ 0   cosφ  sinφ ]
            [ 0  -sinφ  cosφ ]
		从世界系到机体系的旋转矩阵：	
		R = Rx(φ) * Ry(θ) * Rz(ψ)
		计算重力在机体系中的分量
		g_world = [0, 0, GRAVITY]^T
		gx = 1*(-GRAVITY*sinθ) + 0*0 + 0*(GRAVITY*cosθ) = -GRAVITY*sinθ
		gy = 0*(-GRAVITY*sinθ) + cosφ*0 + sinφ*(GRAVITY*cosθ) = GRAVITY*sinφ*cosθ
		gz = 0*(-GRAVITY*sinθ) - sinφ*0 + cosφ*(GRAVITY*cosθ) = GRAVITY*cosφ*cosθ

    **/
		
		// 计算重力加速度在各个轴上的分量
    CHASSIS.fdb.body.gx = -GRAVITY * sin_pitch;
    CHASSIS.fdb.body.gy = GRAVITY * sin_roll * cos_pitch;
    CHASSIS.fdb.body.gz = GRAVITY * cos_roll * cos_pitch;
		
    
    // 消除重力加速度的影响，获取机体坐标系下的加速度
    CHASSIS.fdb.body.x_accel = ax - CHASSIS.fdb.body.gx;
    CHASSIS.fdb.body.y_accel = ay - CHASSIS.fdb.body.gy;
    CHASSIS.fdb.body.z_accel = az - CHASSIS.fdb.body.gz;
		
		
		// 计算从机体坐标系到大地坐标系的旋转矩阵  R_{body->world}
    float R[3][3] = {
        {cos_pitch * cos_yaw, sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw, cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw},
        {cos_pitch * sin_yaw, sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw, cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw},
        {-sin_pitch         , sin_roll * cos_pitch                               , cos_roll * cos_pitch                               }
    };

    // 更新大地坐标系下的加速度
    CHASSIS.fdb.world.x_accel = R[0][0] * ax + R[0][1] * ay + R[0][2] * az;
    CHASSIS.fdb.world.y_accel = R[1][0] * ax + R[1][1] * ay + R[1][2] * az;
    CHASSIS.fdb.world.z_accel = R[2][0] * ax + R[2][1] * ay + R[2][2] * az - GRAVITY;

    //机体的phi、phi_dot
    CHASSIS.fdb.body.phi = -CHASSIS.fdb.body.roll;
    CHASSIS.fdb.body.phi_dot = -CHASSIS.fdb.body.roll_dot;
    
    //机体位移加速度
    CHASSIS.fdb.body.x_acc = CHASSIS.fdb.body.y_accel;
}


 /**
 * @brief  更新底盘电机数据
 * @param  none
 */


void UpdateMotorStatus(void)
{
    for (uint8_t i = 0; i < 4; i++) 
	  {
     GetMotorMeasure(&CHASSIS.joint_motor[i]);//获取关节电机数据
    }
    for (uint8_t i = 0; i < 2; i++) 
		{
		GetMotorMeasure(&CHASSIS.wheel_motor[i]);	//获取3508数据在  转速是rad/s
    }
}

/**
 * @brief  更新腿部状态·
 * @param  none
 */

float ddot_z_M = 0;
float l0 = 0;
float v_l0 = 0;
float theta = 0;
float w_theta = 0;

float dot_v_l0 = 0;
float dot_w_theta = 0;
float ddot_z_w = 0;
void UpdateLegStatus(void)
{
	  uint8_t i = 0;
  //正视
	//左腿   以关节中心建立坐标系 左关节(phi1) 水平 上转 -pi   ->  0    右关节(phi4) 水平上转  0  ->  -pi        
  //                                      水平 下转  pi ->  0                  水平下转   0  ->  pi
	CHASSIS.fdb.leg[0].joint.Phi1=theta_transform(CHASSIS.joint_motor[0].fdb.pos,J0_ANGLE_OFFSET,J0_DIRECTION,1);
	CHASSIS.fdb.leg[0].joint.Phi4 =theta_transform(CHASSIS.joint_motor[1].fdb.pos,J1_ANGLE_OFFSET,J1_DIRECTION, 1);
	//正视
  //右腿   以关节中心建立坐标系 左关节(phi4) 水平 上转 0  ->  -pi    右关节(phi1) 水平上转  -pi  ->  0        
  //                                      水平 下转 0 ->    pi                 水平下转   pi  ->  0
	CHASSIS.fdb.leg[1].joint.Phi1 =theta_transform(CHASSIS.joint_motor[2].fdb.pos, J2_ANGLE_OFFSET, J2_DIRECTION, 1);
  CHASSIS.fdb.leg[1].joint.Phi4 =theta_transform(CHASSIS.joint_motor[3].fdb.pos, J3_ANGLE_OFFSET, J3_DIRECTION, 1);
	
  CHASSIS.fdb.leg[0].joint.dPhi1 = CHASSIS.joint_motor[0].fdb.vel * (J0_DIRECTION);
  CHASSIS.fdb.leg[0].joint.dPhi4 = CHASSIS.joint_motor[1].fdb.vel * (J1_DIRECTION);
  CHASSIS.fdb.leg[1].joint.dPhi1 = CHASSIS.joint_motor[2].fdb.vel * (J2_DIRECTION);
  CHASSIS.fdb.leg[1].joint.dPhi4 = CHASSIS.joint_motor[3].fdb.vel * (J3_DIRECTION);

  // =====更新关节电机力矩=====
  CHASSIS.fdb.leg[0].joint.T1 = CHASSIS.joint_motor[0].fdb.tor * (J0_DIRECTION);
  CHASSIS.fdb.leg[0].joint.T2 = CHASSIS.joint_motor[1].fdb.tor * (J1_DIRECTION);
  CHASSIS.fdb.leg[1].joint.T1 = CHASSIS.joint_motor[2].fdb.tor * (J2_DIRECTION);
  CHASSIS.fdb.leg[1].joint.T2 = CHASSIS.joint_motor[3].fdb.tor * (J3_DIRECTION);
	
	// =====更新驱动轮角速度===== 反馈转子速度除以减速比
	CHASSIS.fdb.leg[0].wheel.Velocity = CHASSIS.wheel_motor[0].fdb.vel * (W0_DIRECTION)/CHASSIS.wheel_motor[0].reduction_ratio;//rad/s
  CHASSIS.fdb.leg[1].wheel.Velocity = CHASSIS.wheel_motor[1].fdb.vel * (W1_DIRECTION)/CHASSIS.wheel_motor[1].reduction_ratio;//rad/s
	

	// =====更新摆杆姿态=====
  float L0_Phi0[2];
  float dL0_dPhi0[2];

	for (i = 0; i < 2; i++) {
        float last_dL0 = CHASSIS.fdb.leg[i].rod.dL0;
        float last_dPhi0 = CHASSIS.fdb.leg[i].rod.dPhi0;
        float last_dTheta = CHASSIS.fdb.leg[i].rod.dTheta;
        
		    // 更新腿长和摆角位置信息  通过关节电机角度正解到L0和phi0
		    GetL0AndPhi0(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, L0_Phi0);
		    //如果不考虑关节电机的phi1和phi4 那么L0 和  phi0的关系就是
        //屁股往前看               phi0都是  0-->>pi/2-->>pi
        CHASSIS.fdb.leg[i].rod.L0 = L0_Phi0[0]  ;
        CHASSIS.fdb.leg[i].rod.Phi0 = L0_Phi0[1];
		
		    CHASSIS.fdb.leg[i].rod.Theta = M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;
				
				 // 计算雅可比矩阵  根据phi1和phi4计算出末端的雅可比矩阵J
        CalcJacobian(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, CHASSIS.fdb.leg[i].J);
		    
		     // 一阶微分   通过雅可比矩阵求解出l0的微分和phi0的微分 同时求解出theta的微分
        GetdL0AnddPhi0(CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.dPhi1, CHASSIS.fdb.leg[i].joint.dPhi4,dL0_dPhi0);
        
		    //计算得到弹簧前馈补偿力 
		    CHASSIS.fdb.leg[i].rod.F_spring=LegController_CalcSpringForce(CHASSIS.fdb.leg[0].rod.L0);
		
        CHASSIS.fdb.leg[i].rod.dL0 = dL0_dPhi0[0];
        CHASSIS.fdb.leg[i].rod.dPhi0 = dL0_dPhi0[1];
        CHASSIS.fdb.leg[i].rod.dTheta = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;
				CHASSIS.fdb.leg[i].rod.dTheta = CHASSIS.fdb.leg[i].rod.dTheta;
		
        // 更新加速度信息  一阶微分--->>二阶微分  
        float accel = (CHASSIS.fdb.leg[i].rod.dL0 - last_dL0) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddL0 = accel;

        accel = (CHASSIS.fdb.leg[i].rod.dPhi0 - last_dPhi0) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddPhi0 = accel;

        accel = (CHASSIS.fdb.leg[i].rod.dTheta - last_dTheta) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddTheta = accel;


        
		    // 计算支撑力 根据反馈的两个关节电机的力矩  以及计算得出的雅可比矩阵  可以得到等效摆杆地面的支持力 以及转动力
        float F[2];//F   T
        GetLegForce(CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.T1, CHASSIS.fdb.leg[i].joint.T2, F);
				
				CHASSIS.fdb.leg[i].rod.F=F[0];  
				CHASSIS.fdb.leg[i].rod.Tp=F[1];

				float F0 = F[0];
        float Tp = F[1];
				
				float P = F0 * cosf(theta) +( Tp * sinf(theta) / CHASSIS.fdb.leg[i].rod.L0)+CHASSIS.fdb.leg[i].rod.F_spring;

				
        CHASSIS.fdb.leg[i].Fn = P;

        if (CHASSIS.fdb.leg[i].Fn< TAKE_OFF_FN_THRESHOLD) {
            CHASSIS.fdb.leg[i].touch_time = 0;
            CHASSIS.fdb.leg[i].take_off_time += CHASSIS.duration;
        } else {
            CHASSIS.fdb.leg[i].touch_time += CHASSIS.duration;
            CHASSIS.fdb.leg[i].take_off_time = 0;
        }
    }
//	CHASSIS.fdb.two_leg_err=CHASSIS.fdb.leg[0].rod.Theta-CHASSIS.fdb.leg[1].rod.Theta;
}

/**
 * @brief  机体运动状态观测器
 * @param  none
 */
void BodyMotionObserve(void)
{
	   float speed = WHEEL_RADIUS * (CHASSIS.fdb.leg[0].wheel.Velocity + CHASSIS.fdb.leg[1].wheel.Velocity) / 2;
      CHASSIS.fdb.body.x_dot= speed;//原始速度

      // 对加速度进行低通滤波
      float filtered_acc = LowPassFilterCalc(&CHASSIS.lpf.x_acc_lpf, CHASSIS.fdb.body.x_acc);
      OBSERVER.body.v_kf.ControlVector[0] = filtered_acc;
      OBSERVER.body.v_kf.MeasuredVector[0] = CHASSIS.fdb.body.x;                   
      OBSERVER.body.v_kf.MeasuredVector[1] = speed;  
	
	    Kalman_Filter_Update(&OBSERVER.body.v_kf);
//      CHASSIS.fdb.body.x_obv = OBSERVER.body.v_kf.xhat_data[0];//滤波后的位移
//      CHASSIS.fdb.body.x_dot_obv= OBSERVER.body.v_kf.xhat_data[1];//滤波后的速度
    
    // 不加入卡曼滤波
     CHASSIS.fdb.body.x_dot_obv = speed;
     CHASSIS.fdb.body.x_acc_obv = CHASSIS.fdb.body.x_acc;
	
    if (fabs(CHASSIS.ref.speed_vector.vx) < WHEEL_DEADZONE) 
		{
        CHASSIS.fdb.body.x += CHASSIS.fdb.body.x_dot_obv * CHASSIS.duration * MS_TO_S;
    }
		else //有速度时候把反馈速度设置为0
		{
        CHASSIS.fdb.body.x = 0;
    }
		
		 // 更新2条腿的状态向量
     uint8_t i = 0;
     for (i = 0; i < 2; i++) {
        CHASSIS.fdb.leg_state[i].theta     =  M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;
        CHASSIS.fdb.leg_state[i].theta_dot = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;
        CHASSIS.fdb.leg_state[i].x         =  CHASSIS.fdb.body.x;
        CHASSIS.fdb.leg_state[i].x_dot     =  CHASSIS.fdb.body.x_dot_obv;
        CHASSIS.fdb.leg_state[i].phi       =  CHASSIS.fdb.body.phi;
        CHASSIS.fdb.leg_state[i].phi_dot   =  CHASSIS.fdb.body.phi_dot;
    }
}




#define StateTransfer()    \
    CHASSIS.step_time = 0; \
    CHASSIS.step = TRANSITION_MATRIX[CHASSIS.step];

static void UpdateStepStatus(void)
{
	
if (CHASSIS.mode == CHASSIS_FREE) {
        if (keyboard_data.Remote_Key_B==1) {  // 电脑键盘按下B键  跳跃
            CHASSIS.step_time = 0;
            CHASSIS.step = JUMP_STEP_SQUST;
        } else if (CHASSIS.step == JUMP_STEP_SQUST) {  // 跳跃——蹲下蓄力状态
            if (CHASSIS.fdb.leg[0].rod.L0 < MIN_LEG_LENGTH + 0.02f &&
                CHASSIS.fdb.leg[1].rod.L0 < MIN_LEG_LENGTH + 0.02f) {
                StateTransfer();
            }
        } else if (CHASSIS.step == JUMP_STEP_JUMP) {  // 跳跃——起跳状态
					
            if (CHASSIS.fdb.leg[0].rod.L0 > MAX_LEG_LENGTH  &&
                CHASSIS.fdb.leg[1].rod.L0 > MAX_LEG_LENGTH ) {
                StateTransfer();
            }
        } else if (CHASSIS.step == JUMP_STEP_RECOVERY) {  // 跳跃——收腿状态
            if (CHASSIS.step_time > 300) {               // 500ms后切换状态
                StateTransfer();
            }
        } else if (CHASSIS.step != NORMAL_STEP && CHASSIS.step_time > MAX_STEP_TIME) {
            // 状态持续时间超过 MAX_STEP_TIME ，自动切换到NORMAL状态
            CHASSIS.step_time = 0;
            CHASSIS.step = NORMAL_STEP;
        }
    } else {
        CHASSIS.step_time = 0;
        CHASSIS.step = NORMAL_STEP;
    }
  
}

#undef StateTransfer



 /**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
 float length = 0.135;//正常腿长
 ChassisSpeedVector_t target_v_set = {0.0f, 0.0f, 0.0f};

float vx_ramp_rate = 6.5f; // 每秒增加6.5m/s
float current_set_vx = 0.0f;//当前设置的速度
 void ChassisReference(void)
 {
	  int16_t rc_x = 0, rc_wz = 0;
    int16_t rc_length = 0, rc_angle = 0;
    float rc_pitch = 0;
    rc_deadband_limit(CHASSIS.rc->ch1, rc_x, CHASSIS_RC_DEADLINE);    //右竖直拨杆控制前进后退
    rc_deadband_limit(CHASSIS.rc->ch2, rc_wz, CHASSIS_RC_DEADLINE);   //右水平拨杆控制旋转
    rc_deadband_limit(CHASSIS.rc->ch4, rc_length, CHASSIS_RC_DEADLINE);
//  rc_deadband_limit(CHASSIS.rc->ch3, rc_pitch, CHASSIS_RC_PITCH_DEADLINE);
	 
    // 计算速度向量
	  target_v_set.vx = rc_x * RC_TO_ONE * MAX_SPEED_VECTOR_VX;
		target_v_set.vx +=keyboard_data.Remote_Key_W*2.5;
		target_v_set.vx -=keyboard_data.Remote_Key_S*2.5;
    if (target_v_set.vx > current_set_vx) {
    current_set_vx += vx_ramp_rate * CHASSIS.duration * MS_TO_S;
    if (current_set_vx > target_v_set.vx) {
        current_set_vx = target_v_set.vx;
    }
} else if (target_v_set.vx < current_set_vx) {
    current_set_vx -= vx_ramp_rate * CHASSIS.duration * MS_TO_S;
    if (current_set_vx < target_v_set.vx) {
        current_set_vx = target_v_set.vx;
    }
}

// 在ChassisReference函数中
static float wz_smooth = 0.0f;  // 平滑后的角速度
static float smooth_factor = 0.05f;  // 平滑因子，可根据实际情况调整

// 计算目标角速度
float target_wz;
float rc_wz_input = -rc_wz*RC_TO_ONE*MAX_SPEED_VECTOR_WZ;
if (fabs(rc_wz_input) > 0.001f) {
    // 小陀螺模式
    target_wz = rc_wz_input;
} else {
    // 云台跟随模式
#ifdef OPEN_CHASSIS_FOLLOW_GIMBAL
    float yaw_angle_diff = angle_difference(GIMBAL_DIRECT_YAW_MID, CHASSIS.fdb.gimbal.gimbal_yaw_6020);
		yaw_angle_diff=fp32_constrain(yaw_angle_diff,-M_PI*0.3,M_PI*0.3);
    float corrected_yaw_target = CHASSIS.fdb.gimbal.gimbal_yaw_6020 + yaw_angle_diff;
    target_wz = -PID_calc(&CHASSIS.pid.chassis_follow_gimbal, CHASSIS.fdb.gimbal.gimbal_yaw_6020, corrected_yaw_target);
#else
    target_wz = -rc_wz*RC_TO_ONE*MAX_SPEED_VECTOR_WZ;
#endif
}
// 平滑过渡
wz_smooth = wz_smooth * (1.0f - smooth_factor) + target_wz * smooth_factor;
target_v_set.wz = wz_smooth;



		

		
		switch (CHASSIS.mode) {
        case CHASSIS_FREE: {  // 底盘自由模式下，控制量为底盘坐标系下的速度
            CHASSIS.ref.speed_vector.vx = current_set_vx;
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = target_v_set.wz;
            break;
        }
				case CHASSIS_OFF_HOOK:
				{
					 CHASSIS.ref.speed_vector.vx = 0;
           CHASSIS.ref.speed_vector.vy = 0;
           CHASSIS.ref.speed_vector.wz = 0;
				}break;
        default:
            CHASSIS.ref.speed_vector.vx = 0;
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = 0;
            break;
    }
		
		// 计算期望状态
    for (uint8_t i = 0; i < 2; i++) {
        CHASSIS.ref.leg_state[i].theta     =  0;
        CHASSIS.ref.leg_state[i].theta_dot =  0;
        CHASSIS.ref.leg_state[i].x         =  0;
        CHASSIS.ref.leg_state[i].x_dot     =  CHASSIS.ref.speed_vector.vx;
        CHASSIS.ref.leg_state[i].phi       =  0;
        CHASSIS.ref.leg_state[i].phi_dot   =  0;
    }
		 // 腿部控制
    switch (CHASSIS.mode) {
				case CHASSIS_OFF_HOOK:
        case CHASSIS_FREE: {
				         length=length-rc_length*RC_TO_ONE*0.001f;
					  if (CHASSIS.step == JUMP_STEP_SQUST) 
            {
              length = MIN_LEG_LENGTH;
            } 
            else if (CHASSIS.step == JUMP_STEP_JUMP) 
            {
              length = MAX_LEG_LENGTH;
            } 
            else if (CHASSIS.step == JUMP_STEP_RECOVERY) 
            {
              length = MIN_LEG_LENGTH ;
            }
						else
						{
							
						}
						
        } break;
        default: {
            length = 0.14f;
        }
    }
    length = fp32_constrain(length, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
		

		 CHASSIS.ref.rod_L0[0] = length;
     CHASSIS.ref.rod_L0[1] = length;

    CHASSIS.ref.body.pitch = fp32_constrain(-rc_pitch * RC_TO_ONE * MAX_PITCH, MIN_PITCH, MAX_PITCH);
 }
static void ConsoleZeroForce(void);//零力控制
static void ConsoleNormal(void);//正常控制
static void ConsoleCalibrate(void);//校准控制
static void ConsoleStandUp(void);//起立控制
static void LocomotionController(void);
static void LegTorqueController(void);
static void Auto_standup(void);//智能收腿起立

 //计算控制量
 void ChassisConsole(void)
 {
 switch (CHASSIS.mode) {
	      case CHASSIS_FREE:
        {
          ConsoleNormal();//正常控制
        }break;
	      case CHASSIS_STAND_UP:
				{
          ConsoleNormal();//正常控制
				}break;
        case CHASSIS_CALIBRATE:
        {
          ConsoleCalibrate();//校准控制
        }break;
				case CHASSIS_OFF_HOOK:
				{
					Auto_standup();
				}break;
        case CHASSIS_OFF:
        case CHASSIS_SAFE:
        default: {

					ConsoleZeroForce();
        }
 }
 }
extern State StandUP_state_L;   
extern State StandUP_state_R;   

 
 static void Auto_standup(void)
 {
	 
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;
	  if(StandUP_state_L==STATE_stretchleg)
		{
	  CHASSIS.joint_motor[0].set.vel = Leg_Stretch_velocity;
    CHASSIS.joint_motor[1].set.vel = -Leg_Stretch_velocity;
		}
		if(StandUP_state_R==STATE_stretchleg)
		{
    CHASSIS.joint_motor[2].set.vel = -Leg_Stretch_velocity;
    CHASSIS.joint_motor[3].set.vel = Leg_Stretch_velocity;
		}
		
		if(StandUP_state_L==STATE_Backleg)
		{
		 CHASSIS.joint_motor[0].set.vel = -Leg_Stretch_velocity;
     CHASSIS.joint_motor[1].set.vel = -Leg_Stretch_velocity;
		}
		if(StandUP_state_R==STATE_Backleg)
		{
     CHASSIS.joint_motor[2].set.vel = Leg_Stretch_velocity;
     CHASSIS.joint_motor[3].set.vel = Leg_Stretch_velocity;
		}
		if(StandUP_state_L==STATE_COMPLETE)
		{
    CHASSIS.joint_motor[0].set.vel = -CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[1].set.vel = CALIBRATE_VELOCITY;

		}
		if(StandUP_state_R==STATE_COMPLETE)
		{
    CHASSIS.joint_motor[2].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[3].set.vel = -CALIBRATE_VELOCITY;
		}
	  CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
 }
static void ConsoleNormal(void)
{
    LocomotionController();
	//LQR 12个力矩输出  yaw轴转向环2个力矩输出
    LegTorqueController();
	//两个fn腿长PID力矩输出 防劈叉两个tp力矩输出

    // 给关节电机赋值
    CHASSIS.joint_motor[0].set.tor = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    CHASSIS.joint_motor[1].set.tor = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    CHASSIS.joint_motor[2].set.tor = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    CHASSIS.joint_motor[3].set.tor = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

     for (uint8_t i = 0; i < 4; i++) {
        if (CHASSIS.mode==CHASSIS_STAND_UP) {
            CHASSIS.joint_motor[i].set.tor = fp32_constrain(CHASSIS.joint_motor[i].set.tor,min_joint_tor_stand, max_joint_tor_stand);
        } 
				else 
					{
            CHASSIS.joint_motor[i].set.tor =fp32_constrain(CHASSIS.joint_motor[i].set.tor,min_joint_tor_move,max_joint_tor_move);
        }
    }

    CHASSIS.wheel_motor[0].set.tor = (CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION));
    CHASSIS.wheel_motor[1].set.tor = (CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION));

}
static void ConsoleZeroForce(void)
{
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;

    CHASSIS.joint_motor[0].set.vel = 0;
    CHASSIS.joint_motor[1].set.vel = 0;
    CHASSIS.joint_motor[2].set.vel = 0;
    CHASSIS.joint_motor[3].set.vel = 0;

    CHASSIS.wheel_motor[0].set.vel = 0;
    CHASSIS.wheel_motor[1].set.vel = 0;

    CHASSIS.wheel_motor[0].set.value =PID_calc(&CHASSIS.pid.wheel_stop[0], CHASSIS.wheel_motor[0].fdb.vel, 0);
    CHASSIS.wheel_motor[1].set.value =PID_calc(&CHASSIS.pid.wheel_stop[1], CHASSIS.wheel_motor[1].fdb.vel, 0);
}

static void ConsoleCalibrate(void)
{   
    CHASSIS.joint_motor[0].set.vel = -CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[1].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[2].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[3].set.vel = -CALIBRATE_VELOCITY;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
}


/**
 * @brief         矩阵相乘，计算LQR输出
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    T_Tp 反馈数据T和Tp
 * @return        none
 */

static void CalcLQR(float k[2][6], float x[6], float T_Tp[2])
{   
	  CHASSIS.lqr_out.wheel_theta=k[0][0]*x[0];
	  CHASSIS.lqr_out.wheel_theta_dot=k[0][1]*x[1];
		CHASSIS.lqr_out.wheel_x=k[0][2]*x[2];	  
		CHASSIS.lqr_out.wheel_vel=k[0][3]*x[3];
		CHASSIS.lqr_out.wheel_phi=k[0][4];
		CHASSIS.lqr_out.wheel_phi_dot=k[0][5]*x[5];
	  CHASSIS.lqr_out.joint_theta=k[1][0]*x[0];
		CHASSIS.lqr_out.joint_theta_dot=k[1][1]*x[1];
	  CHASSIS.lqr_out.joint_x=k[1][2]*x[2];
	  CHASSIS.lqr_out.joint_vel=k[1][3]*x[3];
	  CHASSIS.lqr_out.joint_phi=k[1][4]*x[4];
	  CHASSIS.lqr_out.joint_phi_dot=k[1][5]*x[5];

    T_Tp[0] = k[0][0]*x[0]+k[0][1]*x[1]+k[0][2]*x[2]+k[0][3]*x[3]+k[0][4]*x[4]+k[0][5]*x[5];
    T_Tp[1] = k[1][0]*x[0]+k[1][1]*x[1]+k[1][2]*x[2]+k[1][3]*x[3]+k[1][4]*x[4]+k[1][5]*x[5];
}


fp32 x0_OFFSET=0.0f;   // 目标theta偏移量
fp32 x1_OFFSET=0.0f;   // 目标theta_dot偏移量
fp32 x2_OFFSET=0.0f; // 目标x偏移量
fp32 x3_OFFSET=0.0f;   // 目标x_dot偏移量
fp32 x4_OFFSET=0.00f;   // 目标phi偏移量
fp32 x5_OFFSET=0.0f;   // 目标phi_dot偏移量
float Ld0=0.0;
float L_diff=0.0;
static void LocomotionController(void)
{
	    // 计算LQR增益
    float k[2][6];
    float x[6];
    float T_Tp[2];
    


    for (uint8_t i = 0; i < 2; i++) {
        GetK(CHASSIS.fdb.leg[i].rod.L0,k,CHASSIS.fdb.leg[i].is_take_off);

        x[0] = (x0_OFFSET + (CHASSIS.fdb.leg_state[i].theta     - CHASSIS.ref.leg_state[i].theta));
        x[1] = (x1_OFFSET + (CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.ref.leg_state[i].theta_dot));
        x[2] = x2_OFFSET + (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x); 
        x[3] = x3_OFFSET + (CHASSIS.fdb.leg_state[i].x_dot     - CHASSIS.ref.leg_state[i].x_dot);
        x[4] = x4_OFFSET + (CHASSIS.fdb.leg_state[i].phi       - CHASSIS.ref.leg_state[i].phi);
        x[5] = x5_OFFSET + (CHASSIS.fdb.leg_state[i].phi_dot   - CHASSIS.ref.leg_state[i].phi_dot);

        CalcLQR(k, x, T_Tp);
			  
				CHASSIS.cmd.leg[i].wheel.T = T_Tp[0];
        CHASSIS.cmd.leg[i].rod.Tp = T_Tp[1];


    }

//转向控制================================================
//#ifdef OPEN_CHASSIS_FOLLOW_GIMBAL
//		
//		  float yaw_angle_diff = angle_difference(GIMBAL_DIRECT_YAW_MID,CHASSIS.fdb.gimbal.gimbal_yaw_6020);
//		  float corrected_yaw_target =  CHASSIS.fdb.gimbal.gimbal_yaw_6020 + yaw_angle_diff;
//		  PID_calc(&CHASSIS.pid.chassis_follow_gimbal,CHASSIS.fdb.gimbal.gimbal_yaw_6020,corrected_yaw_target);
//			CHASSIS.cmd.leg[0].wheel.T += CHASSIS.pid.chassis_follow_gimbal.out;
//      CHASSIS.cmd.leg[1].wheel.T -= CHASSIS.pid.chassis_follow_gimbal.out;
//#endif

			PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
		  CHASSIS.cmd.leg[0].wheel.T -= CHASSIS.pid.yaw_velocity.out;
      CHASSIS.cmd.leg[1].wheel.T += CHASSIS.pid.yaw_velocity.out;


  
		    for (uint8_t i = 0; i < 2; i++) 
				{
					if(CHASSIS.fdb.leg[i].is_take_off)
					{
						CHASSIS.cmd.leg[i].wheel.T = 0;
					}
				}
		
		


    
}
/**
 * @brief 腿部力矩控制
 */
float F0, F_leg;
static void LegTorqueController(void)
{
	  if(CHASSIS.mode==CHASSIS_FREE)
		{
			
						for (uint8_t i = 0; i < 2; i++) 
						{
							if (CHASSIS.step == JUMP_STEP_JUMP) 
							{
							// 直接给一个超大力F起飞
							CHASSIS.cmd.leg[i].rod.F = 130;
							}
				  		else if(CHASSIS.step==JUMP_STEP_RECOVERY)
							{
								F0 =50;
								F_leg = PID_calc(&CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0,CHASSIS.ref.rod_L0[i]);
								CHASSIS.cmd.leg[i].rod.F =F0+F_leg- CHASSIS.fdb.leg[i].rod.F_spring; 
							}
							else if(CHASSIS.fdb.leg[0].is_take_off&&CHASSIS.fdb.leg[0].is_take_off)
							{
								F0=0;
								F_leg = PID_calc(&CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0,0.23);
								CHASSIS.cmd.leg[i].rod.F = F0 + F_leg- CHASSIS.fdb.leg[i].rod.F_spring; 
							}
							else
							{				
								F0 =BODY_MASS_FN;
								F_leg = PID_calc(&CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0,CHASSIS.ref.rod_L0[i]);
								CHASSIS.cmd.leg[i].rod.F = F0 + F_leg- CHASSIS.fdb.leg[i].rod.F_spring; 
							}
						}		
					//横滚角稳定
					PID_calc(&CHASSIS.pid.pitch_angle,CHASSIS.fdb.body.pitch,CHASSIS.ref.body.pitch);//右边抬起是pitch增大
					if(CHASSIS.fdb.leg[0].is_take_off)
		      {
						CHASSIS.pid.pitch_angle.out=0;
		      }
					if(CHASSIS.fdb.leg[1].is_take_off)
		      {
						CHASSIS.pid.pitch_angle.out=0;
		      }

					
					if(CHASSIS.fdb.leg[0].is_take_off&&CHASSIS.fdb.leg[1].is_take_off)
					{
						
					}
					CHASSIS.cmd.leg[0].rod.F-=CHASSIS.pid.pitch_angle.out;
					CHASSIS.cmd.leg[1].rod.F+=CHASSIS.pid.pitch_angle.out;
					
					//防劈叉  TP
					PID_calc(&CHASSIS.pid.leg_chase_L_to_R,CHASSIS.fdb.leg[0].rod.Theta,CHASSIS.fdb.leg[1].rod.Theta);
          CHASSIS.cmd.leg[0].rod.Tp-=CHASSIS.pid.leg_chase_L_to_R.out;
          PID_calc(&CHASSIS.pid.leg_chase_R_to_L,CHASSIS.fdb.leg[1].rod.Theta,CHASSIS.fdb.leg[0].rod.Theta);
          CHASSIS.cmd.leg[1].rod.Tp-=CHASSIS.pid.leg_chase_R_to_L.out;

		}

		

		/*====================起立态====================*/
		if(CHASSIS.mode==CHASSIS_STAND_UP)
		{
						for (uint8_t i = 0; i < 2; i++) 
					 {
            // 计算前馈力				
						F0=0;
						F_leg = PID_calc(&CHASSIS.pid.stand_up, CHASSIS.fdb.leg[i].rod.L0,CHASSIS.ref.rod_L0[i]);
						CHASSIS.cmd.leg[i].rod.F = F0 + F_leg;
					 }						 
		}

		
		
    // 转换为关节力矩  杆支持力     杆的旋转力     雅可比矩阵   ---->>>两个关节的需要力矩
    CalcVmc(CHASSIS.cmd.leg[0].rod.F, CHASSIS.cmd.leg[0].rod.Tp, CHASSIS.fdb.leg[0].J,CHASSIS.cmd.leg[0].joint.T);
    CalcVmc(CHASSIS.cmd.leg[1].rod.F, CHASSIS.cmd.leg[1].rod.Tp, CHASSIS.fdb.leg[1].J,CHASSIS.cmd.leg[1].joint.T);
}

#define DM_DELAY 250 
#define DEBUG_KP 17
#define DEBUG_KD 2
 
void SendJointMotorCmd(void);
void SendWheelMotorCmd(void);
 //发送控制量
 void ChassisSendCmd(void)
 {
	      SendJointMotorCmd();
        SendWheelMotorCmd();
 }

 /**
 * @brief 发送关节电机控制指令
 * @param[in] chassis
 */
 
 void SendJointMotorCmd(void)
 {
			switch (CHASSIS.mode)
			{ 	
        case CHASSIS_STAND_UP: 
				case CHASSIS_FREE:  //底盘自由模式
				{
					DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
					osDelay(1);
          DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
					osDelay(1);
          DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
					osDelay(1);
          DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
				}break;
				case CHASSIS_OFF_HOOK:  //底盘脱困模式
				{
          DmMitCtrlVelocity(&CHASSIS.joint_motor[0], STAND_UP_VEL_KD);
          osDelay(1);
          DmMitCtrlVelocity(&CHASSIS.joint_motor[1], STAND_UP_VEL_KD);
          osDelay(1);
          DmMitCtrlVelocity(&CHASSIS.joint_motor[2], STAND_UP_VEL_KD);
          osDelay(1);
          DmMitCtrlVelocity(&CHASSIS.joint_motor[3], STAND_UP_VEL_KD);

				}break;
        case CHASSIS_CALIBRATE:  //底盘校准模式
        {
          DmMitCtrlVelocity(&CHASSIS.joint_motor[0], CALIBRATE_VEL_KD);
          osDelay(1);
          DmMitCtrlVelocity(&CHASSIS.joint_motor[1], CALIBRATE_VEL_KD);
          osDelay(1);
          DmMitCtrlVelocity(&CHASSIS.joint_motor[2], CALIBRATE_VEL_KD);
          osDelay(1);
          DmMitCtrlVelocity(&CHASSIS.joint_motor[3], CALIBRATE_VEL_KD);
          // 校准完成  车体平放在地上 两腿放置在后方
          if (CALIBRATE.reached[0] && CALIBRATE.reached[1] && CALIBRATE.reached[2] &&CALIBRATE.reached[3]) 
          {
          DmSavePosZero(&CHASSIS.joint_motor[0]);
          osDelay(1);
          DmSavePosZero(&CHASSIS.joint_motor[1]);
          osDelay(1);
          DmSavePosZero(&CHASSIS.joint_motor[2]);
          osDelay(1);
          DmSavePosZero(&CHASSIS.joint_motor[3]);
          }
        }break;
				default:    //底盘其他模式
				{
					CHASSIS.joint_motor[0].set.tor=0;
					CHASSIS.joint_motor[1].set.tor=0;			
					CHASSIS.joint_motor[2].set.tor=0;				
					CHASSIS.joint_motor[3].set.tor=0;
					DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
					osDelay(1);
          DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
					osDelay(1);
          DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
					osDelay(1);
          DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
				}
			}
 }
 /**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
 
 fp32 current_to_torque=0.00042063763;
 void SendWheelMotorCmd(void)
 {
	     switch (CHASSIS.mode) {
				 case CHASSIS_FREE: 
					 {
              CHASSIS.wheel_motor[0].set.curr=CHASSIS.wheel_motor[0].set.tor/current_to_torque;
              CHASSIS.wheel_motor[1].set.curr=CHASSIS.wheel_motor[1].set.tor/current_to_torque;
              //扭矩转化为电流
              CanCmdDjiMotor(&hfdcan2,0x200,CHASSIS.wheel_motor[0].set.curr,CHASSIS.wheel_motor[1].set.curr,0,0);
					
            } break;
				 default:
				 {
				      CanCmdDjiMotor(&hfdcan2,0x200,0,0,0,0);
				 }
				 
			 }
 }
	 
void ChassisHandleException()
{
  if(CHASSIS.joint_motor[0].fdb.state==0||CHASSIS.joint_motor[1].fdb.state==0||CHASSIS.joint_motor[2].fdb.state==0||CHASSIS.joint_motor[3].fdb.state==0)
	{
    DmEnable(&CHASSIS.joint_motor[0]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[1]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[2]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[3]);
	} 

  if (CHASSIS.mode==CHASSIS_FREE&&CHASSIS.last_mode!=CHASSIS_FREE)
  {
    memset(&CHASSIS.fdb,0,sizeof(CHASSIS.fdb));
    CHASSIS.fdb.leg[0].is_take_off = false;
    CHASSIS.fdb.leg[1].is_take_off = false;
  }
  
	CHASSIS.last_mode=CHASSIS.mode;    


}




