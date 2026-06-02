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
#include "TOF_distance.h"
#include "chassis_power_control.h"
#define CHASSIS_TASK_INIT_TIME 300
#define CHASSIS_CONTROL_TIME_MS 3  //底盘控制周期3ms
#define MS_TO_S 0.001f
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
extern TOF_data_t tof_data;
extern LossControlDetector leg_loss_control[2]; // 左右腿失控检测器
	//控制超级电容启用与重启
typedef enum 
{
 SuperCapOFF = 0X00,
 SuperCapON = 0X01,
 SuperCapRESET = 0X02,
}SuperCap_mode;
				
//声明SuperCap的数据发送结构体
SuperCap_tx_t SuperCap_tx;
extern SuperCap_rx_t SuperCap_rx;

CTOM_message_t ctom_message;
//机器人结构体
Chassis_s CHASSIS = {
	.mode=CHASSIS_SAFE
};
fp32 BODY_MASS_FN=(100.0f);      // 前馈抵消重量    N
Observer_t OBSERVER;
bool is_auto_climbing;
uint16_t climb_state;
Calibrate_s CALIBRATE = {
    .cali_cnt = 0,
    .velocity = {0.0f, 0.0f, 0.0f, 0.0f},
    .stpo_time = {0, 0, 0, 0},
    .reached = {false, false, false, false},
    .calibrated = false,
};
fp32 move_scale=0.0f;
int8_t TRANSITION_MATRIX[10] = {0};
 void ChassisInit(void);
 void ChassisHandleException(void);

 void ChassisSetMode(void);
void ChassisObserver(void);
 void ChassisReference(void);
 void ChassisConsole(void);
 void ChassisSendCmd(void);
 void ChassisHandleException(void);
static void Auto_ClimbStep(void);
void chassis_task(void const * pvParamewwters)
{
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // 初始化底盘
    ChassisInit();
    while (1) {
        // 更新状态量**************************************************
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
        osDelay(1);
			  
    }
}


 void ChassisInit(void)
 {
	 	CHASSIS.rc = get_remote_control_point();  //获取遥控器指针 
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
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[0], LEG_SUPPORT_FORCE_LPF_ALPHA);//支持力滤波
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[1], LEG_SUPPORT_FORCE_LPF_ALPHA);
	  LowPassFilterInit(&CHASSIS.lpf.VX_filter, 0.9);//键鼠速度滤波
    // 初始化加速度低通滤波器
    LowPassFilterInit(&CHASSIS.lpf.x_acc_lpf, 0.9);//可根据需要调整alpha值
		
		LowPassFilterInit(&CHASSIS.lpf.dtheta,0.4);
    // 初始化机体速度观测器
		Kalman_Filter_Init(&OBSERVER.body.v_kf, 2, 0, 2);
		float F[4] = {1, 0.005, 0, 1}; // 状态转移矩阵不变（采样时间0.005s）
		// 1. 减小过程噪声：更信任模型预测，降低波动
		float Q[4] = {0.01f, 0, 0, 0.005f}; // 从0.05/0.02→0.01/0.005，大幅降低模型噪声权重
		// 2. 增大观测噪声：过滤传感器尖峰噪声，更不信任传感器异常读数
		float R[4] = {8.0f, 0, 0, 3.0f};   // 从1.0/0.0→8.0/3.0，提高速度观测噪声惩罚，彻底过滤尖峰
		// 3. 观测矩阵不变：仅观测速度
		float H[4] = {1, 0, 0, 0};         
		// 4. 减小初始协方差：让滤波快速收敛，避免初始波动
		float P[4] = {10.0f, 0, 0, 10.0f};  // 从100000→10，初始阶段更信任模型
    memcpy(OBSERVER.body.v_kf.F_data, F, sizeof(F));
    memcpy(OBSERVER.body.v_kf.P_data, P, sizeof(P));
    memcpy(OBSERVER.body.v_kf.Q_data, Q, sizeof(Q));
    memcpy(OBSERVER.body.v_kf.R_data, R, sizeof(R));
    memcpy(OBSERVER.body.v_kf.H_data, H, sizeof(H));
		


		//初始化发送值
		SuperCap_tx.SuperCap_mode = SuperCapON;
		SuperCap_tx.feedbackRefereeEnergyBuffer = 60;
		SuperCap_tx.feedbackRefereePowerLimit = 45;
		
 }

 //大疆遥控器设置模式
 void ChassisSetMode(void)
 {
	  if(switch_is_down(CHASSIS.rc->sw2)&&switch_is_down(CHASSIS.rc->sw1))
		{
        CHASSIS.mode = CHASSIS_SAFE; //双下无力安全模式
    } 
	  else if ((switch_is_mid(CHASSIS.rc->sw2)&&switch_is_down(CHASSIS.rc->sw1))||keyboard_data.Remote_Key_R==1) 
		{
        CHASSIS.mode = CHASSIS_STAND_UP;  // 左中右下，底盘起立，从倒地状态到站立状态的中间过程左中右下
    }
		else if (switch_is_up(CHASSIS.rc->sw2)&&switch_is_down(CHASSIS.rc->sw1)) 
		{
        CHASSIS.mode =CHASSIS_FREE;       //左上右下   底盘自由移动  
		}
		else if (switch_is_down(CHASSIS.rc->sw2)&&switch_is_mid(CHASSIS.rc->sw1)) 
		{
        CHASSIS.mode =CHASSIS_selfon;       //左上右下   底盘自由移动  
		}
    else 
    {
        CHASSIS.mode = CHASSIS_SAFE; // 默认为安全模式
    }	

#if TAKE_OFF_DETECT
    // 离地状态切换
//		if(CHASSIS.rc->ch3==-660)
//		{
			    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.fdb.leg[i].is_take_off &&CHASSIS.fdb.leg[i].touch_time > TOUCH_TOGGLE_THRESHOLD) 
					{ 
            CHASSIS.fdb.leg[i].is_take_off = false;

						CHASSIS.pid.leg_length_length[i].max_out=170;



          } 
						else if (!CHASSIS.fdb.leg[i].is_take_off &&CHASSIS.fdb.leg[i].take_off_time > TOUCH_TOGGLE_THRESHOLD) 
					{
            CHASSIS.fdb.leg[i].is_take_off = true;
						CHASSIS.pid.leg_length_length[i].max_out=120;


          }
          } 
//		}
//		else
//		{ 
//				CHASSIS.fdb.leg[0].is_take_off = false;
//				CHASSIS.fdb.leg[1].is_take_off = false;

//		}
    
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
}

 static void UpdateGimbalStatus(void)
 {
	   CHASSIS.fdb.gimbal.gimbal_yaw_6020=ctom_message.gimbal_yaw_6020;
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
    CHASSIS.fdb.body.phi = -CHASSIS.fdb.body.roll;//机体pitch
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
		    CHASSIS.fdb.leg[i].rod.F_spring=LegController_CalcSpringForce(CHASSIS.fdb.leg[i].rod.L0);
		
        CHASSIS.fdb.leg[i].rod.dL0 = dL0_dPhi0[0];
        CHASSIS.fdb.leg[i].rod.dPhi0 = dL0_dPhi0[1];
        CHASSIS.fdb.leg[i].rod.dTheta = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;
				CHASSIS.fdb.leg[i].rod.dTheta = LowPassFilterCalc(&CHASSIS.lpf.dtheta, CHASSIS.fdb.leg[i].rod.dTheta);

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
        } 
				else {
            CHASSIS.fdb.leg[i].touch_time += CHASSIS.duration;
            CHASSIS.fdb.leg[i].take_off_time = 0;
        }
    }


}
/**
 * @brief  机体运动状态观测器
 * @param  none
 */
void BodyMotionObserve(void)
{
	   float left_leg_wheelspeed = WHEEL_RADIUS *(CHASSIS.fdb.leg[0].wheel.Velocity-CHASSIS.fdb.leg[0].rod.dTheta+CHASSIS.fdb. body.phi_dot)
																+CHASSIS.fdb.leg[0].rod.L0*CHASSIS.fdb.leg[0].rod.dTheta*arm_cos_f32(CHASSIS.fdb.leg[0].rod.Theta)
																+CHASSIS.fdb.leg[0].rod.dL0*arm_sin_f32(CHASSIS.fdb.leg[0].rod.Theta);
		 
		 float right_leg_wheelspeed = WHEEL_RADIUS *(CHASSIS.fdb.leg[1].wheel.Velocity-CHASSIS.fdb.leg[1].rod.dTheta+CHASSIS.fdb.body.phi_dot)
																+CHASSIS.fdb.leg[1].rod.L0*CHASSIS.fdb.leg[1].rod.dTheta*arm_cos_f32(CHASSIS.fdb.leg[1].rod.Theta)
																+CHASSIS.fdb.leg[1].rod.dL0*arm_sin_f32(CHASSIS.fdb.leg[1].rod.Theta);
			
		 float speed=(left_leg_wheelspeed+right_leg_wheelspeed)/2.0f;

			CHASSIS.fdb.body.x_dot= speed;//原始速度

      // 对加速度进行低通滤波
      float filtered_acc = LowPassFilterCalc(&CHASSIS.lpf.x_acc_lpf, CHASSIS.fdb.body.x_acc);

		 
		 // 使用kf同时估计加速度和速度,滤波更新
				OBSERVER.body.v_kf.MeasuredVector[0] = speed;                   // 输入轮速
				OBSERVER.body.v_kf.MeasuredVector[1] = CHASSIS.lpf.x_acc_lpf.out;  // 输入加速度
				OBSERVER.body.v_kf.F_data[1] = CHASSIS.duration * MS_TO_S;      // 更新采样时间
				Kalman_Filter_Update(&OBSERVER.body.v_kf);
//		 
		  	CHASSIS.fdb.body.x_dot_obv = OBSERVER.body.v_kf.xhat_data[0];//滤波后的速度
			  CHASSIS.fdb.body.x_acc_obv= OBSERVER.body.v_kf.xhat_data[1];//滤波后的·加速度
			  
				// CHASSIS.fdb.body.x_dot_obv = speed;
				// CHASSIS.fdb.body.x_acc_obv = CHASSIS.fdb.body.x_acc;
	
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
        CHASSIS.fdb.leg_state[i].theta     =  CHASSIS.fdb.leg[i].rod.Theta;
        CHASSIS.fdb.leg_state[i].theta_dot = CHASSIS.fdb.leg[i].rod.dTheta;
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
    
//    / 1. 获取双腿平均摆角和角速度
        float avg_theta = (CHASSIS.fdb.leg_state[0].theta + CHASSIS.fdb.leg_state[1].theta) / 2.0f;
        float avg_theta_dot = (CHASSIS.fdb.leg_state[0].theta_dot + CHASSIS.fdb.leg_state[1].theta_dot) / 2.0f;
        // 2. 获取机体 X 轴大地加速度
//        float ax = CHASSIS.fdb.world.x_accel; 


        if (!is_auto_climbing) 
        {
				if ((avg_theta > 0.22f && avg_theta_dot > 0.12f)&&((CHASSIS.rc->ch2>=100)||(keyboard_data.Remote_Key_D==1)||(CHASSIS.fdb.leg[0].rod.L0+CHASSIS.fdb.leg[1].rod.L0)>=0.7)) {
                is_auto_climbing = true; // 
        }
        // 添加2秒冷却机制，防止频繁触发跳跃
     static uint32_t last_jump_time = 0;
     uint32_t current_time = HAL_GetTick();
     
     if ((keyboard_data.Remote_Key_B==1) && 
         (current_time - last_jump_time > 2000)){  // 电脑键盘按下B键  删跳跃//(CHASSIS.rc->ch2<=-200) 
             CHASSIS.step_time = 0; 
             CHASSIS.step = JUMP_STEP_SQUST; 
             last_jump_time = current_time; // 更新最后一次跳跃时间
         } else if (CHASSIS.step == JUMP_STEP_SQUST) {  // 跳跃——蹲下蓄力状态 
             if (CHASSIS.fdb.leg[0].rod.L0 < MIN_LEG_LENGTH + 0.02f && 
                 CHASSIS.fdb.leg[1].rod.L0 < MIN_LEG_LENGTH + 0.02f) { 
                 StateTransfer(); 
             } 
         } else if (CHASSIS.step == JUMP_STEP_JUMP) {  // 跳跃——起跳状态 
  				   
             if (CHASSIS.fdb.leg[0].rod.L0 > MAX_LEG_LENGTH-0.03  && 
                 CHASSIS.fdb.leg[1].rod.L0 > MAX_LEG_LENGTH-0.03 ) { 
                 StateTransfer(); 
             }  
         } else if (CHASSIS.step == JUMP_STEP_RECOVERY) {  // 跳跃——收腿状态 
             if (CHASSIS.step_time > 300) {               // 300ms后切换状态 
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
 }


#undef StateTransfer 
//----------------------有功率控制---------------------------------------------------------------------------------
  float length = 0.135;//正常腿长
  ChassisSpeedVector_t target_v_set = {0.0f, 0.0f, 0.0f};
 float vx_ramp_rate = 0.05f; 
 float current_vx = 0.0f;
 float target_wz=0.0f,current_wz = 0.0f,wz_ramp_rate = 15.5f;
static void Auto_ClimbStep(void)
{

    static uint32_t climb_timer = 0;
    static float current_target_Phi0[2] = {0.0f, 0.0f}; 
    static float current_target_L0[2] = {0.145f, 0.145f}; 
    static float initial_Phi0[2] = {0.9f, 0.9f}; 

    // 【核心修改】：通过标志位的边沿触发初始化，而不是 mode
    static bool last_is_auto_climbing = false;
    float dt = CHASSIS.duration * 0.001f; 

		if (climb_state == 0) {
			climb_state = 1;
			climb_timer = 0;

			// 记录原始物m理摆角
			initial_Phi0[0] = CHASSIS.fdb.leg[0].rod.Phi0;
			initial_Phi0[1] = CHASSIS.fdb.leg[1].rod.Phi0;

			// 斜坡起点对齐
			current_target_Phi0[0] = initial_Phi0[0];
			current_target_Phi0[1] = initial_Phi0[1];

			current_target_L0[0] = CHASSIS.fdb.leg[0].rod.L0;
			current_target_L0[1] = CHASSIS.fdb.leg[1].rod.L0;
    }

    climb_timer += CHASSIS.duration;

    float swing_speed = 5.0f;   // 摆角速度
    float retract_speed = 1.0f; // 缩腿速度
    float step_phi = swing_speed * dt;
    float step_L0 = retract_speed * dt;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;

    switch (climb_state) {
        // --- 阶段 1：向后摆腿 ---
        case 1: 
        {
            float final_target_Phi0 = 0.3f; 
            bool is_phi_reached = true;     
            for (uint8_t i = 0; i < 2; i++) {
                if (current_target_Phi0[i] < final_target_Phi0 - step_phi) {
                    current_target_Phi0[i] += step_phi; is_phi_reached = false; 
                } else if (current_target_Phi0[i] > final_target_Phi0 + step_phi) {
                    current_target_Phi0[i] -= step_phi; is_phi_reached = false;
                } else {
                    current_target_Phi0[i] = final_target_Phi0;
                }
            }
            if (is_phi_reached || climb_timer > 2000) {
                climb_state = 2; climb_timer = 0;
            }
            CHASSIS.wheel_motor[0].set.tor = -0.0f * W0_DIRECTION;
            CHASSIS.wheel_motor[1].set.tor = -0.0f * W1_DIRECTION;
        } break;

        // --- 阶段 2：缩腿到最短 ---
        case 2: 
        {
            float final_target_L0 = MIN_LEG_LENGTH; 
            bool is_L0_reached = true;              
            for (uint8_t i = 0; i < 2; i++) {
                if (current_target_L0[i] < final_target_L0 - step_L0) {
                    current_target_L0[i] += step_L0; is_L0_reached = false;
                } else if (current_target_L0[i] > final_target_L0 + step_L0) {
                    current_target_L0[i] -= step_L0; is_L0_reached = false;
                } else {
                    current_target_L0[i] = final_target_L0;
                }
            }
            if (is_L0_reached || climb_timer > 2000) {
                climb_state = 3; climb_timer = 0;
            }
            CHASSIS.wheel_motor[0].set.tor = -0.0f * W0_DIRECTION;
            CHASSIS.wheel_motor[1].set.tor = -0.0f * W1_DIRECTION;
        } break;

        // --- 阶段 3：摆角恢复，轮子拉起车身 ---
				case 3:
        {
            bool is_phi_reached = true; 

            for (uint8_t i = 0; i < 2; i++) {
                // 【核心修复】：不要使用撞击时的 initial_Phi0，直接让腿恢复到垂直向下 M_PI_2！
                // 这样当控制权还给 LQR 时，机器人处于最完美的平衡受力姿态！
                float final_target_Phi0 = M_PI_2; 

                if (current_target_Phi0[i] < final_target_Phi0 - step_phi) {
                    current_target_Phi0[i] += step_phi;
                    is_phi_reached = false;
                } else if (current_target_Phi0[i] > final_target_Phi0 + step_phi) {
                    current_target_Phi0[i] -= step_phi;  
                    is_phi_reached = false;
                } else {
                    current_target_Phi0[i] = final_target_Phi0;
                }
            }

            // 保持正向驱动力把底盘“拽”上台阶
            CHASSIS.wheel_motor[0].set.tor = -0.2f * W0_DIRECTION;
            CHASSIS.wheel_motor[1].set.tor = -0.2f * W1_DIRECTION;

            if (is_phi_reached || climb_timer > 2000) {
                climb_state = 4; 
                climb_timer = 0;
            }
        } break;

        // ----------------------------------------------------
        // 阶段 4：跨越完成，自动退回自由模式
        // ----------------------------------------------------
        case 4:
        {
            // 【核心修改】：自动退出并清理 VMC 积分！
            extern bool is_auto_climbing; // 引用你刚刚加的全局标志位
           
            is_auto_climbing = false; 
						is_auto_climbing = 0;
            climb_state = 0;

            // 清理 VMC 的控制积攒，防止交接给 LQR 瞬间产生抽搐
            PID_clear(&CHASSIS.pid.leg_length_length[0]);
            PID_clear(&CHASSIS.pid.leg_length_length[1]);
            length = 0.135;//正常腿长
            // 立即返回，本帧不再发力，下一帧无缝衔接 ConsoleNormal() 的 LQR
            return; 
        } break;
    }

    // --- 底层 VMC 独立闭环控制 ---
    for (uint8_t i = 0; i < 2; i++) {
        float F_compensate = PID_calc(&CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0, current_target_L0[i]);
        float F_spring = LegController_CalcSpringForce(CHASSIS.fdb.leg[i].rod.L0);
        float F_motor = F_compensate - F_spring; 

        float err_phi = current_target_Phi0[i] - CHASSIS.fdb.leg[i].rod.Phi0;
        float d_err_phi = 0.0f - CHASSIS.fdb.leg[i].rod.dPhi0;
        float kp_phi = 60.0f, kd_phi = 4.6;  
        float Tp = kp_phi * err_phi + kd_phi * d_err_phi;

        CHASSIS.cmd.leg[i].rod.F = F_motor;
        CHASSIS.cmd.leg[i].rod.Tp = Tp;

        CalcVmc(F_motor, Tp, CHASSIS.fdb.leg[i].J, CHASSIS.cmd.leg[i].joint.T);

        CHASSIS.cmd.leg[i].joint.T[0] = fp32_constrain(CHASSIS.cmd.leg[i].joint.T[0], -40, 40);
        CHASSIS.cmd.leg[i].joint.T[1] = fp32_constrain(CHASSIS.cmd.leg[i].joint.T[1], -40, 40);

        CHASSIS.joint_motor[i*2].set.tor   = CHASSIS.cmd.leg[i].joint.T[0] * (i==0 ? J0_DIRECTION : J2_DIRECTION);
        CHASSIS.joint_motor[i*2+1].set.tor = CHASSIS.cmd.leg[i].joint.T[1] * (i==0 ? J1_DIRECTION : J3_DIRECTION);
    }
}


static float gyro_angle = 0.0f;
#define GYRO_MOVE_AMPLITUDE    0.6f

  void ChassisReference(void)
  {
 	  int16_t rc_x = 0, rc_wz = 0;
    int16_t rc_length = 0, rc_angle = 0,rc_follow_gimbal=0;
    float rc_pitch = 0;
    rc_deadband_limit(CHASSIS.rc->ch1, rc_x, CHASSIS_RC_DEADLINE);    //右竖直拨杆控制前进后退
    rc_deadband_limit(CHASSIS.rc->ch4, rc_length, CHASSIS_RC_DEADLINE);
		rc_deadband_limit(CHASSIS.rc->ch0, rc_follow_gimbal,CHASSIS_RC_DEADLINE);
		float rc_follow_gimbal_input=-rc_follow_gimbal*RC_TO_ONE;//前馈云台跟随
	 
 //--------------------------------------小陀螺处理---------------------------------------------------------------------------------

 //// 判断是否进入小陀螺模式  按下ctrl键，左边y拨杆拨到竖直
 if ( keyboard_data.Remote_Key_Ctrl == 1
			||(CHASSIS.rc->ch3==-660)) 
 {
		 //小陀螺标志位置为1
     CHASSIS.fdb.tell_gimbal_thing = 1; 
 } 
 
 // 检测到开启小陀螺  
 if (CHASSIS.fdb.tell_gimbal_thing==1) 
{
		float super_wz=0;
	if(SuperCap_rx.capEnergy>50&&SuperCap_rx.errorCode==0)
	{
		super_wz=4;
	}
	else
	{
		super_wz=2;
	}
	
	   if(JudgementData.robot_status_t.robot_level==1)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 6+(CHASSIS.rc->ch3==-660)*6+keyboard_data.Remote_Key_Shift*super_wz;
		 }
		 else if(JudgementData.robot_status_t.robot_level==2)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 7+(CHASSIS.rc->ch3==-660)*7+keyboard_data.Remote_Key_Shift*super_wz;
		 }
		 else if(JudgementData.robot_status_t.robot_level==3)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 8+(CHASSIS.rc->ch3==-660)*8+keyboard_data.Remote_Key_Shift*super_wz;
		 }
		 else if(JudgementData.robot_status_t.robot_level==4)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 9+(CHASSIS.rc->ch3==-660)*8+keyboard_data.Remote_Key_Shift*super_wz;
		 }
		 else if(JudgementData.robot_status_t.robot_level==5)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 9+(CHASSIS.rc->ch3==-660)*8+keyboard_data.Remote_Key_Shift*super_wz;
		 }
		 else if(JudgementData.robot_status_t.robot_level==6)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 10+(CHASSIS.rc->ch3==-660)*8+keyboard_data.Remote_Key_Shift*super_wz;
		 }		
		 else if(JudgementData.robot_status_t.robot_level==7)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 10+(CHASSIS.rc->ch3==-660)*8+keyboard_data.Remote_Key_Shift*super_wz;
		 }		
		 else if(JudgementData.robot_status_t.robot_level==8)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 10+(CHASSIS.rc->ch3==-660)*8+keyboard_data.Remote_Key_Shift*super_wz;
		 }
		 		 
		 else if(JudgementData.robot_status_t.robot_level==9)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 10+(CHASSIS.rc->ch3==-660)*8+keyboard_data.Remote_Key_Shift*super_wz;
		 }		
		 else if(JudgementData.robot_status_t.robot_level==10)
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 10+(CHASSIS.rc->ch3==-660)*8+keyboard_data.Remote_Key_Shift*super_wz;
		 }
		 else
		 {
			  target_wz =keyboard_data.Remote_Key_Ctrl * 6+(CHASSIS.rc->ch3==-660)*6+keyboard_data.Remote_Key_Shift*super_wz;
		 }
    
     // 使用斜坡函数逐渐增加到目标角速度
     if (target_wz > current_wz) {
         current_wz += wz_ramp_rate * CHASSIS.duration * MS_TO_S;
         if (current_wz > target_wz) {
             current_wz = target_wz;
         }
     } else if (target_wz < current_wz) {
         current_wz -= wz_ramp_rate * CHASSIS.duration * MS_TO_S;
         if (current_wz < target_wz) {
             current_wz = target_wz;
         }
     }

        target_v_set.wz = current_wz;
     // 当小陀螺速度降到3m/s时停止小陀螺模式
     if (fabs(current_wz) < 0.3f) {
			   gyro_angle = 0.0f;
         CHASSIS.fdb.tell_gimbal_thing = 0;
     }
 }
//未开启小陀螺为 云台跟随模式
 else 
      {
				// 计算两个中值的误差，选择更近的中值进行跟随
				float diff1 = angle_difference(GIMBAL_DIRECT_YAW_MID, CHASSIS.fdb.gimbal.gimbal_yaw_6020);
				float diff2 = angle_difference(GIMBAL_DIRECT_YAW_MID - PI, CHASSIS.fdb.gimbal.gimbal_yaw_6020);
				float yaw_angle_diff=0;
		if(!keyboard_data.Remote_Key_X)
		{

			  	 yaw_angle_diff = (fabs(diff1) < fabs(diff2)) ? diff1 : diff2;

		}
		else
		{
			   yaw_angle_diff=diff1;
		}
    float corrected_yaw_target = CHASSIS.fdb.gimbal.gimbal_yaw_6020 + yaw_angle_diff;
    target_wz = -PID_calc(&CHASSIS.pid.chassis_follow_gimbal, CHASSIS.fdb.gimbal.gimbal_yaw_6020, corrected_yaw_target)+rc_follow_gimbal_input*5.5-keyboard_data.Remote_Mouse_RL*0.028;
    //直接赋值旋转速度
      target_v_set.wz = target_wz;
      }
 //---------------------------------纵向控制----------------------------------------------------------------------------------
 if (CHASSIS.fdb.tell_gimbal_thing==0) 
{
 				// 计算两个中值的误差，选择更近的中值进行跟随
        float diff1 = angle_difference(GIMBAL_DIRECT_YAW_MID, CHASSIS.fdb.gimbal.gimbal_yaw_6020);
        float diff2 = angle_difference(GIMBAL_DIRECT_YAW_MID - PI,CHASSIS.fdb.gimbal.gimbal_yaw_6020);
			  

	
	      
        int8_t speed_direction = 1; // 速度方向，1为正常，-1为取反
				
				if(!keyboard_data.Remote_Key_X)
				{
						if(fabs(diff1) < fabs(diff2)) 
						{
								speed_direction = 1; // 跟随GIMBAL_DIRECT_YAW_MID，速度正常
						} 
						else 
						{
								speed_direction = -1; // 跟随GIMBAL_DIRECT_YAW_MID - PI，速度取反
						}
				}
	       float super_aceel=0;
				if(SuperCap_rx.capEnergy>50&&SuperCap_rx.errorCode==0)
				{
					super_aceel=1.0;
				}
				else
				{
					super_aceel=0.3;
				}

        target_v_set.vx = rc_x * RC_TO_ONE * MAX_SPEED_VECTOR_VX * speed_direction;

        if(0.5*(CHASSIS.fdb.leg[0].rod.L0+CHASSIS.fdb.leg[1].rod.L0)>0.32) //磕台阶速度
        {
            target_v_set.vx += keyboard_data.Remote_Key_W*0.9*speed_direction;
            target_v_set.vx -= keyboard_data.Remote_Key_S*0.9*speed_direction;	
        }
        else
        {      
						   if(JudgementData.robot_status_t.robot_level==1)
							 {
									if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.2)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.2)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }
							 else if(JudgementData.robot_status_t.robot_level==2)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.3)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.3)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }
							 	else if(JudgementData.robot_status_t.robot_level==3)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.4)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.4)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }
							else if(JudgementData.robot_status_t.robot_level==4)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.5)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.5)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }
							 else if(JudgementData.robot_status_t.robot_level==5)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.6)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.6)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }
							  else if(JudgementData.robot_status_t.robot_level==6)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }
							 							 
							 else if(JudgementData.robot_status_t.robot_level==7)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }							  
							 else if(JudgementData.robot_status_t.robot_level==8)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }
							 	else if(JudgementData.robot_status_t.robot_level==9)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }							 
							 else if(JudgementData.robot_status_t.robot_level==10)
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.7)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }
							 else
							 {
								 	if(keyboard_data.Remote_Key_W==1)
									{
										target_v_set.vx += speed_direction*(1.2)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
									if(keyboard_data.Remote_Key_S==1)
									{
									 target_v_set.vx -= speed_direction*(1.2)+speed_direction*keyboard_data.Remote_Key_Shift*super_aceel;
									}
							 }


        }

       //速度斜波
 			if (target_v_set.vx > current_vx) {
 			current_vx += vx_ramp_rate * CHASSIS.duration * MS_TO_S;
 			if (current_vx > target_v_set.vx) {
 					current_vx = target_v_set.vx;
 			}
 	} else if (target_v_set.vx < current_vx) {
 			current_vx -= vx_ramp_rate * CHASSIS.duration * MS_TO_S;
 			if (current_vx < target_v_set.vx) {
 					current_vx = target_v_set.vx;
 			}
 	}
}

// 1. 动态获取当前允许的速度/旋转步长 (受功率严格制约)
    float current_vx_step = vx_ramp_rate;
    float current_wz_step = wz_ramp_rate; 
    
    if (move_scale < 0.3f)   //功率吃紧
		{
      if (SuperCap_rx.capEnergy<10)
      {
            current_vx_step *= 0.1f; // 无超电  基本锁死目标值
				    current_wz_step *= 0.3f;
      }
			else
			{
						current_vx_step *= 0.5f; // 有超电  允许步长变化较大
					  current_wz_step *= 1.0f;
			}
    } 
		else if (move_scale < 0.8f) //功率剩余较多
		{
      if (SuperCap_rx.errorCode<10)
      {
            current_vx_step *= 0.7f; // 无超电  
				    current_wz_step *= 0.7f;
      }
			else
			{
						current_vx_step *= 0.9f; // 有超电  允许步长变化较大
					  current_wz_step *= 1.0f;
			}
    }

    // 2. 判断前进/后退 (Vx) 的加速与减速状态
    bool is_decelerating = (fabsf(target_v_set.vx ) < fabsf(current_vx)) || (target_v_set.vx  * current_vx < 0);

    if (is_decelerating) 
    {
        // 减速状态 (刹车)：同样受电流热损耗 I^2R 限制，用受控的 current_vx_step 降速
        if (target_v_set.vx  > current_vx + current_vx_step) current_vx += current_vx_step;
        else if (target_v_set.vx  < current_vx - current_vx_step) current_vx -= current_vx_step;
        else current_vx = target_v_set.vx ;
    } 
    else 
    {
        // 加速状态
			if (move_scale < 0.3f) //迫近功率上限
				{
            // 不能加速，主动缓慢降速
            if (current_vx > current_vx_step) current_vx -= current_vx_step;
            else if (current_vx < -current_vx_step) current_vx += current_vx_step;
            else current_vx = 0.0f;
        } 
				else 
				{
            // 正常/受限 加速
            if (target_v_set.vx  > current_vx + current_vx_step) current_vx += current_vx_step;
            else if (target_v_set.vx  < current_vx - current_vx_step) current_vx -= current_vx_step;
            else current_vx = target_v_set.vx ;
        }
    }
    // 3. 旋转 (Wz) 斜坡处理：
    if (target_v_set.wz > current_wz + current_wz_step) current_wz += current_wz_step;
    else if (target_v_set.wz < current_wz - current_wz_step) current_wz -= current_wz_step;
    else current_wz = target_v_set.wz;
    
    //最终赋值给设定值
    current_vx = current_vx;   
    target_v_set.wz = current_wz;

 //----------------------------------------------------------------------------------------------------------------------------

 		switch (CHASSIS.mode) {
         case CHASSIS_FREE: {  // 底盘自由模式下，控制量为底盘坐标系下的速度
             CHASSIS.ref.speed_vector.vx = current_vx;	
             CHASSIS.ref.speed_vector.vy = 0;
             CHASSIS.ref.speed_vector.wz = target_v_set.wz;
             break;
         }
         default:
             CHASSIS.ref.speed_vector.vx = 0;
             CHASSIS.ref.speed_vector.vy = 0;
             CHASSIS.ref.speed_vector.wz = 0;
             break;
     }
		
 		// 计算期望状态
     for (uint8_t i
			 = 0; i < 2; i++) {
         CHASSIS.ref.leg_state[i].theta     =  0;
         CHASSIS.ref.leg_state[i].theta_dot =  0;
         CHASSIS.ref.leg_state[i].x         =  0;
         CHASSIS.ref.leg_state[i].x_dot     =  CHASSIS.ref.speed_vector.vx;
         CHASSIS.ref.leg_state[i].phi       =  0;
         CHASSIS.ref.leg_state[i].phi_dot   =  0;
     }
 		 // 腿部控制
     switch (CHASSIS.mode) {
         case CHASSIS_FREE: {
 				     length=length-rc_length*RC_TO_ONE*0.001f;
				   if(keyboard_data.Remote_Key_E)
					 {
						 	// 斜坡变化到目标腿长
						 	float target_length = move_mid_length;
						 	float delta = target_length - length;
						 	if(fabs(delta) > 0.001f) {
								 length += delta * 0.007f; // 10%步长斜坡变化
						 	} else {
								 length = target_length;
						 	}
					 }
           	 if(keyboard_data.Remote_Key_Q||keyboard_data.Remote_Key_A)
					 {
						 	// 斜坡变化到目标腿长
						 	float target_length = MIN_LEG_LENGTH;
						 	float delta = target_length - length;
						 	if(fabs(delta) > 0.0006f) {
								 length += delta * 0.01f; // 10%步长斜坡变化
						 	} else {
								 length = target_length;
						 	}
					 }
           	if(keyboard_data.Remote_Key_D)
					 {
						 	// 斜坡变化到最大
						 	float target_length = move_max_length;
						 	float delta = target_length - length;
						 	if(fabs(delta) > 0.001f) {
								 length += delta * 0.005f; // 10%步长斜坡变化
						 	} else {
								 length = target_length;
						 	}
					 }

           
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
               length = MIN_LEG_LENGTH+0.01;
             }
         } break;
         default: {
             length = 0.14f;
         }
     }
 length=fp32_constrain(length,MIN_LEG_LENGTH,MAX_LEG_LENGTH);

 						 CHASSIS.ref.rod_L0[0] = length;
 						 CHASSIS.ref.rod_L0[1] = length;
  }

static void ConsoleZeroForce(void);//零力控制
static void ConsoleNormal(void);//正常控制
static void Consoleselfstart(void);//翻倒自启
static void ConsoleCalibrate(void);//校准控制
static void LocomotionController(void);
static void LegTorqueController(void);

 //计算控制量
 void ChassisConsole(void)
 {
 switch (CHASSIS.mode) {
	      case CHASSIS_FREE:
        {
			if (is_auto_climbing) {
                Auto_ClimbStep(); 
            }
            else {
                ConsoleNormal();
            }
        }break;
	      case CHASSIS_STAND_UP:
				{
          ConsoleNormal();//正常控制
				}break;
        case CHASSIS_SAFE:
        default: {
					ConsoleZeroForce();
        }
 }
 } 
 
 
 static void Consoleselfstart(void)
 //翻倒自启
{
		
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
				else if(CHASSIS.mode==CHASSIS_FREE)
				{
            CHASSIS.joint_motor[i].set.tor =fp32_constrain(CHASSIS.joint_motor[i].set.tor,min_joint_tor_move,max_joint_tor_move);
        }
    }

    CHASSIS.wheel_motor[0].set.tor  = (CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION));
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



float Ld0=0.0;
float L_diff=0.0;
 fp32 current_to_torque=0.000480637608;

static void LocomotionController(void)
{
	    // 计算LQR增益
    float k[2][6];
    float x[6];
    float T_Tp[2];

	    // 定义变量存储分离后的力矩 (单位: Nm)
    float T_total[2] = {0}; // 总力矩
    float T_vel[2]   = {0}; // 仅由速度误差产生的力矩 (移动分量)
    float T_bal[2]   = {0}; // 剩下的平衡分量 (总 - 移动)
	  float T_Tp_dummy[2]; // 用于占位  
			
    for (uint8_t i = 0; i < 2; i++) {
        GetK(CHASSIS.fdb.leg[i].rod.L0,k,CHASSIS.fdb.leg[i].is_take_off);
        x[0] = (CHASSIS.fdb.leg_state[i].theta     - CHASSIS.ref.leg_state[i].theta);
        x[1] = (CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.ref.leg_state[i].theta_dot);
        x[2] = (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x); 
        x[3] = (CHASSIS.fdb.leg_state[i].x_dot     - CHASSIS.ref.leg_state[i].x_dot);
        x[4] = (CHASSIS.fdb.leg_state[i].phi       - CHASSIS.ref.leg_state[i].phi);
        x[5] = (CHASSIS.fdb.leg_state[i].phi_dot   - CHASSIS.ref.leg_state[i].phi_dot);
			


        CalcLQR(k, x, T_Tp_dummy);
			  T_total[i]=T_Tp_dummy[0]; 
			  
        CHASSIS.cmd.leg[i].rod.Tp = T_Tp_dummy[1];
			
			   // 2. 分离移动力矩
        T_vel[i] = k[0][3] * x[3]; 

        // 3. 计算平衡力矩
        T_bal[i] = T_total[i] - T_vel[i];
			}
			// 4. 计算旋转力矩 (Yaw)
			PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
		  CHASSIS.cmd.leg[0].wheel.T-=CHASSIS.pid.yaw_velocity.out;
      CHASSIS.cmd.leg[1].wheel.T+=CHASSIS.pid.yaw_velocity.out;
      
			//解决站高时候抽搐的问题 由于腿长过高造成轮毂YAW轴分配力矩不足原因
			//判断腿长较长时候 yaw轴输出力矩0.3
      float T_yaw = 0.0f;
			if(0.5*(CHASSIS.fdb.leg[0].rod.L0+CHASSIS.fdb.leg[1].rod.L0)>move_mid_length+0.05)
			{
				T_yaw = 0.4*CHASSIS.pid.yaw_velocity.out; 
			}
			else
			{
				T_yaw = CHASSIS.pid.yaw_velocity.out; 
			}
			
				 
				// 5.功率控制数据
			float I_bal_L = T_bal[0] / current_to_torque;
			float I_bal_R = T_bal[1] / current_to_torque;
		 
		// 移动分量 = LQR速度分量 + Yaw分量
    // 左轮：LQR速度 - Yaw
    // 右轮：LQR速度 + Yaw
    float T_mov_L_Nm = T_vel[0] - T_yaw;
    float T_mov_R_Nm = T_vel[1] + T_yaw;
    
    float I_mov_L = T_mov_L_Nm / current_to_torque;
    float I_mov_R = T_mov_R_Nm / current_to_torque;

    // 获取当前转速 (RPM)
    float speed_L = CHASSIS.wheel_motor[0].fdb.vel / 0.1047197551f;
    float speed_R = CHASSIS.wheel_motor[1].fdb.vel / 0.1047197551f;
		
    // 6. 计算功率限制缩放系数
     move_scale = 1.0f;
 // 只有在自由模式或站立模式才限制
    if (CHASSIS.mode == CHASSIS_FREE )
    {
        Chassis_Power_Limit_Calc(I_bal_L, I_mov_L, I_bal_R, I_mov_R, speed_L, speed_R, &move_scale);
    }
   // 7. 应用缩放并合成最终力矩
    // Final = Balance + Scale * (Velocity + Yaw)
    CHASSIS.cmd.leg[0].wheel.T = T_bal[0] + move_scale * T_mov_L_Nm;
    CHASSIS.cmd.leg[1].wheel.T = T_bal[1] + move_scale * T_mov_R_Nm;
    

      for (uint8_t i = 0; i < 2; i++) 
				{
					if(CHASSIS.fdb.leg[i].is_take_off// 离地
            ||leg_loss_control[i].state==LOSS_CONTROL_OVERTURN //翻车
            ||leg_loss_control[i].state==LOSS_CONTROL_CONFIRMED //失控
            ||CHASSIS.mode==CHASSIS_STAND_UP)//起立状态
					{
						CHASSIS.cmd.leg[i].wheel.T = 0;
					}
				}
}
/**
 * @brief 腿部力矩控制
 */
float F0, F_leg,touch_ground_kd_leg=40;
static void LegTorqueController(void)
{
	 CHASSIS.ref.rod_L0[0]  = fp32_constrain(CHASSIS.ref.rod_L0[0], MIN_LEG_LENGTH, MAX_LEG_LENGTH);
	 CHASSIS.ref.rod_L0[1]  = fp32_constrain(CHASSIS.ref.rod_L0[1], MIN_LEG_LENGTH, MAX_LEG_LENGTH);

	  if(CHASSIS.mode==CHASSIS_FREE)
		{
						for (uint8_t i = 0; i < 2; i++) 
						{
							if (CHASSIS.step == JUMP_STEP_JUMP) 
							{
							// 直接给一个超大力F起飞
							CHASSIS.cmd.leg[i].rod.F = 160;
							}
				  		else if(CHASSIS.step==JUMP_STEP_RECOVERY)
							{  
								F0 =-20;
								F_leg = PID_calc(&CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0,CHASSIS.ref.rod_L0[i]);
								CHASSIS.cmd.leg[i].rod.F =F0+F_leg- CHASSIS.fdb.leg[i].rod.F_spring ; 
							}
							else if(CHASSIS.mode==CHASSIS_FREE&&CHASSIS.step==NORMAL_STEP)
							{				
								F0 =BODY_MASS_FN;
								if(CHASSIS.fdb.leg[i].touch_time>2000)
								{
									F_leg = PID_calc(&CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0,CHASSIS.ref.rod_L0[i]);
								}
								else
								{
									F_leg = PID_calc(&CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0,CHASSIS.ref.rod_L0[i])-fp32_constrain(touch_ground_kd_leg*CHASSIS.fdb.leg[i].rod.dL0,-100,100);
								}
								CHASSIS.cmd.leg[i].rod.F = F0 + F_leg- CHASSIS.fdb.leg[i].rod.F_spring; 
							}
						}		
					//横滚角稳定
					PID_calc(&CHASSIS.pid.pitch_angle,CHASSIS.fdb.body.pitch,CHASSIS.ref.body.pitch);//右边抬起是pitch增大
					if(CHASSIS.fdb.leg[0].is_take_off&&CHASSIS.fdb.leg[1].is_take_off)
		      {
						CHASSIS.pid.pitch_angle.out=0;
		      }
					CHASSIS.cmd.leg[0].rod.F-=CHASSIS.pid.pitch_angle.out;
					CHASSIS.cmd.leg[1].rod.F+=CHASSIS.pid.pitch_angle.out;
					//防劈叉  TP
					CHASSIS.fdb.two_leg_err=CHASSIS.fdb.leg[0].rod.Theta-CHASSIS.fdb.leg[1].rod.Theta;
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
						F0=-30;
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
 int self_toghter=0;
 void SendJointMotorCmd(void)
 {
			switch (CHASSIS.mode)
			{ 	
        case CHASSIS_STAND_UP: 
				case CHASSIS_FREE:  //底盘自由模式
				{

						DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
						DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
						osDelay(1);
						DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
						DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
				}break;
				case CHASSIS_selfon:  //
				{
				if(leg_loss_control[0].state==LOSS_CONTROL_OVERTURN&&leg_loss_control[1].state==LOSS_CONTROL_OVERTURN)
					{
						
						CHASSIS.joint_motor[0].set.vel=3;
						CHASSIS.joint_motor[1].set.vel=3;
						
						
						if(fabsf(CHASSIS.fdb.leg[0].rod.Phi0-CHASSIS.fdb.leg[1].rod.Phi0)<0.1)
						{
									self_toghter=1;
						}
						if(self_toghter==1)
						{
								CHASSIS.joint_motor[2].set.vel=-3;
								CHASSIS.joint_motor[3].set.vel=-3;
						}
						else
						{
								CHASSIS.joint_motor[2].set.vel=0;
								CHASSIS.joint_motor[3].set.vel=0;
						}
						CHASSIS.wheel_motor[0].set.tor = 0;
						CHASSIS.wheel_motor[1].set.tor = 0;
						
						DmMitCtrlVelocity(&CHASSIS.joint_motor[0],STAND_UP_VEL_KD);
						osDelay(1);
						DmMitCtrlVelocity(&CHASSIS.joint_motor[1],STAND_UP_VEL_KD);
						osDelay(1);
						DmMitCtrlVelocity(&CHASSIS.joint_motor[2],STAND_UP_VEL_KD);
						osDelay(1);
						DmMitCtrlVelocity(&CHASSIS.joint_motor[3],STAND_UP_VEL_KD);
					}
					  if(leg_loss_control[0].state==LOSS_CONTROL_CONFIRMED&&leg_loss_control[1].state==LOSS_CONTROL_CONFIRMED)
					{
						self_toghter=0;
						CHASSIS.joint_motor[0].set.vel=-3;
						CHASSIS.joint_motor[1].set.vel=-3;
						CHASSIS.joint_motor[2].set.vel= 3;
						CHASSIS.joint_motor[3].set.vel= 3;
						CHASSIS.wheel_motor[0].set.tor= 0;
						CHASSIS.wheel_motor[1].set.tor= 0;
					  //确认失控					
						DmMitCtrlVelocity(&CHASSIS.joint_motor[0],1.5);
						osDelay(1);
						DmMitCtrlVelocity(&CHASSIS.joint_motor[1],1.5);
						osDelay(1);
						DmMitCtrlVelocity(&CHASSIS.joint_motor[2],1.5);
						osDelay(1);
						DmMitCtrlVelocity(&CHASSIS.joint_motor[3],1.5);
					}

				}break;
				default:    //底盘其他模式
				{
					CHASSIS.joint_motor[0].set.tor=0;
					CHASSIS.joint_motor[1].set.tor=0;			
					CHASSIS.joint_motor[2].set.tor=0;				
					CHASSIS.joint_motor[3].set.tor=0;
					DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
          DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
					osDelay(1);
          DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
          DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
				}
			}
 }
 /**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
 
 void SendWheelMotorCmd(void)
 {
	     switch (CHASSIS.mode) {
				 case CHASSIS_FREE: 
					 {
              CHASSIS.wheel_motor[0].set.curr=CHASSIS.wheel_motor[0].set.tor/current_to_torque;
              CHASSIS.wheel_motor[1].set.curr=CHASSIS.wheel_motor[1].set.tor/current_to_torque;
						 
						    CHASSIS.wheel_motor[0].set.curr=fp32_constrain(   CHASSIS.wheel_motor[0].set.curr,-16384,16384);
						 	  CHASSIS.wheel_motor[1].set.curr=fp32_constrain(   CHASSIS.wheel_motor[1].set.curr,-16384,16384);

                //扭矩转化为电流
                CanCmdDjiMotor(&hfdcan2,0x200,CHASSIS.wheel_motor[0].set.curr,CHASSIS.wheel_motor[1].set.curr,0,0);
					 		CAN_cmd_supercap(&hfdcan2,0x061,SuperCapON,JudgementData.robot_status_t.chassis_power_limit,JudgementData.power_heat_data_t.buffer_energy);
            } break;
				 default:
				 {
				      CanCmdDjiMotor(&hfdcan2,0x200,0,0,0,0);
					 		CAN_cmd_supercap(&hfdcan2,0x061,SuperCapON,JudgementData.robot_status_t.chassis_power_limit,JudgementData.power_heat_data_t.buffer_energy);
				 }
				 
			 }

			 
 }
	 extern TIM_HandleTypeDef htim12;
 extern osThreadId Music_taskHandle;

void ChassisHandleException()
{
  if(CHASSIS.joint_motor[0].fdb.state==0||CHASSIS.joint_motor[1].fdb.state==0||CHASSIS.joint_motor[2].fdb.state==0||CHASSIS.joint_motor[3].fdb.state==0||keyboard_data.Remote_Key_V==1)
	{
    DmEnable(&CHASSIS.joint_motor[0]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[1]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[2]);
    osDelay(1);
    DmEnable(&CHASSIS.joint_motor[3]);
		osDelay(1);
	} 
	//当切换到底盘控制模式/按键按下R/底盘起立模式
  if ((CHASSIS.mode==CHASSIS_FREE&&CHASSIS.last_mode!=CHASSIS_FREE)||keyboard_data.Remote_Key_R||(CHASSIS.mode==CHASSIS_STAND_UP&&CHASSIS.last_mode!=CHASSIS_STAND_UP))
  {
    memset(&CHASSIS.fdb,0,sizeof(CHASSIS.fdb));
    CHASSIS.fdb.leg[0].is_take_off = false;
    CHASSIS.fdb.leg[1].is_take_off = false;
		length=0.135;
		
		

  }
	
			if(ctom_message.aim_live==0)
		{
	     HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_2);
			vTaskSuspend(Music_taskHandle);
		}
		else
		{
		 HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
			vTaskResume(Music_taskHandle);
		}

	CHASSIS.last_mode=CHASSIS.mode;    


}




