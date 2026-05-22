#include "shoot_fric_trigger.h"
#include "shoot_task.h"
#include "CAN_receive.h"
#include "AutoGimbal.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
extern visionDataStu_t visionDataStu;
extern Keyboard_Data keyboard_data;
extern MTOC_message_t mtoc_mesasge;
extern Gimbal_s gimbal_direct;

Shoot_s SHOOT = {
  .mode = LOAD_STOP,
  .state = FRIC_NOT_READY,
  .fric_flag = 0,
  .move_flag = 0,
  .ecd_count = 0,
  .shoot_flag = 0,
  .heat = 0,
  .heat_limit = 0,
};

fp32 TRIGGER_SPEED=400.0f;  
//fp32 TRIGGER_SPEED=450.0f; 

uint8_t fric_ui;
fp32 delta;
void ShootInit(void) 
{ 
  //获取遥控器指针
  SHOOT.rc = get_remote_control_point(); 

  //摩擦轮相关
  MotorInit(&SHOOT.fric_motor[0],FRIC_MOTOR_R_ID, FRIC_MOTOR_R_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化R摩擦轮电机结构体
  MotorInit(&SHOOT.fric_motor[1],FRIC_MOTOR_L_ID, FRIC_MOTOR_L_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化L摩擦轮电机结构体

  const fp32 pid_fric[3] = {FRIC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FRIC_SPEED_PID_KD};//摩擦轮速度环

  PID_init(&SHOOT.fric_pid[0], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
  PID_init(&SHOOT.fric_pid[1], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);//摩擦轮初始化pid

  //拨弹盘相关
  MotorInit(&SHOOT.trigger_motor,TRIGGER_MOTOR_ID, TRIGGER_MOTOR_CAN, TRIGGER_MOTOR_TYPE, 1, 1.0f, 0);//初始化拨弹盘电机结构体
 if (TRIGGER_MOTOR_TYPE == DJI_M2006)
 {
  const fp32 pid_angel_trigger[3] = {TRIGGER_ANGEL_PID_KP, TRIGGER_ANGEL_PID_KI, TRIGGER_ANGEL_PID_KD};//拨弹盘角度环
  const fp32 pid_speed_trigger[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};//拨弹盘速度环

  PID_init(&SHOOT.trigger_angel_pid, PID_POSITION, pid_angel_trigger, TRIGGER_ANGEL_PID_MAX_OUT, TRIGGER_ANGEL_PID_MAX_IOUT);
  PID_init(&SHOOT.trigger_speed_pid, PID_POSITION, pid_speed_trigger, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);  //拨弹盘初始化pid
 }

}

uint8_t fric_flag=0;
void ShootSetMode(void)
{

	// 通信标志位控制摩擦轮
	if(mtoc_mesasge.fric_flag==1)
	{
		fric_flag=1;
	}
		if(mtoc_mesasge.fric_flag==0)
	{  
		fric_flag=0;
	}
	
	// 设置摩擦轮速度
	if(fric_flag==1||gimbal_direct.mode==GIMBAL_HAND)
	{
		SHOOT.REF.fric_speed_ref_L=FRIC_L_SPEED;
		SHOOT.REF.fric_speed_ref_R=FRIC_R_SPEED;
	}
	else
	{
		SHOOT.REF.fric_speed_ref_L=0;
		SHOOT.REF.fric_speed_ref_R=0;
	}
	

}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ShootObserver(void) 
{
  GetMotorMeasure(&SHOOT.trigger_motor);
  GetMotorMeasure(&SHOOT.fric_motor[0]);
  GetMotorMeasure(&SHOOT.fric_motor[1]);

  SHOOT.FDB.fric_speed_fdb_R = SHOOT.fric_motor[0].fdb.vel;
  SHOOT.FDB.fric_speed_fdb_L = SHOOT.fric_motor[1].fdb.vel;

  SHOOT.FDB.trigger_speed_fdb = SHOOT.trigger_motor.fdb.vel;
}



/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
#define shoot_limit_power 120
void ShootReference(void) 
{
		if(((visionDataStu.mode == 2 &&mtoc_mesasge.mouse_press_l && mtoc_mesasge.mouse_press_r)||(mtoc_mesasge.trigger_flag==1)||(mtoc_mesasge.mouse_press_l==1&&mtoc_mesasge.mouse_press_r!=1))&&mtoc_mesasge.power_heat<=(mtoc_mesasge.shoot_heat_limit-30))
		{
			  if((mtoc_mesasge.power_heat>(mtoc_mesasge.shoot_heat_limit-50))&&(mtoc_mesasge.power_heat<(mtoc_mesasge.shoot_heat_limit-30)))
				{
					SHOOT.REF.trigger_speed_ref = TRIGGER_SPEED;
				}
				else
				{
					SHOOT.REF.trigger_speed_ref = TRIGGER_SPEED-200;
				}
		}
		else if(mtoc_mesasge.key_v>>10==1)
		{
				SHOOT.REF.trigger_speed_ref =-500;
		}
		else
		{
				SHOOT.REF.trigger_speed_ref = 0;
		} 

}


/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ShootConsole(void) 
{
	
  SHOOT.fric_motor[0].set.curr= PID_calc(&SHOOT.fric_pid[0], SHOOT.FDB.fric_speed_fdb_R,SHOOT.REF.fric_speed_ref_R);
  SHOOT.fric_motor[1].set.curr= PID_calc(&SHOOT.fric_pid[1], SHOOT.FDB.fric_speed_fdb_L,SHOOT.REF.fric_speed_ref_L);
	SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);

}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ShootSendCmd(void) 
{		

					CanCmdDjiMotor(1,STD_ID,0,0,SHOOT.trigger_motor.set.curr,0);

			    CanCmdDjiMotor(2,STD_ID,SHOOT.fric_motor[1].set.curr,SHOOT.fric_motor[0].set.curr,0,0);

}


