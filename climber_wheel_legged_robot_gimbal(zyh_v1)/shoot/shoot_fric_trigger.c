#include "shoot_fric_trigger.h"
#include "shoot_task.h"
#include "CAN_receive.h"
#include "AutoGimbal.h"
#include "remote_control.h"
extern visionDataStu_t visionDataStu;
extern Keyboard_Data keyboard_data;

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

fp32 TRIGGER_SPEED=530.0f;

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

void ShootSetMode(void)
{
	if(switch_is_down(SHOOT.rc->rc.s[1])&&switch_is_down(SHOOT.rc->rc.s[0]))//双下无力
	{
		SHOOT.state=FRIC_NOT_READY;//不准备发射
		SHOOT.mode = LOAD_STOP;   //停止拨盘

	}
	else if (switch_is_up(SHOOT.rc->rc.s[1])&&switch_is_up(SHOOT.rc->rc.s[0]))//双上可以射击
  {
      //清弹
    SHOOT.state = FRIC_READY;//准备发射
    SHOOT.mode = LOAD_BURSTFIRE;//连发对速度闭环
 }
  else if (switch_is_up(SHOOT.rc->rc.s[1])&&switch_is_down(SHOOT.rc->rc.s[0]))//左上右下云台手动控制，拨盘 3508同时射击
  {
      //清弹
    SHOOT.state = FRIC_READY;//准备发射
    SHOOT.mode = LOAD_BURSTFIRE;//连发对速度闭环
  }
  else
  { 
	  SHOOT.state=FRIC_NOT_READY;//不准备发射
		SHOOT.mode =LOAD_STOP;   //停止拨盘
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
void ShootReference(void) 
{
	  switch (SHOOT.state)
  {
  case FRIC_NOT_READY:
  SHOOT.REF.fric_speed_ref_R=0.0f;
  SHOOT.REF.fric_speed_ref_L=0.0f;
  break;

  case FRIC_READY:
  SHOOT.REF.fric_speed_ref_R=FRIC_R_SPEED;
  SHOOT.REF.fric_speed_ref_L=FRIC_L_SPEED;
  break;
  
  default:
  break;
  }
	
	 switch (SHOOT.mode)
  {
  case LOAD_STOP:
	{
	SHOOT.REF.trigger_speed_ref=0.0f;
	}
  break;
  case LOAD_BURSTFIRE:
	{  //开火条件，满足遥控器、鼠标左击，鼠标右击需要自瞄给开火位置
		if(SHOOT.rc->rc.ch[4]==660||keyboard_data.Remote_Mouse_KeyL||(visionDataStu.mode == 2 && keyboard_data.Remote_Mouse_KeyR))
		{
				SHOOT.REF.trigger_speed_ref = TRIGGER_SPEED;
		}
		else
		{
				SHOOT.REF.trigger_speed_ref = 0;
		} 
	}
  break;

  default:
    break;
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
    if(SHOOT.mode==LOAD_BURSTFIRE)
		{
					CanCmdDjiMotor(1,STD_ID,0,0,SHOOT.trigger_motor.set.curr,0);
		}
		else
		{
				  CanCmdDjiMotor(1,STD_ID,0,0,0,0);
		}
		
		if(SHOOT.state==FRIC_READY)
		{
			    CanCmdDjiMotor(2,STD_ID,SHOOT.fric_motor[1].set.curr,SHOOT.fric_motor[0].set.curr,0,0);
		}
		else
		{
					CanCmdDjiMotor(2,STD_ID,0,0,0,0);
		}
}
