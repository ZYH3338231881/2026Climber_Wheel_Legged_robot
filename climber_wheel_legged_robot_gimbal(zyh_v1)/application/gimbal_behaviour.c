#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "user_lib.h"
#include "IMU_task.h"
#include "CAN_receive.h"
#include "macro_typedef.h"
#include "AutoGimbal.h"
#include "IMU.h"//陀螺仪文件
#include "sign_gengerator.h"
#include "slidingmodec.h"
LowPassFilter_t Mouse_yaw_Filter;
Gimbal_s gimbal_direct;
PID_t gimbal_direct_pid;
SMC yaw_smc;
extern visionDataStu_t visionDataStu;
extern Keyboard_Data keyboard_data;

void GimbalInit(void) 
{
	 gimbal_direct.rc = get_remote_control_point(); 
	 gimbal_direct.reference.pitch=0;
   gimbal_direct.reference.yaw=0;
	 gimbal_direct.feedback_pos.pitch=0;
   gimbal_direct.feedback_pos.yaw=0;
	 gimbal_direct.feedback_vel.pitch=0;
   gimbal_direct.feedback_vel.yaw=0;
	 gimbal_direct.upper_limit.pitch=GIMBAL_UPPER_LIMIT_PITCH;//rad  pitch的上下限制
   gimbal_direct.lower_limit.pitch=GIMBAL_LOWER_LIMIT_PITCH;
	   //step3 PID数据清零，设置PID参数
   const static fp32 gimbal_yaw_angle[3]={KP_GIMBAL_YAW_ANGLE,KI_GIMBAL_YAW_ANGLE,KD_GIMBAL_YAW_ANGLE};
   const static fp32 gimbal_yaw_velocity[3]={KP_GIMBAL_YAW_VELOCITY,KI_GIMBAL_YAW_VELOCITY,KD_GIMBAL_YAW_VELOCITY};
   
	 const static fp32 gimbal_pitch_angle[3]={KP_GIMBAL_PITCH_ANGLE,KI_GIMBAL_PITCH_ANGLE,KD_GIMBAL_PITCH_ANGLE};
   const static fp32 gimbal_pitch_velocity[3]={KP_GIMBAL_PITCH_VELOCITY,KI_GIMBAL_PITCH_VELOCITY,KD_GIMBAL_PITCH_VELOCITY};
	 
	 PID_init(&gimbal_direct_pid.yaw_angle,PID_POSITION,gimbal_yaw_angle,MAX_OUT_GIMBAL_YAW_ANGLE,MAX_IOUT_GIMBAL_YAW_ANGLE);
   PID_init(&gimbal_direct_pid.yaw_velocity,PID_POSITION,gimbal_yaw_velocity,MAX_OUT_GIMBAL_YAW_VELOCITY,MAX_IOUT_GIMBAL_YAW_VELOCITY);

   PID_init(&gimbal_direct_pid.pitch_angle,PID_POSITION,gimbal_pitch_angle,MAX_OUT_GIMBAL_PITCH_ANGLE,MAX_IOUT_GIMBAL_PITCH_ANGLE);
   PID_init(&gimbal_direct_pid.pitch_velocity,PID_POSITION,gimbal_pitch_velocity,MAX_OUT_GIMBAL_PITCH_VELOCITY,MAX_IOUT_GIMBAL_PITCH_VELOCITY);
	 
	 LowPassFilterInit(&Mouse_yaw_Filter,0.9);
	 
	 SMC_Init(&yaw_smc,80,10,0.0001,6500,0.8,16384);
   //滑膜面参数C 增益K 控制死区单位是弧度   
   //step4 初始化电机
   MotorInit(&gimbal_direct.yaw,GIMBAL_DIRECT_YAW_ID,GIMBAL_DIRECT_YAW_CAN,GIMBAL_DIRECT_YAW_MOTOR_TYPE,GIMBAL_DIRECT_YAW_DIRECTION,GIMBAL_DIRECT_YAW_REDUCTION_RATIO,GIMBAL_DIRECT_YAW_MODE);
   MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,GIMBAL_DIRECT_PITCH_MODE);

	 
	 //step6 模式设置初始化
   gimbal_direct.mode=GIMBAL_ZERO_FORCE;
   gimbal_direct.last_mode = GIMBAL_ZERO_FORCE;
	 
	 
}


void GimbalSetMode(void)
{
	if(switch_is_down(gimbal_direct.rc->rc.s[1])&&switch_is_down(gimbal_direct.rc->rc.s[0]))//双下无力
	{
		gimbal_direct.mode=GIMBAL_ZERO_FORCE;
	}
	else if (switch_is_up(gimbal_direct.rc->rc.s[1])&&switch_is_down(gimbal_direct.rc->rc.s[0]))//左上右下云台手动控制
	{
		gimbal_direct.mode=GIMBAL_HAND;
	}
  else if (switch_is_mid(gimbal_direct.rc->rc.s[1])&&switch_is_down(gimbal_direct.rc->rc.s[0]))//左中右下云台初始化
	{
		gimbal_direct.mode=GIMBAL_INIT;
	}
	  else if (switch_is_up(gimbal_direct.rc->rc.s[1])&&switch_is_up(gimbal_direct.rc->rc.s[0]))//左上右上云台自瞄
	{
		gimbal_direct.mode=GIMBAL_AUTO_AIM;
	}
  else
	{
		gimbal_direct.mode=GIMBAL_ZERO_FORCE;
	}
}

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) 
{
	//电机相关数据更新
  GetMotorMeasure(&gimbal_direct.yaw);
  GetMotorMeasure(&gimbal_direct.pitch);
	
	//IMU相关数据更新
	
	gimbal_direct.feedback_pos.pitch=GetImuAngle(AX_ROLL);
  gimbal_direct.feedback_pos.yaw=GetImuAngle(AX_YAW);

  gimbal_direct.feedback_vel.pitch=GetImuVelocity(AX_ROLL);
  gimbal_direct.feedback_vel.yaw=GetImuVelocity(AX_YAW);

	
	gimbal_direct.duration = xTaskGetTickCount() - gimbal_direct.last_time;
  gimbal_direct.last_time = xTaskGetTickCount();
	
}


/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) 
{
	//切入手动模式，更新数据GIMBAL_HAND
  if((gimbal_direct.mode ==GIMBAL_HAND)&&(gimbal_direct.last_mode!=GIMBAL_HAND))
  {
    gimbal_direct.reference.pitch=0;
    gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;
  }
	//切入自瞄模式更新当前数据GIMBAL_AUTO_AIM
   else if((gimbal_direct.mode ==GIMBAL_AUTO_AIM)&&(gimbal_direct.last_mode!=GIMBAL_AUTO_AIM))
  {
    gimbal_direct.reference.pitch=gimbal_direct.feedback_pos.pitch;
    gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;   
	}

	//切入云台初始化模式更新当前数据GIMBAL_INIT
   else if((gimbal_direct.mode ==GIMBAL_INIT)&&(gimbal_direct.last_mode!=GIMBAL_INIT))
  {
    gimbal_direct.reference.pitch=gimbal_direct.feedback_pos.pitch;
    gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;  
	}
  gimbal_direct.last_mode=gimbal_direct.mode; //上一运行模式更新


}
/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void) 
{
  if (gimbal_direct.mode == GIMBAL_ZERO_FORCE)
  {
    gimbal_direct.pitch.set.curr=0;
    gimbal_direct.yaw.set.curr=0;
  }
	  else if (gimbal_direct.mode == GIMBAL_INIT)
  {
		gimbal_direct.reference.pitch=0;
	  gimbal_direct.reference.pitch=fp32_constrain(gimbal_direct.reference.pitch,gimbal_direct.lower_limit.pitch,gimbal_direct.upper_limit.pitch);
		gimbal_direct.reference.yaw=GIMBAL_DIRECT_YAW_MID;
		
    gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.pitch.direction *gimbal_direct.feedback_pos.pitch,gimbal_direct.reference.pitch);
    gimbal_direct.pitch.set.curr= PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.pitch.direction *gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);
    // 使用角度差值归一化来处理跨越π边界的问题
    float yaw_angle_diff = angle_difference(gimbal_direct.reference.yaw, gimbal_direct.yaw.fdb.pos);
    float corrected_yaw_target = gimbal_direct.feedback_pos.yaw + yaw_angle_diff;
    /*              SMC滑膜控制              */
				yaw_smc.ref=corrected_yaw_target;
				SMC_Tick(&yaw_smc,gimbal_direct.feedback_pos.yaw, gimbal_direct.feedback_vel.yaw);
				gimbal_direct.yaw.set.curr=yaw_smc.u;
	}
	
  else if (gimbal_direct.mode == GIMBAL_HAND)
  {
		gimbal_direct.reference.pitch-=keyboard_data.Remote_Mouse_DU*0.00002;
	  gimbal_direct.reference.pitch=fp32_constrain(gimbal_direct.reference.pitch,gimbal_direct.lower_limit.pitch,gimbal_direct.upper_limit.pitch);
		gimbal_direct.reference.yaw-=gimbal_direct.rc->rc.ch[2]*0.00003;
		  /*              鼠标yaw一阶低通滤波		  */
		  LowPassFilterCalc(&Mouse_yaw_Filter,keyboard_data.Remote_Mouse_RL*0.00006);
		  Mouse_yaw_Filter.out=fp32_constrain(Mouse_yaw_Filter.out,-0.040,+0.040);
	    gimbal_direct.reference.yaw-=Mouse_yaw_Filter.out;


      //速度环yaw轴控制
      gimbal_direct.yaw.set.vel=-gimbal_direct.rc->rc.ch[2]*0.1;
      gimbal_direct.yaw.set.curr=gimbal_direct.yaw.direction * PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.feedback_vel.yaw,gimbal_direct.yaw.set.vel);


      /*              SMC滑膜控制              */
		  // float yaw_angle_diff = angle_difference(gimbal_direct.reference.yaw, gimbal_direct.feedback_pos.yaw);
      // float corrected_yaw_target = gimbal_direct.feedback_pos.yaw + yaw_angle_diff;
			// yaw_smc.ref=corrected_yaw_target;
			// SMC_Tick(&yaw_smc,gimbal_direct.feedback_pos.yaw, gimbal_direct.feedback_vel.yaw);
			// gimbal_direct.yaw.set.curr=yaw_smc.u;

      /*              串级控制                  */
		  gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.pitch.direction *gimbal_direct.feedback_pos.pitch,gimbal_direct.reference.pitch);
      gimbal_direct.pitch.set.curr= PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.pitch.direction *gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);
//    使用角度差值归一化来处理跨越π边界的问题
//    float yaw_angle_diff = angle_difference(gimbal_direct.reference.yaw, gimbal_direct.feedback_pos.yaw);
//    float corrected_yaw_target = gimbal_direct.feedback_pos.yaw + yaw_angle_diff;
//    gimbal_direct.yaw.set.vel=PID_calc(&gimbal_direct_pid.yaw_angle, gimbal_direct.feedback_pos.yaw, corrected_yaw_target);
//    gimbal_direct.yaw.set.curr=gimbal_direct.yaw.direction * PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.feedback_vel.yaw,gimbal_direct.yaw.set.vel);
	}
	 else if (gimbal_direct.mode == GIMBAL_AUTO_AIM)
  {	
    
    if(visionDataStu.mode >= 1)
    {
       gimbal_direct.reference.pitch=-visionDataStu.pitch;
       gimbal_direct.reference.yaw  =visionDataStu.yaw;
    }    
    if(visionDataStu.mode==0&&visionDataStu.last_mode!=0)
   {
      gimbal_direct.reference.pitch=gimbal_direct.feedback_pos.pitch;
      gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;
   }  
    visionDataStu.last_mode=visionDataStu.mode;
    gimbal_direct.reference.pitch=fp32_constrain(gimbal_direct.reference.pitch,gimbal_direct.lower_limit.pitch,gimbal_direct.upper_limit.pitch);
	  gimbal_direct.reference.yaw=fp32_constrain(gimbal_direct.reference.yaw,-3,3);
    gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.pitch.direction *gimbal_direct.feedback_pos.pitch,gimbal_direct.reference.pitch);
    gimbal_direct.pitch.set.curr= PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.pitch.direction *gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);
    /*              SMC滑膜控制              */
		float yaw_angle_diff = angle_difference(gimbal_direct.reference.yaw, gimbal_direct.feedback_pos.yaw);
    float corrected_yaw_target = gimbal_direct.feedback_pos.yaw + yaw_angle_diff;
		yaw_smc.ref=corrected_yaw_target;
		SMC_Tick(&yaw_smc,gimbal_direct.feedback_pos.yaw, gimbal_direct.feedback_vel.yaw);
		gimbal_direct.yaw.set.curr=yaw_smc.u;
	 
	 // 使用角度差值归一化来处理跨越π边界的问题
//    float yaw_angle_diff = angle_difference(gimbal_direct.reference.yaw, gimbal_direct.feedback_pos.yaw);
//    float corrected_yaw_target = gimbal_direct.feedback_pos.yaw + yaw_angle_diff;
//    gimbal_direct.yaw.set.vel=PID_calc(&gimbal_direct_pid.yaw_angle, gimbal_direct.feedback_pos.yaw, corrected_yaw_target);
//    gimbal_direct.yaw.set.curr=gimbal_direct.yaw.direction * PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.feedback_vel.yaw,gimbal_direct.yaw.set.vel);
   	}
	else
	{
		gimbal_direct.pitch.set.curr=0;
    gimbal_direct.yaw.set.curr=0;
	}
}
/*-------------------- Cmd --------------------*/
void C_communication_M();//C板传输给妙板的数据

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void) 
{
    CanCmdDjiMotor(GIMBAL_CAN_CMD_YAW,GIMBAL_STDID_1,gimbal_direct.yaw.set.curr,0,0,0);
	  CanCmdDjiMotor(GIMBAL_CAN_CMD_PITCH,GIMBAL_STDID_1,0,gimbal_direct.pitch.set.curr,0,0);
	  C_communication_M();
}


void C_communication_M() 
{ 
    // 将float类型的yaw转换为int16
    int16_t yaw_int16 = (int16_t)((gimbal_direct.yaw.fdb.pos) * 1000); // 假设放大1000倍保留精度，可根据实际需求调整
    CToM_sendControl(1, 0X666, yaw_int16);
}

