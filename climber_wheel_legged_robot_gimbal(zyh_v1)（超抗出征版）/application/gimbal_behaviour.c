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
LowPassFilter_t Mouse_yaw_Filter,Mouse_pitch_Filter,yaw_speed_filter,pitch_speed_filter;
Gimbal_s gimbal_direct;
PID_t gimbal_direct_pid,DM_4310_pid;
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
   
	 const static fp32 gimbal_4310_angle[3]={KP_GIMBAL_YAW_ANGLE_4310,KI_GIMBAL_YAW_ANGLE_4310,KD_GIMBAL_YAW_ANGLE_4310};
   const static fp32 gimbal_4310_velocity[3]={KP_GIMBAL_YAW_VELOCITY_4310,KI_GIMBAL_YAW_VELOCITY_4310,KD_GIMBAL_YAW_VELOCITY_4310};
	 
	 const static fp32 gimbal_pitch_angle[3]={KP_GIMBAL_PITCH_ANGLE,KI_GIMBAL_PITCH_ANGLE,KD_GIMBAL_PITCH_ANGLE};
   const static fp32 gimbal_pitch_velocity[3]={KP_GIMBAL_PITCH_VELOCITY,KI_GIMBAL_PITCH_VELOCITY,KD_GIMBAL_PITCH_VELOCITY};
	 
	 PID_init(&gimbal_direct_pid.yaw_angle,PID_POSITION,gimbal_yaw_angle,MAX_OUT_GIMBAL_YAW_ANGLE,MAX_IOUT_GIMBAL_YAW_ANGLE);
   PID_init(&gimbal_direct_pid.yaw_velocity,PID_POSITION,gimbal_yaw_velocity,MAX_OUT_GIMBAL_YAW_VELOCITY,MAX_IOUT_GIMBAL_YAW_VELOCITY);
	 
	 PID_init(&DM_4310_pid.yaw_angle,PID_POSITION,gimbal_4310_angle,MAX_OUT_GIMBAL_YAW_ANGLE_4310,MAX_IOUT_GIMBAL_YAW_ANGLE_4310);
   PID_init(&DM_4310_pid.yaw_velocity,PID_POSITION,gimbal_4310_velocity,MAX_OUT_GIMBAL_YAW_VELOCITY_4310,MAX_IOUT_GIMBAL_YAW_VELOCITY_4310);

   PID_init(&gimbal_direct_pid.pitch_angle,PID_POSITION,gimbal_pitch_angle,MAX_OUT_GIMBAL_PITCH_ANGLE,MAX_IOUT_GIMBAL_PITCH_ANGLE);
   PID_init(&gimbal_direct_pid.pitch_velocity,PID_POSITION,gimbal_pitch_velocity,MAX_OUT_GIMBAL_PITCH_VELOCITY,MAX_IOUT_GIMBAL_PITCH_VELOCITY);
	 
	 LowPassFilterInit(&Mouse_yaw_Filter,0.9);
	 LowPassFilterInit(&Mouse_pitch_Filter,0.9);
	 LowPassFilterInit(&yaw_speed_filter,0.9);
	 LowPassFilterInit(&pitch_speed_filter,0.9);



	 
SMC_Init(&yaw_smc, 5, 0.5, 0.005, 10, 0.8, 10);   //滑膜面参数C 增益K 控制死区单位是弧度   
	 
	 
   //step4 初始化电机
   MotorInit(&gimbal_direct.yaw,GIMBAL_DIRECT_YAW_ID,GIMBAL_DIRECT_YAW_CAN,GIMBAL_DIRECT_YAW_MOTOR_TYPE,GIMBAL_DIRECT_YAW_DIRECTION,1,DM_MODE_MIT);
   MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,1,DM_MODE_MIT);

	 
	 DmEnable(&gimbal_direct.yaw);
	 DmEnable(&gimbal_direct.pitch);

	 
	 
	 //step6 模式设置初始化
   gimbal_direct.mode=GIMBAL_ZERO_FORCE;
   gimbal_direct.last_mode = GIMBAL_ZERO_FORCE;
	 
	 
	 
	 
}


void GimbalSetMode(void)
{

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
	gimbal_direct.feedback_pos.roll=GetImuAngle(AX_PITCH);
  
  gimbal_direct.feedback_vel.pitch=GetImuVelocity(AX_ROLL);
  gimbal_direct.feedback_vel.yaw=GetImuVelocity(AX_YAW);

	
	gimbal_direct.duration = xTaskGetTickCount() - gimbal_direct.last_time;
  gimbal_direct.last_time = xTaskGetTickCount();
	
	if(gimbal_direct.yaw.fdb.state==0||gimbal_direct.pitch.fdb.state==0||(mtoc_mesasge.key_v>>14)==1)
	{
	 DmEnable(&gimbal_direct.yaw);
	 osDelay(1);
	 DmEnable(&gimbal_direct.pitch);
	 osDelay(1);
	 DmEnable(&gimbal_direct.yaw);
	 osDelay(1);
	 DmEnable(&gimbal_direct.pitch);
	 osDelay(1);
	}
	
}


/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) 
{	
	// 保存当前模式
	uint8_t current_mode = gimbal_direct.mode;
	// 更新新模式
	gimbal_direct.mode = mtoc_mesasge.mode;
	// 更新上一模式
	gimbal_direct.last_mode = current_mode;
	
	// 模式切换处理
	if (gimbal_direct.mode != gimbal_direct.last_mode)
	{
		// 切入手动模式
		if (gimbal_direct.mode == GIMBAL_HAND)
		{
			gimbal_direct.reference.pitch = gimbal_direct.feedback_pos.pitch;
			gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
		}
		// 切入初始化模式
		else if (gimbal_direct.mode == GIMBAL_INIT)
		{
			gimbal_direct.reference.pitch = 0;
			gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
		}
		// 其他模式切换可以在这里添加
	}
}
/*-------------------- Console --------------------*/
 bool is_small_spin_mode = false;
/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void) 
{
  if (gimbal_direct.mode == GIMBAL_ZERO_FORCE)
  {
    gimbal_direct.pitch.set.tor=0;
    gimbal_direct.yaw.set.tor=0;
  }
	   if (gimbal_direct.mode == GIMBAL_INIT)
  {
//     pitch串级
		gimbal_direct.reference.pitch = 0 ;
	  gimbal_direct.reference.pitch=fp32_constrain(gimbal_direct.reference.pitch,gimbal_direct.lower_limit.pitch,gimbal_direct.upper_limit.pitch);
    gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.pitch.direction *gimbal_direct.feedback_pos.pitch,gimbal_direct.reference.pitch);
    gimbal_direct.pitch.set.tor= PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.pitch.direction *gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);
		

//                    yaw轴-串级控制
		gimbal_direct.reference.yaw=GIMBAL_DIRECT_YAW_MID;
    gimbal_direct.yaw.set.vel=PID_calc(&DM_4310_pid.yaw_angle, gimbal_direct.yaw.fdb.pos, gimbal_direct.reference.yaw);
    gimbal_direct.yaw.set.tor=gimbal_direct.yaw.direction * PID_calc(&DM_4310_pid.yaw_velocity,gimbal_direct.feedback_vel.yaw,gimbal_direct.yaw.set.vel);
	}
	
  if (gimbal_direct.mode == GIMBAL_HAND&&mtoc_mesasge.mouse_press_r!=1)
  {

				if(mtoc_mesasge.receive_chassis_thing==1)//底盘开启小陀螺模式
				{
					if(!is_small_spin_mode)
					{
						// 模式切换时，将参考值设置为当前反馈位置，避免跳变
						gimbal_direct.reference.yaw = gimbal_direct.feedback_pos.yaw;
						is_small_spin_mode = true;
					}
					
					gimbal_direct.reference.yaw -= mtoc_mesasge.yaw_control * 0.000013;
					LowPassFilterCalc(&Mouse_yaw_Filter,mtoc_mesasge.mouse_RL);
					gimbal_direct.reference.yaw -= Mouse_yaw_Filter.out * 0.00005;
					float yaw_angle_diff = angle_difference(gimbal_direct.reference.yaw, gimbal_direct.feedback_pos.yaw);
					float corrected_yaw_target = gimbal_direct.feedback_pos.yaw + yaw_angle_diff;
					gimbal_direct.yaw.set.vel = PID_calc(&gimbal_direct_pid.yaw_angle, gimbal_direct.feedback_pos.yaw, corrected_yaw_target);
					gimbal_direct.yaw.set.tor = gimbal_direct.yaw.direction * PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.feedback_vel.yaw,gimbal_direct.yaw.set.vel);
				}
				else
				{
					
					is_small_spin_mode = false;
					LowPassFilterCalc(&Mouse_yaw_Filter,mtoc_mesasge.mouse_RL);
					fp32 yaw_control=float_constrain(Mouse_yaw_Filter.out*0.00007,-0.070,+0.070);

					gimbal_direct.reference.yaw -=mtoc_mesasge.yaw_control*0.00002+yaw_control;
					float yaw_angle_diff = angle_difference(gimbal_direct.reference.yaw, gimbal_direct.feedback_pos.yaw);
					float corrected_yaw_target = gimbal_direct.feedback_pos.yaw + yaw_angle_diff;
					gimbal_direct.yaw.set.vel = PID_calc(&gimbal_direct_pid.yaw_angle, gimbal_direct.feedback_pos.yaw, corrected_yaw_target);
					gimbal_direct.yaw.set.tor = gimbal_direct.yaw.direction * PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.feedback_vel.yaw,gimbal_direct.yaw.set.vel);
				}
      /*             pitch-串级控制                    */
		gimbal_direct.reference.pitch+=mtoc_mesasge.mouse_UD*0.00003;
		gimbal_direct.reference.pitch=fp32_constrain(gimbal_direct.reference.pitch,gimbal_direct.lower_limit.pitch,gimbal_direct.upper_limit.pitch);
		gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.pitch.direction *gimbal_direct.feedback_pos.pitch,gimbal_direct.reference.pitch);
        gimbal_direct.pitch.set.tor= PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.pitch.direction *gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);

	}
  if (gimbal_direct.mode == GIMBAL_HAND&&mtoc_mesasge.mouse_press_r==1)
  {	
    if(visionDataStu.mode >= 1)
    {
       gimbal_direct.reference.pitch=visionDataStu.pitch;
       gimbal_direct.reference.yaw  =visionDataStu.yaw;
    }    
   else{
//      gimbal_direct.reference.pitch=-gimbal_direct.feedback_pos.pitch;
//      gimbal_direct.reference.yaw=gimbal_direct.feedback_pos.yaw;
    }  
    gimbal_direct.reference.pitch=fp32_constrain(gimbal_direct.reference.pitch,gimbal_direct.lower_limit.pitch,gimbal_direct.upper_limit.pitch);
    gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.pitch.direction *gimbal_direct.feedback_pos.pitch,gimbal_direct.reference.pitch)+1.5*visionDataStu.pitch_vel;
    gimbal_direct.pitch.set.tor= PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.pitch.direction *gimbal_direct.feedback_vel.pitch,gimbal_direct.pitch.set.vel);

	 
	 // 使用角度差值归一化来处理跨越π边界的问题
    float yaw_angle_diff = angle_difference(gimbal_direct.reference.yaw, gimbal_direct.feedback_pos.yaw);
    float corrected_yaw_target = gimbal_direct.feedback_pos.yaw + yaw_angle_diff;
    gimbal_direct.yaw.set.vel=PID_calc(&gimbal_direct_pid.yaw_angle, gimbal_direct.feedback_pos.yaw, corrected_yaw_target)+1*visionDataStu.yaw_vel;
    gimbal_direct.yaw.set.tor=gimbal_direct.yaw.direction * PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.feedback_vel.yaw,gimbal_direct.yaw.set.vel);
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

		DmMitCtrl(&gimbal_direct.yaw,0,0);
		DmMitCtrl(&gimbal_direct.pitch,0,0);
		AutoGimbal_Heartbeat_Check();
	  C_communication_M();
}
uint8_t motor_offfline=0;
void C_communication_M() 
{ 
    // 将float类型的yaw转换为int16
    int16_t yaw_int16 = (int16_t)((gimbal_direct.yaw.fdb.pos) * 1000); // 假设放大1000倍保留精度，可根据实际需求调整
	  if(gimbal_direct.yaw.offline==1||gimbal_direct.pitch.offline==1)
		{
			motor_offfline=1;
		}
		else
		{
			motor_offfline=0;
		}
       CToM_sendControl(1, 0X666, yaw_int16,motor_offfline,visionDataStu.aim_live);
}

