#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "IMU_task.h"
#include "MahonyAHRS.h"
#include "pid.h"
#include "IMU_task.h"
#include "chassis_task.h"
#define cheat TRUE  //作弊模式 去掉较小的gyro值
#define correct_Time_define 1000    //上电去0飘 1000次取平均
#define temp_times 300       //探测温度阈值
pid_type_def Temperature_PID={0};
float Temperature_PID_Para[3]={1600,50,40};
extern Chassis_s CHASSIS;
float gyro[3], accel[3], temp;
float gyro_correct[3]={0};
float RefTemp = 40;   //Destination
float roll,pitch,yaw=0;
uint8_t attitude_flag=2;   //可以手动设置  1开始1000次零漂校准  2直接算出零漂值
uint32_t correct_times=0;


void INS_Init(void)
{
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf初始化
		PID_init(&Temperature_PID, PID_POSITION,Temperature_PID_Para,2000,200); //加热pidlimit
		Mahony_Init(1000);  //mahony姿态解算初始化
    // imu heat init
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init());  //陀螺仪初始化
}

uint32_t temp_temperature=0;
void IMU_Temperature_Ctrl(){
	  PID_calc(&Temperature_PID, temp, RefTemp); //温度pid  //需要调一下pid使得温度在40°左右
		temp_temperature=(uint32_t)Temperature_PID.out; 
		if(Temperature_PID.out<0)
		{
			temp_temperature=0;
		}
    htim3.Instance->CCR4 = temp_temperature;
}
/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 任务参数
 * @retval: void
 * @details: IMU姿态控制任务函数
*/
static uint8_t first_mahony=0; 
void INS_Task(void)
{
     uint32_t count = 0;
    // ins update
    if ((count % 1) == 0)//永远成立每 1ms 执行一次
    {
        BMI088_read(gyro, accel, &temp);
				if(first_mahony==0)
				{
					first_mahony++;
					MahonyAHRSinit(accel[0],accel[1],accel[2],0,0,0);  
				}
				if(attitude_flag==2)  //ekf的姿态解算
				{
//					gyro[0]-=gyro_correct[0];   //减去陀螺仪0飘
//					gyro[1]-=gyro_correct[1];
//					gyro[2]-=gyro_correct[2];
					
					  gyro[0]-=0.00280696363;   //减去陀螺仪0飘
				  	gyro[1]-=-0.003578217;
				  	gyro[2]-=-0.00302640628;
					
					#if cheat              //作弊 可以让yaw很稳定 去掉比较小的值
						if(fabsf(gyro[2])<0.003f)
							gyro[2]=0;
					#endif
					//===========================================================================
						//ekf姿态解算部分
					IMU_QuaternionEKF_Update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]);//EKF姿态解算（高级版）
                    //得到
                        //QEKF_INS.q[0~3]   // 四元数（姿态）
                     //QEKF_INS.Roll
                     //QEKF_INS.Pitch
                     //QEKF_INS.Yaw
                     //QEKF_INS.GyroBias // 陀螺仪零漂
                        
					//mahony姿态解算部分
					Mahony_update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],0,0,0);
					Mahony_computeAngles(); //角度计算     把四元数转成欧拉角,移植到别的平台需要替换掉对应的arm_atan2_f32 和 arm_asin
					//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
					//=============================================================================
					//ekf获取姿态角度函数
					pitch=Get_Pitch()/57.29578f; //获得pitch  -pi 到  pi
					roll=Get_Roll()/57.29578f;//获得roll      -pi 到  pi
					yaw=Get_Yaw()/57.29578f;//获得yaw         -pi 到  pi
					
          
							
					//==============================================================================
                    CHASSIS.imu->angle[0]=roll;
					CHASSIS.imu->angle[1]=pitch;
                    CHASSIS.imu->angle[2]=yaw;

					CHASSIS.imu->gyro[0]=gyro[0];
				    CHASSIS.imu->gyro[1]=gyro[1];
					CHASSIS.imu->gyro[2]=gyro[2];

				    CHASSIS.imu->accel[0]=accel[0];
				    CHASSIS.imu->accel[1]=accel[1];
					CHASSIS.imu->accel[2]=accel[2];


						
						
				}
				else if(attitude_flag==1)   //状态1 开始1000次的陀螺仪0飘初始化
				{
						//gyro correct
						gyro_correct[0]+=	gyro[0];
						gyro_correct[1]+=	gyro[1];
						gyro_correct[2]+=	gyro[2];
						correct_times++;
						if(correct_times>=correct_Time_define)
						{
							gyro_correct[0]/=correct_Time_define;
							gyro_correct[1]/=correct_Time_define;
							gyro_correct[2]/=correct_Time_define;
							attitude_flag=2; //go to 2 state
						}
				}
    }

    // temperature control
    if ((count % 10) == 0)//每 10ms 执行一次温度控制
    {
        // 100hz 的温度控制pid
        IMU_Temperature_Ctrl();
				
				uint32_t temp_Ticks=0;
				if((fabsf(temp-RefTemp)<0.5f)&&attitude_flag==0) //接近额定温度之差小于0.5° 开始计数
				{
					temp_Ticks++;
					if(temp_Ticks>temp_times)   //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
					{
						attitude_flag=1;  //go to correct state
					}
				}
    }
    count++;
}

void imu_task(void  * argument)
{
    INS_Init();
    /* Infinite loop */
    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
}


/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }
}
