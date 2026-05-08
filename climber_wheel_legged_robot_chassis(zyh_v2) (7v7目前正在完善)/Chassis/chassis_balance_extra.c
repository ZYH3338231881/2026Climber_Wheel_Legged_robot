#include "chassis_balance_extra.h"
#include "math.h"
#include "chassis_task.h"
#include "stdio.h"
#include "usart.h"
#include "dsp/fast_math_functions.h"    
#include "arm_math.h"
extern Chassis_s CHASSIS ;

/**
 * @brief 获取K矩阵
 * @param[in]  l 腿长
 * @param[out] k K矩阵  
 */
 
 void GetK(float l, float k[2][6], bool is_take_off)
{
    float t1 = l;
    float t2 = l * l; 
    float t3 = l * l * l;

if(CHASSIS.mode==CHASSIS_STAND_UP)  
{
 k[0][0] = -198.5526f * t3 + 240.5928f * t2 + -122.8236f * t1 + -6.0633f;
 k[0][1] = 0.2905f * t3 + 0.5795f * t2 + -8.4417f * t1 + -0.3246f;
 k[0][2] = -18.6488f * t3 + 22.9639f * t2 + -10.3291f * t1 + -1.3522f;
 k[0][3] = -12.1718f * t3 + 17.0661f * t2 + -9.2703f * t1 + -2.6949f;
 k[0][4] = -69.9934f * t3 + 126.2554f * t2 + -87.6682f * t1 + 28.4210f;
 k[0][5] = -20.9971f * t3 + 28.8769f * t2 + -16.3728f * t1 + 4.9776f;
k[1][0] = 141.8965f * t3 + -75.0062f * t2 + -33.0022f * t1 + 36.7073f;
k[1][1] = 13.2390f * t3 + -12.6443f * t2 + 2.6696f * t1 + 2.6246f;
// k[1][2] = -5.0505f * t3 + 22.8887f * t2 + -21.7466f * t1 + 7.9310f;
// k[1][3] = -51.3276f * t3 + 75.7273f * t2 + -44.7869f * t1 + 12.7109f;
// k[1][4] = 610.5073f * t3 + -705.5404f * t2 + 297.0660f * t1 + 13.3755f;
// k[1][5] = 97.5366f * t3 + -112.6718f * t2 + 47.8899f * t1 + 0.0769f;
}   
else if(CHASSIS.mode==CHASSIS_FREE)
{
//加装云台参数
k[0][0] = -49.8019f * t3 + 129.0930f * t2 + -104.2663f * t1 + 1.2802f;
k[0][1] = 18.9768f * t3 + -15.5600f * t2 + -3.4158f * t1 + 0.1196f;
k[0][2] = -8.7353f * t3 + 25.1947f * t2 + -17.9838f * t1 + 0.8085f;
k[0][3] = -8.0550f * t3 + 24.8828f * t2 + -18.7111f * t1 + 0.6119f;
k[0][4] = 401.4921f * t3 + -307.4983f * t2 + 14.5249f * t1 + 36.2099f;
k[0][5] = 12.5739f * t3 + -6.9702f * t2 + -4.3708f * t1 + 4.1889f;
k[1][0] = 1478.5027f * t3 + -1361.3845f * t2 + 299.3170f * t1 + 59.4659f;
k[1][1] = 112.3038f * t3 + -133.1143f * t2 + 50.3704f * t1 + 1.2789f;
k[1][2] = 266.2529f * t3 + -203.7179f * t2 + 10.7515f * t1 + 22.5716f;
k[1][3] = 153.4088f * t3 + -85.3909f * t2 + -33.8792f * t1 + 29.9701f;
k[1][4] = 1039.9078f * t3 + -2177.1517f * t2 + 1396.7532f * t1 + -86.0345f;
k[1][5] = 86.2313f * t3 + -178.5719f * t2 + 117.8528f * t1 + -13.1555f;
}

	//起飞过程中保留对腿部摆杆的控制  并且此时不是起立态
    if (is_take_off&&CHASSIS.mode==CHASSIS_FREE) {
        k[0][0] = 0;
        k[0][1] = 0;
        k[0][2] = 0;
        k[0][3] = 0;
        k[0][4] = 0;
        k[0][5] = 0;
        //k[1][0] = 0;
       // k[1][1] = 0;
        k[1][2] = 0;
        k[1][3] = 0;
        k[1][4] = 0;
        k[1][5] = 0;
    }
}


/**
 *正运动学分析
 * @brief 通过关节phi1和phi4的值获取L0和Phi0
 * @param[in]  phi1
 * @param[in]  phi4
 * @param[out] L0_Phi0 L0和Phi0
 */
void GetL0AndPhi0(float phi1, float phi4, float L0_Phi0[2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, phi2, XC, YC;
    float L0, Phi0;
    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    Phi0 = atan2(YC, (XC - LEG_L5 / 2));

    L0_Phi0[0] = L0;
    L0_Phi0[1] = Phi0;
}

/**
 *根据正运动学模型计算雅可比矩阵
 * @brief 计算雅可比矩阵
 * @param phi1 
 * @param phi4 
 * @param J 
 */
void CalcJacobian(float phi1, float phi4, float J[2][2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, XC, YC;
    float phi2, phi3;
    float L0, phi0;
    float j11, j12, j21, j22;

    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    phi3 = atan2(YB - YD + LEG_L2 * sin(phi2), XB - XD + LEG_L2 * cos(phi2));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    phi0 = atan2(YC, XC - LEG_L5 / 2);
    
    //JT
    j11 = (LEG_L1 * sin(phi0 - phi3) * sin(phi1 - phi2)) / sin(phi3 - phi2);
    j12 = (LEG_L4 * sin(phi0 - phi2) * sin(phi3 - phi4)) / sin(phi3 - phi2);
    j21 = (LEG_L1 * cos(phi0 - phi3) * sin(phi1 - phi2)) / (L0 * sin(phi3 - phi2));
    j22 = (LEG_L4 * cos(phi0 - phi2) * sin(phi3 - phi4)) / (L0 * sin(phi3 - phi2));
	  
		//J
    J[0][0] = j11;
    J[0][1] = j12;
    J[1][0] = j21;
    J[1][1] = j22;
}


//弹簧作用在竖直方向上的力 虚功原理 输入L0虚拟腿长
/*  l1大腿 l2小腿 Fs气弹簧的力300N
3.08923278 =177度
*/
float LegController_CalcSpringForce(float L0)
{
	float l1=0.210f,l2=0.250f,Fs=400.0f,s2=0.065f,l3=0.092f,l4=0.115f;
	float cos_theta3,sin_theta3,theta3,s3,alpha_s;
    float ls,Fv;//气弹簧长度、气弹簧沿着腿部的力
	
	cos_theta3=(l1*l1+l2*l2-L0*L0)/(2*l1*l2);
	theta3=acosf(cos_theta3);

    sin_theta3=arm_sin_f32(theta3);
    
    s3=sqrtf(l3*l3+l4*l4-2*l3*l4*arm_cos_f32(3.08923278f-theta3));
  
    alpha_s=asinf((l3/s3)*arm_sin_f32(3.08923278f-theta3));

    ls=sqrtf(s2*s2+s3*s3-2*s2*s3*arm_cos_f32(theta3-alpha_s));

    Fv=Fs*(s2*s3*arm_sin_f32(theta3-alpha_s)*L0)/(l1*l2*arm_sin_f32(theta3)*ls);
    return Fv;
}


/**
 * @brief 计算VMC
 * @param[in]  F0 沿杆方向的力
 * @param[in]  Tp 髋关节力矩
 * @param[in]  J 雅可比矩阵
 * @param[out] T 2个关节的输出力矩
 */
void CalcVmc(float F0, float Tp, float J[2][2], float T[2])
{
    // clang-format off
    float JT[2][2] = {{J[0][0],J[1][0]}, // 转置矩阵
                      {J[0][1],J[1][1]}};
    float F[2] = {F0, Tp};
    // clang-format on
    float T1 = JT[0][0] * F[0] + JT[0][1] * F[1];
    float T2 = JT[1][0] * F[0] + JT[1][1] * F[1];

    T[0] = T1;
    T[1] = T2;
}
/**
 * @brief 获取dL0和dPhi0
 * @param[in]  J 雅可比矩阵
 * @param[in]  d_phi1 
 * @param[in]  d_phi4 
 */
void GetdL0AnddPhi0(float J[2][2], float d_phi1, float d_phi4, float dL0_dPhi0[2])
{
    // clang-format off
    float d_l0   = J[0][0] * d_phi1 + J[0][1] * d_phi4;
    float d_phi0 = J[1][0] * d_phi1 + J[1][1] * d_phi4;
    // clang-format on
    dL0_dPhi0[0] = d_l0;
    dL0_dPhi0[1] = d_phi0;
}

/**
 * @brief 获取腿部摆杆的等效力
 * @param[in]  J 雅可比矩阵
 * @param[in]  T1 
 * @param[in]  T2 
 * @param[out] F 0-F0 1-Tp
 */
void GetLegForce(float J[2][2], float T1, float T2, float F[2])
{
    float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    // clang-format off
    float inv_J[4] = {J[1][1] / det, -J[0][1] / det, 
                     -J[1][0] / det,  J[0][0] / det};
    // clang-format on
    //F = (inv_J.') * T
    float F0 = inv_J[0] * T1 + inv_J[2] * T2;
    float Tp = inv_J[1] * T1 + inv_J[3] * T2;

    F[0] = F0;
    F[1] = Tp;
}
/**
 * @brief 通过当前底盘姿态和目标roll角计算两腿长度期望差值
 * @param[in]  Ld0 (m)当前左右腿长度差值(L0l - L0r)
 * @param[in]  theta0 (rad)当前底盘roll角
 * @param[in]  theta1 (rad)目标roll角
 * @return 两腿长度期望差值(m)(L1l - L1r)
 */
inline float CalcLegLengthDiff(float Ld0, float theta0, float theta1)
{
    return WHEEL_BASE * tanf(theta1) -cosf(theta0) / cosf(theta1) * (WHEEL_BASE * tanf(theta0) - Ld0);
}
/*
 *函数简介:Pitch补偿
 *参数说明:左腿ΔL0
 *参数说明:右腿ΔL0
 *参数说明:左腿ΔF
 *参数说明:右腿ΔF
 *返回类型:无
 *备注:无
 */

//void Leg_Controller_Pitch_Control(float Pitch_Target,float *LeftLeg_DeltaL0,float *RightLeg_DeltaL0,float *LeftLeg_DeltaF,float *RightLeg_DeltaF)
//{
//	float Rl=WHEEL_BASE/2; //轮子轴距一半
//	float Delta_L0=CHASSIS.fdb.leg[1].rod.L0-CHASSIS.fdb.leg[0].rod.L0;//右腿减去左腿
//	float BC=Delta_L0*arm_cos_f32(CHASSIS.lpf.pitch.out-Pitch_Target)-2.0f*Rl*arm_sin_f32(CHASSIS.lpf.pitch.out-Pitch_Target);
//	float FD=Delta_L0*arm_sin_f32(CHASSIS.lpf.pitch.out-Pitch_Target)+2.0f*Rl*arm_cos_f32(CHASSIS.lpf.pitch.out-Pitch_Target);
//	float tan_delta,L0d_r,L0d_l;
//	if(FD==0){L0d_r=L0d_l=0;}
//	else
//	{
//		tan_delta=BC/FD;
//		L0d_r=Rl*tan_delta;
//		L0d_l=-Rl*tan_delta;
//	}
//	(*LeftLeg_DeltaL0)=L0d_l;
//	(*RightLeg_DeltaL0)=L0d_r;
//	//暂时未考虑高速离心力补偿
//}

/**
 * @brief 通过L0和Phi0的值计算关节phi1和phi4
 * @param[in]  phi0
 * @param[in]  l0
 * @param[out] phi1_phi4 phi1和phi4
 * @note 用于位置控制时求逆解
 */
void CalcPhi1AndPhi4(float phi0, float l0, float phi1_phi4[2])
{
    float L5_2_pow;
    float Lca2, Lce2;
    float cos_phi11, cos_phi12, cos_phi41, cos_phi42;
    float phi11, phi12, phi41, phi42;
    float phi1, phi4;

    L5_2_pow = (LEG_L5 / 2) * (LEG_L5 / 2);  //(LEG_L5 / 2)^2
    Lca2 = l0 * l0 + L5_2_pow + l0 * LEG_L5 * cos(phi0);
    Lce2 = l0 * l0 + L5_2_pow - l0 * LEG_L5 * cos(phi0);

    cos_phi11 = (L5_2_pow + Lca2 - l0 * l0) / (LEG_L5 * sqrt(Lca2));
    cos_phi12 = (LEG_L1 * LEG_L1 + Lca2 - LEG_L2 * LEG_L2) / (2 * LEG_L1 * sqrt(Lca2));
    cos_phi41 = (L5_2_pow + Lce2 - l0 * l0) / (LEG_L5 * sqrt(Lce2));
    cos_phi42 = (LEG_L4 * LEG_L4 + Lce2 - LEG_L3 * LEG_L3) / (2 * LEG_L5 * sqrt(Lce2));

    phi11 = acos(cos_phi11);
    phi12 = acos(cos_phi12);

    phi41 = acos(cos_phi41);
    phi42 = acos(cos_phi42);

    phi1 = phi11 + phi12;
    phi4 = M_PI - (phi41 + phi42);

    phi1_phi4[0] = phi1;
    phi1_phi4[1] = phi4;
}

void Chassis_ModelJump(void)
{
  #define shoutuiTime			150
	#define shangtuiTime		120
	#define suotuiTime			100
	#define luodiTime			40
	
	
}

