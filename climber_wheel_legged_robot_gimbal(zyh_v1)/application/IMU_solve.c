/**
 ******************************************************************************
 * @file    IMU_solve.c/h
 * @brief   6轴陀螺仪解算部分，通过EKF算法滤除噪声，得到姿态角。
 *          解算得到 姿态角、角速度、加速度
 * @note    仅在IMU解算任务中使用，不对其他模块外提供接口
 *          禁止在其他模块中导入该文件
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025-04-05      Penguin         1. done
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 ******************************************************************************
 @verbatim
 ==============================================================================
 详细原理参考王工的文章：
    四元数EKF姿态更新算法 https://zhuanlan.zhihu.com/p/454155643

 ==============================================================================
 @endverbatim
*/

#include "IMU_solve.h"
#include "stdbool.h"
#define __GYRO_BIAS_YAW  0.003096855f   
#define TRUE 1
#define FALSE 0

/*******************************************************************************/
/* 欧拉角解算需要用到的变量                                                      */
/*******************************************************************************/

// clang-format off
float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0,
                                 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0,
                                 0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 10000, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 10000};
float IMU_QuaternionEKF_Q[36] = {0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.000001, 0,
                                 0, 0, 0, 0, 0, 0.000001};
float IMU_QuaternionEKF_R[9] = {1000000, 0, 0,
                                0, 1000000, 0,
                                0, 0, 1000000};

float IMU_QuaternionEKF_K[36] = {1, 0, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0,
                                 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0,
                                 0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_H[36] = {1, 0, 0, 0, 0, 0,
                                 0, 1, 0, 0, 0, 0,
                                 0, 0, 1, 0, 0, 0,
                                 0, 0, 0, 1, 0, 0,
                                 0, 0, 0, 0, 1, 0,
                                 0, 0, 0, 0, 0, 1};
// clang-format on

INS_t INS = {0};

/*******************************************************************************/
/* 加速度解算需要用到的变量                                                      */
/*******************************************************************************/

// clang-format off
KalmanFilter_t gEstimateKF;	   // 卡尔曼滤波器结构体
float gVec[3];    // 重力加速度向量 用于存储估计值供其他函数调用

float gEstimateKF_F[9] = {1, 0, 0,
                          0, 1, 0,
                          0, 0, 1};	   // 状态转移矩阵其余项在滤波器更新时更新
float gEstimateKF_P[9] = {100, 0.1, 0.1,
                          0.1, 100, 0.1,
                          0.1, 0.1, 100};    // 后验估计协方差初始值
static float gEstimateKF_Q[9] = {0.01, 0, 0,
                                 0, 0.01, 0,
                                 0, 0, 0.01};    // Q矩阵初始值（其实这里设置多少都无所谓）
static float gEstimateKF_R[9] = {100000, 0, 0,
                                 0, 100000, 0,
                                 0, 0, 100000};    // R矩阵初始值（其实这里设置多少都无所谓）
float gEstimateKF_K[9];
const float gEstimateKF_H[9] = {1, 0, 0,
                                0, 1, 0,
                                0, 0, 1};	// 由于不需要异步量测自适应，这里直接设置矩阵H为常量
// clang-format on


/*******************************************************************************/
/* 函数部分                                                                     */
/*******************************************************************************/

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*******************************************************************************/
/* User Functions                                                              */
/*     IMU_QuaternionEKF_User_Func1                                            */
/*******************************************************************************/

static void IMU_QuaternionEKF_User_Func1(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;
	static float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // 四元数归一化
    qInvNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
	// 补充F矩阵
    kf->F_data[4] = q1 * INS.dt / 2;
    kf->F_data[5] = q2 * INS.dt / 2;

    kf->F_data[10] = -q0 * INS.dt / 2;
    kf->F_data[11] = q3 * INS.dt / 2;

    kf->F_data[16] = -q3 * INS.dt / 2;
    kf->F_data[17] = -q0 * INS.dt / 2;

    kf->F_data[22] = q2 * INS.dt / 2;
    kf->F_data[23] = -q1 * INS.dt / 2;
}

static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    static float doubleq0, doubleq1, doubleq2, doubleq3;
    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    */

    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];
    // 复位H矩阵为0矩阵
    memset(kf->H_data, 0, sizeof_float * kf->zSize * kf->xhatSize);

    // 设置H矩阵
    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;
}

static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static float q0, q1, q2, q3;

    // 计算残差方差 inv(H·P'(k)·HT + R)
    kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
    kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    // 计算h(xhat'(k))
    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];
    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算残差z(k) - h(xhat'(k))
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // 卡方检验 计算检验函数r
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix); // temp_matrix = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    kf->MatStatus = Matrix_Transpose(&kf->temp_vector1, &kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    kf->temp_matrix.numRows = 1;
    kf->temp_matrix.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->temp_vector, &kf->temp_vector1, &kf->temp_matrix);

    // 检测函数
    INS.ChiSquare = kf->temp_matrix.pData[0];
    if (INS.ChiSquare < 0.1f * INS.ChiSquareTestThreshold)
        INS.ConvergeFlag = 1;
    if (INS.ChiSquare > INS.ChiSquareTestThreshold && INS.ConvergeFlag)
    {
        // 未通过卡方检验 仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
        kf->SkipEq5 = TRUE;
        return;
    }
    else
    {
        // 应用渐消因子
        kf->P_data[28] /= INS.lambda;
        kf->P_data[35] /= INS.lambda;
        kf->SkipEq5 = FALSE;
    }

    // 通过卡方检验，进行量测更新
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
    kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
    kf->temp_vector.pData[3] = 0;	// 应用M矩阵
    kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);// xhat = xhat'(k) + M·K(k)·(z(k) - h(xhat'(k)))
}

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    memcpy(IMU_QuaternionEKF_P, kf->P_data, sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K, kf->K_data, sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H, kf->H_data, sizeof(IMU_QuaternionEKF_H));
}

/*******************************************************************************/
/* Main Functions                                                              */
/*     gEstimateKF_Init                                                        */
/*     gEstimateKF_Update                                                      */
/*     IMU_QuaternionEKF_Init                                                  */
/*     IMU_QuaternionEKF_Update                                                */
/*******************************************************************************/

void gEstimateKF_Init(float process_noise, float measure_noise)
{
    for (uint8_t i = 0; i < 9; i += 4)
    {
        // 初始化过程噪声与量测噪声
        gEstimateKF_Q[i] = process_noise;
        gEstimateKF_R[i] = measure_noise;
    }

    Kalman_Filter_Init(&gEstimateKF, 3, 0, 3);	// 状态向量3维 无控制部分 测量向量3维
    // gEstimateKF.User_Func0_f = gEstimateKF_Tuning;
    memcpy(gEstimateKF.F_data, gEstimateKF_F, sizeof(gEstimateKF_F));
    memcpy(gEstimateKF.P_data, gEstimateKF_P, sizeof(gEstimateKF_P));
    memcpy(gEstimateKF.Q_data, gEstimateKF_Q, sizeof(gEstimateKF_Q));
    memcpy(gEstimateKF.R_data, gEstimateKF_R, sizeof(gEstimateKF_R));
    memcpy(gEstimateKF.H_data, gEstimateKF_H, sizeof(gEstimateKF_H));
}

void gEstimateKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{

    // 空间换时间 避免重复运算
    static float gxdt, gydt, gzdt;
    gxdt = gx * dt;
    gydt = gy * dt;
    gzdt = gz * dt;

    // 由于本例中状态转移矩阵为时变矩阵
    // 需要在卡尔曼滤波器更新前更新转移矩阵F的值
    gEstimateKF.F_data[1] = gzdt;
    gEstimateKF.F_data[2] = -gydt;

    gEstimateKF.F_data[3] = -gzdt;
    gEstimateKF.F_data[5] = gxdt;

    gEstimateKF.F_data[6] = gydt;
    gEstimateKF.F_data[7] = -gxdt;

    // 卡尔曼滤波器测量值更新
    // 不一定写在滤波器更新函数之前，也可写在与传感器通信的回调函数中
    gEstimateKF.MeasuredVector[0] = ax;
    gEstimateKF.MeasuredVector[1] = ay;
    gEstimateKF.MeasuredVector[2] = az;

    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(&gEstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 3; i++)
    {
        gVec[i] = gEstimateKF.FilteredValue[i];
    }
}

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 */
void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda)
{
    INS.Q1 = process_noise1;
    INS.Q2 = process_noise2;
    INS.R = measure_noise;
    INS.ChiSquareTestThreshold = 0.01f;
    INS.ConvergeFlag = 0;
    if (lambda > 1)
        lambda = 1;
    INS.lambda = lambda;
    Kalman_Filter_Init(&INS.IMU_QuaternionEKF, 6, 0, 3);
    INS.IMU_QuaternionEKF.xhat_data[0] = 1;
    INS.IMU_QuaternionEKF.xhat_data[1] = 0;
    INS.IMU_QuaternionEKF.xhat_data[2] = 0;
    INS.IMU_QuaternionEKF.xhat_data[3] = 0;
    INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_User_Func1;
    INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;
    INS.IMU_QuaternionEKF.SkipEq3 = TRUE;
    INS.IMU_QuaternionEKF.SkipEq4 = TRUE;
    memcpy(INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    memcpy(INS.IMU_QuaternionEKF.P_data, IMU_QuaternionEKF_P, sizeof(IMU_QuaternionEKF_P));
    memcpy(INS.IMU_QuaternionEKF.Q_data, IMU_QuaternionEKF_Q, sizeof(IMU_QuaternionEKF_Q));
    memcpy(INS.IMU_QuaternionEKF.R_data, IMU_QuaternionEKF_R, sizeof(IMU_QuaternionEKF_R));
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z 
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    static float halfgxdt, halfgydt, halfgzdt;
	 static  float accelInvNorm;

    /*
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    18    19    20    21    22    23
    24    25    26    27    28    29
    30    31    32    33    34    35
    */
    INS.dt = dt;

    halfgxdt = 0.5f * (gx - INS.GyroBias[0]) * dt;
    halfgydt = 0.5f * (gy - INS.GyroBias[1]) * dt;
    halfgzdt = 0.5f * (gz - INS.GyroBias[2]) * dt;

    // 初始化F矩阵为单位阵
    memcpy(INS.IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));
    // 设置F矩阵用于过程更新
    INS.IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    INS.IMU_QuaternionEKF.F_data[2] = -halfgydt;
    INS.IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    INS.IMU_QuaternionEKF.F_data[6] = halfgxdt;
    INS.IMU_QuaternionEKF.F_data[8] = halfgzdt;
    INS.IMU_QuaternionEKF.F_data[9] = -halfgydt;

    INS.IMU_QuaternionEKF.F_data[12] = halfgydt;
    INS.IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    INS.IMU_QuaternionEKF.F_data[15] = halfgxdt;

    INS.IMU_QuaternionEKF.F_data[18] = halfgzdt;
    INS.IMU_QuaternionEKF.F_data[19] = halfgydt;
    INS.IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    // 归一化加速度向量作为量测向量
    accelInvNorm = invSqrt(ax * ax + ay * ay + az * az);
    INS.IMU_QuaternionEKF.MeasuredVector[0] = ax * accelInvNorm;
    INS.IMU_QuaternionEKF.MeasuredVector[1] = ay * accelInvNorm;
    INS.IMU_QuaternionEKF.MeasuredVector[2] = az * accelInvNorm;

    // 设置Q,R矩阵
    INS.IMU_QuaternionEKF.Q_data[0] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[7] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[14] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[21] = INS.Q1 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[28] = INS.Q2 * INS.dt;
    INS.IMU_QuaternionEKF.Q_data[35] = INS.Q2 * INS.dt;
    INS.IMU_QuaternionEKF.R_data[0] = INS.R;
    INS.IMU_QuaternionEKF.R_data[4] = INS.R;
    INS.IMU_QuaternionEKF.R_data[8] = INS.R;

    // 卡尔曼滤波器更新
    Kalman_Filter_Update(&INS.IMU_QuaternionEKF);

    // 估计结果导出
    INS.q[0] = INS.IMU_QuaternionEKF.FilteredValue[0];
    INS.q[1] = INS.IMU_QuaternionEKF.FilteredValue[1];
    INS.q[2] = INS.IMU_QuaternionEKF.FilteredValue[2];
    INS.q[3] = INS.IMU_QuaternionEKF.FilteredValue[3];
    INS.GyroBias[0] = INS.IMU_QuaternionEKF.FilteredValue[4];
    INS.GyroBias[1] = INS.IMU_QuaternionEKF.FilteredValue[5];
		
    INS.GyroBias[2] = __GYRO_BIAS_YAW; // 陀螺仪yaw零飘，单位rad/s(在参数文件中配置)

    // 四元数反解欧拉角
    INS.angle[0] = atan2f(2.0f * (INS.q[0] * INS.q[1] + INS.q[2] * INS.q[3]), 2.0f * (INS.q[0] * INS.q[0] + INS.q[3] * INS.q[3]) - 1.0f);
    INS.angle[1] = asinf(-2.0f * (INS.q[1] * INS.q[3] - INS.q[0] * INS.q[2]));
    INS.angle[2] = atan2f(2.0f * (INS.q[0] * INS.q[3] + INS.q[1] * INS.q[2]), 2.0f * (INS.q[0] * INS.q[0] + INS.q[1] * INS.q[1]) - 1.0f);
}


float GetEkfAngle(int i){
    return INS.angle[i];
}

float GetEkfAccel(int i){
    return gVec[i];
}
/*------------------------------ End of File ------------------------------*/