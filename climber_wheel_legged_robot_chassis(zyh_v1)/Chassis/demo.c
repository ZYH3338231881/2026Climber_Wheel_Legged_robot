static void LocomotionController(void)
{
    // 计算LQR增益
    float k[2][6];
    float x[6];
    
    // 定义变量存储分离后的力矩 (单位: Nm)
    float T_total[2] = {0}; // 总力矩
    float T_vel[2]   = {0}; // 仅由速度误差产生的力矩 (移动分量)
    float T_bal[2]   = {0}; // 剩下的平衡分量 (总 - 移动)
    
    float T_Tp_dummy[2]; // 用于占位

    for (uint8_t i = 0; i < 2; i++) 
    {
        GetK(CHASSIS.fdb.leg[i].rod.L0, k, CHASSIS.fdb.leg[i].is_take_off);

        // 状态向量
        x[0] = (x0_OFFSET + (CHASSIS.fdb.leg_state[i].theta     - CHASSIS.ref.leg_state[i].theta));
        x[1] = (x1_OFFSET + (CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.ref.leg_state[i].theta_dot));
        x[2] = x2_OFFSET + (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x); 
        x[3] = x3_OFFSET + (CHASSIS.fdb.leg_state[i].x_dot     - CHASSIS.ref.leg_state[i].x_dot);
        x[4] = x4_OFFSET + (CHASSIS.fdb.leg_state[i].phi       - CHASSIS.ref.leg_state[i].phi);
        x[5] = x5_OFFSET + (CHASSIS.fdb.leg_state[i].phi_dot   - CHASSIS.ref.leg_state[i].phi_dot);

        // 1. 计算标准 LQR 总输出
        CalcLQR(k, x, T_Tp_dummy);
        T_total[i] = T_Tp_dummy[0]; // 轮子力矩
			
				if (CHASSIS.fdb.body.is_slipping) {
            // 打滑时
            CHASSIS.cmd.leg[i].rod.Tp = T_Tp_dummy[1] * 0.15f; 
        } else {
            // 正常抓地
            CHASSIS.cmd.leg[i].rod.Tp = T_Tp_dummy[1]; 
        }
//        CHASSIS.cmd.leg[i].rod.Tp = T_Tp_dummy[1]; // 关节切向力(不动)

        // 2. 分离移动力矩

        T_vel[i] = k[0][3] * x[3]; 

        // 3. 计算平衡力矩
        T_bal[i] = T_total[i] - T_vel[i];
    }

		// 4. 计算旋转力矩 (Yaw)
    float T_yaw = 0.0f;
    
    // 打滑时，强制斩断两轮的差速扭矩
    if (CHASSIS.fdb.body.is_slipping) {
        PID_clear(&CHASSIS.pid.yaw_velocity); // 清除底层速度环的积分，防止落地瞬间爆发！
        T_yaw = 0.0f; 
    } 
    else {
        PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
        T_yaw = CHASSIS.pid.yaw_velocity.out; 
    }

    // 5. 功率控制数据
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
			if(CHASSIS.fdb.leg[i].is_take_off)
			{
				CHASSIS.cmd.leg[i].wheel.T = 0;
			}
			
			float max_theta = fabsf(CHASSIS.fdb.leg_state[0].theta);
			if (fabsf(CHASSIS.fdb.leg_state[1].theta) > max_theta) {
					max_theta = fabsf(CHASSIS.fdb.leg_state[1].theta);
			}
			// 2. 获取当前机体的绝对俯仰角
			float body_pitch = fabsf(CHASSIS.fdb.body.roll);

			// 3. 构建姿态危险惩罚指数 (设定安全死区，超过后开始产生压缩惩罚)

			if (max_theta > 0.50f) { // 摆角超过约 11 度开始惩罚
					CHASSIS.cmd.leg[i].wheel.T = 0;
			}

			if (body_pitch > 0.3f) { // 俯仰角超过约 8.5 度开始惩罚
					CHASSIS.cmd.leg[i].wheel.T = 0;
			}
		}
//
		
}


// 1. 动态获取当前允许的速度/旋转步长 (受功率严格制约)
    float current_vx_step = vx_step;
    float current_wz_step = dynamic_step; 
    
    // 功率越低，斜坡爬升越慢（不仅限制扭矩，更要限制目标期望的增长）
    if (move_scale < 0.3f) {
        current_vx_step *= 0.1f; // 极度缺电：基本锁死目标值
        current_wz_step *= 0.1f;
    } else if (move_scale < 0.8f) {
        current_vx_step *= 0.5f; // 功率吃紧：放缓加速和刹车力度
        current_wz_step *= 0.5f;
    }

    // 2. 判断前进/后退 (Vx) 的加速与减速状态
    bool is_decelerating = (fabsf(target_vx_raw) < fabsf(target_vx_ramp)) || 
                           (target_vx_raw * target_vx_ramp < 0);

    if (is_decelerating) 
    {
        // 减速状态 (刹车)：同样受电流热损耗 I^2R 限制，用受控的 current_vx_step 降速
        if (target_vx_raw > target_vx_ramp + current_vx_step) target_vx_ramp += current_vx_step;
        else if (target_vx_raw < target_vx_ramp - current_vx_step) target_vx_ramp -= current_vx_step;
        else target_vx_ramp = target_vx_raw;
    } 
    else 
    {
        // 加速状态
        if (move_scale < 0.3f) {
            // 不能加速，主动缓慢降速
            if (target_vx_ramp > current_vx_step) target_vx_ramp -= current_vx_step;
            else if (target_vx_ramp < -current_vx_step) target_vx_ramp += current_vx_step;
            else target_vx_ramp = 0.0f;
        } else {
            // 正常/受限 加速
            if (target_vx_raw > target_vx_ramp + current_vx_step) target_vx_ramp += current_vx_step;
            else if (target_vx_raw < target_vx_ramp - current_vx_step) target_vx_ramp -= current_vx_step;
            else target_vx_ramp = target_vx_raw;
        }
    }

    // 3. 旋转 (Wz) 斜坡处理：
    if (target_wz_raw > target_wz_ramp + current_wz_step) target_wz_ramp += current_wz_step;
    else if (target_wz_raw < target_wz_ramp - current_wz_step) target_wz_ramp -= current_wz_step;
    else target_wz_ramp = target_wz_raw;
    
    // 最终赋值给设定值
    v_set.vx = target_vx_ramp;   
    v_set.wz = target_wz_ramp;