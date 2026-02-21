# 2026Climber Wheel-Legged Robot

常州大学Climber战队2026串联腿步兵控制算法开源项目

## 团队成员

- 机械设计: cxp
- 电控设计: zyh

## 机器人概览

<div align="center">
  <img src="./图片/整体照片1.png" width="45%" style="display: inline-block; margin-right: 10px;">
  <img src="./图片/整体照片2.png" width="45%" style="display: inline-block;">
</div>

## 技术架构

### 硬件配置
- **射击系统**: 2个3508电机 + 1个2006电机
- **云台系统**: 2个6020电机
- **底盘关节**: 4个达妙8009P电机
- **轮毂驱动**: 3508电机

### 控制算法
#### 云台控制
- Yaw轴: SMC滑膜控制（预留串级PID接口）
- Pitch轴: 串级PID控制

#### 底盘控制
- 运动学模型: 五连杆建模
- 控制算法: LQR控制器
- 单腿模型: 包含气弹簧处理（基于虚功原理）

## 功能特点

- 轮腿复合式移动机构
- 高精度云台控制系统
- 自适应地形能力

## 使用说明

详细的操作流程和注意事项将在后续更新中完善。