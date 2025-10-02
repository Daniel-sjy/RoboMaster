#include <cmath>

#include "can/can.hpp"
#include "cmsis_os.h"
#include "motor/motor.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"
#include "uart/uart.hpp"

// 麦轮几何参数定义
const float WHEEL_RADIUS = 0.077f;  // 轮子半径 R (米)
const float HALF_LENGTH = 0.165f;   // 前后轮距离的一半 L (米)
const float HALF_WIDTH = 0.185f;    // 左右轮距离的一半 W (米)

// 实例化 Mecanum 对象
// 参数: R, L, W, reverse_lf, reverse_lr, reverse_rf, reverse_rr
sp::Mecanum mecanum_chassis(WHEEL_RADIUS, HALF_LENGTH, HALF_WIDTH, false, false, true, true);

GimbalData rm_motor_data;
sp::DBusSwitchMode last_sw_l = remote.sw_l;

//                        dt     kp     ki    kd    mo    mio  alpha  ang? dynamic?
sp::PID rm_motor_pid_lf(0.001f, 1.0f, 0.05f, 0.0f, 3.0f, 0.5f, 0.3f, false, true);  // 左前
sp::PID rm_motor_pid_lb(0.001f, 1.0f, 0.05f, 0.0f, 3.0f, 0.5f, 0.3f, false, true);  // 左后
sp::PID rm_motor_pid_rf(0.001f, 1.0f, 0.05f, 0.0f, 3.0f, 0.5f, 0.3f, false, true);  // 右前
sp::PID rm_motor_pid_rb(0.001f, 1.0f, 0.05f, 0.0f, 3.0f, 0.5f, 0.3f, false, true);  // 右后

extern "C" void control_task()
{
  remote.request();

  can1.config();
  can1.start();
  can2.config();
  can2.start();

  // 初始化电机数据
  rm_motor_data.absolute_angle_set = 0.0f;
  rm_motor_data.absolute_speed_set = 0.0f;
  rm_motor_data.given_torque = 0.0f;

  // 最大底盘线速度 (Vx/Vy Max, 单位: m/s)
  const float MAX_LINEAR_VELOCITY = 1.0f;
  // 最大底盘角速度 (Wz Max, 单位: rad/s)
  const float MAX_ANGULAR_VELOCITY = 2.0f;

  while (true) {
    if (remote.sw_r == sp::DBusSwitchMode::MID) {
      // 1. 获取遥控器输入
      float Vx_input = remote.ch_lv;
      float Vy_input = remote.ch_lh;
      float W_input = remote.ch_rh;

      // 计算底盘速度分量 (V_x, V_y 和 W_z)

      // V_x (前进/后退速度, m/s)
      float Vx = Vx_input * MAX_LINEAR_VELOCITY;

      // V_y (左右侧移速度, m/s)
      float Vy = Vy_input * MAX_LINEAR_VELOCITY;

      // W_z (净转向角速度, rad/s)
      float Wz = W_input * MAX_ANGULAR_VELOCITY;

      // 计算结果 (rad/s) 存储在 mecanum_chassis.speed_lf/lr/rf/rr 中
      mecanum_chassis.calc(Vx, Vy, Wz);

      // 目标转速单位: rad/s
      float target_lf = mecanum_chassis.speed_lf;
      float target_lb = mecanum_chassis.speed_lr;
      float target_rf = mecanum_chassis.speed_rf;
      float target_rb = mecanum_chassis.speed_rr;

      // PID 计算与指令发送
      rm_motor_pid_lf.calc(target_lf, rm_motor_lf.speed);
      rm_motor_lf.cmd(rm_motor_pid_lf.out);

      rm_motor_pid_lb.calc(target_lb, rm_motor_lb.speed);
      rm_motor_lb.cmd(rm_motor_pid_lb.out);

      rm_motor_pid_rf.calc(target_rf, rm_motor_rf.speed);
      rm_motor_rf.cmd(rm_motor_pid_rf.out);

      rm_motor_pid_rb.calc(target_rb, rm_motor_rb.speed);
      rm_motor_rb.cmd(rm_motor_pid_rb.out);

      // 存储 PID 输出用于调试
      rm_motor_data.absolute_speed_set = target_lf;
      rm_motor_data.given_torque = rm_motor_pid_lf.out;
    }
    else {
      // 拨杆不在 MID 位置，重置所有 PID 状态并停止所有电机
      rm_motor_data.absolute_speed_set = 0.0f;

      rm_motor_lf.cmd(0.0f);
      rm_motor_lb.cmd(0.0f);
      rm_motor_rf.cmd(0.0f);
      rm_motor_rb.cmd(0.0f);

      rm_motor_data.given_torque = 0.0f;
    }

    send_rm_motor();

    osDelay(1);
  }

  return;
}
