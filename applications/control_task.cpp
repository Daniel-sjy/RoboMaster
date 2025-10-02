#include "can/can.hpp"
#include "cmsis_os.h"
#include "motor/motor.hpp"
#include "tools/pid/pid.hpp"
#include "uart/uart.hpp"

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

  // 最大轮速
  const float TARGET_MAX_SPEED = 5.0f;

  // 旋转灵敏度系数
  const float ROTATION_FACTOR = 0.8f;

  while (true) {
    if (remote.sw_r == sp::DBusSwitchMode::MID) {
      // 1. 获取遥控器输入
      float Vx_input = remote.ch_lv;
      float W_input = remote.ch_rh;

      // 2. 计算速度分量 (V_x 和 Omega)

      // V_x (前进/后退速度)
      float Vx = Vx_input * TARGET_MAX_SPEED;

      // W (净转向角速度) = W_input * 灵敏度
      float W = W_input * TARGET_MAX_SPEED * ROTATION_FACTOR;

      // // Vy (左右侧移速度) 暂时设为 0
      // float Vy = 0.0f;

      // 3. 麦轮反运动学计算目标轮速 (V_i = Vx ± Vy ± W)
      // 简化后：V_i = Vx ± W

      // 左侧轮 (使用 Vx + W)
      float target_lf = Vx + W;
      float target_lb = Vx + W;

      // 右侧轮 (使用 Vx - W)
      float target_rf = Vx - W;
      float target_rb = Vx - W;

      // 4. PID 计算与指令发送

      rm_motor_pid_lf.calc(target_lf, rm_motor_lf.speed);
      rm_motor_lf.cmd(rm_motor_pid_lf.out);

      rm_motor_pid_lb.calc(target_lb, rm_motor_lb.speed);
      rm_motor_lb.cmd(rm_motor_pid_lb.out);

      rm_motor_pid_rf.calc(target_rf, rm_motor_rf.speed);
      rm_motor_rf.cmd(rm_motor_pid_rf.out);

      rm_motor_pid_rb.calc(target_rb, rm_motor_rb.speed);
      rm_motor_rb.cmd(rm_motor_pid_rb.out);

      // 5. 存储 PID 输出用于调试 (使用 lf 数据作为示例)
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

    // 确保在控制逻辑结束后发送指令
    send_rm_motor();

    osDelay(1);
  }

  return;
}
