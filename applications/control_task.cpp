#include "cmsis_os.h"
#include "can/can.hpp"
#include "motor/motor.hpp"
#include "uart/uart.hpp"
#include "tools/pid/pid.hpp"

GimbalData dm_motor_data;
sp::DBusSwitchMode last_sw_l = remote.sw_l;

//                           dt     kp    ki    kd    mo   mio   alpha  ang? dynamic?
sp::PID dm_motor_pid_angle(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, true, true);
sp::PID dm_motor_pid_speed(0.001f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, false, true);

extern "C" void control_task()
{
    remote.request();

	can1.config();
	can1.start();
	can2.config();
	can2.start();

    // 初始化
    dm_motor_data.absolute_angle_set = dm_motor_x.angle;
    dm_motor_data.absolute_speed_set = 0.0f;
    dm_motor_data.given_torque = 0.0f;

    while (true) {

        // 电机使能控制计数，周期为15s
        if (motor_enable_freq == 5000) motor_enable = true;
        motor_enable_freq = (motor_enable_freq + 1) % 15000;

        if (motor_enable) {
            enable_dm_motor();
            motor_enable = false;
            continue;
        }


        if (remote.sw_r == sp::DBusSwitchMode::MID) {

            // 双环pid
            // if (remote.sw_l == sp::DBusSwitchMode::DOWN && last_sw_l == sp::DBusSwitchMode::MID)
            //     dm_motor_data.absolute_angle_set = dm_motor_x.angle + 0.0f;

            // dm_motor_pid_angle.calc(dm_motor_data.absolute_angle_set, dm_motor_x.angle);
            // dm_motor_data.absolute_speed_set = dm_motor_pid_angle.out;
            // dm_motor_pid_speed.calc(dm_motor_data.absolute_speed_set, dm_motor_x.speed);
            // dm_motor_data.given_torque = dm_motor_pid_speed.out;

            // 单速度环
            dm_motor_pid_speed.calc(dm_motor_data.absolute_speed_set, dm_motor_x.speed);
            dm_motor_data.given_torque = dm_motor_pid_speed.out;
        }
        else {
            dm_motor_data.given_torque = 0.0f;
        }
        
        dm_motor_x.cmd(dm_motor_data.given_torque);
        send_dm_motor();

        last_sw_l = remote.sw_l;

        osDelay(1);
    }

    return;
}
