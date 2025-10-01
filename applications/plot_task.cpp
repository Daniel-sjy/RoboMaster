#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "can/can.hpp"
#include "motor/motor.hpp"
#include "uart/uart.hpp"

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
  while (true) {
    plotter.plot(

      lk_motor_x.angle,
      lk_motor_x.speed,
      lk_motor_x.torque,

      // rm_motor_x.angle,
      // rm_motor_x.speed,
      // rm_motor_x.torque,

      dm_motor_x.angle,
      dm_motor_x.speed,
      dm_motor_x.torque
    
    );

    osDelay(10);  // 100Hz
  }
}