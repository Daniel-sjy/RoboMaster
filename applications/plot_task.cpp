#include "can/can.hpp"
#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "motor/motor.hpp"
#include "uart/uart.hpp"

extern sp::DBus remote;

extern sp::RM_Motor rm_motor_lf;
extern sp::RM_Motor rm_motor_lb;
extern sp::RM_Motor rm_motor_rf;
extern sp::RM_Motor rm_motor_rb;

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
  while (true) {
    plotter.plot(remote.ch_lv, rm_motor_lf.speed);

    osDelay(10);  // 100Hz
  }
}