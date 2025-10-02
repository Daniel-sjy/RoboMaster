#include "can.hpp"

#include "cmsis_os.h"
#include "motor/motor.hpp"

void send_rm_motor()
{
  rm_motor_rf.write(can2.tx_data);
  rm_motor_lf.write(can2.tx_data);
  rm_motor_lb.write(can2.tx_data);
  rm_motor_rb.write(can2.tx_data);
  can2.send(rm_motor_rf.tx_id);
}