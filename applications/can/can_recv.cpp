#include "can.hpp"
#include "cmsis_os.h"
#include "motor/motor.hpp"

void can1_recv(uint32_t stamp_ms) { can1.recv(); }

void can2_recv(uint32_t stamp_ms)
{
  can2.recv();

  if (can2.rx_id == rm_motor_lf.rx_id) {
    rm_motor_lf.read(can2.rx_data, stamp_ms);
  }
  else if (can2.rx_id == rm_motor_lb.rx_id) {
    rm_motor_lb.read(can2.rx_data, stamp_ms);
  }
  else if (can2.rx_id == rm_motor_rf.rx_id) {
    rm_motor_rf.read(can2.rx_data, stamp_ms);
  }
  else if (can2.rx_id == rm_motor_rb.rx_id) {
    rm_motor_rb.read(can2.rx_data, stamp_ms);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1)
      can1_recv(stamp_ms);

    else if (hcan == &hcan2)
      can2_recv(stamp_ms);
  }
}