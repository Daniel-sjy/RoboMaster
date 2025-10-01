#include "cmsis_os.h"
#include "can.hpp"
#include "motor/motor.hpp"

void enable_dm_motor() {
    dm_motor_x.write_clear_error(can1.tx_data);
    can1.send(dm_motor_x.tx_id);
    osDelay(1);
    dm_motor_x.write_enable(can1.tx_data);
    can1.send(dm_motor_x.tx_id);
    osDelay(1);
}

void send_dm_motor() {
	dm_motor_x.write(can1.tx_data);
    can1.send(dm_motor_x.tx_id);
}

void send_rm_motor() {
    rm_motor_x.write(can1.tx_data);
    can1.send(rm_motor_x.tx_id);
}

void send_lk_motor() {
    lk_motor_x.write(can1.tx_data);
    can1.send(lk_motor_x.tx_id);
}