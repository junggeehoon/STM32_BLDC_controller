/*
 * control_table.c
 *
 *  Created on: Sep 26, 2025
 *      Author: pscad
 */


#include "control_table.h"
#include "dynamixel_protocol.h"

// Define the global instance of our control table
volatile ControlTable_t control_table;

void control_table_init(void)
{
	control_table.model_number = DXL_MODEL_NUMBER;
	control_table.firmware_version = DXL_FIRMWARE_VERSION;
	control_table.operating_mode = CURRENT_CONTROL;
	control_table.id = DXL_ID;
	control_table.baud_rate = 2;
	control_table.torque_enable = 0;
	control_table.led = 0;
	control_table.realtime_tick = 0;

//	control_table.present_position = 2048;

	control_table.current_p_gain = 9800;
	control_table.current_i_gain = 180;

	control_table.velocity_p_gain = 160;
	control_table.velocity_i_gain = 300;

	control_table.position_p_gain = 90;
	control_table.position_d_gain = 4;
	control_table.position_i_gain = 10;

	control_table.pwm_limit = 600;
	control_table.velocity_limit = 131;
	control_table.current_limit = 2000;

	control_table.goal_pwm = 0;
	control_table.goal_current = 0;
	control_table.goal_velocity = 0;
	control_table.goal_position = 0;
}
