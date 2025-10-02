/*
 * control_table.h
 *
 *  Created on: Sep 26, 2025
 *      Author: pscad
 */

#ifndef INC_CONTROL_TABLE_H_
#define INC_CONTROL_TABLE_H_

#include <stdint.h>

typedef enum {
    CURRENT_CONTROL          = 0,
    VELOCITY_CONTROL         = 1,
    POSITION_CONTROL         = 3,
    EXTENDED_POSITION_CONTROL= 4,
    CURRENT_BASED_POSITION   = 5,
    PWM_CONTROL              = 16
} OperatingMode_t;

// Ensure Byte Alignment (Prevent compiler to add padding between struct members.)
#pragma pack(1)

typedef struct {
    // EEPROM Area
    uint16_t model_number;          // Address: 0, Length: 2
    uint32_t model_information;     // Address: 2, Length: 4
    uint8_t  firmware_version;      // Address: 6, Length: 1
    uint8_t  id;                    // Address: 7, Length: 1
    uint8_t  baud_rate;             // Address: 8, Length: 1
    uint8_t  return_delay_time;     // Address: 9, Length: 1
    uint8_t  drive_mode;            // Address: 10, Length: 1
    uint8_t  operating_mode;        // Address: 11, Length: 1
    uint8_t  secondary_id;          // Address: 12, Length: 1
    uint8_t  protocol_type;         // Address: 13, Length: 1
    uint16_t history_shutdown_volt; // Address: 14, Length: 2
    uint8_t  history_power_on_count;// Address: 16, Length: 1
    uint8_t  history_shutdown_count;// Address: 17, Length: 1
    uint8_t  history_shutdown_temp; // Address: 18, Length: 1
    uint8_t  _pad_19;               // Padding for address 19
    int32_t  homing_offset;         // Address: 20, Length: 4
    uint32_t moving_threshold;      // Address: 24, Length: 4
    uint16_t ecc_address;           // Address: 28, Length: 2
    uint8_t  ecc_error_count;       // Address: 30, Length: 1
    uint8_t  temperature_limit;     // Address: 31, Length: 1
    uint16_t max_voltage_limit;     // Address: 32, Length: 2
    uint16_t min_voltage_limit;     // Address: 34, Length: 2
    uint16_t pwm_limit;             // Address: 36, Length: 2
    uint16_t current_limit;         // Address: 38, Length: 2
    uint8_t  _pad_40_43[4];         // Padding for addresses 40-43
    uint32_t velocity_limit;        // Address: 44, Length: 4
    uint32_t max_position_limit;    // Address: 48, Length: 4
    uint32_t min_position_limit;    // Address: 52, Length: 4
    uint8_t  _pad_56_59[4];         // Padding for addresses 56-59
    uint8_t  startup_configuration; // Address: 60, Length: 1
    uint8_t  _pad_61;               // Padding for address 61
    uint8_t  pwm_slope;             // Address: 62, Length: 1
    uint8_t  shutdown;              // Address: 63, Length: 1

    // RAM Area (Addresses 64-230+)
	uint8_t  torque_enable;         // Address: 64, Length: 1
	uint8_t  led;                   // Address: 65, Length: 1
	uint8_t  _pad_66_67[2];         // Padding for addresses 66-67
	uint8_t  status_return_level;   // Address: 68, Length: 1
	uint8_t  registered_instruction;// Address: 69, Length: 1
	uint8_t  hardware_error_status; // Address: 70, Length: 1
	uint8_t  calibration_status;    // Address: 71, Length: 1
	uint16_t current_i_gain;        // Address: 72, Length: 2
	uint16_t current_p_gain;        // Address: 74, Length: 2
	uint16_t velocity_i_gain;       // Address: 76, Length: 2
	uint16_t velocity_p_gain;       // Address: 78, Length: 2
	uint16_t position_d_gain;       // Address: 80, Length: 2
	uint16_t position_i_gain;       // Address: 82, Length: 2
	uint16_t position_p_gain;       // Address: 84, Length: 2
	uint8_t  _pad_86_87[2];         // Padding for addresses 86-87
	uint16_t feedforward_2nd_gain;  // Address: 88, Length: 2
	uint16_t feedforward_1st_gain;  // Address: 90, Length: 2
	uint8_t  qc_command;            // Address: 92, Length: 1
	uint8_t  qc_response;           // Address: 93, Length: 1
	uint16_t qc_address;            // Address: 94, Length: 2
	uint16_t qc_data;               // Address: 96, Length: 2
	uint8_t  bus_watchdog;          // Address: 98, Length: 1
	uint8_t  _pad_99;               // Padding for address 99
	int16_t  goal_pwm;              // Address: 100, Length: 2
	int16_t  goal_current;          // Address: 102, Length: 2
	int32_t  goal_velocity;         // Address: 104, Length: 4
	uint32_t profile_acceleration;  // Address: 108, Length: 4
	uint32_t profile_velocity;      // Address: 112, Length: 4
	int32_t  goal_position;         // Address: 116, Length: 4
	int16_t  realtime_tick;         // Address: 120, Length: 2
	uint8_t  moving;                // Address: 122, Length: 1
	uint8_t  moving_status;         // Address: 123, Length: 1
	int16_t  present_pwm;           // Address: 124, Length: 2
	int16_t  present_current;       // Address: 126, Length: 2
	int32_t  present_velocity;      // Address: 128, Length: 4
	int32_t  present_position;      // Address: 132, Length: 4
	int32_t  velocity_trajectory;   // Address: 136, Length: 4
	int32_t  position_trajectory;   // Address: 140, Length: 4
	uint16_t present_input_voltage; // Address: 144, Length: 2
	uint8_t  present_temperature;   // Address: 146, Length: 1
	uint8_t  backup_ready;          // Address: 147, Length: 1
	uint16_t backup_count;          // Address: 148, Length: 2
	uint8_t  _pad_150;              // Padding for address 150
	uint8_t  operating_mode_ram;    // Address: 151, Length: 1
	uint8_t  _pad_152_159[8];       // Padding for addresses 152-159
	uint32_t build_date;            // Address: 160, Length: 4
	uint16_t position_raw;          // Address: 164, Length: 2
	uint16_t backup_size;           // Address: 166, Length: 2
	uint8_t _pad_168_227[60];       // Padding for addresses 168-227
	uint16_t eep_erase_cnt;      	// Address: 228, Length: 2
	uint8_t  eep_wr_offset;         // Address: 230, Length: 1
} ControlTable_t;

#pragma pack()

extern volatile ControlTable_t control_table;

// Function to initialize the control table with default values
void control_table_init(void);

#endif /* INC_CONTROL_TABLE_H_ */
