/*
 * dynamixel_protocol.c
 *
 *  Created on: Sep 26, 2025
 *      Author: pscad
 */


#include "dynamixel_protocol.h"
#include "control_table.h"
#include <string.h>

// We need access to the UART handle for transmission
extern UART_HandleTypeDef huart1;

// Buffer for building response packets
uint8_t txBuffer[255];

static void send_status_packet(uint8_t id, uint8_t error, uint8_t* params, uint16_t param_len);
static void handle_ping(uint8_t id, const uint8_t* params, uint16_t param_len);
static void handle_read(uint8_t id, const uint8_t* params, uint16_t param_len);
static void handle_write(uint8_t id, const uint8_t* params, uint16_t param_len);


bool process_packet(uint8_t* packetBuffer, uint16_t packetLength)
{
    uint16_t totalPacketSize = 7 + packetLength;
    uint16_t received_crc = (uint16_t)(packetBuffer[totalPacketSize - 1] << 8 | packetBuffer[totalPacketSize - 2]);
    uint16_t calculated_crc = update_crc(0, packetBuffer, totalPacketSize - 2);

    // CRC validation
    if (received_crc != calculated_crc) {
        return false;
    }

    uint8_t id = packetBuffer[4];
    uint8_t instruction = packetBuffer[7];
    uint8_t* params = &packetBuffer[8]; 	// Instruction parameters start at index 8
    uint16_t param_len = packetLength - 3;  // Length of param field = Total Len - Inst(1) - CRC(2)


    if (id == DXL_ID)
    {
        switch (instruction)
        {
            case PING:
                handle_ping(id, params, param_len);
                return true;

            case READ:
                handle_read(id, params, param_len);
                return true;

            case WRITE:
                handle_write(id, params, param_len);
                return true;

            default:
                break;
        }
    }

    else if (id == BROADCAST_ID)
    {
        switch (instruction)
        {
            case PING:
                handle_ping(id, params, param_len);
                return true;

            default:
                break;
        }
    }

    return false;
}


//==============================================================================
// Instruction Handlers (Implement your logic here)
//==============================================================================

static void handle_ping(uint8_t id, const uint8_t* params, uint16_t param_len)
{
    uint8_t response_params[3];
    response_params[0] = DXL_MODEL_NUMBER & 0xFF;
    response_params[1] = (DXL_MODEL_NUMBER >> 8) & 0xFF;
    response_params[2] = DXL_FIRMWARE_VERSION;
    send_status_packet(id, NO_ERR, response_params, 3);
}

static void handle_read(uint8_t id, const uint8_t* params, uint16_t param_len)
{
	// Reverse Little-Endian order
	uint16_t read_address = (params[1] << 8) | params[0]; // Address High Byte + Address Low Byte
	uint16_t read_length  = (params[3] << 8) | params[2]; // Length High Byte + Length Low Byte

	// Address is out of range, send an error packet
	if ((read_address + read_length) > sizeof(ControlTable_t))
	{
		send_status_packet(id, INSTRUCTION_ERR, NULL, 0);
		return;
	}

	uint8_t* data_to_send = (uint8_t*)&control_table + read_address;
	send_status_packet(id, NO_ERR, data_to_send, read_length);
}

static void handle_write(uint8_t id, const uint8_t* params, uint16_t param_len)
{
	uint16_t write_address = (params[1] << 8) | params[0];
	uint16_t write_length = param_len - 2;

	if ((write_address + write_length) > sizeof(ControlTable_t)) {
		send_status_packet(id, INSTRUCTION_ERR, NULL, 0);
		return;
	}

	// Handle torque enable
	if (write_address == offsetof(ControlTable_t, torque_enable) && write_length == 1)
	{
		uint8_t torque_value = params[2];

		if (torque_value == 1) {
			torque_enable();
		} else {
			torque_disable();
		}
	}
	// Handle operating mode
	else if (write_address == offsetof(ControlTable_t, operating_mode) && write_length == 1)
	{
		uint8_t new_mode = params[2];
		control_table.operating_mode = new_mode;
		reset_pid_controllers();
	}

	else
	{
		if (write_length > 0) {
		    uint8_t* dest = (uint8_t*)&control_table + write_address;
		    const uint8_t* src = &params[2];
		    memcpy(dest, src, write_length); // The line in question
		}
	}

    send_status_packet(id, NO_ERR, NULL, 0);
}


//==============================================================================
// Low-level Packet Transmission
//==============================================================================

/**
 * @brief Constructs and sends a generic Status Packet.
 */
static void send_status_packet(uint8_t id, uint8_t error, uint8_t* params, uint16_t param_len)
{
    uint16_t packet_len_field = 1 + 1 + param_len + 2; // Instruction + Error + Params + CRC

    txBuffer[0] = 0xFF;
    txBuffer[1] = 0xFF;
    txBuffer[2] = 0xFD;
    txBuffer[3] = 0x00;
    txBuffer[4] = id;
    txBuffer[5] = packet_len_field & 0xFF;
    txBuffer[6] = (packet_len_field >> 8) & 0xFF;
    txBuffer[7] = STATUS;
    txBuffer[8] = error;

    for (uint16_t i = 0; i < param_len; i++) {
        txBuffer[9 + i] = params[i];
    }

    uint16_t crc_calc_len = 9 + param_len;
    uint16_t crc = update_crc(0, txBuffer, crc_calc_len);
    txBuffer[crc_calc_len] = crc & 0xFF;
    txBuffer[crc_calc_len + 1] = (crc >> 8) & 0xFF;

    RS485_Transmit(txBuffer, crc_calc_len + 2);
}

void RS485_Transmit(uint8_t *pData, uint16_t size) {
    RS485_TX_ENABLE();
    HAL_UART_Transmit_IT(&huart1, pData, size);
}

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
