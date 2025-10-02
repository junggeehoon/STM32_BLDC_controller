/*
 * dynamixel_protocol.h
 *
 *  Created on: Sep 26, 2025
 *      Author: pscad
 */

#ifndef INC_DYNAMIXEL_PROTOCOL_H_
#define INC_DYNAMIXEL_PROTOCOL_H_

#include "main.h"
#include <stdbool.h>

//==============================================================================
// Instruction Defines
//==============================================================================

#define DXL_ID               1
#define DXL_MODEL_NUMBER     1200
#define DXL_FIRMWARE_VERSION 0x26

#define PING 				 0x01
#define READ				 0x02
#define WRITE 				 0x03
#define REG_WRITE 			 0x04
#define ACTION 				 0x05
#define FACTORY_RESET 		 0x06
#define REBOOT 				 0x08
#define CLEAR 				 0x10
#define CONTROL_TABLE_BACKUP 0x20
#define STATUS         		 0x55
#define SYNC_READ	   		 0x82
#define SYNC_WRITE     		 0x83
#define FAST_SYNC_READ 		 0x8A
#define BULK_READ      		 0x92
#define BULK_WRITE 	   		 0x93
#define FAST_BULK_READ 		 0x9A

#define NO_ERR				 0x00
#define RESULT_FAIL_ERR      0x01
#define INSTRUCTION_ERR      0X02
#define CRC_ERR				 0X03
#define DATA_RANGE_ERR  	 0X04
#define DATA_LENGTH_ERR		 0X05
#define DATA_LIMIT_ERR		 0X06
#define ACCESS_ERR			 0X07

#define BROADCAST_ID		 0xFE

//  Label="Current control:Velocity control:-:Position control:Extended Position control:Current-based Position:-:-:-:-:-:-:-:-:-:-:PWM control"
//#define CURRENT_CONTROL      0x00
//#define VELOCITY_CONTROL     0x01
//#define POSITION_CONTROL     0x03
//#define PWM_CONTROL          0x10

//==============================================================================
// Function Prototypes
//==============================================================================

/**
 * @brief Processes a fully received and aligned DYNAMIXEL packet.
 * @param packetBuffer The buffer containing the full packet.
 * @param packetLength The value from the packet's length field.
 * @return true if a response was transmitted, false otherwise.
 */
bool process_packet(uint8_t* packetBuffer, uint16_t packetLength);

/**
 * @brief Transmits data over the RS485 bus.
 * @param pData Pointer to the data to send.
 * @param size The number of bytes to send.
 */
void RS485_Transmit(uint8_t *pData, uint16_t size);

/**
 * @brief Calculates the CRC-16-CCITT for a block of data.
 * @param crc_accum The initial CRC value (usually 0).
 * @param data_blk_ptr Pointer to the data block.
 * @param data_blk_size The size of the data block.
 * @return The calculated CRC.
 */
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);


#endif /* INC_DYNAMIXEL_PROTOCOL_H_ */
