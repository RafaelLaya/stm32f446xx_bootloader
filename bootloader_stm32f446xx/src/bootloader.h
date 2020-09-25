/*
 * bootloader.h
 *
 *  Created on: Sep 17, 2020
 *      Author: rafael
 */

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include <inttypes.h>
#include <string.h>
#include <stdbool.h>

#define DEBUG_MESSAGES_ENABLED		0x0u


/** @addtogroup BL_CMD
 *  @brief Command Codes recognized by the Bootloader
 *  @{
 */
#define BL_CMD_GET_VERSION					0x51u
#define BL_CMD_GET_HELP						0x52u
#define BL_CMD_GET_CID						0x53u
#define BL_CMD_GET_RDP_STATUS				0x54u
#define BL_CMD_GO_TO_ADDR					0x55u
#define BL_CMD_FLASH_ERASE					0x56u
#define BL_CMD_MEM_WRITE					0x57u
#define BL_CMD_EN_RW_PROTECT				0x58u
#define BL_CMD_DIS_RW_PROTECTION			0x59u
#define BL_CMD_READ_SECTOR_STATUS			0x5Au
#define BL_CMD_MEM_READ						0x5Bu
#define BL_CMD_GO_TO_APP					0x5Cu
/**
 * @}
 */

/** @addtogroup BL_IMPORTANT_PACKET_LENGTHS
 *  @brief Important lengths of important components in a packet
 *  @{
 */
#define BL_BYTES_OF_LENGTH					1u
#define BL_BYTES_OF_COMMAND_CODE			1u
#define BL_PREAMBLE_SIZE					(BL_BYTES_OF_LENGTH + BL_BYTES_OF_COMMAND_CODE)
#define BL_BYTES_IN_CRC32					4u
#define BL_RESPONSE_OVERHEAD				(BL_BYTES_OF_LENGTH + BL_BYTES_IN_CRC32)
/**
 *  @}
 */

/** @addtogroup BL_MISC_LENGTHS
 *  @{
 */
#define BL_VERSION_LENGTH					1u
#define BL_NUMBER_OF_COMMANDS				0xCu
#define BL_CID_LENGTH						2u
#define BL_REV_LENGTH						2u
#define BL_RDP_STATUS_LENGTH				1u
#define BL_SUCCESS_VALUE_LENGTH				1u
#define BL_SIZE_OF_SECTOR_MASK 				1u
#define BL_SIZE_OF_PROTECTION_CODE 			1u
#define BL_MAX_WRITE_PAYLOAD_LENGTH	(0xFFu - BL_BYTES_OF_COMMAND_CODE - 2 * sizeof(uint32_t) - 1)
#define BL_MAX_READ_PAYLOAD_LENGTH  (0xFFu - BL_BYTES_OF_LENGTH - BL_BYTES_IN_CRC32)
/**
 *  @}
 */

/** @addtogroup BL_RESPONSE_LENGTH
 *  @brief The length of the response for commands whose length is fixed
 *  @{
 */
#define BL_GET_VERSION_RESP_LENGTH			(BL_RESPONSE_OVERHEAD + 2 * BL_VERSION_LENGTH)
#define BL_GET_HELP_RESP_LENGTH				(BL_RESPONSE_OVERHEAD + BL_NUMBER_OF_COMMANDS)
#define BL_GET_CID_RESP_LENGTH				(BL_RESPONSE_OVERHEAD + BL_CID_LENGTH + BL_REV_LENGTH)
#define BL_GET_RDP_STATUS_RESP_LENGTH		(BL_RESPONSE_OVERHEAD + BL_RDP_STATUS_LENGTH)
#define BL_GET_GO_TO_ADDR_RESP_LENGTH		(BL_RESPONSE_OVERHEAD + BL_SUCCESS_VALUE_LENGTH)
#define BL_FLASH_ERASE_RESP_LENGTH			(BL_RESPONSE_OVERHEAD + BL_SUCCESS_VALUE_LENGTH)
#define BL_MEM_WRITE_RESP_LENGTH			(BL_RESPONSE_OVERHEAD + BL_SUCCESS_VALUE_LENGTH)
#define BL_EN_RW_PROTECTION_RESP_LENGTH		(BL_RESPONSE_OVERHEAD + BL_SUCCESS_VALUE_LENGTH)
#define BL_DIS_RW_PROTECTION_RESP_LENGTH	(BL_RESPONSE_OVERHEAD + BL_SUCCESS_VALUE_LENGTH)
#define BL_READ_SECTOR_STATUS_RESP_LENGTH	(BL_RESPONSE_OVERHEAD + sizeof(uint32_t))
#define BL_GO_TO_APP_RESP_LENGTH			(BL_RESPONSE_OVERHEAD + BL_SUCCESS_VALUE_LENGTH)
/**
 *  @}
 */

/** @addtogroup BL_ACKNOWLEDGEMENT
 *  @brief Corresponds to positive and negative acknowledgement
 *  @{
 */
#define BL_ACK			0xA5u
#define BL_NACK			0x7Fu
/**
 * @}
 */

/** @addtogroup BL_VERSION
 *  @brief Corresponds to the version of the bootloader
 *  @{
 */
#define BL_VERSION		0x01u
#define BL_SUBVERSION	0x01u
/**
 *  @}
 */

/** @addtogroup BL_MEMORIES
 *  @brief Some parameters for some memories
 *  @{
 */
#define FLASH_SECTOR_1_BASE		0x08004000u
#define BL_FLASH_START			0x08000000u
#define BL_FLASH_END			0x081FFFFFu
#define BL_SRAM_START			0x20000000u
#define BL_SRAM_END				0x2001FFFFu
#define BL_BKP_SRAM_START		0x40024000u
#define BL_BKP_SRAM_END			0x40024FFFu
#define BL_OTP_START			0x1FFF7800u
#define BL_OTP_END				(0x1FFF7A00u + 0x10u - 0x1u)
#define BL_MAX_NUM_OF_SECTORS			8u
#define BL_MASS_ERASE_INITIAL_FLAG		0xFFu
#define BL_PROTECTION_CODE_RW 			0xC1u
#define BL_PROTECTION_CODE_W			0xC2u
/**
 *  @}
 */

/** @addtogroup BL_SUCCESS
 *  Corresponds to success and failure codes
 *  @{
 */
#define BL_SUCCESS			0xA6u
#define BL_NO_SUCCESS		0xB4u
#define BL_VALID_ADDR		BL_SUCCESS
#define BL_INVALID_ADDR		BL_NO_SUCCESS
/**
 *  @}
 */

/** @addtogroup BL_BUFFER_SIZES
 *  @brief The size of some buffers
 *  @{
 */
#define BL_RX_BUFFER_SIZE		(0xFFu + 0x10u)
#define BL_TX_BUFFER_SIZE		(0xFFu + 0x10u)
/**
 *  @}
 */

/**
 *  @brief A reset handler is of the following type
 */
typedef void (*ResetHandler_t)(void);

/**
 *  @brief The main application must configure the CRC engine as well
 *  as the UART for the Bootloader
 */
extern UART_HandleTypeDef BL_UartHandle;
extern CRC_HandleTypeDef BL_CrcHandle;




/**
 *  @brief 	Main loop to communicate with the Host application.
 *  		This is done by waiting for the length of the request,
 *  		then receiving the request and decoding the command code.
 */
void BL_CommunicateWithHost(void);

/**
 *  @brief 	Runs an user application located at ProgramAddress.
 *  		i.e.: A reset-handler is located at the next 4-byte word
 *  		after ProgramAddress.
 * 	1. Get MSP (first 4-byte word)
 *	2. Gets Reset Handler (pointer is in the second 4-byte word)
 *  3. Sets the Thumb Mode bit
 *  4. De-configures all peripherals
 *  5. Re-starts the clock configuration
 *  6. VTOR should be fixed by the reset handler of the user application
 *  7. Calls the reset handler of the user application
 */
void BL_RunUserApplication(uint32_t ProgramAddress);

/**
 *  @brief  Prints a message through UART for debugging purposes.
 */
extern void PrintDebugMessage(char *Format, ...);

#endif /* BOOTLOADER_H_ */
