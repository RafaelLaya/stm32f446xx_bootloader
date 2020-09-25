/*
 * bootloader.c
 *
 *  Created on: Sep 17, 2020
 *      Author: rafael
 */

#include "bootloader.h"

static uint8_t BL_RxBuffer[BL_RX_BUFFER_SIZE];
static uint8_t BL_TxBuffer[BL_TX_BUFFER_SIZE];

CRC_HandleTypeDef BL_CrcHandle;
UART_HandleTypeDef BL_UartHandle;

/**
 *  @brief 	Forces the length to be an 8-bit number.
 *  		This is useful because a package length is an 8-bit number,
 *  		but the length of the internal BL_TxBuffer might be momentarily
 *  		and slightly bigger. Thus we make sure to always store the output
 *  		of this function onto the first byte of every package.
 *  @param Length The length of the data to transfer (data and crc).
 *  @return The value of Length but overflow of 8-bit numbers is enforced
 *  to 0xFFu
 */
static inline uint8_t BL_EnforceLength(uint16_t Length)
{
	if (Length > 0xFFu) {
		PrintDebugMessage("Length was forced from %" PRIx16 " to " PRIx16 "\r\n",
							Length, 0xFFu);
		return 0xFFu;
	}

	return Length;
}

/**
 *  @brief 	Sends the data of @p Length bytes on the @p Data buffer
 *  		through UART in a polling fashion.
 *  @param Data The buffer where the data is located.
 *  @param Length The number of bytes to be transferred.
 */
static void BL_SendData(uint8_t *Data, uint16_t Length)
{
	PrintDebugMessage("Sending:\r\n");
	for (uint32_t i = 0; i < Length; i++) {
		PrintDebugMessage("%" PRIx8 "\r\n", Data[i]);
	}

	HAL_UART_Transmit(&BL_UartHandle, Data, Length, HAL_MAX_DELAY);
}

/**
 *  @brief 	Receives data of @p Length bytes on the @p Data buffer
 *  		through UART in a polling fashion.
 *  @param Data The buffer of data where information will be stored.
 *  @param Length The number of bytes to receive
 */
static void BL_ReceiveData(uint8_t *Data, uint16_t Length)
{
	HAL_UART_Receive(&BL_UartHandle, Data, Length, HAL_MAX_DELAY);

	PrintDebugMessage("Received:\r\n");
	for (uint32_t i = 0; i < Length; i++) {
		PrintDebugMessage("%" PRIx8 "\r\n", Data[i]);
	}
}

/**
 *  @brief Handles positive acknowledgement.
 */
static void BL_Ack(void)
{
	PrintDebugMessage("Will send ACK\r\n");
	uint8_t Ack = BL_ACK;
	BL_SendData(&Ack, sizeof(uint8_t));
}

/**
 *  @brief Handles negative acknowledgement.
 */
static void BL_Nack(void)
{
	PrintDebugMessage("Will send NACK\r\n");
	uint8_t Nack = BL_NACK;
	BL_SendData(&Nack, sizeof(uint8_t));
}

/**
 *  @brief 	Calculates the CRC of a buffer. The CRC is a CRC32
 *  		calculation performed on each byte. The polynomial is hard-coded
 *  		by the vendor to be 0x4C11DB7.
 *  @param Buffer Contains the data whose CRC will be calculated.
 *  @param Length The number of bytes in @p Buffer.
 *  @return The calculated CRC value.
 */
static uint32_t BL_CalculateCrc(uint8_t *Buffer, uint16_t Length)
{
	uint32_t Crc_Calculated = 0xFFu;

	__HAL_CRC_DR_RESET(&BL_CrcHandle);
	for (uint32_t i = 0; i < Length; i++) {
		uint32_t Input = (uint32_t) Buffer[i];
		Crc_Calculated = HAL_CRC_Accumulate(&BL_CrcHandle,
											&Input,
											sizeof(Input) / sizeof(uint32_t));
	}

	PrintDebugMessage("Crc_Calculated is %" PRIx32 "\r\n", Crc_Calculated);
	return Crc_Calculated;
}

/**
 *  @brief Checks if the CRC calculated matches the received CRC.
 *  @param Buffer The buffer whose CRC will be calculated.
 *  @param Length The number of bytes in @p Buffer.
 *  @param CrcReceived The CRC received.
 *  @return true if the calculated CRC (from Buffer and Length) matches CrcReceived.
 */
static bool BL_IsCrcValid(uint8_t *Buffer, uint16_t Length, uint32_t CrcReceived)
{
	return BL_CalculateCrc(Buffer, Length) == CrcReceived;
}

/**
 *  @brief Retrieves the version of this bootloader.
 *  @return The version of this bootloader.
 */
static uint8_t BL_GetVersion(void)
{
	PrintDebugMessage("Bootloader version is %" PRIx8 "\r\n", (uint8_t) BL_VERSION);
	return BL_VERSION;
}

/**
 *  @brief 	When a command is received, this function checks the validity of the
 *  		message through a CRC check. Then acknowledges the message.
 *  @param Buffer The buffer where the received command is stored.
 *  @param Length The number of bytes in @p Buffer.
 *  @return 0 on success, 1 if CRC failed.
 */
static uint8_t BL_HandleCommandSetup(uint8_t *Buffer, uint16_t Length)
{
	uint32_t CrcReceived = *((uint32_t *) (&Buffer[Length - sizeof(uint32_t)]));

	PrintDebugMessage("Length of Received message is %" PRIx16 "\r\n", Length);
	PrintDebugMessage("CrcReceived is %" PRIx32 "\r\n", CrcReceived);

	if (!BL_IsCrcValid(Buffer, Length - sizeof(uint32_t), CrcReceived)) {
		PrintDebugMessage("CRC Check Failed\r\n");
		BL_Nack();
		return 1;
	}

	PrintDebugMessage("CRC Check Successful\r\n");
	BL_Ack();
	return 0;
}

/**
 *	@brief 	When a command has been processed, this function calculates the CRC of the
 *  		response and then proceeds to send the response to the host application.
 *  @param Buffer The buffer where the response is stored.
 *  @param 	ResponseLength The number of bytes in the response, including the length
 *  		of the response.
 */
static void BL_HandleCommandTeardown(uint8_t *Buffer, uint16_t ResponseLength)
{
	PrintDebugMessage("Response Length is %" PRIx16 "\r\n", ResponseLength);
	uint32_t crc = BL_CalculateCrc(	Buffer,
									ResponseLength - sizeof(uint32_t));
	*((uint32_t *) &Buffer[ResponseLength - sizeof(uint32_t)]) = crc;

	BL_SendData(Buffer, ResponseLength);
}

/**
 *  @brief Retrieves the Sub-version of this bootloader.
 *  @return The Sub-version of this bootloader.
 */
static inline uint8_t BL_GetSubversion(void)
{
	return BL_SUBVERSION;
}

/**
 *  @brief Handles the command to retrieve the version of this bootloader.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleGetVersionCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	BL_TxBuffer[0] = BL_EnforceLength(BL_GET_VERSION_RESP_LENGTH - BL_BYTES_OF_LENGTH);
	BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_GetVersion();
	BL_TxBuffer[BL_BYTES_OF_LENGTH + 1] = BL_GetSubversion();

	BL_HandleCommandTeardown((uint8_t *) BL_TxBuffer, BL_GET_VERSION_RESP_LENGTH);
}

/**
 *  @brief Handles the command to retrieve all the commands supported by this bootloader.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleGetHelpCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}
	BL_TxBuffer[0] = BL_EnforceLength(BL_GET_HELP_RESP_LENGTH - BL_BYTES_OF_LENGTH);

	for (uint8_t i = 1; i < BL_NUMBER_OF_COMMANDS + 1; i++) {
		BL_TxBuffer[i] = BL_CMD_GET_VERSION + i - 1;
	}

	BL_HandleCommandTeardown(BL_TxBuffer, BL_GET_HELP_RESP_LENGTH);
}

/**
 *  @brief 	Handles the command to retrieve the Chip ID and Revision Number of
 *  		our STM32F4 device.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleGetChipIdCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	uint16_t ChipId = DBGMCU->IDCODE & DBGMCU_IDCODE_DEV_ID_Msk;
	uint16_t RevId = (DBGMCU->IDCODE & DBGMCU_IDCODE_REV_ID_Msk) >> DBGMCU_IDCODE_REV_ID_Pos;
	BL_TxBuffer[0] = BL_EnforceLength(BL_GET_CID_RESP_LENGTH - BL_BYTES_OF_LENGTH);
	*((uint16_t *) &BL_TxBuffer[BL_BYTES_OF_LENGTH]) = ChipId;
	*((uint16_t *) &BL_TxBuffer[BL_BYTES_OF_LENGTH + BL_CID_LENGTH]) = RevId;

	BL_HandleCommandTeardown(BL_TxBuffer, BL_GET_CID_RESP_LENGTH);
}

/**
 *  @brief Handles the command to retrieve the status of RDP protection.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleGetRdpStatusCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	FLASH_OBProgramInitTypeDef FlashOBP;
	HAL_FLASHEx_OBGetConfig(&FlashOBP);
	uint8_t RdpStatus = FlashOBP.RDPLevel;

	BL_TxBuffer[0] = BL_EnforceLength(BL_GET_RDP_STATUS_RESP_LENGTH - BL_BYTES_OF_LENGTH);
	BL_TxBuffer[BL_BYTES_OF_LENGTH] = RdpStatus;

	BL_HandleCommandTeardown(BL_TxBuffer, BL_GET_RDP_STATUS_RESP_LENGTH);
}

/**
 *  @brief 	Handles the command to jump to a given address in memory.
 *  		Usually, a reset handler or similar will be located at
 *  		JumpAddress.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
uint8_t BL_VerifyAddr(uint32_t JumpAddress)
{
	if (JumpAddress >= BL_SRAM_START && JumpAddress <= BL_SRAM_END) {
		return BL_VALID_ADDR;
	}
	else if (JumpAddress >= BL_FLASH_START && JumpAddress <= BL_FLASH_END) {
		return BL_VALID_ADDR;
	}
	else if (JumpAddress >= BL_BKP_SRAM_START && JumpAddress <= BL_BKP_SRAM_END) {
		return BL_VALID_ADDR;
	}

	PrintDebugMessage("%" PRIx32 " is a Valid Address To Jump\r\n", JumpAddress);
	return BL_INVALID_ADDR;
}

/**
 *  @brief Handles the command to retrieve the version of this bootloader.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleGoToAddrCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	// See comment on BL_RunUserApplication
	uint32_t JumpAddr = *((uint32_t *) (&Buffer[BL_PREAMBLE_SIZE])) | 0x1u;

	ResetHandler_t JumpFunction = (ResetHandler_t) JumpAddr;

	uint8_t JumpStatus = BL_VerifyAddr(JumpAddr);

	BL_TxBuffer[0] = BL_EnforceLength(BL_GET_GO_TO_ADDR_RESP_LENGTH - BL_BYTES_OF_LENGTH);
	BL_TxBuffer[BL_BYTES_OF_LENGTH] = JumpStatus;

	BL_HandleCommandTeardown(BL_TxBuffer, BL_GET_GO_TO_ADDR_RESP_LENGTH);

	// De-configure all peripherals
	HAL_DeInit();

	// Re-start clock configuration
	SystemInit();

	// Fence
	__DSB();
	__ISB();

	(*JumpFunction)();
}

/**
 *  @brief Performs an erase on memory.
 *  @param Initial The first sector to erase (0 through 7).
 *  @param Number The number of sectors to erase.
 *  @SectorError 	Same as @p SectorError in function HAL_FLASHEx_Erase()
 *  				which indicates an error if different than 0xFFFFFFFFu.
 *  @return 0 on Success, otherwise failure.
 */
static uint8_t BL_PerformErase(uint8_t Initial, uint8_t Number, uint32_t *SectorError)
{
	PrintDebugMessage("Initial Sector is %" PRIx8 "\r\n", Initial);
	PrintDebugMessage("Number of sectors is %" PRIx8 "\r\n", Number);

	if (Initial != BL_MASS_ERASE_INITIAL_FLAG) {
		if (Initial >= BL_MAX_NUM_OF_SECTORS) {
			PrintDebugMessage("Invalid input: Initial >= BL_MAX_NUM_OF_SECTORS\r\n");
			return 1;
		}
		else if (Number < 1) {
			PrintDebugMessage("Invalid input: Number < 1\r\n");
			return 2;
		}
		else if (Number > BL_MAX_NUM_OF_SECTORS - Initial) {
			PrintDebugMessage("Invalid input: Number > BL_MAX_NUM_OF_SECTORS - Initial\r\n");
			return 3;
		}
	}

	FLASH_EraseInitTypeDef Erase;
	Erase.NbSectors = Number;
	Erase.Sector = Initial;
	Erase.Banks = FLASH_BANK_1;
	Erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	if (Initial == BL_MASS_ERASE_INITIAL_FLAG) {
		PrintDebugMessage("Will perform a Mass Erase\r\n");
		Erase.TypeErase = FLASH_TYPEERASE_MASSERASE;
	}
	else {
		PrintDebugMessage("Will perform a Sector Erase\r\n");
		Erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	}

	HAL_FLASH_Unlock();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_FLASHEx_Erase(&Erase, SectorError);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_FLASH_Lock();

	return 0;
}

/**
 *  @brief Handles the command to erase sectors of flash.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleFlashEraseCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	uint8_t InitialSector = Buffer[BL_PREAMBLE_SIZE];
	uint8_t NumberOfSectors = Buffer[BL_PREAMBLE_SIZE + 1];

	// do erase and reply
	BL_TxBuffer[0] = BL_EnforceLength(BL_FLASH_ERASE_RESP_LENGTH - BL_BYTES_OF_LENGTH);

	uint32_t SectorError;
	if (BL_PerformErase(InitialSector, NumberOfSectors, &SectorError) != 0) {
		PrintDebugMessage("Error: BL_PerformErase() did not return 0\r\n");
		BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_NO_SUCCESS;
	}
	else if (SectorError != 0xFFFFFFFFu) {
		PrintDebugMessage("Error: SectorError != 0xFFFFFFFFu\r\n");
		BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_NO_SUCCESS;
	}
	else {
		PrintDebugMessage("Success in Erasing Flash\r\n");
		BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_SUCCESS;
	}

	BL_HandleCommandTeardown(BL_TxBuffer, BL_FLASH_ERASE_RESP_LENGTH);
}

/**
 *  @brief Handles the command to program flash memory.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleMemWriteCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	uint32_t BaseAddress = *((uint32_t *) (&Buffer[BL_PREAMBLE_SIZE]));
	uint8_t PayloadLength = Buffer[BL_PREAMBLE_SIZE + sizeof(uint32_t)];

	BL_TxBuffer[0] = BL_EnforceLength(BL_MEM_WRITE_RESP_LENGTH - BL_BYTES_OF_LENGTH);

	if (PayloadLength > BL_MAX_WRITE_PAYLOAD_LENGTH) {
			PrintDebugMessage("Payload has been forced from %" PRIx8 " to " PRIx8 "\r\n",
								PayloadLength, BL_MAX_WRITE_PAYLOAD_LENGTH);
			PayloadLength = BL_MAX_WRITE_PAYLOAD_LENGTH;
	}

	if (BaseAddress >= FLASH_SECTOR_1_BASE && ((BaseAddress + PayloadLength - 1u) <= BL_FLASH_END)) {
		PrintDebugMessage("Valid Address to Program the Flash: %" PRIx32 "\r\n",
							BaseAddress);
		BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_SUCCESS;
		HAL_FLASH_Unlock();
		for (uint32_t i = 0; i < PayloadLength; i++) {
			if (i == 224) {
				PrintDebugMessage("i is %" PRIu32 "\r\n", i);
			}
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,
							  BaseAddress,
							  Buffer[BL_PREAMBLE_SIZE + sizeof(uint32_t) + 1 + i]);
			if ( *((uint8_t *) BaseAddress) != (Buffer[BL_PREAMBLE_SIZE + sizeof(uint32_t) + 1 + i]) ) {
				PrintDebugMessage("i is %" PRIu32 "\r\n", i);
				BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_NO_SUCCESS;
				break;
			}
			BaseAddress++;
		}
		HAL_FLASH_Lock();
	}
	else {
		PrintDebugMessage("Address is not valid to Program the Flash: %" PRIx32 "\r\n",
							BaseAddress);
		BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_NO_SUCCESS;
	}

	BL_HandleCommandTeardown(BL_TxBuffer, BL_MEM_WRITE_RESP_LENGTH);
}

/**
 *  @brief Handles the command to enable or disable W or RW protections.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleEnRwProtectionCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	BL_TxBuffer[0] = BL_EnforceLength(BL_EN_RW_PROTECTION_RESP_LENGTH - BL_BYTES_OF_LENGTH);

	// parse command
	uint8_t sector_mask = Buffer[BL_PREAMBLE_SIZE];
	uint8_t protection_code = Buffer[BL_PREAMBLE_SIZE + 1u];

	PrintDebugMessage("sector_mask is %" PRIx8 "\r\n", sector_mask);
	PrintDebugMessage("protection_code is %" PRIx8 "\r\n", protection_code);
	PrintDebugMessage("FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
	PrintDebugMessage("FLASH->SR = %" PRIx32 "\r\n", FLASH->SR);

	// Unlock
	FLASH->OPTKEYR = 0x08192A3Bu;
	FLASH->OPTKEYR = 0x4C5D6E7Fu;

	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	// configure FLASH_OPTCR
	FLASH->OPTCR &= ~(FLASH_OPTCR_nWRP_Msk);
	if (protection_code == BL_PROTECTION_CODE_RW) {
		PrintDebugMessage("Protection is RW\r\n");
		FLASH->OPTCR |= FLASH_OPTCR_SPRMOD_Msk;
		FLASH->OPTCR &= ~(FLASH_OPTCR_nWRP_Msk);
		FLASH->OPTCR |= (sector_mask << FLASH_OPTCR_nWRP_Pos) & FLASH_OPTCR_nWRP_Msk;
	}
	else {
		PrintDebugMessage("Protection is W\r\n");
		FLASH->OPTCR &= ~(FLASH_OPTCR_SPRMOD_Msk);
		FLASH->OPTCR |= FLASH_OPTCR_nWRP_Msk;
		FLASH->OPTCR &= ~((sector_mask << FLASH_OPTCR_nWRP_Pos) & FLASH_OPTCR_nWRP_Msk);
	}

	PrintDebugMessage("FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
	PrintDebugMessage("FLASH->SR = %" PRIx32 "\r\n", FLASH->SR);

	// Start
	FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT_Msk;

	// Lock
	FLASH->OPTCR |= (FLASH_OPTCR_OPTLOCK_Msk);

	// Wait until operation is finished
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	PrintDebugMessage("sector_mask is %" PRIx8 "\r\n", sector_mask);
	PrintDebugMessage("protection_code is %" PRIx8 "\r\n", protection_code);
	PrintDebugMessage("FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
	PrintDebugMessage("FLASH->SR = %" PRIx32 "\r\n", FLASH->SR);

	BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_SUCCESS;
	BL_HandleCommandTeardown((uint8_t *) BL_TxBuffer, BL_EN_RW_PROTECTION_RESP_LENGTH);
}

/**
 *  @brief Handles the command to read from memory.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleMemReadCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	uint32_t BaseAddress = *((uint32_t *) (&Buffer[BL_PREAMBLE_SIZE]));
	uint8_t PayloadLength = Buffer[BL_PREAMBLE_SIZE + sizeof(uint32_t)];

	if (PayloadLength > BL_MAX_READ_PAYLOAD_LENGTH) {
		PrintDebugMessage("Payload has been forced from %" PRIx8 " to " PRIx8 "\r\n",
							PayloadLength, BL_MAX_READ_PAYLOAD_LENGTH);
		PayloadLength = BL_MAX_READ_PAYLOAD_LENGTH;
	}

	if (PayloadLength == 0) {
		BL_TxBuffer[0] = BL_EnforceLength(BL_BYTES_IN_CRC32 - BL_BYTES_OF_LENGTH);
		BL_HandleCommandTeardown(BL_TxBuffer, BL_BYTES_IN_CRC32);
		return;
	}
	else if ((BL_VerifyAddr(BaseAddress) != BL_VALID_ADDR || BL_VerifyAddr(BaseAddress + PayloadLength - 1u) != BL_VALID_ADDR) &&
			   (!(BaseAddress >= BL_OTP_START && (BaseAddress + PayloadLength - 1u) <= BL_OTP_END))) {
		PrintDebugMessage("Address range is not valid to read %" PRIx32 "-%" PRIx32 "\r\n",
							BaseAddress, BaseAddress + PayloadLength - 1u);
		BL_TxBuffer[0] = BL_EnforceLength(BL_BYTES_IN_CRC32 - BL_BYTES_OF_LENGTH);
		BL_HandleCommandTeardown(BL_TxBuffer, BL_BYTES_IN_CRC32);
		return;
	}

	BL_TxBuffer[0] = BL_EnforceLength(PayloadLength + BL_BYTES_IN_CRC32);

	for (uint32_t i = 0; i < PayloadLength; i++) {
		BL_TxBuffer[BL_BYTES_OF_LENGTH + i] = *((uint8_t *) BaseAddress + i);
	}

	BL_HandleCommandTeardown(BL_TxBuffer, BL_BYTES_OF_LENGTH + PayloadLength + BL_BYTES_IN_CRC32);
}

/**
 *  @brief Handles the command to read the protection status of a sector.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleReadSectorStatusCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	BL_TxBuffer[0] = BL_EnforceLength(BL_READ_SECTOR_STATUS_RESP_LENGTH - BL_BYTES_OF_LENGTH);

	// Unlock
	FLASH->OPTKEYR = 0x08192A3Bu;
	FLASH->OPTKEYR = 0x4C5D6E7Fu;

	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	*((uint32_t *) &BL_TxBuffer[BL_BYTES_OF_LENGTH]) = FLASH->OPTCR;

	PrintDebugMessage("FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
	PrintDebugMessage("Should match = %" PRIx32 "\r\n", *((uint32_t *) &BL_TxBuffer[BL_BYTES_OF_LENGTH]));

	// Lock
	FLASH->OPTCR |= (FLASH_OPTCR_OPTLOCK_Msk);

	// Wait until operation is finished
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	BL_HandleCommandTeardown((uint8_t *) BL_TxBuffer, BL_READ_SECTOR_STATUS_RESP_LENGTH);
}

/**
 *  @brief 	Disables protections on each sector while being
 *  		on level 0 and PCROP has been disabled.
 */
static void BL_DisableProtections(void)
{
	HAL_FLASH_OB_Unlock();

	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	if (FLASH->OPTCR & FLASH_OPTCR_SPRMOD_Msk) {
		// if PCROP
		FLASH->OPTCR &= ~(FLASH_OPTCR_nWRP_Msk);
	}
	else {
		// no PCROP
		FLASH->OPTCR |= (FLASH_OPTCR_nWRP_Msk);
	}

	FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT_Msk;
	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	HAL_FLASH_OB_Lock();
}

/**
 *  @brief Sets PCROP protection ON.
 */
static void BL_SetPCROCProtection(void)
{
	HAL_FLASH_OB_Unlock();

	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	// PCROC Protection
	FLASH->OPTCR |= FLASH_OPTCR_SPRMOD_Msk;

	FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT_Msk;
	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	HAL_FLASH_OB_Lock();
}

/**
 *  @brief Sets Protection level from level 0 to level 1.
 */
static void BL_SetProtectionLevel1(void)
{
	HAL_FLASH_OB_Unlock();

	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	// Protection Level 1
	FLASH->OPTCR &= ~(FLASH_OPTCR_RDP_Msk);

	FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT_Msk;
	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	HAL_FLASH_OB_Lock();
}

/**
 *  @brief 	Sets protection level from level 1 to level 0, sets write protections and
 *  		disables protection on each sector.
 */
static void BL_SetProtectionLevel0_SetWriteProtection_DisableProtections(void)
{
	HAL_FLASH_OB_Unlock();

	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	// Protection Level 0
	FLASH->OPTCR &= ~(FLASH_OPTCR_RDP_Msk);
	FLASH->OPTCR |= (0xAAu << FLASH_OPTCR_RDP_Pos);

	// Write Protection
	FLASH->OPTCR &= ~(FLASH_OPTCR_SPRMOD_Msk);

	// Disable Protections
	FLASH->OPTCR |= (FLASH_OPTCR_nWRP_Msk);

	FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT_Msk;
	// Wait until available
	while (FLASH->SR & FLASH_SR_BSY_Msk);

	HAL_FLASH_OB_Lock();
}

/**
 *  @brief 	Handles the command to go to an application whose base address is known.
 *  		i.e.: There is a reset handler at the next 4-byte word after the given base address.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleGoToAppCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}
	BL_TxBuffer[0] = BL_GO_TO_APP_RESP_LENGTH - 1;

	uint32_t BaseAddress = *((uint32_t *) (&Buffer[BL_PREAMBLE_SIZE]));
	PrintDebugMessage("BaseAddress = %" PRIx32 "\r\n", BaseAddress);

	if (BL_VerifyAddr(BaseAddress) != BL_VALID_ADDR) {
		PrintDebugMessage("Can't go to address %" PRIx32 "\r\n", BaseAddress);
		BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_NO_SUCCESS;
		BL_HandleCommandTeardown((uint8_t *) BL_TxBuffer, BL_GO_TO_APP_RESP_LENGTH);
		return;
	}

	PrintDebugMessage("Address has been checked %" PRIx32 "\r\n", BaseAddress);
	BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_SUCCESS;
	BL_HandleCommandTeardown((uint8_t *) BL_TxBuffer, BL_GO_TO_APP_RESP_LENGTH);

	BL_RunUserApplication(BaseAddress);
}

/**
 *  @brief Handles the command to disable W and/or RW (PCROP) protections.
 *  @param Buffer The buffer where the received command is stored.
 *  @param RequestLength The number of bytes in @p Buffer.
 */
static void BL_HandleDisableRwProtectionCommand(uint8_t *Buffer, uint16_t RequestLength)
{
	if (BL_HandleCommandSetup(Buffer, RequestLength) != 0) {
		PrintDebugMessage("Could not perform Setup\r\n");
		return;
	}

	BL_TxBuffer[0] = BL_EnforceLength(BL_DIS_RW_PROTECTION_RESP_LENGTH - BL_BYTES_OF_LENGTH);

	FLASH->SR |= FLASH_SR_WRPERR_Msk | FLASH_SR_RDERR_Msk | FLASH_SR_PGSERR_Msk | FLASH_SR_PGPERR_Msk | FLASH_SR_PGAERR_Msk;

	PrintDebugMessage("Start FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
	PrintDebugMessage("Start FLASH_SR = %" PRIx32 "\r\n", FLASH->SR);

	if ((FLASH->OPTCR & FLASH_OPTCR_RDP_Msk) == (0xCCu << FLASH_OPTCR_RDP_Pos)) {
		// Level 2 is permanent, report no success
		BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_NO_SUCCESS;
		BL_HandleCommandTeardown((uint8_t *) BL_TxBuffer, BL_DIS_RW_PROTECTION_RESP_LENGTH);
		PrintDebugMessage("RDP Protection Level 2 is permanent!\r\n");
		return;
	}


	if ((FLASH->OPTCR & FLASH_OPTCR_RDP_Msk) == (0xAAu << FLASH_OPTCR_RDP_Pos)) {
		// Level 0
		if (FLASH->OPTCR & FLASH_OPTCR_SPRMOD_Msk) {
			// PCROC
			BL_SetPCROCProtection();
			PrintDebugMessage("[A] FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
			PrintDebugMessage("[A] FLASH_SR = %" PRIx32 "\r\n", FLASH->SR);
			BL_SetProtectionLevel1();
			PrintDebugMessage("[B] FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
			PrintDebugMessage("[B] FLASH_SR = %" PRIx32 "\r\n", FLASH->SR);
			BL_SetProtectionLevel0_SetWriteProtection_DisableProtections();
			PrintDebugMessage("[C] FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
			PrintDebugMessage("[C] FLASH_SR = %" PRIx32 "\r\n", FLASH->SR);
			BL_DisableProtections();
		}
		else {
			// No PCROC
			BL_DisableProtections();
		}
	}
	else {
		// Level 1
		BL_SetProtectionLevel0_SetWriteProtection_DisableProtections();
		BL_DisableProtections();
	}

	PrintDebugMessage("End FLASH->OPTCR = %" PRIx32 "\r\n", FLASH->OPTCR);
	PrintDebugMessage("End FLASH_SR = %" PRIx32 "\r\n", FLASH->SR);

	BL_TxBuffer[BL_BYTES_OF_LENGTH] = BL_SUCCESS;
	BL_HandleCommandTeardown((uint8_t *) BL_TxBuffer, BL_DIS_RW_PROTECTION_RESP_LENGTH);
}

void BL_CommunicateWithHost(void)
{
	uint16_t Length;
	while (1) {
		memset((void *) (uint8_t *) &BL_RxBuffer, 0u, BL_RX_BUFFER_SIZE);

		// get the Length (1 byte)
		PrintDebugMessage("Getting Length...\r\n");
		BL_ReceiveData((uint8_t *) &BL_RxBuffer, 1u);

		Length = BL_EnforceLength(BL_RxBuffer[0]);

		// get the Data
		PrintDebugMessage("Length is %" PRIx16 "\r\n", Length);
		PrintDebugMessage("Getting Data...\r\n");
		BL_ReceiveData((uint8_t *) &BL_RxBuffer[1], Length);

		// first byte is the command code
		switch (BL_RxBuffer[1]) {
			case (BL_CMD_GET_VERSION):
				PrintDebugMessage("Will handle BL_CMD_GET_VERSION\r\n");
				BL_HandleGetVersionCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_GET_HELP):
				PrintDebugMessage("Will handle BL_CMD_GET_HELP\r\n");
				BL_HandleGetHelpCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_GET_CID):
				PrintDebugMessage("Will handle BL_CMD_GET_CID\r\n");
				BL_HandleGetChipIdCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_GET_RDP_STATUS):
				PrintDebugMessage("Will handle BL_CMD_GET_RDP_STATUS\r\n");
				BL_HandleGetRdpStatusCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_GO_TO_ADDR):
				PrintDebugMessage("Will handle BL_CMD_GO_TO_ADDR\r\n");
				BL_HandleGoToAddrCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_FLASH_ERASE):
				PrintDebugMessage("Will handle BL_CMD_FLASH_ERASE\r\n");
				BL_HandleFlashEraseCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_MEM_WRITE):
				PrintDebugMessage("Will handle BL_CMD_MEM_WRITE\r\n");
				BL_HandleMemWriteCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_EN_RW_PROTECT):
				PrintDebugMessage("Will handle BL_CMD_EN_RW_PROTECT\r\n");
				BL_HandleEnRwProtectionCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_MEM_READ):
				PrintDebugMessage("Will handle BL_CMD_MEM_READ\r\n");
				BL_HandleMemReadCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_READ_SECTOR_STATUS):
				PrintDebugMessage("Will handle BL_CMD_READ_SECTOR_STATUS\r\n");
				BL_HandleReadSectorStatusCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_GO_TO_APP):
				PrintDebugMessage("Will handle BL_CMD_GO_TO_APP\r\n");
				BL_HandleGoToAppCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			case (BL_CMD_DIS_RW_PROTECTION):
				PrintDebugMessage("Will handle BL_CMD_DISABLE_RW_PROTECT\r\n");
				BL_HandleDisableRwProtectionCommand((uint8_t *) &BL_RxBuffer, Length + 1u);
				break;
			default:
				PrintDebugMessage("Unknown Command\r\n");
				break;
		}
	}
}

void BL_RunUserApplication(uint32_t ProgramAddress)
{
	// Get MSP from the first 4-byte word
	uint32_t msp = *((volatile uint32_t *) ProgramAddress);

	// Get PC from the second 4-byte word which is the Reset Handler
	uint32_t ResetHandlerAddr = ProgramAddress + sizeof(uint32_t);
	ResetHandlerAddr = (uint32_t) *((ResetHandler_t *) ResetHandlerAddr);

	// Instructions are 2-byte aligned, so ARM takes advantage of this by encoding thumb mode
	// in the LSB, which must always be set in Cortex M4
	ResetHandlerAddr |= 0x1u;
	ResetHandler_t ResetHandler = (ResetHandler_t) ResetHandlerAddr;

	PrintDebugMessage("Reset Handler is %" PRIxPTR "\r\n", (void *) ResetHandler);

	// De-configure all peripherals
	HAL_DeInit();

	// Re-start clock configuration
	SystemInit();

	// No need to fix VTOR, since it must be set within the Reset Handler of the user
	// application

	// Set MSP
	__set_MSP(msp);

	// Fence
	__DSB();
	__ISB();

	// Set PC to Reset Handler of user application
	(*ResetHandler)();
}
