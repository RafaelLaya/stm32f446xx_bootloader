/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "bootloader.h"

#if DEBUG_MESSAGES_ENABLED
#include "printf.h"
#include <stdarg.h>
#define DEBUG_BUFFER_SIZE			100u
static uint8_t DebugBuffer[DEBUG_BUFFER_SIZE];
#endif /* DEBUG_MESSAGES_ENABLED */

static UART_HandleTypeDef DebugUartHandle;

static void SystemClockConfig(void)
{
	RCC_ClkInitTypeDef RCC_ClockInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClockInitStruct.ClockType = 	(RCC_CLOCKTYPE_SYSCLK |
									 	 RCC_CLOCKTYPE_HCLK   |
										 RCC_CLOCKTYPE_PCLK1  |
										 RCC_CLOCKTYPE_PCLK2);
	RCC_ClockInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClockInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClockInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClockInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClockInitStruct, FLASH_LATENCY_0);
}

static void ConfigDebugUART(void)
{
	GPIO_InitTypeDef TxPin, RxPin;
	TxPin.Alternate = GPIO_AF7_USART1;
	TxPin.Mode = GPIO_MODE_AF_PP;
	TxPin.Pin = GPIO_PIN_9;
	TxPin.Pull = GPIO_PULLUP;
	TxPin.Speed = GPIO_SPEED_FREQ_HIGH;
	__GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &TxPin);

	RxPin.Alternate = GPIO_AF7_USART1;
	RxPin.Mode = GPIO_MODE_AF_PP;
	RxPin.Pin = GPIO_PIN_10;
	RxPin.Pull = GPIO_PULLUP;
	RxPin.Speed = GPIO_SPEED_FREQ_HIGH;
	__GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &RxPin);

	DebugUartHandle.Init.BaudRate = 115200;
	DebugUartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	DebugUartHandle.Init.Mode = UART_MODE_TX_RX;
	DebugUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	DebugUartHandle.Init.Parity = UART_PARITY_EVEN;
	DebugUartHandle.Init.StopBits = UART_STOPBITS_1;
	DebugUartHandle.Init.WordLength = UART_WORDLENGTH_9B;
	DebugUartHandle.Instance = USART1;

	__HAL_RCC_USART1_CLK_ENABLE();
	HAL_UART_Init(&DebugUartHandle);
}

static void ConfigBootloaderUART(void)
{
	GPIO_InitTypeDef TxPin, RxPin;
	TxPin.Alternate = GPIO_AF7_USART2;
	TxPin.Mode = GPIO_MODE_AF_PP;
	TxPin.Pin = GPIO_PIN_2;
	TxPin.Pull = GPIO_PULLUP;
	TxPin.Speed = GPIO_SPEED_FREQ_HIGH;
	__GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &TxPin);

	RxPin.Alternate = GPIO_AF7_USART2;
	RxPin.Mode = GPIO_MODE_AF_PP;
	RxPin.Pin = GPIO_PIN_3;
	RxPin.Pull = GPIO_PULLUP;
	RxPin.Speed = GPIO_SPEED_FREQ_HIGH;
	__GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &RxPin);

	BL_UartHandle.Init.BaudRate = 115200;
	BL_UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	BL_UartHandle.Init.Mode = UART_MODE_TX_RX;
	BL_UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	BL_UartHandle.Init.Parity = UART_PARITY_EVEN;
	BL_UartHandle.Init.StopBits = UART_STOPBITS_1;
	BL_UartHandle.Init.WordLength = UART_WORDLENGTH_9B;
	BL_UartHandle.Instance = USART2;

	__HAL_RCC_USART2_CLK_ENABLE();
	HAL_UART_Init(&BL_UartHandle);
}

static void ConfigCrc(void)
{
	__HAL_RCC_CRC_CLK_ENABLE();

	BL_CrcHandle.Instance = CRC;
	HAL_CRC_Init(&BL_CrcHandle);

	__HAL_CRC_DR_RESET(&BL_CrcHandle);
}

static void ConfigUserButton(void)
{
	GPIO_InitTypeDef gpio;
	gpio.Alternate = 0;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pin = GPIO_PIN_13;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	__HAL_RCC_GPIOC_CLK_ENABLE();
	HAL_GPIO_Init(GPIOC, &gpio);
}

extern void PrintDebugMessage(char *Format, ...)
{
#if DEBUG_MESSAGES_ENABLED
	va_list ArgsList;
	va_start(ArgsList, Format);

	vsnprintf((char *) &DebugBuffer, DEBUG_BUFFER_SIZE, Format, ArgsList);
	HAL_UART_Transmit(&DebugUartHandle,
					  (uint8_t *) &DebugBuffer,
					  strlen((char *) &DebugBuffer),
					  HAL_MAX_DELAY);

	va_end(ArgsList);
#endif /* DEBUG_MESSAGES_ENABLED */
}

static void ConfigLed(void)
{
	GPIO_InitTypeDef led;
	led.Mode = GPIO_MODE_OUTPUT_PP;
	led.Pin = GPIO_PIN_5;
	led.Pull = GPIO_NOPULL;
	led.Speed = GPIO_SPEED_FREQ_HIGH;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &led);
}

int main(void)
{
	HAL_Init();
	SystemCoreClockUpdate();
	SystemClockConfig();

	ConfigDebugUART();
	ConfigBootloaderUART();
	ConfigUserButton();
	ConfigCrc();
	ConfigLed();

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
		BL_CommunicateWithHost();
	}
	else {
		BL_RunUserApplication(FLASH_SECTOR_1_BASE);
	}

	while (1);

	return 0;
}
