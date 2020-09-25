/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "printf.h"
#include <inttypes.h>
#include <string.h>
#include <stdarg.h>

#define DEBUG_BUFFER_SIZE			100u

static uint8_t DebugBuffer[DEBUG_BUFFER_SIZE];
static UART_HandleTypeDef DebugUartHandle;

void PrintMsg(char *Format, ...)
{
	va_list ArgsList;
	va_start(ArgsList, Format);

	vsnprintf((char *) &DebugBuffer, DEBUG_BUFFER_SIZE, Format, ArgsList);
	HAL_UART_Transmit(&DebugUartHandle,
					  (uint8_t *) &DebugBuffer,
					  strlen((char *) &DebugBuffer),
					  HAL_MAX_DELAY);

	va_end(ArgsList);
}

static void config_Led(void)
{
	GPIO_InitTypeDef Led;
	Led.Mode = GPIO_MODE_OUTPUT_PP;
	Led.Pin = GPIO_PIN_5;
	Led.Pull = GPIO_NOPULL;
	Led.Speed = GPIO_SPEED_FREQ_LOW;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &Led);
}

static void ConfigButton(void)
{
	GPIO_InitTypeDef Button;
	Button.Mode = GPIO_MODE_IT_FALLING;
	Button.Pin = GPIO_PIN_13;
	Button.Pull = GPIO_NOPULL;
	Button.Speed = GPIO_SPEED_FREQ_LOW;
	__HAL_RCC_GPIOC_CLK_ENABLE();
	HAL_GPIO_Init(GPIOC, &Button);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15U, 15U);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
}

void EXTI15_10_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

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

int main(void)
{
	HAL_Init();
	SystemCoreClockUpdate();
	SystemClockConfig();

	ConfigDebugUART();
	config_Led();
	ConfigButton();

	PrintMsg("Hello, world!\r\n");

	uint8_t Data;
	while (1) {
		HAL_UART_Receive(&DebugUartHandle, &Data, 1u, HAL_MAX_DELAY);
		HAL_UART_Transmit(&DebugUartHandle, &Data, 1u, HAL_MAX_DELAY);
	}
	while (1);

	return 0;
}

