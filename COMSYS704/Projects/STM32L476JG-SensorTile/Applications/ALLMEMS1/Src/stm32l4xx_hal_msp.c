/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "ALLMEMS1_config.h"
#include "SensorTile.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
//  GPIO_InitTypeDef  GPIO_InitStruct;
//
//  /* Configure USB FS GPIOs */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//  /* Configure DM DP Pins */
//  GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_12);
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /* Enable USB FS Clock */
//  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
//
//  /* Set USB FS Interrupt priority */
//  HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
//
//  /* Enable USB FS Interrupt */
//  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/**
  * @brief  De-Initializes the PCD MSP.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
//  /* Disable USB FS Clock */
//  __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
//  __HAL_RCC_SYSCFG_CLK_DISABLE();
}



/**
  * @brief TIM MSP Initialization
  * This function configures the hardware resources used in this example:
  *  - Peripheral's clock enable
  *  - Peripheral's Interrupt Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0xF, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  #ifdef ALLMEMS1_MOTIONFA
  } else if(htim->Instance == TIM3) {
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0xF, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  #endif /* ALLMEMS1_MOTIONFA */
  } else if (htim->Instance == TIM5) {
     /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    /* Set Interrupt Group Priority */
    HAL_NVIC_SetPriority(TIM5_IRQn, 0xF, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  }

}  

/**
  * @brief TIM OC MSP Initialization
  * This function configures the hardware resources used in this example:
  *  - Peripheral's clock enable
  *  - Peripheral's Interrupt Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{ 
  /* TIM1 Peripheral clock enable */
  __HAL_RCC_TIM1_CLK_ENABLE();

  /* Enable TIM1 global Interrupt & set priority  */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0x8, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

/**
  * @brief CRC MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
  /* CRC Peripheral clock enable */
  __HAL_RCC_CRC_CLK_ENABLE();
}

/**
  * @brief CRC MSP De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hcrc: CRC handle pointer
  * @retval None
  */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
  /* Enable CRC reset state */
  __HAL_RCC_CRC_FORCE_RESET();

  /* Release CRC from reset state */
  __HAL_RCC_CRC_RELEASE_RESET();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5) {
//		BSP_LED_On(LED1);
		  GPIO_InitTypeDef  GPIO_InitStruct;

//		  USARTx_TX_GPIO_CLK_ENABLE();
		  __HAL_RCC_GPIOC_CLK_ENABLE();
//		  USARTx_RX_GPIO_CLK_ENABLE();
		  __HAL_RCC_GPIOD_CLK_ENABLE();

		  /* Enable USARTx clock */
//		  USARTx_CLK_ENABLE();
		  __HAL_RCC_UART5_CLK_ENABLE();



		  /*##-2- Configure peripheral GPIO ##########################################*/
		  /* UART TX GPIO pin configuration  */
		  GPIO_InitStruct.Pin       = GPIO_PIN_12;
		  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		  GPIO_InitStruct.Pull      = GPIO_NOPULL;
		  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;

		  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		  /* UART RX GPIO pin configuration  */
		  GPIO_InitStruct.Pin = GPIO_PIN_2;
		  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;

		  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		  /*##-3- Configure the NVIC for UART ########################################*/
		  /* NVIC for USART */
		  HAL_NVIC_SetPriority(UART5_IRQn, 0, 1);
		  HAL_NVIC_EnableIRQ(UART5_IRQn);

	}

	if (huart->Instance == USART1) {
//		BSP_LED_On(LED1);
		  GPIO_InitTypeDef  GPIO_InitStruct;

//		  USARTx_TX_GPIO_CLK_ENABLE();
		  __HAL_RCC_GPIOG_CLK_ENABLE();
//		  USARTx_RX_GPIO_CLK_ENABLE();
//		  __HAL_RCC_GPIOD_CLK_ENABLE();

		  /* Enable USARTx clock */
//		  USARTx_CLK_ENABLE();
		  __HAL_RCC_USART1_CLK_ENABLE();



		  /*##-2- Configure peripheral GPIO ##########################################*/
		  /* UART TX GPIO pin configuration  */
		  GPIO_InitStruct.Pin       = GPIO_PIN_9;
		  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		  GPIO_InitStruct.Pull      = GPIO_NOPULL;
		  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

		  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

		  /* UART RX GPIO pin configuration  */
		  GPIO_InitStruct.Pin = GPIO_PIN_10;
		  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

		  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

		  /*##-3- Configure the NVIC for UART ########################################*/
		  /* NVIC for USART */
//		  HAL_NVIC_SetPriority(UART5_IRQn, 0, 1);
//		  HAL_NVIC_EnableIRQ(UART5_IRQn);

	}


}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  __HAL_RCC_UART5_FORCE_RESET();
  __HAL_RCC_UART5_RELEASE_RESET();
//  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

  /*##-3- Disable the NVIC for UART ##########################################*/
  HAL_NVIC_DisableIRQ(UART5_IRQn);
}


