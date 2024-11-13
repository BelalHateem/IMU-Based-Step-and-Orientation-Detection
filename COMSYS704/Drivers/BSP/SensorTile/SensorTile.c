/**
******************************************************************************
* @file    SensorTile.c
* @author  SRA - Central Labs
* @version v2.1.6
* @date    10-Feb-2022
* @brief   This file provides low level functionalities for SensorTile board
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
#include "SensorTile.h"


/** @addtogroup BSP
* @{
*/ 

/** @addtogroup SENSORTILE
* @{
*/

  /** @addtogroup SENSORTILE_LOW_LEVEL
  * @brief This file provides a set of low level firmware functions 
  * @{
  */

/** @defgroup SENSORTILE_LOW_LEVEL_Private_TypesDefinitions SENSORTILE_LOW_LEVEL Private Typedef
* @{
*/

/**
* @}
*/

/** @defgroup SENSORTILE_LOW_LEVEL__Private_Defines SENSORTILE_LOW_LEVEL Private Defines
* @{
*/


/**
* @brief SensorTile BSP Driver version number v2.1.6
*/
#define __SensorTile_BSP_VERSION_MAIN   (0x02) /*!< [31:24] main version */
#define __SensorTile_BSP_VERSION_SUB1   (0x01) /*!< [23:16] sub1 version */
#define __SensorTile_BSP_VERSION_SUB2   (0x04) /*!< [15:8]  sub2 version */
#define __SensorTile_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __SensorTile_BSP_VERSION         ((__SensorTile_BSP_VERSION_MAIN << 24)\
|(__SensorTile_BSP_VERSION_SUB1 << 16)\
  |(__SensorTile_BSP_VERSION_SUB2 << 8 )\
    |(__SensorTile_BSP_VERSION_RC))


/**
* @}
*/

/** @defgroup SENSORTILE_LOW_LEVEL_Private_Variables SENSORTILE_LOW_LEVEL Private Variables 
* @{
*/

//SPI_HandleTypeDef SPI_SD_Handle;
//DMA_HandleTypeDef hdma_tx;

GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LEDSWD_GPIO_PORT};
const uint32_t GPIO_PIN[LEDn] = {LED1_PIN, LEDSWD_PIN};

//uint32_t SpixTimeout = SENSORTILE_SD_SPI_TIMEOUT_MAX;        /*<! Value of Timeout when SPI communication fails */

/**
* @}
*/






/** @defgroup SENSORTILE_LOW_LEVEL_Exported_Functions SENSORTILE_LOW_LEVEL Exported Functions
  * @{
  */


/**
* @brief  This method returns the STM32446E EVAL BSP Driver revision
* @param  None
* @retval version: 0xXYZR (8bits for each decimal, R for RC)
*/
uint32_t BSP_GetVersion(void)
{
  return __SensorTile_BSP_VERSION;
}


/**
* @brief  Configures LEDs.
* @param  Led: LED to be configured. 
*          This parameter can be one of the following values:
*            @arg  LED1
* @retval None
*/
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable VddIO2 for GPIOG  */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /* Enable the GPIO_LED clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = GPIO_PIN[Led];
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);
}


/**
* @brief  DeInit LEDs.
* @param  Led: LED to be configured. 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
* @retval None
*/
void BSP_LED_DeInit(Led_TypeDef Led)
{
  
}

/**
* @brief  Turns selected LED On.
* @param  Led: LED to be set on 
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_On(Led_TypeDef Led)
{
  if(Led == LED1)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
  }
  else if (Led == LEDSWD)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }
}

/**
* @brief  Turns selected LED Off. 
* @param  Led: LED to be set off
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_Off(Led_TypeDef Led)
{
  if(Led == LED1)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  }
  else if (Led == LEDSWD)
  {
    HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
  }
}

/**
* @brief  Toggles the selected LED.
* @param  Led: LED to be toggled
*          This parameter can be one of the following values:
*            @arg  LED1
*            @arg  LED2
*            @arg  LED3
*            @arg  LED4
* @retval None
*/
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}





/**
* @}
*/

/**
* @}
*/ 

/**
* @}
*/

/**
* @}
*/    
