/**
  ******************************************************************************
  * @file    SensorTile_conf.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains definitions for the components bus interfaces
  *          This file should be copied to the application folder and renamed
  *          to SensorTile_conf.h. 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SENSORTILE_CONF_H__
#define __SENSORTILE_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Replace the header file names with the ones of the target platform */
//#include "stm32yyxx_hal.h"
//#include "nucleo_xyyyzz_bus.h"
//#include "nucleo_xyyyzz_errno.h"
#include "stm32l4xx_hal.h"
#include "SensorTile_bus.h"
#include "SensorTile_errno.h"
  


#ifdef __cplusplus
}
#endif

#endif /* __SENSORTILE_CONF_H__*/



