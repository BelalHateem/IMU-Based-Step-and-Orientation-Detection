/**
  ******************************************************************************
  * @file    MotionPM_Manager.c
  * @author  MEMS Software Solutions Team
  * @version 4.2.0
  * @date    07-Feb-2022
  * @brief   This file contains Pedo Meter interface functions
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
#include "TargetFeatures.h"

/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup STANDING_SITTING_DESK STANDING SITTING DESK
 * @{
 */

/* exported Variable -------------------------------------------------------------*/
MPM_output_t PedometerCode;

/* Private typedef -----------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Initialize the MotionSD engine
 * @param  None
 * @retval None
 */
void MotionPM_manager_init(void)
{
  char LibVersion[36];

  MotionPM_Initialize();
  MotionPM_GetLibVersion(LibVersion);


  TargetBoardFeatures.MotionPMIsInitalized=1;
  ALLMEMS1_PRINTF("Initialized %s\r\n", LibVersion);
}

/**
 * @brief  Run pedometer algorithm
 * @param  data_in  Structure containing input data
 * @param  data_out Structure containing output data
 * @retval None
 */
void MotionPM_manager_run(MPM_input_t *data_in, MPM_output_t *data_out)
{
	MotionPM_Update(data_in, data_out);
}

/**
 * @brief  Reset algorithm
 * @param  None
 * @retval None
 */
void MotionPM_manager_reset(void)
{
	MotionPM_ResetStepCount();
}

/**
 * @brief  Get the library version
 * @param  version  Library version string (must be array of 35 char)
 * @param  length  Library version string length
 * @retval None
 */
void MotionPM_manager_get_version(char *version, int *length)
{
  *length = (int)MotionPM_GetLibVersion(version);
}

/**
 * @}
 */

/**
 * @}
 */

