/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"

#include "eeprom.h"
#include "settings.h"
#include "system/system.h"
#include "system/init.h"
#include "system/adc.h"
#include "motor/motor.h"
#include "input/input_dshot.h"
#include "input/input_multishot.h"
#include "input/input_oneshot.h"
#include "input/input_proshot.h"
#include "input/input_pwm.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
long map(long x, long in_min, long in_max, long out_min, long out_max);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

extern int newinput;
extern int count;
extern int error;
extern int e_com_time;
extern int input;
extern int running;
extern int commandcount;
extern int tocheck;
extern int duty_cycle;
extern int filter_delay;
extern char filter_level;
extern char polling_mode;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
