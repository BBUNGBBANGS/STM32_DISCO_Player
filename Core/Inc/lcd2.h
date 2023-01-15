/**
  ******************************************************************************
  * @file    stm32746g_discovery_lcd.h
  * @author  MCD Application Team
  * @brief   This file contains the common defines and functions prototypes for
  *          the stm32746g_discovery_lcd.c driver.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

/* Includes ------------------------------------------------------------------*/
/* Include LCD component Driver */
/* LCD RK043FN48H-CT672B 4,3" 480x272 pixels */
#include "main.h"
extern uint8_t LCD_Feature;
extern void LCD_SetHint(void);
extern void LCD_Show_Feature(uint8_t feature);
#endif /* __STM32746G_DISCOVERY_LCD_H */

