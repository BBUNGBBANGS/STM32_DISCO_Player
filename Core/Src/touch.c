/**
  ******************************************************************************
  * @file    stm32746g_discovery_ts.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the Touch 
  *          Screen on STM32746G-Discovery board.
  *
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
  @verbatim
   1. How To use this driver:
   --------------------------
      - This driver is used to drive the touch screen module of the STM32746G-Discovery
        board on the RK043FN48H-CT672B 480x272 LCD screen with capacitive touch screen.
      - The FT5336 component driver must be included in project files according to
        the touch screen driver present on this board.
   
   2. Driver description:
   ---------------------
     + Initialization steps:
        o Initialize the TS module using the BSP_TS_Init() function. This 
          function includes the MSP layer hardware resources initialization and the
          communication layer configuration to start the TS use. The LCD size properties
          (x and y) are passed as parameters.
        o If TS interrupt mode is desired, you must configure the TS interrupt mode
          by calling the function BSP_TS_ITConfig(). The TS interrupt mode is generated
          as an external interrupt whenever a touch is detected. 
          The interrupt mode internally uses the IO functionalities driver driven by
          the IO expander, to configure the IT line.
     
     + Touch screen use
        o The touch screen state is captured whenever the function BSP_TS_GetState() is 
          used. This function returns information about the last LCD touch occurred
          in the TS_StateTypeDef structure.
        o If TS interrupt mode is used, the function BSP_TS_ITGetStatus() is needed to get
          the interrupt status. To clear the IT pending bits, you should call the 
          function BSP_TS_ITClear().
        o The IT is handled using the corresponding external interrupt IRQ handler,
          the user IT callback treatment is implemented on the same external interrupt
          callback.
  @endverbatim
  ******************************************************************************
  */ 

/* Dependencies
- stm32746g_discovery_lcd.c
- ft5336.c
EndDependencies */

/* Includes ------------------------------------------------------------------*/
#include "touch.h"
#include "lcd.h"
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32746G_DISCOVERY
  * @{
  */ 
  
/** @defgroup STM32746G_DISCOVERY_TS STM32746G_DISCOVERY_TS
  * @{
  */   

/** @defgroup STM32746G_DISCOVERY_TS_Private_Types_Definitions STM32746G_DISCOVERY_TS Types Definitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32746G_DISCOVERY_TS_Private_Defines STM32746G_DISCOVERY_TS Types Defines
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32746G_DISCOVERY_TS_Private_Macros STM32746G_DISCOVERY_TS Private Macros
  * @{
  */ 
/**
  * @}
  */

/** @defgroup STM32746G_DISCOVERY_TS_Imported_Variables STM32746G_DISCOVERY_TS Imported Variables
  * @{
  */
  /**
    * @}
    */

/** @defgroup STM32746G_DISCOVERY_TS_Private_Variables STM32746G_DISCOVERY_TS Private Variables
  * @{
  */ 
static TS_DrvTypeDef *tsDriver;
static uint16_t tsXBoundary, tsYBoundary; 
static uint8_t tsOrientation;
static uint8_t I2cAddress;
static TS_StateTypeDef TS_State;
uint8_t Toutch_Status = 1;
/**
  * @}
  */ 

static void Touch_Select_Zone(uint8_t x, uint8_t y, uint8_t * status);
static void Touch_Display_Zone(uint8_t *status,uint32_t color);

/** @defgroup STM32746G_DISCOVERY_TS_Private_Function_Prototypes STM32746G_DISCOVERY_TS Private Function Prototypes
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32746G_DISCOVERY_TS_Exported_Functions STM32746G_DISCOVERY_TS Exported Functions
  * @{
  */ 
void Touch_Operation(void)
{
    uint8_t text[30] = {0,};
    uint16_t x, y;
    static uint8_t status_old;
    BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected)
    {
        /* Get X and Y position of the touch post calibrated */
        TS_State.touchX[0]++;
        TS_State.touchY[0]++;
        x = TS_State.touchX[0];
        y = TS_State.touchY[0];
        Touch_Select_Zone(x,y,&Toutch_Status);
        if(status_old != Toutch_Status)
        {
            Touch_Display_Zone(&status_old,LCD_COLOR_WHITE);
            Touch_Display_Zone(&Toutch_Status,LCD_COLOR_LIGHTYELLOW);
        }
        status_old = Toutch_Status;
        BSP_LCD_SetTextColor(LCD_COLOR_LIGHTYELLOW);
        sprintf((char*)text, "%d", Toutch_Status);
        BSP_LCD_DisplayStringAt(325, 232, (uint8_t *)&text, LEFT_MODE);
    }
}

static void Touch_Select_Zone(uint8_t x, uint8_t y, uint8_t * status)
{       
    if ((y > 30) && (y < 60))
    {
        *status = 1;
    }
    else if ((y > 60) && (y < 90))
    {
        *status = 2;
    }
    else if ((y > 90) && (y < 120))
    {
        *status = 3;
    }
    else if ((y > 120) && (y < 150))
    {
        *status = 4;
    }
    else if ((y > 150) && (y < 180))
    {
        *status = 5;
    }
    else if ((y > 180) && (y < 210))
    {
        *status = 6;
    }
    else if ((y > 210) && (x > 0) && (x < 160))
    {
        *status = 7;
    }
    else if ((y > 210) && (x > 160) && (x < 320))
    {
        *status = 8;
    }
    else if ((y > 210) && (x > 320) && (x < 480))
    {
        *status = 9;
    }
}

static void Touch_Display_Zone(uint8_t *status,uint32_t color)
{
    BSP_LCD_SetTextColor(color);
    if (*status == 1)
    {
        BSP_LCD_DrawHLine(0,30,BSP_LCD_GetXSize());
        BSP_LCD_DrawHLine(0,60,BSP_LCD_GetXSize());
        BSP_LCD_DrawVLine(0,30,30);
        BSP_LCD_DrawVLine(30,30,30);
        BSP_LCD_DrawVLine(480,30,30);
    }
    else if (*status == 2)
    {
        BSP_LCD_DrawHLine(0,60,BSP_LCD_GetXSize());
        BSP_LCD_DrawHLine(0,90,BSP_LCD_GetXSize());
        BSP_LCD_DrawVLine(0,60,30);
        BSP_LCD_DrawVLine(30,60,30);
        BSP_LCD_DrawVLine(480,60,30);        
    }
    else if (*status == 3)
    {
        BSP_LCD_DrawHLine(0,90,BSP_LCD_GetXSize());
        BSP_LCD_DrawHLine(0,120,BSP_LCD_GetXSize());
        BSP_LCD_DrawVLine(0,90,30);
        BSP_LCD_DrawVLine(30,90,30);
        BSP_LCD_DrawVLine(480,90,30);                
    }
    else if (*status == 4)
    {
        BSP_LCD_DrawHLine(0,120,BSP_LCD_GetXSize());
        BSP_LCD_DrawHLine(0,150,BSP_LCD_GetXSize());
        BSP_LCD_DrawVLine(0,120,30);
        BSP_LCD_DrawVLine(30,120,30);
        BSP_LCD_DrawVLine(480,120,30);                
    }
    else if (*status == 5)
    {
        BSP_LCD_DrawHLine(0,150,BSP_LCD_GetXSize());
        BSP_LCD_DrawHLine(0,180,BSP_LCD_GetXSize());
        BSP_LCD_DrawVLine(0,150,30);
        BSP_LCD_DrawVLine(30,150,30);
        BSP_LCD_DrawVLine(480,150,30);                
    }
    else if (*status == 6)
    {
        BSP_LCD_DrawHLine(0,180,BSP_LCD_GetXSize());
        BSP_LCD_DrawHLine(0,210,BSP_LCD_GetXSize());
        BSP_LCD_DrawVLine(0,180,30);
        BSP_LCD_DrawVLine(30,180,30);
        BSP_LCD_DrawVLine(480,180,30);                
    }
}
/**
  * @brief  Initializes and configures the touch screen functionalities and 
  *         configures all necessary hardware resources (GPIOs, I2C, clocks..).
  * @param  ts_SizeX: Maximum X size of the TS area on LCD
  * @param  ts_SizeY: Maximum Y size of the TS area on LCD
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_Init(uint16_t ts_SizeX, uint16_t ts_SizeY)
{
  uint8_t status = TS_OK;
  tsXBoundary = ts_SizeX;
  tsYBoundary = ts_SizeY;
  
  /* Read ID and verify if the touch screen driver is ready */
  ft5336_ts_drv.Init(TS_I2C_ADDRESS);
  if(ft5336_ts_drv.ReadID(TS_I2C_ADDRESS) == FT5336_ID_VALUE)
  { 
    /* Initialize the TS driver structure */
    tsDriver = &ft5336_ts_drv;
    I2cAddress = TS_I2C_ADDRESS;
    tsOrientation = TS_SWAP_XY;

    /* Initialize the TS driver */
    tsDriver->Start(I2cAddress);
  }
  else
  {
    status = TS_DEVICE_NOT_FOUND;
  }

  return status;
}

/**
  * @brief  DeInitializes the TouchScreen.
  * @retval TS state
  */
uint8_t BSP_TS_DeInit(void)
{ 
  /* Actually ts_driver does not provide a DeInit function */
  return TS_OK;
}

/**
  * @brief  Configures and enables the touch screen interrupts.
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_ITConfig(void)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Configure Interrupt mode for SD detection pin */
  gpio_init_structure.Pin = TS_INT_PIN;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init_structure);

  /* Enable and set Touch screen EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(TS_INT_EXTI_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));

  /* Enable the TS ITs */
  tsDriver->EnableIT(I2cAddress);

  return TS_OK;  
}

/**
  * @brief  Gets the touch screen interrupt status.
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_ITGetStatus(void)
{
  /* Return the TS IT status */
  return (tsDriver->GetITStatus(I2cAddress));
}

/**
  * @brief  Returns status and positions of the touch screen.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State)
{
  static uint32_t _x[TS_MAX_NB_TOUCH] = {0, 0};
  static uint32_t _y[TS_MAX_NB_TOUCH] = {0, 0};
  uint8_t ts_status = TS_OK;
  uint16_t x[TS_MAX_NB_TOUCH];
  uint16_t y[TS_MAX_NB_TOUCH];
  uint16_t brute_x[TS_MAX_NB_TOUCH];
  uint16_t brute_y[TS_MAX_NB_TOUCH];
  uint16_t x_diff;
  uint16_t y_diff;
  uint32_t index;
#if (TS_MULTI_TOUCH_SUPPORTED == 1)
  uint32_t weight = 0;
  uint32_t area = 0;
  uint32_t event = 0;
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  /* Check and update the number of touches active detected */
  TS_State->touchDetected = tsDriver->DetectTouch(I2cAddress);
  
  if(TS_State->touchDetected)
  {
    for(index=0; index < TS_State->touchDetected; index++)
    {
      /* Get each touch coordinates */
      tsDriver->GetXY(I2cAddress, &(brute_x[index]), &(brute_y[index]));

      if(tsOrientation == TS_SWAP_NONE)
      {
        x[index] = brute_x[index];
        y[index] = brute_y[index];
      }

      if(tsOrientation & TS_SWAP_X)
      {
        x[index] = 4096 - brute_x[index];
      }

      if(tsOrientation & TS_SWAP_Y)
      {
        y[index] = 4096 - brute_y[index];
      }

      if(tsOrientation & TS_SWAP_XY)
      {
        y[index] = brute_x[index];
        x[index] = brute_y[index];
      }

      x_diff = x[index] > _x[index]? (x[index] - _x[index]): (_x[index] - x[index]);
      y_diff = y[index] > _y[index]? (y[index] - _y[index]): (_y[index] - y[index]);

      if ((x_diff + y_diff) > 5)
      {
        _x[index] = x[index];
        _y[index] = y[index];
      }

      if(I2cAddress == FT5336_I2C_SLAVE_ADDRESS)
      {
        TS_State->touchX[index] = x[index];
        TS_State->touchY[index] = y[index];
      }
      else
      {
        /* 2^12 = 4096 : indexes are expressed on a dynamic of 4096 */
        TS_State->touchX[index] = (tsXBoundary * _x[index]) >> 12;
        TS_State->touchY[index] = (tsYBoundary * _y[index]) >> 12;
      }

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

      /* Get touch info related to the current touch */
      ft5336_TS_GetTouchInfo(I2cAddress, index, &weight, &area, &event);

      /* Update TS_State structure */
      TS_State->touchWeight[index] = weight;
      TS_State->touchArea[index]   = area;

      /* Remap touch event */
      switch(event)
      {
        case FT5336_TOUCH_EVT_FLAG_PRESS_DOWN	:
          TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
          break;
        case FT5336_TOUCH_EVT_FLAG_LIFT_UP :
          TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
          break;
        case FT5336_TOUCH_EVT_FLAG_CONTACT :
          TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
          break;
        case FT5336_TOUCH_EVT_FLAG_NO_EVENT :
          TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
          break;
        default :
          ts_status = TS_ERROR;
          break;
      } /* of switch(event) */

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

    } /* of for(index=0; index < TS_State->touchDetected; index++) */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
    /* Get gesture Id */
    ts_status = BSP_TS_Get_GestureId(TS_State);
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  } /* end of if(TS_State->touchDetected != 0) */

  return (ts_status);
}

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
/**
  * @brief  Update gesture Id following a touch detected.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if all initializations are OK. Other value if error.
  */
uint8_t BSP_TS_Get_GestureId(TS_StateTypeDef *TS_State)
{
  uint32_t gestureId = 0;
  uint8_t  ts_status = TS_OK;

  /* Get gesture Id */
  ft5336_TS_GetGestureID(I2cAddress, &gestureId);

  /* Remap gesture Id to a TS_GestureIdTypeDef value */
  switch(gestureId)
  {
    case FT5336_GEST_ID_NO_GESTURE :
      TS_State->gestureId = GEST_ID_NO_GESTURE;
      break;
    case FT5336_GEST_ID_MOVE_UP :
      TS_State->gestureId = GEST_ID_MOVE_UP;
      break;
    case FT5336_GEST_ID_MOVE_RIGHT :
      TS_State->gestureId = GEST_ID_MOVE_RIGHT;
      break;
    case FT5336_GEST_ID_MOVE_DOWN :
      TS_State->gestureId = GEST_ID_MOVE_DOWN;
      break;
    case FT5336_GEST_ID_MOVE_LEFT :
      TS_State->gestureId = GEST_ID_MOVE_LEFT;
      break;
    case FT5336_GEST_ID_ZOOM_IN :
      TS_State->gestureId = GEST_ID_ZOOM_IN;
      break;
    case FT5336_GEST_ID_ZOOM_OUT :
      TS_State->gestureId = GEST_ID_ZOOM_OUT;
      break;
    default :
      ts_status = TS_ERROR;
      break;
  } /* of switch(gestureId) */

  return(ts_status);
}
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

/**
  * @brief  Clears all touch screen interrupts.
  */
void BSP_TS_ITClear(void)
{
  /* Clear TS IT pending bits */
  tsDriver->ClearIT(I2cAddress); 
}


/** @defgroup STM32756G_DISCOVERY_TS_Private_Functions TS Private Functions
  * @{
  */


/**
  * @brief  Function used to reset all touch data before a new acquisition
  *         of touch information.
  * @param  TS_State: Pointer to touch screen current state structure
  * @retval TS_OK if OK, TE_ERROR if problem found.
  */
uint8_t BSP_TS_ResetTouchData(TS_StateTypeDef *TS_State)
{
  uint8_t ts_status = TS_ERROR;
  uint32_t index;

  if (TS_State != (TS_StateTypeDef *)NULL)
  {
    TS_State->gestureId = GEST_ID_NO_GESTURE;
    TS_State->touchDetected = 0;

    for(index = 0; index < TS_MAX_NB_TOUCH; index++)
    {
      TS_State->touchX[index]       = 0;
      TS_State->touchY[index]       = 0;
      TS_State->touchArea[index]    = 0;
      TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
      TS_State->touchWeight[index]  = 0;
    }

    ts_status = TS_OK;

  } /* of if (TS_State != (TS_StateTypeDef *)NULL) */

  return (ts_status);
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

/**
  * @}
  */

