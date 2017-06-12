/**
  ******************************************************************************
  * @file    rm68140.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    02-December-2014
  * @brief   This file contains all the functions prototypes for the rm68140.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RM68140_H
#define __RM68140_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "../Common/lcd.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup rm68140
  * @{
  */

/** @defgroup RM68140_Exported_Types
  * @{
  */
   
/**
  * @}
  */ 

/** @defgroup RM68140_Exported_Constants
  * @{
  */
/** 
  * @brief  RM68140 ID  
  */  
#define  RM68140_ID                    0x66
#define  ADDR_MODE_TOP_TO_BOTTOM       (0 )
#define  ADDR_MODE_BOTTOM_TO_TOP       (1<<7) 
#define  ADDR_MODE_LEFT_TO_RIGHT       (0)
#define  ADDR_MODE_RIGHT_TO_LEFT       (1<<6) 
#define  ADDR_MODE_COLUMN_ROW_NORMAL   (0)
#define  ADDR_MODE_COLUMN_ROW_EXCHANGE (1<<5)
#define  ADDR_MODE_VERTICAL_NORMAL     (0)
#define  ADDR_MODE_VERTICAL_FLIPPED    (1<<0)
#define  ADDR_MODE_HORIZONTAL_NORMAL   (0)
#define  ADDR_MODE_HORIZONTAL_FLIPPED  (1<<1)   
/** 
  * @brief  RM68140 Size  
  */  
#define  RM68140_LCD_PIXEL_WIDTH    ((uint16_t)320)
#define  RM68140_LCD_PIXEL_HEIGHT   ((uint16_t)240)
   
/** 
  * @brief  RM68140 Registers  
  */ 
#define LCD_REG_CMD_NOP                       0x00
#define LCD_REG_CMD_SOFT_RESET                0x01
#define LCD_REG_CMD_DISPLAY_ID                0x04
#define LCD_REG_CMD_ENTER_SLEEP_MODE          0x10
#define LCD_REG_CMD_EXIT_SLEEP_MODE           0x11
#define LCD_REG_CMD_ENTER_PARTICAL_MODE       0x12
#define LCD_REG_CMD_ENTER_NORMAL_MODE         0x13
#define LCD_REG_CMD_ENTER_INVERT_MODE         0x21
#define LCD_REG_CMD_EXIT_INVERT_MODE          0x20
#define LCD_REG_CMD_SET_DISPLAY_ON            0x29
#define LCD_REG_CMD_SET_DISPLAY_OFF           0x28
#define LCD_REG_CMD_SET_COLUMN_ADDR           0x2A 
#define LCD_REG_CMD_SET_PAGE_ADDR             0x2B
#define LCD_REG_CMD_WRITE_MEM_START           0x2C
#define LCD_REG_CMD_READ_MEM_START            0x2E
#define LCD_REG_CMD_SET_PARTICAL_AREA         0x30
#define LCD_REG_CMD_SET_TEAR_ON               0x35   
#define LCD_REG_CMD_SET_ADDR_MODE             0x36
#define LCD_REG_CMD_SET_PIXEL_FORMAT          0x3A
#define LCD_REG_CMD_WRITE_MEM_CONTINUE        0x3C
#define LCD_REG_CMD_READ_MEM_CONTINUE         0x3E
#define LCD_REG_CMD_SET_DISPLAY_BRIGHTNESS    0x51
#define LCD_REG_CMD_SET_CONTROL_DISPLAY       0x53
#define LCD_REG_CMD_INTERFACE_MODE_CONTROL     0xB0
#define LCD_REG_CMD_FRAMERATE_CONTROL_NORMAL   0xB1
#define LCD_REG_CMD_FRAMERATE_CONTROL_PARTICAL 0xB3
#define LCD_REG_CMD_FRAMERATE_CONTROL_IDLE     0xB2
#define LCD_REG_CMD_DISPLAY_INVERSION_CONTROL  0xB4
#define LCD_REG_CMD_DISPLAY_FUNCTION_CONTROL   0xB6
#define LCD_REG_CMD_ENTRY_MODE_SET             0xB7
#define LCD_REG_CMD_PWR_CONTROL_1             0xC0
#define LCD_REG_CMD_PWR_CONTROL_2             0xC1
#define LCD_REG_CMD_PWR_CONTROL_3             0xC2
#define LCD_REG_CMD_PWR_CONTROL_4             0xC3
#define LCD_REG_VCOM_CONTROL_1                0xC5
#define LCD_REG_CMD_GAMMA_SETTING             0xE0

/**
  * @}
  */
  
/** @defgroup RM68140_Exported_Functions
  * @{
  */ 
void     rm68140_Init(void);
uint16_t rm68140_ReadID(void);
void     rm68140_WriteReg(uint8_t LCDReg, uint16_t* ptr_param,uint32_t param_cnt);
void     rm68140_ReadReg(uint8_t LCDReg, uint16_t* ptr_param,uint32_t param_cnt);

void     rm68140_DisplayOn(void);
void     rm68140_DisplayOff(void);
void     rm68140_SetCursor(uint16_t Xpos, uint16_t Ypos);
void     rm68140_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode);
uint16_t rm68140_ReadPixel(uint16_t Xpos, uint16_t Ypos);

void     rm68140_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     rm68140_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length);
void     rm68140_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp);
void     rm68140_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata);

void     rm68140_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);


uint16_t rm68140_GetLcdPixelWidth(void);
uint16_t rm68140_GetLcdPixelHeight(void);

/* LCD driver structure */
extern LCD_DrvTypeDef   rm68140_drv;

/* LCD IO functions */
void     LCD_IO_Init(void);
void     LCD_IO_WriteMultipleData(uint16_t *pData, uint32_t Size);
void     LCD_IO_WriteCmd(uint8_t Cmd);
void     LCD_IO_ReadMultipleData(uint16_t *ptr_param,uint32_t param_cnt);

/**
  * @}
  */ 
      
#ifdef __cplusplus
}
#endif

#endif /* __RM68140_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
