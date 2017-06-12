/**
  ******************************************************************************
  * @file    rm68140.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    02-December-2014
  * @brief   This file includes the LCD driver for RM68140 LCD.
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

/* Includes ------------------------------------------------------------------*/
#include "rm68140.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup rm68140
  * @brief     This file provides a set of functions needed to drive the 
  *            RM68140 LCD.
  * @{
  */

/** @defgroup RM68140_Private_TypesDefinitions
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup RM68140_Private_Defines
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup RM68140_Private_Macros
  * @{
  */
     
/**
  * @}
  */  

/** @defgroup RM68140_Private_Variables
  * @{
  */ 
LCD_DrvTypeDef   rm68140_drv = 
{
  rm68140_Init,
  rm68140_ReadID,
  rm68140_DisplayOn,
  rm68140_DisplayOff,
  rm68140_SetCursor,
  rm68140_WritePixel,
  rm68140_ReadPixel,
  rm68140_SetDisplayWindow,
  rm68140_DrawHLine,
  rm68140_DrawVLine,
  rm68140_GetLcdPixelWidth,
  rm68140_GetLcdPixelHeight,
  rm68140_DrawBitmap,
  rm68140_DrawRGBImage,  
};

static uint16_t ArrayRGB[480] = {0};
/**
  * @}
  */ 
  
/** @defgroup RM68140_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup RM68140_Private_Functions
  * @{
  */   

/**
  * @brief  Initialize the RM68140 LCD Component.
  * @param  None
  * @retval None
  */
void rm68140_Init(void)
{  
  uint16_t param[15];
  /* Initialize RM68140 low level bus layer ----------------------------------*/
  //LCD_IO_Init();
  rm68140_WriteReg(LCD_REG_CMD_SOFT_RESET, param,0); /* Set SS and SM bit */
  /* Start Initial Sequence --------------------------------------------------*/
  rm68140_WriteReg(LCD_REG_CMD_EXIT_SLEEP_MODE, param,0); /* Set SS and SM bit */
  
  param[0]=0x000a;
  param[1]=0x000a;
  rm68140_WriteReg(LCD_REG_CMD_PWR_CONTROL_1, param,2); /* Set SS and SM bit */
  
  param[0]=0x0000;
  param[1]=0x0042;
  param[2]=0x0080;
  rm68140_WriteReg(LCD_REG_VCOM_CONTROL_1, param,3); /* Set SS and SM bit */
  
  param[0]=0x0033;
  rm68140_WriteReg(LCD_REG_CMD_PWR_CONTROL_3, param,1); /* Set SS and SM bit */
  
  param[0]=0x0000;
  rm68140_WriteReg(LCD_REG_CMD_SET_TEAR_ON, param,1); /* Set SS and SM bit */
  
  param[0]=0x00b0;
  param[1]=0x0011;
  rm68140_WriteReg(LCD_REG_CMD_FRAMERATE_CONTROL_NORMAL, param,2); /* Set SS and SM bit */
  
  param[0]=0x0002;
  rm68140_WriteReg(LCD_REG_CMD_DISPLAY_INVERSION_CONTROL, param,1); /* Set SS and SM bit */
  
  param[0]=0x0000;
  param[1]=0x0000;
  param[2]=0x003b;
  rm68140_WriteReg(LCD_REG_CMD_DISPLAY_FUNCTION_CONTROL, param,3); /* Set SS and SM bit */
  
  param[0]=0x0007;
  rm68140_WriteReg(LCD_REG_CMD_ENTRY_MODE_SET, param,1); /* Set SS and SM bit */
  
  param[0]=0x0055;
  rm68140_WriteReg(LCD_REG_CMD_SET_PIXEL_FORMAT, param,1); /* Set SS and SM bit */
   
  param[0]=ADDR_MODE_TOP_TO_BOTTOM|ADDR_MODE_LEFT_TO_RIGHT|ADDR_MODE_COLUMN_ROW_EXCHANGE;
  rm68140_WriteReg(LCD_REG_CMD_SET_ADDR_MODE, (uint16_t*)param,1);
  
  param[0]=0x0000;
  param[1]=0x0035;
  param[2]=0x0033;
  param[3]=0x0000;
  param[4]=0x0000;
  param[5]=0x0000;
  param[6]=0x0000;
  param[7]=0x0035;
  param[8]=0x0033;
  param[9]=0x0000;
  param[10]=0x0000;
  param[11]=0x0000;
  rm68140_WriteReg(LCD_REG_CMD_GAMMA_SETTING, param,12); /* Set SS and SM bit */
 
  rm68140_WriteReg(LCD_REG_CMD_SET_DISPLAY_ON, param,0); /* Set SS and SM bit */
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void rm68140_DisplayOn(void)
{
 uint16_t param[15];
  /* Power On sequence -------------------------------------------------------*/
 rm68140_WriteReg(LCD_REG_CMD_SET_DISPLAY_ON, param,0); /* Set SS and SM bit */
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void rm68140_DisplayOff(void)
{
  
  /* Display Off */
 uint16_t param[15];
  /* Power On sequence -------------------------------------------------------*/
 rm68140_WriteReg(LCD_REG_CMD_SET_DISPLAY_OFF, param,0); /* Set SS and SM bit */
}

/**
  * @brief  Get the LCD pixel Width.
  * @param  None
  * @retval The Lcd Pixel Width
  */
uint16_t rm68140_GetLcdPixelWidth(void)
{
 return (uint16_t)480;
}

/**
  * @brief  Get the LCD pixel Height.
  * @param  None
  * @retval The Lcd Pixel Height
  */
uint16_t rm68140_GetLcdPixelHeight(void)
{
 return (uint16_t)320;
}

/**
  * @brief  Get the RM68140 ID.
  * @param  None
  * @retval The RM68140 ID 
  */
uint16_t rm68140_ReadID(void)
{
  
  uint16_t id[4];
  rm68140_ReadReg(LCD_REG_CMD_DISPLAY_ID,id,4);
  
  return id[3];
}

/**
  * @brief  Set Cursor position.
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @retval None
  */
void rm68140_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
  rm68140_SetDisplayWindow(Xpos,Ypos,1,1);  
}

/**
  * @brief  Write pixel.   
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  RGBCode: the RGB pixel color
  * @retval None
  */
void rm68140_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode)
{
  /* Set Cursor */
  rm68140_SetCursor(Xpos, Ypos);
  
  /* Prepare to write GRAM */
  rm68140_WriteReg(LCD_REG_CMD_WRITE_MEM_START,&RGBCode,1);
}

/**
  * @brief  Read pixel.
  * @param  None
  * @retval The RGB pixel color
  */
uint16_t rm68140_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  uint16_t RGBCode;
  /* Set Cursor */
  rm68140_SetCursor(Xpos, Ypos);
  
  /* Prepare to write GRAM */
   rm68140_ReadReg(LCD_REG_CMD_WRITE_MEM_START,&RGBCode,1);

  return RGBCode;
}

/**
  * @brief  Writes to the selected LCD register.
  * @param  LCDReg: Address of the selected register.
  * @param  LCDRegValue: Value to write to the selected register.
  * @retval None
  */
void rm68140_WriteReg(uint8_t LCDReg, uint16_t* ptr_param,uint32_t param_cnt)
{
  LCD_IO_WriteCmd(LCDReg);
  
  /* Write 16-bit GRAM Reg */
  LCD_IO_WriteMultipleData(ptr_param, param_cnt);
}

/**
  * @brief  Reads the selected LCD Register.
  * @param  LCDReg: address of the selected register.
  * @retval LCD Register Value.
  */
void rm68140_ReadReg(uint8_t LCDReg, uint16_t* ptr_param,uint32_t param_cnt)
{
  /* Write 16-bit Index (then Read Reg) */
  LCD_IO_WriteCmd(LCDReg);
  
  /* Read 16-bit Reg */
  LCD_IO_ReadMultipleData(ptr_param,param_cnt);
}

/**
  * @brief  Sets a display window
  * @param  Xpos:   specifies the X bottom left position.
  * @param  Ypos:   specifies the Y bottom left position.
  * @param  Height: display window height.
  * @param  Width:  display window width.
  * @retval None
  */
void rm68140_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint16_t param[4];
  param[0]=Xpos>>8;
  param[1]=Xpos&0x00ff;
  param[2]=(Xpos+Width-1)>>8;
  param[3]=(Xpos+Width-1)&0x00ff;
  
  /* Horizontal GRAM Start and end Address */
  rm68140_WriteReg(LCD_REG_CMD_SET_COLUMN_ADDR,param,4);

  param[0]=Ypos>>8;
  param[1]=Ypos&0x00ff;
  param[2]=(Ypos+Height-1)>>8;
  param[3]=(Ypos+Height-1)&0x00ff;
  /* Vertical GRAM Start and end Address */
  rm68140_WriteReg(LCD_REG_CMD_SET_PAGE_ADDR,param,4);
}

/**
  * @brief  Draw vertical line.
  * @param  RGBCode: Specifies the RGB color   
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.  
  * @retval None
  */
void rm68140_DrawHLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t counter = 0;
  
  rm68140_SetDisplayWindow(Xpos,Ypos,Length,1);

  /* Sent a complete line */
  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = RGBCode;
  }  
  rm68140_WriteReg(LCD_REG_CMD_WRITE_MEM_START,ArrayRGB,Length);
}

/**
  * @brief  Draw vertical line.
  * @param  RGBCode: Specifies the RGB color    
  * @param  Xpos:     specifies the X position.
  * @param  Ypos:     specifies the Y position.
  * @param  Length:   specifies the Line length.  
  * @retval None
  */
void rm68140_DrawVLine(uint16_t RGBCode, uint16_t Xpos, uint16_t Ypos, uint16_t Length)
{
  uint16_t counter = 0;
  
  rm68140_SetDisplayWindow(Xpos,Ypos,1,Length);

  /* Sent a complete line */
  for(counter = 0; counter < Length; counter++)
  {
    ArrayRGB[counter] = RGBCode;
  }  
  rm68140_WriteReg(LCD_REG_CMD_WRITE_MEM_START,ArrayRGB,Length);  
}

/**
  * @brief  Displays a bitmap picture.
  * @param  BmpAddress: Bmp picture address.
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD    
  * @retval None
  */
void rm68140_DrawBitmap(uint16_t Xpos, uint16_t Ypos, uint8_t *pbmp)
{
  uint32_t index = 0, size = 0;
  uint16_t param[1];
  /* Read bitmap size */
  size = *(volatile uint16_t *) (pbmp + 2);
  size |= (*(volatile uint16_t *) (pbmp + 4)) << 16;
  /* Get bitmap data address offset */
  index = *(volatile uint16_t *) (pbmp + 10);
  index |= (*(volatile uint16_t *) (pbmp + 12)) << 16;
  size = (size - index);
  pbmp += index;

  param[0]=ADDR_MODE_TOP_TO_BOTTOM|ADDR_MODE_RIGHT_TO_LEFT|ADDR_MODE_COLUMN_ROW_EXCHANGE;
  rm68140_WriteReg(LCD_REG_CMD_SET_ADDR_MODE, param,1);
  
  rm68140_WriteReg(LCD_REG_CMD_WRITE_MEM_START,(uint16_t*)pbmp,size);
  
  param[0]=ADDR_MODE_TOP_TO_BOTTOM|ADDR_MODE_LEFT_TO_RIGHT|ADDR_MODE_COLUMN_ROW_EXCHANGE;
  rm68140_WriteReg(LCD_REG_CMD_SET_ADDR_MODE, (uint16_t*)param,1);
}

/**
  * @brief  Displays picture.
  * @param  pdata: picture address.
  * @param  Xpos: Image X position in the LCD
  * @param  Ypos: Image Y position in the LCD
  * @param  Xsize: Image X size in the LCD
  * @param  Ysize: Image Y size in the LCD
  * @retval None
  */
void rm68140_DrawRGBImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
  uint32_t size = 0;

  size = (Xsize * Ysize);

  rm68140_WriteReg(LCD_REG_CMD_WRITE_MEM_START, (uint16_t*)pdata,size);
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
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
