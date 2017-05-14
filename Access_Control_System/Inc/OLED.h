/*
 * File:		nokia.h
 * Purpose:		Serial Input/Output routines
 *
 */

#ifndef _OELD_H
#define _OELD_H

#include "main.h"
#include "stm32f1xx_hal.h"
/*!< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;    /*!< Read Only */

/********************************************************************/
/*-----------------------------------------------------------------------
LCD_init          : OLED初始化

编写日期          ：2012-11-01
最后修改日期      ：2012-11-01
-----------------------------------------------------------------------*/
 extern u8 lanzhou96x64[768];
 void OLED_Init(void);
 void OLED_CLS(void);
 void OLED_P6x8Str(u8 x,u8 y,u8 ch[]);
 void OLED_P8x16Str(u8 x,u8 y,u8 t,u8 ch[]);
 void OLED_P14x16Str(u8 x,u8 y,u8 t,u8 ch[]);
 void OLED_Print(u8 x, u8 y,u8 mode,u8 ch[]);
 void OLED_PutPixel(u8 x,u8 y);
 void OLED_Rectangle(u8 x1,u8 y1,u8 x2,u8 y2,u8 gif);
 void OLED_Set_Pos(u8 x, u8 y);
 void OLED_WrDat(u8 data);
 void Draw_LibLogo(void);
 void Draw_Landzo(void);
 void Draw_BMP(u8 x0,u8 y0,u8 x1,u8 y1,u8 bmp[]);
 void OLED_Fill(u8 dat);
 void Dly_ms(u16 ms);
void pixelC_HW_OLED_FillScreen(u8 bmp[128][8]);
extern void ToSting(float Value,u8 Array[]);
extern void Display_Value(u8 x,u8 y,u8 t,float Value);


/********************************************************************/

#endif
