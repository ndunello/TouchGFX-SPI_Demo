#ifndef INC_ILI9341_H_
#define INC_ILI9341_H_

#ifdef __cplusplus
  extern "C" {
#endif

#include "main.h"

#define GUI_WIDTH 320
#define GUI_HEIGHT 240

void ILI9341_Init(void);
void ILI9341_SetWindow(uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);
void ILI9341_DrawBitmap(uint16_t w, uint16_t h, uint8_t *s);
void ILI9341_WritePixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_EndOfDrawBitmap(void);

void LCD_WR_REG(uint8_t data);
void            LCD_IO_WriteMultipleData(uint16_t *pData, uint32_t Size);

#ifdef __cplusplus
}
#endif

#endif /* INC_ILI9341_H_ */
