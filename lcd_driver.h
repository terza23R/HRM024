#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include "stdint.h"


/* I2C device address, depends on model of the PCF8574 chip,A(7E),T(4E) */
#define LCD_ADDR (0x7E)


/* LCD komande */
#define LCD_FUNCTION_SET1      0x33
#define LCD_FUNCTION_SET2      0x32
#define LCD_4BIT_2LINE_MODE    0x28
#define LCD_DISP_CURS_ON       0x0E
#define LCD_DISP_ON_CURS_OFF   0x0C  
#define LCD_DISPLAY_OFF        0x08
#define LCD_DISPLAY_ON         0x04
#define LCD_CLEAR_DISPLAY      0x01
#define LCD_ENTRY_MODE_SET     0x04
#define LCD_INCREMENT_CURSER   0x06
#define LCD_SET_ROW1_COL1      0x80  //Force cursor to beginning ( 1st line)
#define LCD_SET_ROW2_COL1      0xC0  //Force cursor to beginning ( 2nd line)
#define LCD_MOVE_DISPLAY_LEFT  0x18 
#define LCD_MOVE_DISPLAY_RIGHT 0x1C 

void I2C_Conf(void);
void LCD_Write_Cmd4bit(uint8_t DevAddr, uint8_t data);
void LCD_Write_Cmdbitbit(uint8_t DevAddr, uint8_t data);
void LCD_Write_Data(uint8_t DevAddr,uint8_t data);
void LCD_Init();
void LCD_Cursor(int r, int c);
void LCD_Send_String (char *str);
void LCD_Clear(void);

#endif


