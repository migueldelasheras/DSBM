#ifndef __LCD_DRV__
#define __LCD_DRV__


// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET 0x20
 
// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
 
// flag for entry mode
#define LCD_ENTRYLEFT 0x02
 
// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_2LINE 0x08
#define LCD_5x10DOTS 0x04

//colors
#define WHITE           0
#define RED             1
#define GREEN           2
#define BLUE            3

#define REG_RED         0x04        // pwm2
#define REG_GREEN       0x03        // pwm1
#define REG_BLUE        0x02        // pwm0

void setRGB(unsigned char r, unsigned char g, unsigned char b);
void printLCD(char *str);
void locateLCD(unsigned char col, unsigned char row);
void clearLCD();
//void displayOn();
void initLCD();

#endif
