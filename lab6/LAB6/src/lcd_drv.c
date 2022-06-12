#include "lcd_drv.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define RGB (0x62<<1)
#define LCD (0x3E<<1)

char _displayfunction;
char _displaycontrol;


void sendCommand(char value)
{
    unsigned char dta[2] = {0x80, value};
    //i2c_send_byteS(dta, 2);
    for(int i=0;i<2;i++){
    HAL_I2C_Master_Transmit(&hi2c1,LCD,&dta[i],sizeof(dta),100);
    }
}


void setReg(char addr, char val)
{
    uint8_t datos[2]={addr,val};
    HAL_I2C_Master_Transmit(&hi2c1,RGB,datos,sizeof(datos),100);
}



void setRGB(unsigned char r, unsigned char g, unsigned char b)
{   
    setReg(REG_RED, r);
    setReg(REG_GREEN, g);
    setReg(REG_BLUE, b);
}


void printLCD(char *str)
{   
    HAL_I2C_Master_Transmit(&hi2c1,RGB,str,sizeof(str),100);
}


void locateLCD(unsigned char col, unsigned char row)
{
    col = (row == 0 ? col|0x80 : col|0xc0);
    unsigned char dta[2] = {0x80, col};
    for(int i=0;i<2;i++){
    HAL_I2C_Master_Transmit(&hi2c1,LCD,&dta[i],sizeof(dta),100);
    }
}


void displayOn() 
{
    _displaycontrol |= LCD_DISPLAYON;
    sendCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}


void clearLCD()
{
    sendCommand(LCD_CLEARDISPLAY);        
    HAL_Delay(2);         
}


void initLCD() 
{   
    //Initialize displayfunction parameter for setting up LCD display
   _displayfunction |= LCD_2LINE;
   _displayfunction |= LCD_5x10DOTS;
 
   //Wait for more than 30 ms after power rises above 4.5V per the data sheet
    HAL_Delay(50);
 
 
    // Send first function set command. Wait longer that 39 us per the data sheet
    sendCommand(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(1);  
    
    // turn the display on
    displayOn();
 
    // clear the display
    clearLCD();
    
    // Initialize backlight
    setReg(0, 0);
    setReg(1, 0);
    setReg(0x08, 0xAA);   
}
