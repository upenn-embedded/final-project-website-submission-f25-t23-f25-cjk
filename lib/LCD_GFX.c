/*
 * LCD_GFX.c
 *
 * Created: 9/20/2021 6:54:25 PM
 *  Author: You
 */ 

#include "LCD_GFX.h"
#include "ST7735.h"
#include <stddef.h>
#include <stdlib.h>

/******************************************************************************
* Local Functions
******************************************************************************/



/******************************************************************************
* Global Functions
******************************************************************************/

/**************************************************************************//**
* @fn			uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
* @brief		Convert RGB888 value to RGB565 16-bit color data
* @note
*****************************************************************************/
uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
	return ((((31*(red+4))/255)<<11) | (((63*(green+2))/255)<<5) | ((31*(blue+4))/255));
}

/**************************************************************************//**
* @fn			void LCD_drawPixel(uint8_t x, uint8_t y, uint16_t color)
* @brief		Draw a single pixel of 16-bit rgb565 color to the x & y coordinate
* @note
*****************************************************************************/
void LCD_drawPixel(uint8_t x, uint8_t y, uint16_t color) {
	LCD_setAddr(x,y,x,y);
	SPI_ControllerTx_16bit(color);
}

/**************************************************************************//**
* @fn			void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fColor, uint16_t bColor)
* @brief		Draw a character starting at the point with foreground and background colors
* @note
*****************************************************************************/

void LCD_drawChar(uint8_t x, uint8_t y, uint16_t character, uint16_t fColor, uint16_t bColor){
	uint16_t row = character - 0x20;		//Determine row of ASCII table starting at space
	int i, j;
	if ((LCD_WIDTH-x>7)&&(LCD_HEIGHT-y>7)){
		for(i=0;i<5;i++){
			uint8_t pixels = ASCII[row][i]; //Go through the list of pixels
			for(j=0;j<8;j++){
				if ((pixels >> j) & 0x01){
					LCD_drawPixel(x+i,y+j,fColor);
				}
				else {
					LCD_drawPixel(x+i,y+j,bColor);
				}
			}
		}
	}
}


/******************************************************************************
* LAB 4 TO DO. COMPLETE THE FUNCTIONS BELOW.
* You are free to create and add any additional files, libraries, and/or
*  helper function. All code must be authentically yours.
******************************************************************************/

/**************************************************************************//**
* @fn			void LCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius,uint16_t color)
* @brief		Draw a colored circle of set radius at coordinates
* @note
*****************************************************************************/
void LCD_drawCircle(uint8_t x0, uint8_t y0, uint8_t radius, uint16_t color)
{
    int x = 0;
    int y = radius;
    int d = 3 - 2 * radius;

    while (y >= x)
    {
        // Draw 8 symmetric points
        LCD_drawPixel(x0 + x, y0 + y, color);
        LCD_drawPixel(x0 - x, y0 + y, color);
        LCD_drawPixel(x0 + x, y0 - y, color);
        LCD_drawPixel(x0 - x, y0 - y, color);
        LCD_drawPixel(x0 + y, y0 + x, color);
        LCD_drawPixel(x0 - y, y0 + x, color);
        LCD_drawPixel(x0 + y, y0 - x, color);
        LCD_drawPixel(x0 - y, y0 - x, color);

        if (d > 0)
        {
            y--;
            d = d + 4 * (x - y) + 10;
        }
        else
        {
            d = d + 4 * x + 6;
        }
        x++;
    }
}

/**************************************************************************//**
* @fn			void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
* @brief		Draw a line from and to a point with a color
* @note
*****************************************************************************/
void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
{
	int dx = abs((int)x1 - (int)x0);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = -abs((int)y1 - (int)y0);
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;          /* e_xy */

    for (;;) {
        if ((unsigned)x0 < LCD_WIDTH && (unsigned)y0 < LCD_HEIGHT)
            LCD_drawPixel((uint8_t)x0, (uint8_t)y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err << 1;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}



/**************************************************************************//**
* @fn			void LCD_drawBlock(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint16_t color)
* @brief		Draw a colored block at coordinates
* @note
*****************************************************************************/
static inline void swap_u8(uint8_t *a, uint8_t *b){ uint8_t t=*a; *a=*b; *b=t; }
void LCD_drawBlock(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1,uint16_t color)
{
    if (x0 > x1) swap_u8(&x0,&x1);
    if (y0 > y1) swap_u8(&y0,&y1);

    if (x1 >= LCD_WIDTH)  x1 = LCD_WIDTH  - 1;
    if (y1 >= LCD_HEIGHT) y1 = LCD_HEIGHT - 1;

    LCD_setAddr(x0, y0, x1, y1);

    clear(LCD_PORT, LCD_TFT_CS);
    clear(LCD_PORT, LCD_DC);
    SPI_ControllerTx_stream(ST7735_RAMWR);      //  RAM
    set(LCD_PORT, LCD_DC);                       

    uint32_t count = (uint32_t)(x1 - x0 + 1) * (uint32_t)(y1 - y0 + 1);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    while (count--) {
        SPI_ControllerTx_stream(hi);
        SPI_ControllerTx_stream(lo);
    }
    set(LCD_PORT, LCD_TFT_CS); 
        
}

/**************************************************************************//**
* @fn			void LCD_setScreen(uint16_t color)
* @brief		Draw the entire screen to a color
* @note
*****************************************************************************/
void LCD_setScreen(uint16_t color) {
    // Fill this range - hard-coded entirety of screen.
    LCD_setAddr(0,0,159,127);

    //preparing for larger, batch writing
    clear(LCD_PORT, LCD_TFT_CS);   // Select LCD
    clear(LCD_PORT, LCD_DC);       // Command mode
    
    //
    SPI_ControllerTx_stream(ST7735_RAMWR); // RAM write command
    set(LCD_PORT, LCD_DC);         // Switch to data mode
    
    // Setting total pixels to stream within address range
    // Intuitively: height of display x width
    //  filled roughly half of screen, manually ticking up from 2x that
    for(uint32_t pix = 0; pix < (uint32_t)2*160*128; pix++) {
        SPI_ControllerTx_stream(color);
    }
        
}

/**************************************************************************//**
* @fn			void LCD_drawString(uint8_t x, uint8_t y, char* str, uint16_t fg, uint16_t bg)
* @brief		Draw a string starting at the point with foreground and background colors
* @note
*****************************************************************************/
void LCD_drawString(uint8_t x, uint8_t y, char* str, uint16_t fg, uint16_t bg)
{
    uint8_t cx = x, cy = y;
    for (size_t i = 0; str[i] != '\0'; ++i) {
        char ch = str[i];
        if (ch == '\n') {               // line feed
            cy += 8; cx = x;
            continue;
        }
        if (cx + 5 >= LCD_WIDTH) {      // auto linefeed
            cy += 8; cx = x;
        }
        if (cy + 8 > LCD_HEIGHT) break; // Stop beyond the screen

        LCD_drawChar(cx, cy, (uint8_t)ch, fg, bg);
        cx += 6;                         // 5 columns of font + 1 column spacing
    }
    
}

void LCD_drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c)
{
    if (!w || !h) return;
    LCD_drawLine(x, y, x+w-1, y, c);
    LCD_drawLine(x, y+h-1, x+w-1, y+h-1, c);
    LCD_drawLine(x, y, x, y+h-1, c);
    LCD_drawLine(x+w-1, y, x+w-1, y+h-1, c);
}
void LCD_fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t c)
{
    if (!w || !h) return;
    LCD_drawBlock(x, y, x+w-1, y+h-1, c);
}