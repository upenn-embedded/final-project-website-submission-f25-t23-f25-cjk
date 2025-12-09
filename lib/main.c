//#include "uart.h"
//#include "joystick.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "ST7735.h"
#include "LCD_GFX.h"
#include "uart.h"
//#include "pong.h"
//#include "motor.h"

void main(){
    lcd_init();
    //joystick_init();
    uart_init(115200);
    
    LCD_drawCircle(0x40, 0x40, 10, WHITE);
    //LCD_setScreen(RED);

    while(1){
    }
}