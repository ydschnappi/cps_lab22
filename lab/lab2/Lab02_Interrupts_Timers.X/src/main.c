#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL 
#include <libpic30.h>

#include "lcd.h"
#include "led.h"
#include "lab02.h"

/* Configuration of the Chip */
// Initial Oscillator Source Selection = Primary (XT, HS, EC) Oscillator with PLL
#pragma config FNOSC = PRIPLL
// Primary Oscillator Mode Select = XT Crystal Oscillator mode
#pragma config POSCMD = XT
// OSC2 Pin Function: OSC2 is Clock Output
#pragma config OSCIOFNC = ON
// Watchdog Timer Enable = Watchdog Timer enabled/disabled by user software
// (LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
#pragma config FWDTEN = OFF
uint8_t COUNT=0;
int main(){
    //Init LCD and LEDs
    lcd_initialize();
    led_init();
	
    // Clear the Screen and reset the cursor
    lcd_clear();
    lcd_locate(0, 0);
    
    // Initialize Lab 02 Timers
    initialize_timer();
    
    // Start Lab02 Main Program
    timer_loop();
    
    

    // Stop
    while(1){
   
    }
        
        
        
        ;
}

