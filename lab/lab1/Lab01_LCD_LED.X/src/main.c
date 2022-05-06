#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL 
#include <libpic30.h>

#include "lcd.h"
#include "led.h"

/* Configuration of the Chip */
// Initial Oscillator Source Selection = Primary (XT, HS, EC) Oscillator with PLL
#pragma config FNOSC = PRIPLL
// Primary Oscillator Mode Select = XT Crystal Oscillator mode
#pragma config POSCMD = XT
// Watchdog Timer Enable = Watchdog Timer enabled/disabled by user software
// (LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
#pragma config FWDTEN = OFF
uint8_t COUNT = 0;

void timer_1_init(){
    __builtin_write_OSCCONL(OSCCONL | 2);
    T1CONbits.TON = 0; //Disable Timer
    T1CONbits.TCS = 1; //Select external clock
    T1CONbits.TSYNC = 0; //Disable Synchronization
    T1CONbits.TCKPS = 0b00; //Select 1:1 Prescaler
    TMR1 = 0x00; //Clear timer register
    PR1 = 32767; //Load the period value
    IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1;// Enable Timer1 interrupt
    T1CONbits.TON = 1;// Start Timer
}
void name_print(){
    lcd_locate(0, 0);
    lcd_printf("Zhelin Yang");
    lcd_locate(0, 1);
    lcd_printf("Dian Yuan");
}

void led_print(uint8_t num){
        LED5_PORT = num % 2;
        num/=2;
        LED4_PORT = num % 2;
        num/=2;
        LED3_PORT = num % 2;
        num/=2;
        LED2_PORT = num % 2;
        num/=2;
        LED1_PORT = num % 2;
        num/=2;
}

void __attribute__((__interrupt__)) _T1Interrupt(void)
{
    COUNT++;
    if(COUNT == 32){
        // lcd_clear_row(2);
        lcd_clear();
        name_print();
        lcd_locate(0, 2);
        COUNT = 0;
        lcd_printf("%d",COUNT);
        led_print(COUNT);
    };
    lcd_locate(0, 2);
    lcd_printf("%d",COUNT);
    led_print(COUNT);
    // count++; // Increment a global counter
    IFS0bits.T1IF = 0; // clear the interrupt flag
}


int main(){
    //Init LCD and LEDs
    lcd_initialize();
    led_init();

    // Clear the Screen and reset the cursor
    lcd_clear();
    name_print();
    lcd_locate(0, 2);
    lcd_printf("%d",COUNT);
    led_print(COUNT);
    timer_1_init();
    
    while(1){
    };
}

