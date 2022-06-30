#include "lab04.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "math.h"

#include "types.h"
#include "lcd.h"
#include "led.h"
#include "dac.h"

// signal parameter
#define FREQUENCY 1
#define SAMPLE_RATE 50*FREQUENCY
#define V_MAX 3.0
#define V_MIN 1.0
#define PI 3.14

/*
 * Timer code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define Cal_PR3(prescale,sample_rate) 12800000/(prescale*sample_rate)   // 12.8Mhz, 256 prescaler

uint16_t counter = 0;

void timer_initialize()
{
    __builtin_write_OSCCONL(OSCCONL | 2); // Disable Timer
    T3CONbits.TON = 0; //Disable Timer
    CLEARBIT(T3CONbits.TCS);
    T3CONbits.TCS = 0; //Select external clock
    T3CONbits.TCKPS = 0b11; //Select 1:256 Prescaler, 50000Hz
    TMR3 = 0x00; //Clear timer register
    PR3 = Cal_PR3(256, SAMPLE_RATE); //Load the period value
    IPC2bits.T3IP = 0x01; // Set Timer1 Interrupt Priority Level
    IFS0bits.T3IF = 0; // Clear Timer1 Interrupt Flag
    IEC0bits.T3IE = 1;// Enable Timer1 interrupt
    T3CONbits.TON = 1;// Start Timer
}




void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{ // invoked every ??
    LED1_PORT ^= 0x01;
    double voltage = 0.5*(V_MAX - V_MIN) * sin(2*PI*FREQUENCY * counter/SAMPLE_RATE) + 0.5*(V_MAX + V_MIN);
    counter ++;
    set_Voltage((int)(1000*voltage));
    //set_Voltage(1500);
    IFS0bits.T3IF = 0;
}
/*
 * main loop
 */


void main_loop()
{
    // print assignment information
    lcd_printf("Lab04: Wave");
    lcd_locate(0, 1);
    lcd_printf("Group: 1");
    
    while(TRUE) { }
}
