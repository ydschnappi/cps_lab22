#include "lab02.h"

#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdio.h>
#include "types.h"
#include "lcd.h"
#include "led.h"

#define FCY_EXT 32768
uint8_t millisec = 0;
uint8_t sec = 0;
uint8_t min = 0;

void initialize_timer()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
   __builtin_write_OSCCONL(OSCCONL | 2); // Disable Timer
   
    CLEARBIT(T2CONbits.TON);
    // Set Prescaler
    T2CONbits.TCKPS = 0b11;
    // Set Clock Source
    CLEARBIT(T2CONbits.TCS);
    // Set Gated Timer Mode -> don't use gating
    CLEARBIT(T2CONbits.TGATE);
    // T1: Set External Clock Input Synchronization -> no sync
    //T1CONbits.TSYNC = 0;
    
    // Clear Timer2 interrupt status flag
    CLEARBIT(IFS0bits.T2IF); 
    
    // Load Timer Periods
    PR2 = 100; // Set timer period 40ms: 2*10^-3*12.8*10^6*1/256
    // Reset Timer Values
    TMR2 = 0x00; // Clear timer register
    // Set Interrupt Priority
    IPC1bits.T2IP = 0x01;
    // Clear Interrupt Flags
    CLEARBIT(IFS0bits.T2IF);
    
    // Enable Interrupts
    SETBIT(IEC0bits.T2IE);
    
    
    // Enable the Timers
    SETBIT(T2CONbits.TON);
    
    //Set for Timer1
   
    T1CONbits.TON = 0; //Disable Timer
    T1CONbits.TCS = 1; //Select external clock
    T1CONbits.TSYNC = 0; //Disable Synchronization
    T1CONbits.TCKPS = 0b11; //Select 1:1 Prescaler
    TMR1 = 0x00; //Clear timer register
    PR1 = 128; //Load the period value
    IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1;// Enable Timer1 interrupt
    T1CONbits.TON = 1;// Start Timer


    //Set for Timer3
   
    T3CONbits.TON = 0; //Disable Timer
    CLEARBIT(T3CONbits.TCS);
    T3CONbits.TCS = 0; //Select external clock
    T3CONbits.TCKPS = 0b00; //Select 1:1 Prescaler
    TMR3 = 0x00; //Clear timer register
    PR3 = 65535; //Load the period value
    IPC2bits.T3IP = 0x01; // Set Timer1 Interrupt Priority Level
    IFS0bits.T3IF = 0; // Clear Timer1 Interrupt Flag
    IEC0bits.T3IE = 1;// Enable Timer1 interrupt
    T3CONbits.TON = 1;// Start Timer
}

void timer_loop()
{   
    
    // print assignment information
    lcd_printf("Lab02: Int & Timer");
    lcd_locate(0, 1);
    lcd_printf("Group: 1");
    
    
    uint16_t iteration = 0;
    // Stop
    
    while(1){
        if (iteration == 2000){
            // uint16_t temp = TMR3;
            lcd_locate(0, 2);
            lcd_printf("%d, %.3f", TMR3,TMR3/12800.0); 
            // lcd_locate(0, 3);
            // lcd_printf("%.3f",temp/12800.0); //
            iteration = 0;
            lcd_locate(0, 4);
            lcd_printf("%02d:%02d:%03d",min,sec,millisec);  
            LED3_PORT ^= BV(0);
            TMR3 = 0x00;
            IFS0bits.T3IF = 0;
            
        }
        
        // nothing happen
        iteration ++;   
    }
    

}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{ // invoked every ??
    LED2_PORT ^= BV(0);
    IFS0bits.T1IF = 0;
    sec = sec + 1;
    if (sec / 60){
        sec= 0;
        min++;
    }
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{ // invoked every ??
  LED1_PORT ^= 0x01;
  IFS0bits.T2IF = 0;
  
  millisec = millisec + 2;  // 0 ms , 1st IT = 2ms, 2nd IT = 2ms * 2 ..... 
  if(millisec == 1000){
      millisec = 0;
  }
}


void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T3Interrupt(void)
{ // invoked every ??
  IFS0bits.T3IF = 0;
}