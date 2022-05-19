#include "lab03.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * DAC code
 */

#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13
    
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13

uint8_t count = 0;

void dac_initialize()
{
    // set AN10, AN11 AN13 to digital mode
    SETBIT(DAC_SDI_AD1CFG); // set Pin to Digital
    SETBIT(DAC_SCK_AD1CFG); // set Pin to Digital
    SETBIT(DAC_LDAC_AD1CFG ); // set Pin to Digital
    SETBIT(DAC_SDI_AD2CFG); // set Pin to Digital
    SETBIT(DAC_SCK_AD2CFG); // set Pin to Digital
    SETBIT(DAC_LDAC_AD2CFG ); // set Pin to Digital
   
    // this means AN10 will become RB10, AN11->RB11, AN13->RB13
    // see datasheet 11.3
    
    // set RD8, RB10, RB11, RB13 as output pins
    CLEARBIT(DAC_CS_TRIS); // set Pin to Output
    CLEARBIT(DAC_SDI_TRIS); // set Pin to Output
    CLEARBIT(DAC_SCK_TRIS); // set Pin to Output
    CLEARBIT(DAC_LDAC_TRIS); // set Pin to Output
    // set default state: /CS=high, SCK=high, SDI=high, /LDAC=high
    PORTD |= BV(8); // CS
    PORTB |= BV(11);//SCK
    PORTB |= BV(10);
    PORTB |= BV(13);
    
    
    
    
}

/*
 * Timer code
 */

#define FCY_EXT   32768UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

void timer_initialize()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);
    // configure timer
    T1CONbits.TON = 0; //Disable Timer
    T1CONbits.TCS = 1; //Select external clock
    T1CONbits.TSYNC = 0; //Disable Synchronization
    T1CONbits.TCKPS = 0b11; //Select 1:1 Prescaler
    TMR1 = 0x00; //Clear timer register
    PR1 = 64; //Load the period value
    IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1;// Enable Timer1 interrupt
    T1CONbits.TON = 1;// Start Timer
    
}

void set_Voltage(float v){
    
}

// interrupt service routine?

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{ // invoked every ??
    count++;
    switch(count)
        case 1:
            //2,5v
            set_Voltage(2.5);
            LED1_PORT ^= 0x01;
            IFS0bits.T2IF = 0;
            break;
        case 5:
            //3,5v
            set_Voltage(3.5);
            LED1_PORT ^= 0x01;
            IFS0bits.T2IF = 0;
            break;
        case 7:
            //1v
            set_Voltage(1.0);
            count = 0;
            
            LED1_PORT ^= 0x01;
            IFS0bits.T2IF = 0;
            
            break;
}


/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: 1");
    
    //first conversion  to 1v
    set_Voltage(1);
    
    while(TRUE)
    {
        
        // main loop code
    }
}
