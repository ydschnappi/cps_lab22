#include "dac.h"
#include "types.h"

// tristate register
#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13

// port register
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

// analog to digital converter 1 port configuration register
#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

// analog to digital converter 2 port configuration register
#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13

void dac_initialize()
{
    // set AN10, AN11 AN13 to digital mode
    SETBIT(DAC_SDI_AD1CFG); // set Pin to Digital
    Nop();
    SETBIT(DAC_SCK_AD1CFG); // set Pin to Digital
    Nop();
    SETBIT(DAC_LDAC_AD1CFG ); // set Pin to Digital
    Nop();
    SETBIT(DAC_SDI_AD2CFG); // set Pin to Digital
    Nop();
    SETBIT(DAC_SCK_AD2CFG); // set Pin to Digital
    Nop();
    SETBIT(DAC_LDAC_AD2CFG); // set Pin to Digital
    Nop();
    // this means AN10 will become RB10, AN11->RB11, AN13->RB13
    // see datasheet 11.3
    CLEARBIT(DAC_CS_TRIS);
    Nop();
    CLEARBIT(DAC_SDI_TRIS); // set Pin to Output
    Nop();
    CLEARBIT(DAC_SCK_TRIS); // set Pin to Output
    Nop();
    CLEARBIT(DAC_LDAC_TRIS); // set Pin to Output
    Nop();
    // set default state: /CS=high, SCK=high, SDI=high, /LDAC=high
    // PORTD |= BV(8); // CS
    SETBIT(DAC_CS_PORT);
    Nop();
    // PORTB |= BV(11);//SCK
    CLEARBIT(DAC_SCK_PORT);
    Nop();
    // PORTB |= BV(10);
    // PORTB |= BV(13);
    SETBIT(DAC_LDAC_PORT);
    Nop();
    CLEARBIT(DAC_SDI_PORT);
    Nop();

}

void set_Voltage(int V)
{
    V = V + 4096;
    // clear CS
    CLEARBIT(DAC_CS_PORT);
    Nop();
    int i=15;
    while(i>=0)
    {
        // uint16_t value = V&BV(i);
        // value = value >> i;
        // DAC_SDI_PORT= value;
        PORTB &= ~BV(10); // Set SDI low
        Nop();
        PORTB |= (V & BV(i)) >> i << 10; // Set SDI high only if bit 2 in var is 1.
        Nop();
        // toggle clock
        SETBIT(DAC_SCK_PORT);
        Nop();
        CLEARBIT(DAC_SCK_PORT);
        Nop(); 
        i--;
    }
    
    //clear CS
    SETBIT(DAC_CS_PORT);
    Nop();
    // clear SDI
    CLEARBIT(DAC_SDI_PORT);
    Nop();
    
    //toggle LDAC
    CLEARBIT(DAC_LDAC_PORT);
    Nop(); 
    Nop(); 
    SETBIT  (DAC_LDAC_PORT);
    Nop();
}

void dac_convert_milli_volt(uint16_t milliVolt)
{
    
}
