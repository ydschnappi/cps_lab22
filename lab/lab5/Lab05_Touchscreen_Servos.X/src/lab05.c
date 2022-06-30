#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


void servo_init(){
    
    //setup OC8 (X axis)
    CLEARBIT(TRISDbits.TRISD7); // Set OC8 as output
    OC8R = 300; // Set the initial duty cycle to 0ms
    OC8RS = 300; // Load OCRS: next pwm duty cycle
    OC8CON = 0x0006; // Set OC8: PWM, no fault check, Timer2
    SETBIT(T2CONbits.TON); // Turn Timer 2 off
    
    //setup OC7 (Y axis)
    CLEARBIT(TRISDbits.TRISD6); // Set OC7 as output
    OC7R = 300; // Set the initial duty cycle to 0ms
    OC7RS = 300; // Load OCRS: next pwm duty cycle
    OC7CON = 0x0006; // Set OC7: PWM, no fault check, Timer2
    SETBIT(T2CONbits.TON); // Turn Timer 2 off  
}

void servo_control (int axis, float duty_cycle){ 

    // axis X = 0, axis Y = 1
    // duty_cycle (ms)
    int OC_Value = (int)(duty_cycle * 4000.0 / 20.0);
    lcd_printf("%d",OC_Value);       
    if (axis == 0){  
        //CLEARBIT(TRISDbits.TRISD7); // Set OC8 as output
        //OC8R = OC_Value; // Set the initial duty cycle to 0ms
        OC8RS = OC_Value; // Load OCRS: next pwm duty cycle
        OC8CON = 0x0006; // Set OC8: PWM, no fault check, Timer2
        //SETBIT(T2CONbits.TON); // Turn Timer 2 on
        
    } else {
        //CLEARBIT(TRISDbits.TRISD6); // Set OC7 as output
        //OC7R = OC_Value; // Set the initial duty cycle to 0ms
        OC7RS = OC_Value; // Load OCRS: next pwm duty cycle
        OC8CON = 0x0006; // Set OC8: PWM, no fault check, Timer2
        //SETBIT(T2CONbits.TON); // Turn Timer 2 on
    }
}

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

void touch_init(){

CLEARBIT(TRISEbits.TRISE1); // I/O pin set to output
CLEARBIT(TRISEbits.TRISE2); // I/O pin set to output
CLEARBIT(TRISEbits.TRISE3); // I/O pin set to output

}

unsigned int touch_output(int axis){

    if(axis == 0){  // read X axis
        CLEARBIT(PORTEbits.RE1);
        SETBIT(PORTEbits.RE2);
        SETBIT(PORTEbits.RE3);
        
        return (PORTBbits.RB15);  //AN15
    } else {
        // read Y axis
        SETBIT(PORTEbits.RE1);
        CLEARBIT(PORTEbits.RE2);
        CLEARBIT(PORTEbits.RE3);
        
        return (PORTBbits.RB9);  //AN9
    }
       
        
}

unsigned int touch_standby(){
        
        SETBIT(PORTEbits.RE1);
        SETBIT(PORTEbits.RE2);
        CLEARBIT(PORTEbits.RE3);
}

void timer_init()
{
    //__builtin_write_OSCCONL(OSCCONL | 2); // Disable Timer
    CLEARBIT(T2CONbits.TON); // Disable Timer
    CLEARBIT(T2CONbits.TCS); // Select internal instruction cycle clock
    CLEARBIT(T2CONbits.TGATE); // Disable Gated Timer mode
    TMR2 = 0x00; // Clear timer register
    T1CONbits.TCKPS = 0b10; // Select 1:64 Prescaler
    CLEARBIT(IFS0bits.T2IF); // Clear Timer2 interrupt status flag
    CLEARBIT(IEC0bits.T2IE); // Disable Timer2 interrupt enable control bit
    PR2 = 4000; // 20 ms  = 0.02 * 12.8 * 1000000 / 64
    
}

void main_control(){

servo_control(0, 1.5);  // 0.9 ms control X axis 0 degree
servo_control(1, 1.5);  //0.9 mscontrol Y axis 0 degree

}

void main_loop()
{
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: 1");
    
    timer_init();
    //touch_init();
    servo_init();
    // initialize touchscreen
    
    // initialize servos
    main_control(); 
    while(TRUE) {
       
       
        
    }
}
