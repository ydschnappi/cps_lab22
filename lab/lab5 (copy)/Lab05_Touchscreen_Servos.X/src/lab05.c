#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

void Servo_Init(){
    
	TRISDbits.TRISD7 = 0; //Set 0C8 as output
	TRISDbits.TRISD6 = 0; //Set 0C7 as output
	
	//Initialize 0C8 (X axis)
	OC8CONbits.OCM = 0b000; // Disable Output Compare Module
	OC8R = 3700; // Write the duty cycle for the first PWM pulse (Duty 1.5 ms)
	OC8RS = 3700; // Write the duty cycle for the second PWM pulse (Duty 1.5 ms)
	OC8CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
	OC8CONbits.OCM = 0b110; // Select the Output Compare mode

	//Initialize 0C7 (Y axis)
	OC7CONbits.OCM = 0b000; // Disable Output Compare Module
	OC7R = 3700; // Write the duty cycle for the first PWM pulse (Duty 1.5 ms)
	OC7RS = 3700; // Write the duty cycle for the second PWM pulse (Duty 1.5 ms)
	OC7CONbits.OCTSEL = 0; // Select Timer 2 as output compare time base
	OC7CONbits.OCM = 0b110; // Select the Output Compare mode

	// Initialize and enable Timer2
	T2CONbits.TON = 0; // Disable Timer
	T2CONbits.TCS = 0; // Select internal instruction cycle clock
	T2CONbits.TGATE = 0; // Disable Gated Timer mode
	T2CONbits.TCKPS = 0b10; // Select 1:1 Prescaler
	TMR2 = 0x00; // Clear timer register
	PR2 = 4000; // Load the period value, 20 ms  = 0.02 * 12.8 * 1,000,000 / 64
	IPC1bits.T2IP = 0x01; // Set Timer 2 Interrupt Priority Level
	IFS0bits.T2IF = 0; // Clear Timer 2 Interrupt Flag
	IEC0bits.T2IE = 0; // Disable Timer 2 interrupt
	T2CONbits.TON = 1; // Start Timer
	__delay_ms(2000);
}

void Servo_Control (int axis, float duty_cycle){ 

    // Input1: axis X = 0, axis Y = 1
    // Input2: duty_cycle (ms)
	
    int OC_Value = (int)(4000.0 - duty_cycle * 4000.0 / 20.0);
      
    if (axis == 0){  
	
        OC8RS = OC_Value; // Load OCRS: next pwm duty cycle  
		
    } else {

        OC7RS = OC_Value; // Load OCRS: next pwm duty cycle
    }
}



void Touch_Init(){

		CLEARBIT(TRISEbits.TRISE1); // I/O pin set to output
		CLEARBIT(TRISEbits.TRISE2); // I/O pin set to output
		CLEARBIT(TRISEbits.TRISE3); // I/O pin set to output

}

void Touch_Standby(){
        
    SETBIT(PORTEbits.RE1);
    SETBIT(PORTEbits.RE2);
    CLEARBIT(PORTEbits.RE3);
		
}

uint16_t ADC_read (int axis){
	
	if(axis == 0){  
	
		// read X axis

		AD1CHS0bits.CH0SA = 0x000F; //set ADC1 to sample AN15
		SETBIT(AD1CON1bits.SAMP); //start to sample
		while(!AD1CON1bits.DONE); //wait for conversion to finish
        CLEARBIT(AD1CON1bits.SAMP);
		CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
        
        return ADC1BUF0;  //AN15
        //return 1;
    
	} else {
        // read Y axis
		
		AD1CHS0bits.CH0SA = 0x0009; //set ADC2 to sample AN9
		SETBIT(AD1CON1bits.SAMP); //start to sample
		while(!AD1CON1bits.DONE); //wait for conversion to finish
        CLEARBIT(AD1CON1bits.SAMP);
		CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
        
        return ADC1BUF0;  //AN9
  
	}


}

uint16_t Touch_Read(int axis){

	// input: X asix (0) or Y axis (1)
	// output: Analog pin

    if(axis == 0){  // read X axis
 

		AD1CHS0bits.CH0SA = 0x000F; //set ADC1 to sample AN15
        CLEARBIT(PORTEbits.RE1);
        SETBIT(PORTEbits.RE2);
        SETBIT(PORTEbits.RE3);
		SETBIT(AD1CON1bits.SAMP); //start to sample
		while(!AD1CON1bits.DONE); //wait for conversion to finish
        CLEARBIT(AD1CON1bits.SAMP);
		CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
        
        return ADC1BUF0;  //AN15
        
        //return ADC_read(0);  //read ADC for AN15
    
	} else {
        // read Y axis

        AD1CHS0bits.CH0SA = 0x0009; //set ADC2 to sample AN9
        SETBIT(PORTEbits.RE1);
        CLEARBIT(PORTEbits.RE2);
        CLEARBIT(PORTEbits.RE3);
		SETBIT(AD1CON1bits.SAMP); //start to sample
		while(!AD1CON1bits.DONE); //wait for conversion to finish
        CLEARBIT(AD1CON1bits.SAMP);
		CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
        
        return ADC1BUF0;  //AN9
        
        // return ADC_read(1);  //read ADC for AN9
    
	}    
}

void ADC_Init (){
	CLEARBIT(TRISEbits.TRISE1); // I/O pin set to output
	CLEARBIT(TRISEbits.TRISE2); // I/O pin set to output
	CLEARBIT(TRISEbits.TRISE3); // I/O pin set to output
    //disable ADC
	CLEARBIT(AD1CON1bits.ADON);
    
    SETBIT(TRISBbits.TRISB15);
    SETBIT(TRISBbits.TRISB9);
    SETBIT(AD1PCFGLbits.PCFG15);
    SETBIT(AD1PCFGLbits.PCFG9);
    
    CLEARBIT(AD1CON1bits.AD12B);
    AD1CON1bits.FORM = 0;
    AD1CON1bits.SSRC = 0x7;
    
    AD1CON2 = 0;
    CLEARBIT(AD1CON3bits.ADRC);
    AD1CON3bits.SAMC = 0x1F;
    AD1CON3bits.ADCS = 0x2;
    SETBIT(AD1CON1bits.ADON);
    
	//Init. ADC for AN15
	//disable ADC
	//CLEARBIT(AD1CON1bits.ADON);
	//initialize PIN
	//SETBIT(TRISBbits.TRISB15); //set TRISB RB15 to input
	//SETBIT(AD1PCFGLbits.PCFG15); //set ADC1 pin AN20 as analog
	//Configure AD1CON1
	//CLEARBIT(AD1CON1bits.AD12B); //set 10b resolution
	//AD1CON1bits.FORM = 0; //set integer output
	//AD1CON1bits.SSRC = 0x7; //set automatic conversion
	//Configure AD1CON2
	//AD1CON2 = 0; //not using scanning sampling
	//Configure AD1CON3
	//CLEARBIT(AD1CON3bits.ADRC); //internal clock source
	//AD1CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
	//AD1CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
	//Leave AD1CON4 at its default value
	//enable ADC
	//SETBIT(AD1CON1bits.ADON);


	//Init. ADC for AN9
	//disable ADC
	//CLEARBIT(AD2CON1bits.ADON);
	//initialize PIN
	//SETBIT(TRISBbits.TRISB9); //set TRISB RB9 to input
	//SETBIT(AD2PCFGLbits.PCFG9); //set ADC2 pin AN9 as analog
	//Configure AD1CON1
	//CLEARBIT(AD2CON1bits.AD12B); //set 10b resolution
	//AD2CON1bits.FORM = 0; //set integer output
	//AD2CON1bits.SSRC = 0x7; //set automatic conversion
	//Configure AD1CON2
	//AD2CON2 = 0; //not using scanning sampling
	//Configure AD1CON3
	//CLEARBIT(AD2CON3bits.ADRC); //internal clock source
	//AD2CON3bits.SAMC = 0x1F; //sample-to-conversion clock = 31Tad
	//AD2CON3bits.ADCS = 0x2; //Tad = 3Tcy (Time cycles)
	//Leave AD1CON4 at its default value
	//enable ADC
	//SETBIT(AD2CON1bits.ADON);
    

}



void main_task(){

// step 1
Touch_Standby();
Servo_Control(0, 0.9);  // 0.9 ms control X axis 0 degree
Servo_Control(1, 0.9);  //0.9 mscontrol Y axis 0 degree
__delay_ms(3000);
lcd_locate(0, 3);
lcd_printf("X-axis: %d, Y-axis: %d", Touch_Read(0), Touch_Read(1));
__delay_ms(1000);

// step 2
Touch_Standby();
Servo_Control(0, 0.9);  // 0.9 ms control X axis 0 degree
Servo_Control(1, 2.1);  //0.9 mscontrol Y axis 0 degree
__delay_ms(3000);
lcd_locate(0, 3);
lcd_printf("X-axis: %d, Y-axis: %d", Touch_Read(0), Touch_Read(1));
__delay_ms(1000);

// step 3
Touch_Standby();
Servo_Control(0, 2.1);  // 0.9 ms control X axis 0 degree
Servo_Control(1, 2.1);  //0.9 mscontrol Y axis 0 degree
__delay_ms(3000);
lcd_locate(0, 3);
lcd_printf("X-axis: %d, Y-axis: %d", Touch_Read(0), Touch_Read(1));
__delay_ms(1000);

// step 4
Touch_Standby();
Servo_Control(0, 2.1);  // 0.9 ms control X axis 0 degree
Servo_Control(1, 0.9);  //0.9 mscontrol Y axis 0 degree
__delay_ms(3000);
lcd_locate(0, 3);
lcd_printf("X-axis: %d, Y-axis: %d", Touch_Read(0), Touch_Read(1));
__delay_ms(1000);

}

void main_loop()
{
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: 1");
    
    //timer_init();
    //touch_init();
    Servo_Init();
	ADC_Init();
	// Touch_Init();
    // initialize touchscreen
    
    // initialize servos
    
    while(TRUE) {
    main_task();   
        
    }
}
