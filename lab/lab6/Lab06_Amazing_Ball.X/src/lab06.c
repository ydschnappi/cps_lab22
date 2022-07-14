#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>

#include <math.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


#define X_DIM 0
#define Y_DIM 1

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03
#define CENTER_X 400 
#define CENTER_Y 350 
#define RADIUS 100
#define SPEED 0.15

uint16_t TOUCH_MIN_X = 90;
uint16_t TOUCH_MAX_X = 710;
uint16_t TOUCH_MIN_Y = 100;
uint16_t TOUCH_MAX_Y = 630;
volatile uint16_t touch_x;
volatile uint16_t touch_y;
volatile uint8_t start = 0;
int tick_50hz; // for interrupt
int CUTOFF = 3;
int SAMPLE_RATE = 50;
double pre_output_x;
double pre_output_y;


//PD controller
double prevErrX = 0;
double prevErrY = 0;
double errorX = 0;
double errorY = 0;
double KpX = 0.4, KdX = 0.2;
double KpY = 0.4, KdY = 0.2;
double derivativeX = 0;
double derivativeY = 0;
double outputX = 0;
double outputY = 0;


volatile uint8_t deadline_miss = 0;


double Xpos_set, Ypos_set;

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


void timers_initialize() {

  //Set Timer1 to generate an interrupt every 10ms (100Hz) ==> PR1=500
  CLEARBIT(T1CONbits.TON); //Disable Timer
  CLEARBIT(T1CONbits.TCS); //Select internal instruction cycle clock
  CLEARBIT(T1CONbits.TGATE); //Disable Gated Timer mode
  T1CONbits.TCKPS = 0b11; //Select 1:256 Prescaler
  PR1 = 500; //Load the period value ==> running at 100Hz now!
  TMR1 = 0x00; //Clear timer register
  IPC0bits.T1IP = 0x01; // Set Timer1 Interrupt Priority Level
  CLEARBIT(IFS0bits.T1IF); // Clear Timer1 Interrupt Flag
  SETBIT(IEC0bits.T1IE); // Enable Timer1 interrupt
  SETBIT(T1CONbits.TON); // Start Timer
}





void Touch_Standby(){
        
    SETBIT(PORTEbits.RE1);
    SETBIT(PORTEbits.RE2);
    CLEARBIT(PORTEbits.RE3);
		
}

uint16_t ADC_read (int axis){
	
	if(axis == 0){  
	
		// read X axis

		AD1CHS0bits.CH0SA = 0x00F; //set ADC1 to sample AN15
		SETBIT(AD1CON1bits.SAMP); //start to sample
		while(!AD1CON1bits.DONE); //wait for conversion to finish
        CLEARBIT(AD1CON1bits.SAMP);
		CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
        
        return ADC1BUF0;  //AN15
        //return 1;
    
	} else {
        // read Y axis
		
		AD1CHS0bits.CH0SA = 0x009; //set ADC2 to sample AN9
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

		AD1CHS0bits.CH0SA = 0x00f; //set ADC1 to sample AN15
        
        CLEARBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        __delay_ms(10);
        
		SETBIT(AD1CON1bits.SAMP); //start to sample
		while(!AD1CON1bits.DONE); //wait for conversion to finish
        // CLEARBIT(AD1CON1bits.SAMP);
		CLEARBIT(AD1CON1bits.DONE); //MUST HAVE! clear conversion done bit
        
        return ADC1BUF0;  //AN15
        
        
        //return ADC_read(0);  //read ADC for AN15
    
	} else {
        // read Y axis

        AD1CHS0bits.CH0SA = 0x009; //set ADC2 to sample AN9
        SETBIT(PORTEbits.RE1);
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        __delay_ms(10);
        
		SETBIT(AD1CON1bits.SAMP); //start to sample
		while(!AD1CON1bits.DONE); //wait for conversion to finish
        // CLEARBIT(AD1CON1bits.SAMP);
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
    Nop();
    SETBIT(TRISBbits.TRISB9);
    Nop();
    SETBIT(AD1PCFGLbits.PCFG15);
    Nop();
    SETBIT(AD1PCFGLbits.PCFG9);
    
    CLEARBIT(AD1CON1bits.AD12B);
    AD1CON1bits.FORM = 0;
    AD1CON1bits.SSRC = 0x7;
    
    AD1CON2 = 0;
    CLEARBIT(AD1CON3bits.ADRC);
    AD1CON3bits.SAMC = 0x1F;
    AD1CON3bits.ADCS = 0x2;
    SETBIT(AD1CON1bits.ADON);
  

}



/*
 * Touch screen code
 */



double pdX_controller(double Xp) {
    // TODO: Implement PD X
    double pid;
    errorX = Xpos_set - Xp;
    derivativeX = (errorX - prevErrX);
    outputX = KpX*errorX + KdX*derivativeX;

    pid =outputX;

    prevErrX = errorX;
    return pid;
}


double pdY_controller(double Yp) {
    // TODO: Implement PD Y
    double pid;
    errorY = Ypos_set - Yp;
    derivativeY = (errorY - prevErrY);
    outputY = KpY*errorY + KdY*derivativeY;

    prevErrY = errorY;
    pid = outputY;

    return pid;
}



/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */

double lowPassFrequency(int axis, double input) 
{ 
    double RC = 1.0/(CUTOFF*2*3.14);  
    double dt = 1.0/SAMPLE_RATE;  
    double alpha = dt/(RC+dt); 
    double filtered_output;
    
    if (axis == 0){
    
        filtered_output = pre_output_x + (alpha*(input - pre_output_x)); 
        pre_output_x = filtered_output;
        
    } else {
    
        filtered_output = pre_output_y + (alpha*(input - pre_output_y)); 
        pre_output_y = filtered_output;
       
    }
    
    return filtered_output;
}  


void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
  IFS0bits.T1IF = 0; // clear interrupt flag

  if(start == 1)
    deadline_miss++;
  
  if (tick_50hz == 2){  // every 50hz 
  
  touch_x = lowPassFrequency(0, Touch_Read(0));
  touch_y = lowPassFrequency(1, Touch_Read(1));
  
  tick_50hz = 0;
  }
  
  tick_50hz += 1;
  
  start = 1;
}

double conv_PD_to_Duty(double pd){

 double duty_cycle = 0.003 * pd + 1.5;
 
 if (duty_cycle >= 2.1){
 
     duty_cycle = 2.1;
 } else if (duty_cycle <= 0.9){
 
     duty_cycle = 0.9;
 }
 
 return duty_cycle;
}


void main_loop()
{
    uint8_t hz50_scaler, hz5_scaler, hz1_scaler, sec;
    uint32_t tick = 0;
    
    // print assignment information
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: GroupName");
    lcd_locate(0, 2);
    
    Servo_Init();  
    __delay_ms(200);
    Touch_Init();
    timers_initialize();
    ADC_Init ();
    Touch_Standby();
    
             
    while(TRUE) {
        

        
  lcd_locate(0, 4);   
  lcd_printf("X-axis: %d", touch_x);
  lcd_locate(0, 5);
  lcd_printf("Y-axis: %d", touch_y);  
  
  
  
  // 50Hz control task
    if(hz50_scaler == 0) {
  
      Xpos_set = CENTER_X + RADIUS * cos(tick * SPEED);
      Ypos_set = CENTER_Y + RADIUS * sin(tick * SPEED);
      
        lcd_locate(0, 2);   
        lcd_printf("X-target: %f", Xpos_set);
        lcd_locate(0, 3);
        lcd_printf("Y-target: %f", Ypos_set); 
      
      tick++;

      double pidX = pdX_controller(touch_x);
      double pidY = pdY_controller(touch_y);

      Servo_Control(0, conv_PD_to_Duty(pidX));  // 0.9 ms control X axis 0 degree
      Servo_Control(1, conv_PD_to_Duty(pidY));  //0.9 mscontrol Y axis 0 degree
      
    }
  
    if(hz5_scaler == 0) {

      if(deadline_miss >= 1) {
        lcd_locate(0,6);
        lcd_printf("%4d d_misses!!!", deadline_miss);
      }
    }

    // 1Hz seconds display task
    if(hz1_scaler == 0) {

      sec++;
    }
    
    hz50_scaler = (hz50_scaler + 1) % 2;
    hz5_scaler = (hz5_scaler + 1) % 20;
    hz1_scaler = (hz1_scaler + 1) % 100;

    start = 0;
  }
  
     
    
}



