#include <math.h>
#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF  // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1              /////// slowest wdt?? PS1048576 
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock (8MHz) to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x0101      // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

// Constants
#define NUM_SAMPS 1000
#define PI 3.14159
#define IODIR 0x00
#define IOCON 0x05
#define GPIO  0x09
#define OLAT 0x0A
#define GYRO 0b1101011
#define OUT_TEMP_L 0x20

void delay(int time) {
    int delaytime = time; //in hz, core timer freq is half sysfreq
    int starttime;
    starttime = _CP0_GET_COUNT(); 
    while ((int)_CP0_GET_COUNT()-starttime < delaytime){
    ;
    }
   
}

int main() {
	 __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0; 
    // do your TRIS and LAT commands here 
    TRISAbits.TRISA4 = 0;        // Pin A4 is the GREEN LED, 0 for output
    LATAbits.LATA4 = 1;          // make GREEN LED pin "high" (1) = "on"
    TRISBbits.TRISB4 = 1;        // B4 (reset button) is an input pin
    TRISBbits.TRISB2 = 0;        // B2 and B3 set to output for LCD 
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB15 = 0;       // B15 set to output 
    // initialize peripherals and chips
   
    __builtin_enable_interrupts();
    
     _CP0_SET_COUNT(0);
    
    
    //// PWM init. //////
    RPA0Rbits.RPA0R = 0b0101; // OC1 = A0
    RPB8Rbits.RPB8R = 0b0101; // OC2 = B8
    OC1CONbits.OCTSEL = 0; // select timer 2 for both
    OC2CONbits.OCTSEL = 0;
    OC1CONbits.OCM = 0b110; // set to PWM mode
    OC2CONbits.OCM = 0b110;
    T2CONbits.TCKPS = 7; //timer 2 pre-scaler = 1:256, N=256
    PR2 = 6249; // period 2 = (PR2+1) * N * 12.5 ns = 0.02 s, 50 Hz                                                                  
    TMR2 = 0;
    OC1RS = 469;
    OC1R = 469; // should be 7.5% duty cycle
    OC2RS = 469;
    OC2R = 469;
    T2CONbits.ON = 1;
    OC1CONbits.ON = 1;
    OC2CONbits.ON = 1;
    
    while(1) {
       ///////////////PWM//////////////////
       
       //OC1RS = floor((ax*1000))+1000; //duty cycle = OC1RS/2000, range is 0,2g
       
       //OC2RS = floor((ay*1000))+1000; //duty cycle = OC1RS/2000, range is 0,2g
           
       //delay(100000);                      // controls frequency
       
    }
     
}
