/*
 * File:   main_class.c
 * Author: klink
 *
 * Created on April 1, 2015, 6:21 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h> // processor SFR definitions
#include <sys/attribs.h> // __ISR macro

// DEVCFGs here


//These are the DEVCFG bits for the NU32 in standalone mode:
//#pragma config DEBUG = OFF          // Background Debugger disabled
//#pragma config FPLLMUL = MUL_20     // PLL Multiplier: Multiply by 20
//#pragma config FPLLIDIV = DIV_2     // PLL Input Divider:  Divide by 2
//#pragma config FPLLODIV = DIV_1     // PLL Output Divider: Divide by 1
//#pragma config FWDTEN = OFF         // WD timer: OFF
//#pragma config POSCMOD = HS         // Primary Oscillator Mode: High Speed xtal
//#pragma config FNOSC = PRIPLL       // Oscillator Selection: Primary oscillator w/ PLL
//#pragma config FPBDIV = DIV_1       // Peripheral Bus Clock: Divide by 1
//#pragma config BWP = OFF            // Boot write protect: OFF
//#pragma config ICESEL = ICS_PGx2    // ICE pins configured on PGx2, Boot write protect OFF.
//#pragma config FSOSCEN = OFF        // Disable second osc to get pins back
//#pragma config FSRSSEL = PRIORITY_7 // Shadow Register Set for interrupt priority 7


// These are the available DEVCFG bits for the PIC32MX250F128B are listed in the documentation that comes with XC32, in microchip/xc32/v1.34/docs/config_docs/32mx250f128b.html

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // not boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 40MHz
#pragma config UPLLIDIV = DIV_2 // divide clock to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0x1234 // some 16bit userid
#pragma config PMDL1WAY = ON // not multiple reconfiguration, check this
#pragma config IOL1WAY = ON // not multimple reconfiguration, check this
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // controlled by USB module

int readADC(void);

int main() {

    // startup
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that
    // kseg0 is cacheable (0x3) or uncacheable (0x2)
    // see Chapter 2 "CPU for Devices with M4K Core"
    // of the PIC32 reference manual
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // no cache on this chip!

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
    DDPCONbits.JTAGEN = 0;

    __builtin_enable_interrupts();


//Analog (ANx) pins default to be analog input. Use ANSELA or ANSELB to make them digital
    ANSELAbits.ANSA0 = 0; // 0 for digital, 1 for analog

  // set up USER pin as input
    ANSELBbits.ANSB13 = 0; // 0 for digital, 1 for analog
    TRISBbits.TRISB13 = 1; // 0 for output, 1 for input


  // set up LED1 pin as a digital output
    RPB7Rbits.RPB7R = 0b0001; // set B7 to U1TX
    TRISBbits.TRISB7 = 0; // 0 for output, 1 for input
    LATBbits.LATB7 = 1;

  // set up LED2 as OC1 using Timer2 at 1kHz
  //For a peripheral that requires an output pin, use the RP(pin name)Rbits.RP(pin name)R to set the pin, using table 12-2 in Chapter 12
    ANSELBbits.ANSB15 = 0; // 0 for digital, 1 for analog
    RPB15Rbits.RPB15R = 0b0101;// set B15 to OC1

  // set up A0 as AN0
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;

    T2CONbits.TCKPS = 0;     // Timer3 prescaler N=2 (1:2)
	PR2 = 39999;              // period = (PR2+1) * N * 12.5 ns = 100 us, 20 kHz 9999
	TMR2 = 0;                // initial TMR2 count is 0 2
	OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
//	OC1CONbits.OCTSEL = 1;   // set Output compare timer select bit to use timer 3 as clock source
	OC1RS = 0;            // duty cycle = OC1RS/(PR2+1) = 75% 5000
	OC1R = 0;             // initialize before turning OC1 on; afterward it is read-only 5000
	T2CONbits.ON = 1;        // turn on Timer2
  	OC1CONbits.ON = 1;       // turn on OC1

 //Use the core timer to double check your CPU clock settings

while (1) {
    _CP0_SET_COUNT(0); // set core timer to 0, remember it counts at half the CPU clock
    LATBINV = 0x0080; // invert a pin

    // wait for half a second, setting LED brightness to pot angle while waiting
        while (_CP0_GET_COUNT() < 10000000) {
            int val = readADC();
            OC1RS = val * PR2/1024;

            if (PORTBbits.RB13 == 1) {
                // nothing
            } else {
                LATBINV = 0x0080;
            }
        }
    }
}

int readADC(void) {
    int elapsed = 0;
    int finishtime = 0;
    int sampletime = 20;
    int a = 0;

    AD1CON1bits.SAMP = 1;
    elapsed = _CP0_GET_COUNT();
    finishtime = elapsed + sampletime;
    while (_CP0_GET_COUNT() < finishtime) {
    }
    AD1CON1bits.SAMP = 0;
    while (!AD1CON1bits.DONE) {
    }
    a = ADC1BUF0;
    return a;
}