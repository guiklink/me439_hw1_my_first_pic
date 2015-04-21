/* 
 * File:   main_class.c
 * Author: klink
 *
 * Created on April 1, 2015, 5:28 PM
 */

#include<xc.h> // processor SFR definitions
#include<sys/attribs.h>

#include "i2c_display.h"
#include "accel.h" // __ISR macro

////////////////////////////// DEVCFGs here /////////////////////////////////
/*These are the DEVCFG bits for the NU32 in standalone mode:
#pragma config DEBUG = OFF          // Background Debugger disabled
#pragma config FPLLMUL = MUL_20     // PLL Multiplier: Multiply by 20
#pragma config FPLLIDIV = DIV_2     // PLL Input Divider:  Divide by 2
#pragma config FPLLODIV = DIV_1     // PLL Output Divider: Divide by 1
#pragma config FWDTEN = OFF         // WD timer: OFF
#pragma config POSCMOD = HS         // Primary Oscillator Mode: High Speed xtal
#pragma config FNOSC = PRIPLL       // Oscillator Selection: Primary oscillator w/ PLL
#pragma config FPBDIV = DIV_1       // Peripheral Bus Clock: Divide by 1
#pragma config BWP = OFF            // Boot write protect: OFF
#pragma config ICESEL = ICS_PGx2    // ICE pins configured on PGx2, Boot write protect OFF.
#pragma config FSOSCEN = OFF        // Disable second osc to get pins back
#pragma config FSRSSEL = PRIORITY_7 // Shadow Register Set for interrupt priority 7
*/

//These are the available DEVCFG bits for the PIC32MX250F128B are listed in the documentation that comes with XC32, in microchip/xc32/v1.33/docs/config_docs/32mx250f128b.html
// DEVCFG0
#pragma config DEBUG = OFF          // no debugging
#pragma config JTAGEN = OFF         // no jtag
#pragma config ICESEL = ICS_PGx1    // use PGED1 and PGEC1
#pragma config PWP = OFF            // no write protect
#pragma config BWP = OFF            // not boot write protect
#pragma config CP = OFF             // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL       // use primary oscillator with pll
#pragma config FSOSCEN = OFF        // turn off secondary oscillator
#pragma config IESO = OFF           // no switching clocks
#pragma config POSCMOD = HS         // high speed crystal mode
#pragma config OSCIOFNC = OFF       // free up secondary osc pins
#pragma config FPBDIV = DIV_1       // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD       // do not enable clock switch
#pragma config WDTPS = PS1048576    // slowest wdt
#pragma config WINDIS = OFF         // no wdt window
#pragma config FWDTEN = OFF         // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 40MHz
#pragma config UPLLIDIV = DIV_2 // divide 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON      // USB clock on

// DEVCFG3
#pragma config USERID = 0       // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON    // allow only one reconfiguration
#pragma config IOL1WAY = ON     // allow only one reconfiguration
#pragma config FUSBIDIO = ON    // USB pins controlled by USB module
#pragma config FVBUSONIO = ON   // controlled by USB module

/////////////////////////////////////////////////////////////////////////////

#define ACCEL_AXIS_SCALE 30/3200

int readADC(void);
void accel_draw_axis(short x, short y);

int main() {

    /////////////////////////////// startup ////////////////////////////////

    //Startup code to run as fast as possible and get pins back from bad defaults

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

    /////////////////////////////////////////////////////////////////////////

    // set up USER pin as input

    ANSELBbits.ANSB13 = 0; // 0 for digital, 1 for analog
    TRISBbits.TRISB13 = 1;  // USER is input

    // set up LED1 pin as a digital output
    RPB7Rbits.RPB7R = 0b0001; // set B7 to U1TX
    TRISBbits.TRISB7 = 0;
    


    // set up LED2 as OC1 using Timer2 at 1kHz
    ANSELBbits.ANSB15 = 0; // 0 for digital, 1 for analog
    //For a peripheral that requires an output pin, use the RP(pin name)Rbits.RP(pin name)R to set the pin, using table 12-2 in Chapter 12
    RPB15Rbits.RPB15R = 0b0101; // set B15 to U1TX

    // set up A0 as AN0
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;

    //LATBSET = 0x80;

    //////////////////////////// Initialize timers ////////////////////////////

    T2CONbits.TCKPS = 1;     // Timer2 prescaler = 2
    PR2 = 19999;              // ClockFrequency / PreScaler / Period = frequency in Hz
    TMR2 = 0;                // initial TMR2 count is 0
    OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
    OC1CONbits.OCTSEL = 0;   // set Output compare timer select bit to use timer 2 as clock source
    OC1RS = 0;               // duty cycle = OC1RS/(PR2+1) = 75%
    OC1R = 0;                // initialize before turning OC1 on; afterward it is read-only
    T2CONbits.ON = 1;        // turn on Timer2
    OC1CONbits.ON = 1;       // turn on OC1

    ///////////////////////////////////////////////////////////////////////////
    //_CP0_SET_COUNT(0);       // init core timer

    // Variables to write msg on display
    char buffer[20];
    short number = 1337;

    //sprintf(buffer,"Hello world %d!",number);

    // Variables to store accel data
    short accels[3]; // accelerations for the 3 axes
    short mags[3]; // magnetometer readings for the 3 axes
    short temp;

    display_init(); // initialize I2C2
    acc_setup();    // initialize accelerometer

    display_clear();
    display_draw();

    // Variables to draw in different axis
    short accel_x = 0;
    short accel_y = 0;
    short accel_z = 0;



    while (1)
    {
        // invert pin every 0.5s, set PWM duty cycle % to the pot voltage output %
        _CP0_SET_COUNT(0); // set core timer to 0, remember it counts at half the CPU clock
        LATBINV = 0x0080; // invert a pin
        // wait for half a second, setting LED brightness to pot angle while waiting

            while (_CP0_GET_COUNT() < 10000000)
            {
                OC1RS = readADC() * PR2/1024;
                if (!PORTBbits.RB13) {
                    LATBINV = 0x0080;
                }


                // read the accelerometer from all three axes
                // the accelerometer and the pic32 are both little endian by default (the lowest address has the LSB)
                // the accelerations are 16-bit twos compliment numbers, the same as a short
                acc_read_register(OUT_X_L_A, (unsigned char *) accels, 6);
                // need to read all 6 bytes in one transaction to get an update.
                acc_read_register(OUT_X_L_M, (unsigned char *) mags, 6);
                // read the temperature data. Its a right justified 12 bit two's compliment number
                acc_read_register(TEMP_OUT_L, (unsigned char *) &temp, 2);

                accel_x = accels[0];
                accel_y = accels[1];
                accel_z = accels[2];

                accel_draw_axis(accel_x,accel_y);

                //Display accel values on the screen
                /*display_clear();

                sprintf(buffer,"x: %d y: %d z: %d", accel_x*ACCEL_AXIS_SCALE, accel_y*ACCEL_AXIS_SCALE, accel_z*ACCEL_AXIS_SCALE);
                display_write_string(buffer,5,5);
                display_draw();*/
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


#define DISPLAY_ALLIGNER 0
#define DISPLAY_MAX_X 128
#define DISPLAY_MAX_Y 64
#define DISPLAY_CENTER_X DISPLAY_MAX_X/2
#define DISPLAY_CENTER_Y DISPLAY_MAX_Y/2

void accel_draw_axis(short x, short y)
{
    display_clear();

    short x_draw, y_draw;

    x_draw = x * ACCEL_AXIS_SCALE;
    y_draw = y * ACCEL_AXIS_SCALE;

    display_pixel_set(DISPLAY_CENTER_Y, DISPLAY_CENTER_X,1);

    short i;
    int isXNeg=0,isYNeg=0;

    if(x_draw < 0)
    {
        isXNeg = 1;
        x_draw *= -1;
    }
    if(x_draw > DISPLAY_CENTER_X)
            x_draw = DISPLAY_CENTER_X;

    if(y_draw < 0)
    {
        isYNeg = 1;
        y_draw *= -1;
    }
    if(y_draw > DISPLAY_CENTER_Y)
            y_draw = DISPLAY_CENTER_Y;
    
    for(i = 1; i <= x_draw; i++)
    {
        if(isXNeg)
        {
            display_pixel_set(DISPLAY_CENTER_Y - 2,DISPLAY_CENTER_X - i,1);
            display_pixel_set(DISPLAY_CENTER_Y - 1,DISPLAY_CENTER_X - i,1);
            display_pixel_set(DISPLAY_CENTER_Y,DISPLAY_CENTER_X - i,1);
            display_pixel_set(DISPLAY_CENTER_Y + 1,DISPLAY_CENTER_X - i,1);
            display_pixel_set(DISPLAY_CENTER_Y + 2,DISPLAY_CENTER_X - i,1);
        }
        else
        {
            display_pixel_set(DISPLAY_CENTER_Y - 2,DISPLAY_CENTER_X + i,1);
            display_pixel_set(DISPLAY_CENTER_Y - 1,DISPLAY_CENTER_X + i,1);
            display_pixel_set(DISPLAY_CENTER_Y,DISPLAY_CENTER_X + i,1);
            display_pixel_set(DISPLAY_CENTER_Y + 1,DISPLAY_CENTER_X + i,1);
            display_pixel_set(DISPLAY_CENTER_Y + 2,DISPLAY_CENTER_X + i,1);
        }
    }

    for(i = 1; i <= y_draw; i++)
    {
        if(isYNeg)
        {
            display_pixel_set(DISPLAY_CENTER_Y - i,DISPLAY_CENTER_X - 2,1);
            display_pixel_set(DISPLAY_CENTER_Y - i,DISPLAY_CENTER_X - 1,1);
            display_pixel_set(DISPLAY_CENTER_Y - i,DISPLAY_CENTER_X,1);
            display_pixel_set(DISPLAY_CENTER_Y - i,DISPLAY_CENTER_X + 1,1);
            display_pixel_set(DISPLAY_CENTER_Y - i,DISPLAY_CENTER_X + 2,1);
        }
        else
        {
            display_pixel_set(DISPLAY_CENTER_Y + i,DISPLAY_CENTER_X - 2,1);
            display_pixel_set(DISPLAY_CENTER_Y + i,DISPLAY_CENTER_X - 1,1);
            display_pixel_set(DISPLAY_CENTER_Y + i,DISPLAY_CENTER_X,1);
            display_pixel_set(DISPLAY_CENTER_Y + i,DISPLAY_CENTER_X + 1,1);
            display_pixel_set(DISPLAY_CENTER_Y + i,DISPLAY_CENTER_X + 2,1);
        }
    }

    display_draw();
}