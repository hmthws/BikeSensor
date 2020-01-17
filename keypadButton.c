/* DESCRIPTION
 * Sample code showing keypad's response to pushing buttons 1 and 2
 * Include pressedKey on debugger's "Expressions" to see the hexaKeys' value when you alternate between the two keys
 * Did not include button debouncer in this (releasing the button does not set pressedKey back to Value 0 '\x00')
 */

#define US_TRIG         BIT0
#define US_ECHO         BIT1

#define US_MASK         US_TRIG | US_ECHO

unsigned int up_counter;
unsigned int distance_cm;

#include <msp430.h>
#include "driverlib.h"
#include <timer_b.h>
#include "Board.h"
#include <stdio.h>
#include <stdlib.h>

// Define word access definitions to LCD memories
#define LCDMEMW ((int*)LCDMEM)

// Workaround LCDBMEM definition bug in IAR header file
#ifdef __IAR_SYSTEMS_ICC__
#define LCDBMEMW ((int*)&LCDM32)
#else
#define LCDBMEMW ((int*)LCDBMEM)
#endif

#define pos4 10  /* Digit A4 - L10 */
#define pos5 2   /* Digit A5 - L2  */
#define pos6 18  /* Digit A6 - L18 */


uint8_t lcdKeys[10] = {0x00,0x60,0xDB,0xF3,0x67,0xB7,0XBF,0xE4,0xFF,0xF7};

uint8_t hexaKeys[4][3] = {
  {0x60,0xDB,0xF3},
  {0x67,0xB7,0XBF},
  {0xE4,0xFF,0xF7},
  {0,0xFC,0} //only use the middle one - for pound or asterisk just ignore or call method to concatenate
};

const char digit[10][2] =
{
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};

void handleKey();
void setupLCD();
void setupKeypad();
void setupDistanceSensor();
void setupLED();
void setLED();
void loop();
void showChar(char c, int position);
//void setupButton();

#define TIMER_PERIOD 511
#define MAX_DISTANCE 2352941

int threshold = 50;
float distance = 0; //distance given from sensor
int keyPressed[3] = {0,0,0};
int i = 0;
int randomNum = 0;
uint16_t counter = 0;

void showChar(char c, int position)
{
    if (c == ' ')
    {
        // Display space
        LCDMEMW[position/2] = 0;
    }
    else if (c >= '0' && c <= '9')
    {
        // Display digit
        LCDMEMW[position/2] = digit[c-48][0] | (digit[c-48][1] << 8);
    }
    else
    {
        // Turn all segments on if character is not a space, digit, or uppercase letter
        LCDMEMW[position/2] = 0xFFFF;
    }
}


void main (void)
{
    __disable_interrupt();
    WDT_A_hold(WDT_A_BASE);     // Stop watchdog timer
    PMM_unlockLPM5();
//    P1DIR |= US_TRIG;
//    P1DIR &= ~US_ECHO;// Set P1.0 to output direction
//    P1OUT &= ~US_TRIG;
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);//2.7 from 1.1 is trigger
    GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN2);//8.2 from 1.0 is echo
//
//    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
//    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);


    setupLCD();
    setupKeypad();
    setupLED();
    setupDistanceSensor();

    __enable_interrupt();
    _EINT();
    //PMM_unlockLPM5();
    //__bis_SR_register(LPM4_bits + GIE);


    loop();
}


void loop(){
    Timer_A_initUpModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    param.timerPeriod = TIMER_PERIOD;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = false;
    Timer_A_initUpMode(TIMER_A0_BASE, &param);

    Timer_A_initUpModeParam param2 = {0};
    param2.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param2.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    param2.timerPeriod = TIMER_PERIOD;
    param2.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    param2.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    param2.timerClear = TIMER_A_DO_CLEAR;
    param2.startTimer = false;
    Timer_A_initUpMode(TIMER_A1_BASE, &param2);

    int pulseReceived = 0;

    for (;;){
        handleKey();
        P2OUT |= 0X80; //-x80                // assert P2.7 = TRIGGER
        __delay_cycles(6);                 // 10us wide 1.2 = ECHOP
        P2OUT &= ~0X80;                // deassert

        Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
        pulseReceived = 0;

        while (pulseReceived == 0 && Timer_A_getCounterValue(TIMER_A1_BASE) < MAX_DISTANCE){//&& Timer_A_getCounterValue(TIMER_A1_BASE) < MAX_DISTANCE
            if (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == GPIO_INPUT_PIN_HIGH){

                Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
                for (;;){
                    if (GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN2) == GPIO_INPUT_PIN_LOW){
                        //make sure this number is

                      distance = Timer_A_getCounterValue(TIMER_A0_BASE);
                     // distance = (distance * 340)/200; //speed of light, divide by 2, divide by 100 to get in cm
                      int hex = (int) distance;
                     // distance = distance/100; //divide by clock speed
//                      int num1 = distance/100;
//                      int num2 = (distance - num1)/10;
//                      int num3 = (distance - num1- num2);
                      __delay_cycles(1000000);
                      int d = hex/100;
                      hex = hex - d*100;
                      showChar((char)(d) + '0', pos4);
                      d = hex/10;
                      hex = hex - d *10;
                      showChar((char)(d) + '0', pos5);

                      showChar((char)(hex) + '0', pos6);

                      setLED();

                      param.timerClear = TIMER_A_DO_CLEAR;
                      param.startTimer = false;
                      Timer_A_initUpMode(TIMER_A0_BASE, &param);

                      param2.timerClear = TIMER_A_DO_CLEAR;
                      param2.startTimer = false;
                      Timer_A_initUpMode(TIMER_A1_BASE, &param2);

                      break;
                    }
                }

                pulseReceived = 1;
            }
        }
    }
}


void testKeypad(){

}

void setLED(){
    if (distance < threshold){
        //turn on red led
        GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);

    }

    else {
        //turn on green
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
    }

}


void setupDistanceSensor(){
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);               //trigger
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);

    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN0);
}


void setupLED(){
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);//red
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);//green
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);//yellow
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
}

void setupKeypad(){
    // ROWS ARE OUTPUTS



    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);                  // Row 1: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);                  // Row 2: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);                  // Row 3: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);

    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);                  // Row 4: Output direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);


    // COLUMNS ARE ISR TRIGGERS

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 1: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5);
   // GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 2: Input direction
     GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN7, GPIO_HIGH_TO_LOW_TRANSITION);
     GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN7);
     GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN7);
    // GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN7);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);     // Column 3: Input direction
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN5);
    //GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN5);


}

void handleKey()
{


        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); // Row 1
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 2
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); // Row 3
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 4

        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == GPIO_INPUT_PIN_LOW){
            threshold = 40;
            GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
            __delay_cycles(100000);
            GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
        }

        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_LOW) {
            threshold = 80;
            GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
                        __delay_cycles(100000);
                        GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
        }

        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW) {
            threshold = 120;
            GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
                        __delay_cycles(100000);
                        GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
        }
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); // Row 1
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 2
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); // Row 3- HIGH
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 4- HIGH

        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == GPIO_INPUT_PIN_LOW){
            threshold = 160;
                }

                if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_LOW) {
                    threshold = 200;
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
                                __delay_cycles(100000);
                                GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
                }

                if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW) {
                    threshold = 240;
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
                                __delay_cycles(100000);
                                GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
                }

               GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); // Row 1- LOW
               GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 2- HIGH
               GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); // Row 3- HIGH
               GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 4- HIGH

               if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5) == GPIO_INPUT_PIN_LOW){
                   threshold = 280;
                   GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
                               __delay_cycles(100000);
                               GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
                       }

               if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_LOW) {
                   threshold = 320;
                   GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
                               __delay_cycles(100000);
                               GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
               }

               if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5) == GPIO_INPUT_PIN_LOW) {
                   threshold = 360;
                   GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
                               __delay_cycles(100000);
                               GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
               }

               GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6); // Row 1- LOW
               GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 2- HIGH
               GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3); // Row 3- HIGH
               GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 4- HIGH

            if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN7) == GPIO_INPUT_PIN_LOW) {
                threshold = 400;
                GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN0);
                            __delay_cycles(100000);
                            GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN0);
            }

        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6); // Row 1- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4); // Row 2- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3); // Row 3- LOW
        GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); // Row 4- LOW

}

void setupLCD(){
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_SEGMENT_LINE_26);

    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_36, LCD_E_SEGMENT_LINE_39);

    LCD_E_initParam initParams = LCD_E_INIT_PARAM;
    initParams.clockSource =  LCD_E_CLOCKSOURCE_XTCLK;
    initParams.clockDivider = LCD_E_CLOCKDIVIDER_8;
    initParams.muxRate  = LCD_E_4_MUX;
    initParams.waveforms  = LCD_E_STANDARD_WAVEFORMS;
    initParams.segments  = LCD_E_SEGMENTS_ENABLED;

    //init LCD as 4-mux mode
    LCD_E_init (LCD_E_BASE, &initParams);

    //LCD operation
    LCD_E_setVLCDSource(LCD_E_BASE, LCD_E_INTERNAL_REFERENCE_VOLTAGE, LCD_E_EXTERNAL_SUPPLY_VOLTAGE);
    LCD_E_setVLCDVoltage (LCD_E_BASE, LCD_E_REFERENCE_VOLTAGE_3_08V);

    LCD_E_enableChargePump(LCD_E_BASE);
    LCD_E_setChargePumpFreq (LCD_E_BASE, LCD_E_CHARGEPUMP_FREQ_16);

    LCD_E_clearAllMemory (LCD_E_BASE);

    LCD_E_setPinAsCOM(LCD_E_BASE,LCD_E_SEGMENT_LINE_0, LCD_E_MEMORY_COM0);
    LCD_E_setPinAsCOM(LCD_E_BASE,LCD_E_SEGMENT_LINE_1, LCD_E_MEMORY_COM1);
    LCD_E_setPinAsCOM(LCD_E_BASE,LCD_E_SEGMENT_LINE_2, LCD_E_MEMORY_COM2);
    LCD_E_setPinAsCOM(LCD_E_BASE,LCD_E_SEGMENT_LINE_3, LCD_E_MEMORY_COM3);

    LCD_E_selectDisplayMemory(LCD_E_BASE, LCD_E_DISPLAYSOURCE_MEMORY);

    //initially display 0
    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, 0xFC);
    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, 0xFC);
    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_18, 0xFC);

    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, 0xFC);
    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, 0xFC);
    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_18, 0xFC);

    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, 0xFC);
    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, 0xFC);
    LCD_E_setMemory (LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_18, 0xFC);


    LCD_E_on(LCD_E_BASE);
}

#pragma vector = PORT1_VECTOR     // Using PORT1_VECTOR interrupt because P1.4 and P1.5 are in port 1
__interrupt void PORT1_ISR(void)
{
   // PMM_unlockLPM5();
    //WDT_A_hold(WDT_A_BASE);
 //__disable_interrupt();
    handleKey();

    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN7);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN5);
 //__enable_interrupt();
}
