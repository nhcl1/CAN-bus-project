/*
 * CAN bus DC motor controller master firmware
 * Written for TI Tiva TM4C123GH6PM
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"

#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"

#include "utils/uartstdio.h"

uint32_t ui32ADC0Value[1];  //to store the value ADC sampled value

volatile bool errFlag = 0;  //transmission error flag
unsigned int sysClock;      //clockspeed in Hz

void delay(unsigned int milliseconds) {
    SysCtlDelay((sysClock / 3) * (milliseconds / 1000.0f));
}

/*
 * CAN interrupt handler
 */
void CANIntHandler(void) {

    unsigned long status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);  //reading interrupt status

    if(status == CAN_INT_INTID_STATUS) {    //controller status interrupt
        status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        errFlag = 1;
    } else if(status == 1) {
        CANIntClear(CAN0_BASE, 1); //interrupt clear
        errFlag = 0;
    } else {
        UARTprintf("Unexpected CAN bus interrupt\n");
    }
}

int main(void) {

    tCANMsgObject msg;  //CAN message object
    unsigned int msgData;   //buffer to store the ADC value

    unsigned char *msgDataPtr = (unsigned char *)&msgData; //pointer to buffer

    // System clock @ 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    sysClock = SysCtlClockGet();

    // Enable Peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);     //enable PWM1 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);     //enable ADC0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);    //enable GPIO for ADC0 and CAN0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    //enable GPIO for UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);    //enable UART0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);     //enable CAN0 module

    // Configure PWM0
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);            //PWM clock divider
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);    //configure PD0 as PWM output
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);   //PWM mode set as "counting down"
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 4095);    //set the period for the PWM generator
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true); //enable the PWM output
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);             //enable PWM generator

    // Configure ADC0
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);  //ADC sequencer 0, processor triggered
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);        //ADC0 module, sequencer 0 , 1 sampling, input is from channel 0 PE3
    ADCSequenceEnable(ADC0_BASE, 1);                               //enable the sequence 1

    // Set up debugging UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, SysCtlClockGet());           //115200 baud rate for UART

    // Set up CAN0
    GPIOPinConfigure(GPIO_PE4_CAN0RX);
    GPIOPinConfigure(GPIO_PE5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    CANInit(CAN0_BASE);                                     //CAN0 controller initialized
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000);    //set CAN bit rate
    CANIntRegister(CAN0_BASE, CANIntHandler);               //use dynamic vector table allocation
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);   //enable interrupt sources
    IntEnable(INT_CAN0);    //enables the CAN0 interrupt
    CANEnable(CAN0_BASE);   //enable the CAN0 controller

    // Set up msg object
    msgData = 0;
    msg.ui32MsgID = 1;
    msg.ui32MsgIDMask = 0;
    msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    msg.ui32MsgLen = sizeof(msgDataPtr);
    msg.pui8MsgData = msgDataPtr;

    unsigned int t = 0; //counter

    while(1) {
        ADCIntClear(ADC0_BASE, 1);      //interrupt clear
        ADCProcessorTrigger(ADC0_BASE, 1);
        ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);    //store ADC value in ui32ADC0Value
        uint8_t* ptr = ui32ADC0Value;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32ADC0Value[0]);   //use ADC value as PWM width


        uint8_t byte3 = *ptr ;      //pointer to LSB
        uint8_t byte2 = *(ptr+1);
        uint8_t byte1 = *(ptr+2);
        uint8_t byte0 = *(ptr+3);   //pointer to MSB

        printf("%i%i%i%i", byte3, byte2, byte1, byte0);

        delay(100);
        msgDataPtr[0] = byte3;
        msgDataPtr[1] = byte2;
        msgDataPtr[2] = byte1;
        msgDataPtr[3] = byte0;

        int message = byte3;
        message += (byte2 << 8);
        message += (byte1 << 16);
        message += (byte0 << 24);

        UARTprintf("Sending ADC0 value:\t %i\n", message);

        CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_TX); //message loaded into CAN peripheral message object 1 so it can trigger interrupts on any matched rx messages

        delay(100);
        if (errFlag)
        {
            UARTprintf("CANBUS error\n");
        }
        t++;
    }

    return 0;
}
