/*
 * CAN bus DC motor controller slave device firmware
 * Written for TI Tiva TM4C1294NCPDT
 */

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"

#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#include "utils/uartstdio.h"

//FIFO Buffer for handling incoming data
#define SIZE_OF_BUFFER 32

volatile uint32_t arrayBuffer[SIZE_OF_BUFFER]; //array for FIFO
volatile uint32_t rxMsgCount;                  //item count for FIFO
volatile uint32_t head;     //head for FIFO
volatile uint32_t tail;     //tail for FIFO
volatile bool rxFlag = 0;   //msg received flag
volatile bool errFlag = 0;  //error flag
unsigned int sysClock;      //clockspeed in Hz

void delay(unsigned int milliseconds) {
    SysCtlDelay((sysClock / 3) * (milliseconds / 1000.0f));
}

/*
 * CAN interrupt handler
 */
void CANIntHandler(void) {
    uint32_t  i;
    tCANMsgObject messgeRx;

    unsigned long status = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE); //reading interrupt status

    if(status == CAN_INT_INTID_STATUS) {    // controller status interrupt
        status = CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);
        errFlag = 1;
    }
    else if((status > 0) & (status < 5)) {
        for(i = status; i < 5; i++) {
            messgeRx.ui32Flags = MSG_OBJ_NO_FLAGS;
            messgeRx.pui8MsgData = (void *)&(arrayBuffer[head]);
            CANMessageGet(CAN1_BASE, i, &messgeRx, true);

            if((messgeRx.ui32Flags & MSG_OBJ_NEW_DATA) == MSG_OBJ_NEW_DATA) {
                head++;
                head = head % SIZE_OF_BUFFER;
                rxMsgCount++;
            }
            else {
                break;
            }
        }
        UARTprintf("Received ADC0 value:\t %i\n", arrayBuffer[head]);
    }

   /* else if(status == 1) {
        CANIntClear(CAN1_BASE, 1); // clear interrupt
        rxFlag = 1;     // set rx flag
        errFlag = 0;    // clear  error flags*/
     else {
        UARTprintf("Unexpected CAN bus interrupt\n");
    }
}

int main(void) {

   tCANMsgObject msg;   //CAN message object

    sysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    //enable UART0 GPIO peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, sysClock); // 115200 baud

    // CAN1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    //enable CAN1 GPIO peripheral
    GPIOPinConfigure(GPIO_PB0_CAN1RX);
    GPIOPinConfigure(GPIO_PB1_CAN1TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
    CANInit(CAN1_BASE);                             //initialize CAN1 module
    CANBitRateSet(CAN1_BASE, sysClock, 1000000);
    CANIntRegister(CAN1_BASE, CANIntHandler);       //use dynamic vector table allocation
    CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS); //enable CAN interrupt
    IntEnable(INT_CAN1);
    CANEnable(CAN1_BASE); //CAN controller enabled

    // Set up msg object
    msg.ui32MsgID = 0;
    msg.ui32MsgIDMask = 0;
    msg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_FIFO | MSG_OBJ_USE_ID_FILTER;
    //msg.ui32MsgLen = 8;

    // 4 message objects are loaded since the receiving message is 32bit long and trigger interrupts on any matched received messages
    CANMessageSet(CAN1_BASE, 1, &msg, MSG_OBJ_TYPE_RX);
    CANMessageSet(CAN1_BASE, 2, &msg, MSG_OBJ_TYPE_RX);
    CANMessageSet(CAN1_BASE, 3, &msg, MSG_OBJ_TYPE_RX);

    msg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;

    CANMessageSet(CAN1_BASE, 4, &msg, MSG_OBJ_TYPE_RX);

    while(1) {

        }

    return 0;
}

