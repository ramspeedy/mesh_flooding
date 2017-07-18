/*
 * app_task.c
 *
 *  Created on: Jun 8, 2017
 *      Author: rsmenon
 */

/***** Includes *****/

#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>

#include <ti/mw/display/Display.h>

/* Board Header files */
#include "Board.h"

#include "RadioProtocol.h"
#include "flooding_task.h"
#include "mac_task.h"

/***** Defines *****/
#define APP_TASK_STACK_SIZE 1024
#define APP_TASK_PRIORITY   3

#define APP_EVENT_ALL                  0xFFFFFFFF
#define APP_EVENT_SEND_DATA    (uint32_t)(1 << 0)
#define APP_EVENT_NEW_DATA     (uint32_t)(1 << 1)
#define APP_EVENT_RESEND       (uint32_t)(1 << 2)


/***** Variable declarations *****/
static Task_Params appTaskParams;
Task_Struct appTask;    /* not static so you can see in ROV */
static uint8_t appTaskStack[APP_TASK_STACK_SIZE];
Event_Struct appEvent;  /* not static so you can see in ROV */
static Event_Handle appEventHandle;

static Display_Handle hDisplayLcd;

static struct ComboPacket latestPacket;
static struct ComboPacket tempNewPacket;
static int8_t latestRssi;
static uint16_t latestAckSeqNo;

static uint16_t rxPacketCount = 0;
static uint16_t counter = 0;
/* Pin driver handle */
static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static bool txFlag;

/* Enable the 3.3V power domain used by the LCD */
PIN_Config pinTable[] = {
    PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Prototypes *****/
static void appTaskFunction(UArg arg0, UArg arg1);
void buttonCallback(PIN_Handle handle, PIN_Id pinId);
static void updateLcd(void);


/***** Function definitions *****/
void appTask_init(void)
{
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    txFlag = 0;
    latestAckSeqNo = 0;
    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&appEvent, &eventParam);
    appEventHandle = Event_handle(&appEvent);


    /* Create the node task */
    Task_Params_init(&appTaskParams);
    appTaskParams.stackSize = APP_TASK_STACK_SIZE;
    appTaskParams.priority = APP_TASK_PRIORITY;
    appTaskParams.stack = &appTaskStack;
    Task_construct(&appTask, appTaskFunction, &appTaskParams, NULL);
}

static void appTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize display and try to open LCD display. */
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;
    /* To display on the Watch DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.*/
    hDisplayLcd = Display_open(Display_Type_LCD, &params);
    /* Check if the selected Display type was found and successfully opened */
    if (hDisplayLcd)
    {
        Display_print0(hDisplayLcd, 0, 0, "Waiting for pkt");
    }


    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if (!buttonPinHandle)
    {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallback) != 0)
    {
        System_abort("Error registering button callback function");
    }


    while(1) {
        uint32_t events = Event_pend(appEventHandle, 0, APP_EVENT_ALL, BIOS_WAIT_FOREVER);

        if(events & APP_EVENT_SEND_DATA) {
          counter += 1;
          tempNewPacket.destAddress = (NODE_ADDR) ? 0x00 : 0x01;
          tempNewPacket.packet.header.sourceAddress = NODE_ADDR;
          tempNewPacket.packet.header.packetType = PacketType_Data;
          tempNewPacket.packet.dataPacket.seqNo = counter;

          if (counter >= 100) {
              txFlag = 0;
          }
          floodTask_sendData(&tempNewPacket);
        }
        if(events & APP_EVENT_RESEND) {
            floodTask_sendData(&tempNewPacket);
        }

        if(events & APP_EVENT_NEW_DATA) {
          //update display
          updateLcd();

          //TODO update relevant packet statistics
        }
    }
}

void appTask_packetReceived(struct ComboPacket* packet, int8_t rssi)
{
        /* Save the values */
        memcpy(&latestPacket, packet, sizeof(struct ComboPacket));
        latestRssi = rssi;

        if(latestPacket.destAddress == NODE_ADDR) {
            rxPacketCount++;
        }
        Event_post(appEventHandle, APP_EVENT_NEW_DATA);
}

void appTask_ackReceived(uint16_t seqNo)
{
    latestAckSeqNo = seqNo;
    if (txFlag) {
        Event_post(appEventHandle, APP_EVENT_SEND_DATA);
    }

        Event_post(appEventHandle, APP_EVENT_NEW_DATA);
}

void appTask_sendFail() {
    if (txFlag) {
        Event_post(appEventHandle, APP_EVENT_RESEND);
    }
}

/* Pin interrupt Callback function board buttons configured in the pinTable. */
void buttonCallback(PIN_Handle handle, PIN_Id pinId)
{
    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);


    if (PIN_getInputValue(Board_BUTTON0) == 0)
    {
        txFlag = ~txFlag;
        Event_post(appEventHandle, APP_EVENT_SEND_DATA);
    }
}

static void updateLcd(void) {

    Display_clear(hDisplayLcd);
    Display_print0(hDisplayLcd, 0, 0, "SAddr DAddr");

    Display_print2(hDisplayLcd, 1, 0, "0x%02x  0x%02x",
                latestPacket.packet.header.sourceAddress, latestPacket.destAddress);
    Display_print0(hDisplayLcd, 2, 0, "SeqNo RSSI");
    Display_print2(hDisplayLcd, 3, 0, "%d   %04d",
                    latestPacket.packet.dataPacket.seqNo, latestRssi);
    Display_print1(hDisplayLcd, 4, 0, "Pkts Rx'd %d", rxPacketCount);
    Display_print1(hDisplayLcd, 5, 0, "Ack Rx'd %d", latestAckSeqNo);
}

