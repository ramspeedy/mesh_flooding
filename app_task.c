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
#include "app_task.h"

/***** Defines *****/
#define APP_TASK_STACK_SIZE 1024
#define APP_TASK_PRIORITY   3

#define APP_EVENT_ALL                  0xFFFFFFFF
#define APP_EVENT_SEND_DATA    (uint32_t)(1 << 0)
#define APP_EVENT_NEW_DATA     (uint32_t)(1 << 1)
#define APP_EVENT_UPDATE_LCD   (uint32_t)(1 << 2)
#define APP_EVENT_UPDATE_LCD_STAT (uint32_t)(1 << 3)

#define PACKET_INTERVAL_MS 100


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

/* Pin driver handle */
static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;
static PIN_Handle buttonPinHandle1;
static PIN_State buttonPinState1;
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static bool txFlag;

Clock_Struct sensorTimerClock;     /* not static so you can see in ROV */
static Clock_Handle sensorTimerClockHandle;

/******** PACKET STATS *********/
static uint16_t latestAckSeqNo;
static uint16_t counter = 0;
static uint16_t rxPacketCount = 0;
 uint16_t txPacketCount = 0;
static uint16_t rxAckCount = 0;
static uint16_t seqNoArray[NUM_NODES] = {0};
static uint16_t routeArray[NUM_NODES][32] = {0};

struct nodeStat statArray[NUM_NODES];

//uint32_t latency_start = 0;
//uint32_t latency_stop = 0;


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
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

PIN_Config buttonPinTable1[] = {
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/***** Prototypes *****/
static void appTaskFunction(UArg arg0, UArg arg1);
void buttonCallback(PIN_Handle handle, PIN_Id pinId);
void buttonCallback1(PIN_Handle handle, PIN_Id pinId);
void sensorTimerCallback(UArg arg0);
static void updateLcd(void);
static void updateLcd2(void);
static void updateLcd3(void);
static void top8(uint16_t* arr, uint8_t len, uint8_t* indexArray);


/***** Function definitions *****/
void appTask_init(void)
{
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle)
    {
        System_abort("Error initializing board 3.3V domain pins\n");
    }

    latestAckSeqNo = 0;
    int i;
    for (i = 0; i < NUM_NODES; i++) {
        statArray[i].rx = 0;
        statArray[i].tx = 0;
    }
    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&appEvent, &eventParam);
    appEventHandle = Event_handle(&appEvent);

    Clock_Params clkParams;
    clkParams.period = 0;
    clkParams.startFlag = FALSE;
    Clock_construct(&sensorTimerClock, sensorTimerCallback, 1, &clkParams);
    sensorTimerClockHandle = Clock_handle(&sensorTimerClock);

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
        Display_print0(hDisplayLcd, 0, 0, "Startup");
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

    buttonPinHandle1 = PIN_open(&buttonPinState1, buttonPinTable1);
    if (!buttonPinHandle1)
    {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle1, &buttonCallback) != 0)
    {
        System_abort("Error registering button callback function");
    }


    Clock_setPeriod(sensorTimerClockHandle, (PACKET_INTERVAL_MS * 1000 / Clock_tickPeriod));
//    Clock_start(sensorTimerClockHandle);


    tempNewPacket.destAddress = 0;
    tempNewPacket.packet.header.sourceAddress = NODE_ADDR;
    tempNewPacket.packet.header.packetType = PacketType_Data;

    static uint8_t state_LCD = 0;

    while(1) {
        uint32_t events = Event_pend(appEventHandle, 0, APP_EVENT_ALL, BIOS_WAIT_FOREVER);

        if(events & APP_EVENT_SEND_DATA) {
            static uint8_t destAddr = 0;
//                if (txFlag = 1) {
                    txFlag = 0;
                    counter++;
                    destAddr++;

                    destAddr = (destAddr == NUM_NODES) ? 0 : destAddr;
                    destAddr = (destAddr == NODE_ADDR) ? ((destAddr == NUM_NODES-1) ? 0 : destAddr+1) : destAddr;

//                    if ((counter%4) == 0) {
//                        destAddr = 2;
//                    }
//                    else {
//                        destAddr = 0;
//                    }

//                    destAddr = 1;

                    tempNewPacket.destAddress = destAddr;
                    tempNewPacket.packet.dataPacket.seqNo = counter;
                    tempNewPacket.packet.dataPacket.length = PACKET_LENGTH;
                    floodTask_sendData(&tempNewPacket);
//                }
        }

        if (events & APP_EVENT_UPDATE_LCD) {
            switch (state_LCD) {
                case 0:
                    updateLcd();
                    state_LCD++;
                    break;
                case 1:
                    updateLcd2();
                    state_LCD++;
                    break;
                case 2:
                case 3:
                case 4:
                case 5:
                    updateLcd3();
                    state_LCD == 5 ? state_LCD = 0 : state_LCD++;
                    break;
            }
        }

    }
}

void appTask_packetReceived(struct ComboPacket* packet, int8_t rssi)
{
        /* Save the values */
        memcpy(&latestPacket, packet, sizeof(struct ComboPacket));
        latestRssi = rssi;

        uint8_t sourceAddress = latestPacket.packet.header.sourceAddress;

        //duplicates

        if (latestPacket.packet.dataPacket.seqNo <= seqNoArray[sourceAddress]) {
            statArray[sourceAddress].duplicate++;
        }

        //Route stats
        routeArray[sourceAddress][latestPacket.packet.header.floodControl]++;

        //PDR stats
        statArray[sourceAddress].rx++;
        rxPacketCount++;
}

void appTask_ackReceived(uint16_t seqNo)
{
    latestAckSeqNo = seqNo;
    rxAckCount++;
    if(latestAckSeqNo == counter) {
        txFlag = 1;
    }
}

void appTask_sendFail()
{
    txFlag = 1;
}

/* Pin interrupt Callback function board buttons configured in the pinTable. */
void buttonCallback(PIN_Handle handle, PIN_Id pinId)
{
    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);


    if (PIN_getInputValue(Board_BUTTON0) == 0)
    {
        static bool state_radio = 0;
        if (state_radio) {
            state_radio = 0;
            Clock_stop(sensorTimerClockHandle);
        }
        else {
            state_radio = 1;
            Clock_start(sensorTimerClockHandle);
        }
//        Event_post(appEventHandle, APP_EVENT_SEND_DATA);
    }
    if (PIN_getInputValue(Board_BUTTON1) == 0)
    {
        Event_post(appEventHandle, APP_EVENT_UPDATE_LCD);
    }
}

void sensorTimerCallback(UArg arg0)
{
    Event_post(appEventHandle, APP_EVENT_SEND_DATA);
    return;
    static uint8_t destAddr = 0;
    if (txFlag) {
        txFlag = 0;
        counter++;
        destAddr++;

        destAddr = (destAddr == NUM_NODES) ? 0 : destAddr;
        destAddr = (destAddr == NODE_ADDR) ? ((destAddr == NUM_NODES-1) ? 0 : destAddr+1) : destAddr;
//        destAddr = 0;

        tempNewPacket.destAddress = destAddr;
        tempNewPacket.packet.dataPacket.seqNo = counter;
        floodTask_sendData(&tempNewPacket);
    }
    Event_post(appEventHandle, APP_EVENT_UPDATE_LCD_STAT);
}

static void updateLcd(void) {

    Display_clear(hDisplayLcd);
    Display_print0(hDisplayLcd, 0, 0, "SAddr DAddr");
    Display_print2(hDisplayLcd, 1, 0, "0x%02x  0x%02x", latestPacket.packet.header.sourceAddress, latestPacket.destAddress);
    Display_print0(hDisplayLcd, 2, 0, "SeqNo AckSeqNo");
    Display_print2(hDisplayLcd, 3, 0, "%d   %d", latestPacket.packet.dataPacket.seqNo, latestAckSeqNo);
    Display_print1(hDisplayLcd, 4, 0, "Pkts Rx'd %d", rxPacketCount);
    Display_print1(hDisplayLcd, 5, 0, "Pkts Tx'd %d", txPacketCount);
    Display_print1(hDisplayLcd, 6, 0, "Ack Rx'd %d", rxAckCount);
}

static void updateLcd2(void) {

//    Display_clear(hDisplayLcd);
    Display_print0(hDisplayLcd, 0, 0, "A Tx Rx");
    uint8_t i = 0;
    for (i = 0; i < NUM_NODES; i++) {
        Display_print3(hDisplayLcd, i+1, 0, "%d %d %d", i, statArray[i].tx, statArray[i].rx);
    }
    for (i = 0; i < NUM_NODES; i++) {
        Display_print3(hDisplayLcd, i+1+NUM_NODES, 0, "%d %d %d", i, statArray[i].reTX, statArray[i].duplicate);
    }

}

static void updateLcd3(void) {
        static uint8_t sourceAddr = 0;
        sourceAddr = (sourceAddr == NUM_NODES) ? 0 : sourceAddr;
        sourceAddr = (sourceAddr == NODE_ADDR) ? ((sourceAddr == NUM_NODES-1) ? 0 : sourceAddr+1) : sourceAddr;

        Display_clear(hDisplayLcd);
        Display_print1(hDisplayLcd, 0, 0, "SourceAddr %d", sourceAddr);
        uint8_t i = 0;
        uint8_t indexArr[8] = {0};
        top8(routeArray[sourceAddr], 32, indexArr);
        for (i = 0; i < 8; i++) {
            Display_print2(hDisplayLcd, i+1, 0, "%d %d", indexArr[i], routeArray[sourceAddr][indexArr[i]]);
        }
        sourceAddr++;
}

static void top8(uint16_t* arr, uint8_t len, uint8_t* indexArray) {
    uint8_t i;

    for(i = 0; i < len; i++) {
        if(arr[i] > arr[indexArray[0]]) {
            indexArray[7] = indexArray[6];
            indexArray[6] = indexArray[5];
            indexArray[5] = indexArray[4];
            indexArray[4] = indexArray[3];
            indexArray[3] = indexArray[2];
            indexArray[2] = indexArray[1];
            indexArray[1] = indexArray[0];
            indexArray[0] = i;
        }
        else if (arr[i] > arr[indexArray[1]]) {
            indexArray[7] = indexArray[6];
            indexArray[6] = indexArray[5];
            indexArray[5] = indexArray[4];
            indexArray[4] = indexArray[3];
            indexArray[3] = indexArray[2];
            indexArray[2] = indexArray[1];
            indexArray[1] = i;
        }
        else if (arr[i] > arr[indexArray[2]]) {
            indexArray[7] = indexArray[6];
            indexArray[6] = indexArray[5];
            indexArray[5] = indexArray[4];
            indexArray[4] = indexArray[3];
            indexArray[3] = indexArray[2];
            indexArray[2] = i;
        }
        else if (arr[i] > arr[indexArray[3]]) {
            indexArray[7] = indexArray[6];
            indexArray[6] = indexArray[5];
            indexArray[5] = indexArray[4];
            indexArray[4] = indexArray[3];
            indexArray[3] = i;
        }
        else if (arr[i] > arr[indexArray[4]]) {
            indexArray[7] = indexArray[6];
            indexArray[6] = indexArray[5];
            indexArray[5] = indexArray[4];
            indexArray[4] = i;
        }
        else if (arr[i] > arr[indexArray[5]]) {
            indexArray[7] = indexArray[6];
            indexArray[6] = indexArray[5];
            indexArray[5] = i;
        }
        else if (arr[i] > arr[indexArray[6]]) {
            indexArray[7] = indexArray[6];
            indexArray[6] = i;
        }
        else if (arr[i] > arr[indexArray[7]]) {
            indexArray[7] = i;
        }
    }
}
