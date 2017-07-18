/*
 * flooding_task.c
 *
 *  Created on: Jun 7, 2017
 *      Author: rsmenon
 *      Description: Behavior for node which sends out sensor data and also participates in flooding
 */

/***** Includes *****/

#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* Board Header files */
#include "Board.h"

#include "mac_task.h"
#include "app_task.h"
#include "RadioProtocol.h"


/***** Defines *****/
#define FLOOD_TASK_STACK_SIZE 1024
#define FLOOD_TASK_PRIORITY   3

#define FLOOD_EVENT_ALL                  0xFFFFFFFF
#define FLOOD_EVENT_SEND_DATA            (uint32_t)(1 << 0)
#define FLOOD_EVENT_FORWARD_PACKET         (uint32_t)(1 << 1)

struct ComboPacket packetSend;
struct ComboPacket packetFlood;

/***** Variable declarations *****/
static Task_Params floodTaskParams;
Task_Struct floodTask;    /* not static so you can see in ROV */
static uint8_t floodTaskStack[FLOOD_TASK_STACK_SIZE];
Event_Struct floodEvent;  /* not static so you can see in ROV */
static Event_Handle floodEventHandle;

/***** Prototypes *****/
static void floodTaskFunction(UArg arg0, UArg arg1);
static bool shouldForward(uint8_t hopCount, uint8_t floodControl);

/***** Function definitions *****/
void floodTask_init(void)
{

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&floodEvent, &eventParam);
    floodEventHandle = Event_handle(&floodEvent);


    /* Create the node task */
    Task_Params_init(&floodTaskParams);
    floodTaskParams.stackSize = FLOOD_TASK_STACK_SIZE;
    floodTaskParams.priority = FLOOD_TASK_PRIORITY;
    floodTaskParams.stack = &floodTaskStack;
    Task_construct(&floodTask, floodTaskFunction, &floodTaskParams, NULL);
}

static void floodTaskFunction(UArg arg0, UArg arg1)
{

    while(1) {
        /* Wait for event */
        uint32_t events = Event_pend(floodEventHandle, 0, FLOOD_EVENT_ALL, BIOS_WAIT_FOREVER);

        if (events & FLOOD_EVENT_SEND_DATA) {
          //fill out empty fields in packet
          packetSend.packet.header.floodControl = 0;
          packetSend.packet.header.hopCount = 2;
          //call mac task
          macTask_sendData(&packetSend);
        }

        if (events & FLOOD_EVENT_FORWARD_PACKET) {
          //check flooding flags
          if (shouldForward(packetFlood.packet.header.hopCount, packetFlood.packet.header.floodControl)) {
            //Modify flood bits in packet
            packetFlood.packet.header.floodControl = (packetFlood.packet.header.floodControl | (1 << NODE_ADDR));
            packetFlood.packet.header.hopCount--;

            //TODO make this into queue
            macTask_forwardPacket(&packetFlood);
          }
        }
    }
}

//helpers

void floodTask_sendData(struct ComboPacket* packet) {
  //TODO Make into a queue

  memcpy(&packetSend, packet, sizeof(struct ComboPacket));
  Event_post(floodEventHandle, FLOOD_EVENT_SEND_DATA);
}

static bool shouldForward(uint8_t hopCount, uint8_t floodControl) {
  //TODO logic based on nodeAddress variable/define
  return ((hopCount > 0) && !((floodControl >> NODE_ADDR) & 0x01));
}

void floodTask_floodPacket(struct ComboPacket* packet, int8_t rssi) { //TODO rethink these params
  //Ask AppTask to update lcd
  appTask_packetReceived(packet, rssi);
  //TODO Make into queue

  memcpy(&packetFlood, packet, sizeof(struct ComboPacket));

  Event_post(floodEventHandle, FLOOD_EVENT_FORWARD_PACKET);
}
