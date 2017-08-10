/*
 * mac_task.c
 *
 *  Created on: Jun 7, 2017
 *      Author: rsmenon
 *      Description: RTOS task which controls radio rx/tx and implements CSMA-CA
 *      Will eventually handle the ACKs as well. Basically the MAC layer.
 */


/***** Includes *****/
#include <xdc/std.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

#include <driverlib/trng.h>

/* Board Header files */
#include "Board.h"

#include <stdlib.h>
#include "easylink/EasyLink.h"

#include "mac_task.h"
#include "flooding_task.h"
#include "app_task.h"

/* Defines */
#define MAC_TASK_STACK_SIZE 1024
#define MAC_TASK_PRIORITY   3

#define RADIO_EVENT_ALL                          0xFFFFFFFF
#define RADIO_EVENT_SEND_DATA                    (uint32_t)(1 << 0)
#define RADIO_EVENT_FORWARD_PACKET               (uint32_t)(1 << 1)
#define RADIO_EVENT_OTHER_PACKET_RECEIVED        (uint32_t)(1 << 2)
#define RADIO_EVENT_DATA_ACK_RECEIVED            (uint32_t)(1 << 3)
#define RADIO_EVENT_VALID_PACKET_RECEIVED        (uint32_t)(1 << 4)
#define RADIO_EVENT_ACK_TIMEOUT                  (uint32_t)(1 << 5)

#define RADIO_MAX_RETRIES 0
#define RADIO_ACK_TIMEOUT_TIME_MS 10
#define NO_TIMEOUT 0

#define RSSI_THRESHOLD -55
#define RSSI_TIMEOUT_MS 15


/***** Type declarations *****/
enum opType {
    sending,
    forwarding,
    receiving,
};

struct RadioOperation {
    EasyLink_TxPacket easyLinkTxPacket;
    uint8_t retriesDone;
    uint8_t maxNumberOfRetries;
    uint32_t ackTimeoutMs;
    enum opType operation;
    enum NodeRadioOperationStatus result;
};


/* Variables */
static Task_Params macTaskParams;
Task_Struct macTask;        /* not static so you can see in ROV */
static uint8_t macTaskStack[MAC_TASK_STACK_SIZE];
Semaphore_Struct radioAccessSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioAccessSemHandle;
Event_Struct radioOperationEvent; /* not static so you can see in ROV */
static Event_Handle radioOperationEventHandle;
Semaphore_Struct radioResultSem;  /* not static so you can see in ROV */
static Semaphore_Handle radioResultSemHandle;

static struct RadioOperation currentRadioOperation;
static EasyLink_TxPacket easyLinkAckTxPacket;

static struct ComboPacket latestRxPacket;
static struct ComboPacket latestTxPacket;
static uint16_t latestAckRxSeqNo;

static uint32_t prevTicks = 0;

static int8_t last_rssi = RSSI_THRESHOLD;
static uint32_t rssi_timestamp = 0;

static bool sleeping;
static EasyLink_Status easystat;
/* Pin driver handle */
extern PIN_Handle ledPinHandle;


/***** Prototypes *****/
static void macTaskFunction(UArg arg0, UArg arg1);
static void returnRadioOperationStatus(enum NodeRadioOperationStatus status);
static void sendDataPacket(struct DataPacket dpacket, uint8_t destAddress);
static void sendAckPacket(uint8_t destAddress, uint16_t seqNo);
static void forwardPacket(union GenericPacket gPacket, uint8_t destAddress);
static void csma();
static void asyncrx(uint32_t timeout);
static void resendPacket();
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);


/* Init Function */
void macTask_init(void) {

    /* Initialize Local Variables */
    currentRadioOperation.maxNumberOfRetries = 0;
    currentRadioOperation.ackTimeoutMs = NO_TIMEOUT;
    currentRadioOperation.retriesDone = 0;
    currentRadioOperation.operation = receiving;
    currentRadioOperation.result = NodeRadioStatus_Failed;

    /* Create semaphore used for exclusive radio access */
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    Semaphore_construct(&radioAccessSem, 1, &semParam);
    radioAccessSemHandle = Semaphore_handle(&radioAccessSem);

    /* Create semaphore used for callers to wait for result */
    Semaphore_construct(&radioResultSem, 0, &semParam);
    radioResultSemHandle = Semaphore_handle(&radioResultSem);

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&radioOperationEvent, &eventParam);
    radioOperationEventHandle = Event_handle(&radioOperationEvent);

    /* Create the radio protocol task */
    Task_Params_init(&macTaskParams);
    macTaskParams.stackSize = MAC_TASK_STACK_SIZE;
    macTaskParams.priority = MAC_TASK_PRIORITY;
    macTaskParams.stack = &macTaskStack;
    Task_construct(&macTask, macTaskFunction, &macTaskParams, NULL);

}


/* Task Function */
static void macTaskFunction(UArg arg0, UArg arg1)
{
    /* Initialize EasyLink: should refer to smartrf settings */
    if(EasyLink_init(EasyLink_Phy_2_4_200kbps2gfsk) != EasyLink_Status_Success) {
        System_abort("EasyLink_init failed");
    }

    uint32_t freq = EasyLink_getFrequency();
    int8_t power = EasyLink_getRfPwr();
    EasyLink_setRfPwr(-10);
    asyncrx(NO_TIMEOUT);

    /* Enter main task loop */
    while (1)
    {
        // Things that can happen
        // Receive Our Data Packet, Receive Our Ack, Receive stranger Data Packet, Receive stranger Ack
        // Send out a Data Packet, Send out an Ack, Reroute a Data Packet (Flooding), Reroute an Ack

        /* Wait for an event */
        uint32_t events = Event_pend(radioOperationEventHandle, 0, RADIO_EVENT_ALL, BIOS_WAIT_FOREVER);

        if (events & RADIO_EVENT_SEND_DATA) {
            sendDataPacket(latestTxPacket.packet.dataPacket, latestTxPacket.destAddress);
        }

        if (events & RADIO_EVENT_FORWARD_PACKET) {
            forwardPacket(latestTxPacket.packet, latestTxPacket.destAddress);
        }
        /* Packet already passed up to flooding so just go into rx mode */
        if (events & RADIO_EVENT_OTHER_PACKET_RECEIVED) {
          asyncrx(currentRadioOperation.ackTimeoutMs);
        }

        /* If we get an ACK intended for us */
        if (events & RADIO_EVENT_DATA_ACK_RECEIVED)
        {
            //if we were expecting ack with this seqNo
            if ((currentRadioOperation.operation == sending) && (latestAckRxSeqNo == latestTxPacket.packet.dataPacket.seqNo)) {
                returnRadioOperationStatus(NodeRadioStatus_Success);

            }

            appTask_ackReceived(latestAckRxSeqNo);
            asyncrx(currentRadioOperation.ackTimeoutMs);
        }


        if (events & RADIO_EVENT_VALID_PACKET_RECEIVED) {
            //Send Rx'd Data packet up to App Task
            appTask_packetReceived(&latestRxPacket, last_rssi);
            sendAckPacket(latestRxPacket.packet.header.sourceAddress, latestRxPacket.packet.dataPacket.seqNo);
        }

        if (events & RADIO_EVENT_ACK_TIMEOUT) {
            //if not in Radio Operation
//            if (Semaphore_getCount(radioResultSemHandle) != 0) {
            if (currentRadioOperation.operation != sending) {
                asyncrx(currentRadioOperation.ackTimeoutMs);
            }
            // If we haven't resent it the maximum number of times yet, then resend packet
            else if (currentRadioOperation.retriesDone < currentRadioOperation.maxNumberOfRetries)
            {
                resendPacket();
            }
            else
            {
                /* Else return send fail */
                returnRadioOperationStatus(NodeRadioStatus_Failed);
                asyncrx(currentRadioOperation.ackTimeoutMs);
            }
        }
    }
}

/* Helper Functions */
uint32_t get_RNG(uint8_t numBits) {
    uint32_t returnValue;

    Power_setDependency(PowerCC26XX_PERIPH_TRNG);
    TRNGEnable();
    /* Do not accept the same address as the concentrator, in that case get a new random value */
    while (!(TRNGStatusGet() & TRNG_NUMBER_READY))
    {
        //wait for random number generator
    }
    returnValue = TRNGNumberGet(TRNG_LOW_WORD);
    TRNGDisable();
    Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);

    returnValue = returnValue >> (32-numBits);
    return returnValue;

}

//Checks RSSI and Timestamp to see whether we should wait arbitrary time before tx or not
static void csma() {
  //RSSI check
  uint32_t currentTicks;
  uint32_t timeSinceRssi;
  static uint8_t expBackoff = 0;

  currentTicks = Clock_getTicks(); //Ticks are 10microsecond intervals

  //check for wrap around
  if (currentTicks > rssi_timestamp)
  {
      //calculate time since last reading in 1ms units
      timeSinceRssi = ((currentTicks - rssi_timestamp) * Clock_tickPeriod) / 1000;
  }
  else
  {
      //calculate time since last reading in 1ms units
      timeSinceRssi = ((prevTicks - rssi_timestamp) * Clock_tickPeriod) / 1000;
  }

  if (last_rssi > RSSI_THRESHOLD && timeSinceRssi < RSSI_TIMEOUT_MS) {
      sleeping = 1;
      Task_sleep(get_RNG(expBackoff)*1000/Clock_tickPeriod); //Sleep for arbitrary amount
      (expBackoff >= 3) ? expBackoff = 0 : expBackoff++;
      sleeping = 0;
  }
  else {
      expBackoff = 1;
  }
}


//Hook to other tasks to send regular data packet
enum NodeRadioOperationStatus macTask_sendData(struct ComboPacket* packet)
{
//    System_printf("Send %d\n", Clock_getTicks());
//    System_flush();
    enum NodeRadioOperationStatus status;

    /* Get radio access semaphore */
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);
    currentRadioOperation.operation = sending;
    /* Save data to send */
    memcpy(&latestTxPacket, packet, sizeof(struct ComboPacket));

    /* Raise RADIO_EVENT_SEND_DATA event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_SEND_DATA);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);

    /* Get result */
    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle);

    return status;
}

//Hook to other tasks to forward a data packet
void macTask_forwardPacket(struct ComboPacket* packet)
{
//    enum NodeRadioOperationStatus status;

    /* Get radio access sempahore */
    Semaphore_pend(radioAccessSemHandle, BIOS_WAIT_FOREVER);
    currentRadioOperation.operation = forwarding;

    /* Save data to send */
    memcpy(&latestTxPacket, packet, sizeof(struct ComboPacket));

    /* Raise RADIO_EVENT_FORWARD_PACKET event */
    Event_post(radioOperationEventHandle, RADIO_EVENT_FORWARD_PACKET);

    /* Wait for result */
    Semaphore_pend(radioResultSemHandle, BIOS_WAIT_FOREVER);

    /* Get result */
//    status = currentRadioOperation.result;

    /* Return radio access semaphore */
    Semaphore_post(radioAccessSemHandle);

    return;
}

//Signals to other tasks that the radio operation is over and also releases resources
static void returnRadioOperationStatus(enum NodeRadioOperationStatus result)
{
    /* Save result */
    currentRadioOperation.result = result;
    currentRadioOperation.ackTimeoutMs = NO_TIMEOUT;
    currentRadioOperation.operation = receiving;
    /* Post result semaphore */
    Semaphore_post(radioResultSemHandle);
}

//Internal send command for Data Packets
static void sendDataPacket(struct DataPacket dPacket, uint8_t destAddress)
{
//    System_printf("sendDataPacket\n");
//    System_flush();


    /* Set destination address in EasyLink API */
    currentRadioOperation.easyLinkTxPacket.dstAddr[0] = destAddress;

    /* Copy data packet to payload
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    memcpy(currentRadioOperation.easyLinkTxPacket.payload, ((uint8_t*)&dPacket), sizeof(struct DataPacket));
    currentRadioOperation.easyLinkTxPacket.len = sizeof(struct DataPacket);

    /* Setup retries */
    currentRadioOperation.maxNumberOfRetries = RADIO_MAX_RETRIES;
    currentRadioOperation.ackTimeoutMs = RADIO_ACK_TIMEOUT_TIME_MS;
    currentRadioOperation.retriesDone = 0;

    /* Abort any previously running rx*/
    if (EasyLink_abort() != EasyLink_Status_Success) {
        System_abort("EasyLink_abort failed");
    }

    csma();
    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }

    /* Enter RX */
    asyncrx(currentRadioOperation.ackTimeoutMs);
}

static void sendAckPacket(uint8_t destAddress, uint16_t seqNo) {
//    System_printf("sendAckPacket\n");
//    System_flush();

    struct AckPacket ackPacket;
    ackPacket.header.floodControl = 0;
    ackPacket.header.hopCount = FLOOD_HOP_COUNT;
    ackPacket.header.packetType = PacketType_Ack;
    ackPacket.header.sourceAddress = NODE_ADDR;
    ackPacket.seqNo = seqNo;

    /* Set destination address in EasyLink API */
    easyLinkAckTxPacket.dstAddr[0] = destAddress;

    /* Copy data packet to payload
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
    memcpy(easyLinkAckTxPacket.payload, ((uint8_t*)&ackPacket), sizeof(struct AckPacket));
    easyLinkAckTxPacket.len = sizeof(struct AckPacket);

    /* Abort any previously running rx*/
    if (EasyLink_abort() != EasyLink_Status_Success) {
        System_abort("EasyLink_abort failed");
    }

    /* Setup retries */

    /* Send packet  */
    if (EasyLink_transmit(&easyLinkAckTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }

    /* Enter RX */
    asyncrx(currentRadioOperation.ackTimeoutMs);
}

//Internal send command for Ack Packets
static void forwardPacket(union GenericPacket gPacket, uint8_t destAddress) {
//    System_printf("forwardPkt\n");
//            System_flush();


    /* Set destination address in EasyLink API */
    currentRadioOperation.easyLinkTxPacket.dstAddr[0] = destAddress;

    //CHECK IF DATA or ACK TODO
    if (gPacket.header.packetType == PacketType_Ack) {
    /* Copy data packet to payload
     * Note that the EasyLink API will implcitily both add the length byte and the destination address byte. */
        memcpy(currentRadioOperation.easyLinkTxPacket.payload, (&gPacket), sizeof(struct AckPacket));
        currentRadioOperation.easyLinkTxPacket.len = sizeof(struct AckPacket);

    }
    else {
        memcpy(currentRadioOperation.easyLinkTxPacket.payload, (&gPacket), sizeof(struct DataPacket));
        currentRadioOperation.easyLinkTxPacket.len = sizeof(struct DataPacket);
    }

    /* Setup retries */
    currentRadioOperation.maxNumberOfRetries = 0;
    currentRadioOperation.ackTimeoutMs = NO_TIMEOUT;
    currentRadioOperation.retriesDone = 0;

    /* Abort any previously running rx*/
    if (EasyLink_abort() != EasyLink_Status_Success) {
        System_abort("EasyLink_abort failed");
    }

    csma();

    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }

    returnRadioOperationStatus(NodeRadioStatus_Success);
    asyncrx(currentRadioOperation.ackTimeoutMs);
}


//Internal func for retrying until ack is received
static void resendPacket()
{

    /* Abort any previously running asyncrx*/
    if (EasyLink_abort() != EasyLink_Status_Success) {
        System_abort("EasyLink_abort failed");
    }

    csma();
    /* Send packet  */
    if (EasyLink_transmit(&currentRadioOperation.easyLinkTxPacket) != EasyLink_Status_Success)
    {
        System_abort("EasyLink_transmit failed");
    }

    /* Increase retries by one */
    currentRadioOperation.retriesDone++;

    asyncrx(currentRadioOperation.ackTimeoutMs);
}

static void asyncrx(uint32_t timeout) {

    /* Abort any previously running asyncrx*/
    if (EasyLink_abort() != EasyLink_Status_Success) {
        System_abort("EasyLink_abort failed");
    }

    EasyLink_setCtrl(EasyLink_Ctrl_AsyncRx_TimeOut, EasyLink_ms_To_RadioTime(timeout));
    easystat = EasyLink_receiveAsync(rxDoneCallback, 0);

    if (easystat != EasyLink_Status_Success)
    {
        System_abort("EasyLink_receiveAsync failed");
    }
}

//Callback for async Rx func
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
//        System_printf("rxDone\n");
//        System_flush();
    struct PacketHeader* packetHeader;

    /* If this callback is called because of a packet received */
    if (status == EasyLink_Status_Success)
    {
        /* Check the payload header */
        packetHeader = (struct PacketHeader*)rxPacket->payload;

        if (rxPacket->dstAddr[0] == NODE_ADDR) {
            last_rssi = rxPacket->rssi; //We Shouldn't have to wait for any other node in this case

          if (packetHeader->packetType == PacketType_Ack)
          {
              // Save Ack Sequence Number
              latestAckRxSeqNo = ((struct AckPacket*)rxPacket->payload)->seqNo;
              Event_post(radioOperationEventHandle, RADIO_EVENT_DATA_ACK_RECEIVED);
          }

          else if (packetHeader->packetType == PacketType_Data)
          {
//              rssi_timestamp = Clock_getTicks();

              // Save Data Packet
              latestRxPacket.destAddress = rxPacket->dstAddr[0];
              memcpy((void*)&(latestRxPacket.packet), &rxPacket->payload, sizeof(struct DataPacket));

              Event_post(radioOperationEventHandle, RADIO_EVENT_VALID_PACKET_RECEIVED);
          }
        }
        else {

            last_rssi = rxPacket->rssi;
            rssi_timestamp = Clock_getTicks();

            // Receive Packet for someone else: Pass up to flooding
            latestRxPacket.destAddress = rxPacket->dstAddr[0];
            if (packetHeader->packetType == PacketType_Ack)
            {
              memcpy(&(latestRxPacket.packet), &rxPacket->payload, sizeof(struct AckPacket));
            }
            else if (packetHeader->packetType == PacketType_Data) {
              memcpy(&(latestRxPacket.packet), &rxPacket->payload, sizeof(struct DataPacket));
            }
            floodTask_floodPacket(&latestRxPacket, rxPacket->rssi);
            Event_post(radioOperationEventHandle, RADIO_EVENT_OTHER_PACKET_RECEIVED);
        }

    }
    // Packet rx was unsuccessful or there was a timeout
    else if (status != EasyLink_Status_Aborted) {
        if (status == EasyLink_Status_Rx_Error) {
            //treat error as a collision
            last_rssi = 0;
            rssi_timestamp = Clock_getTicks();
        }
        Event_post(radioOperationEventHandle, RADIO_EVENT_ACK_TIMEOUT);
    }

}
