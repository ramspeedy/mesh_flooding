/*
 * App Task
 */

#ifndef TASKS_APPTASK_H_
#define TASKS_APPTASK_H_

/* Initializes the Node Task and creates all TI-RTOS objects */
void appTask_init(void);
void appTask_packetReceived(struct ComboPacket* packet, int8_t rssi);
void appTask_ackReceived(uint16_t seqNo);
void appTask_sendFail();

struct nodeStat {
    uint16_t tx;
    uint16_t rx;
};

extern uint16_t txPacketCount;
extern struct nodeStat statArray[NUM_NODES];

#endif /* TASKS_APPTASK_H_ */
