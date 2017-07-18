/*
 * FLooding Task
 */

#ifndef TASKS_FLOODTASK_H_
#define TASKS_FLOODTASK_H_

/* Initializes the Node Task and creates all TI-RTOS objects */
void floodTask_init(void);
void floodTask_floodPacket(struct ComboPacket* packet, int8_t rssi);
void floodTask_sendData(struct ComboPacket* packet);

#endif /* TASKS_FLOODTASK_H_ */
