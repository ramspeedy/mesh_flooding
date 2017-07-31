/*
 *
 */

#ifndef TASKS_MACTASK_H_
#define TASKS_MACTASK_H_

#include "stdint.h"
#include "RadioProtocol.h"

enum NodeRadioOperationStatus {
    NodeRadioStatus_Success,
    NodeRadioStatus_Failed
};

/* Initializes the NodeRadioTask and creates all TI-RTOS objects */
void macTask_init(void);

enum NodeRadioOperationStatus macTask_sendData(struct ComboPacket* packet);
void macTask_forwardPacket(struct ComboPacket* packet);


#endif /* TASKS_MACTASK_H_ */
