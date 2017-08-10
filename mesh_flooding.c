/*
 *  ======== mesh_flooding.c ========
 *
 *  author: R. Menon
 *  date: 06/04/17
 *
 *  description: main func sets up pins,
 *  initializes RTOS tasks, and starts RTOS
 *
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Headers NOT SURE IF NEEDED */
#include <ti/drivers/UART.h>
#include <ti/drivers/SPI.h>

/* Board Header files */
#include "Board.h"

#include "mac_task.h"
#include "app_task.h"
#include "flooding_task.h"


/*
 *  ======== main ========
 */
int main(void) {

    /* Call board init functions */
    Board_initGeneral();

    /* For LCD Display */
    Board_initUART();
    Board_initSPI();

    /* Initialize RTOS Tasks: radio/mac and protocol */
    macTask_init();
    floodTask_init();
    appTask_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
