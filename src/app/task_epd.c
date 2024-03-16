#include <stdio.h>
#include <string.h>

#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

//#include "board.h"
//#include <ti/drivers/uart/UARTCC26XX.h>
#include "task_epd.h"
#include "epd2in13.h"
#include "util.h"

uint8_t VERSION_MAJOR = 0;
uint8_t VERSION_MINOR = 2;

#define EPD_TASK_PRIORITY                     2
#define EPD_TASK_STACK_SIZE                   512
Task_Struct EPDTask;
Char EPDTaskStack[EPD_TASK_STACK_SIZE];

uint32_t epoch_time = 0;
 
#if 0
 enum {
   EPD_CMD_PING = 0x10,
   EPD_CMD_INIT = 0x11,
   EPD_CMD_DEINIT = 0x12,
    
   EPD_CMD_PREPARE_BLK_RAM = 0x13,
   EPD_CMD_WRITE_BLK_RAM = 0x14,
   EPD_CMD_GET_BLK_RAM_CRC = 0x15,
   
   EPD_CMD_PREPARE_RED_RAM = 0x18,
   EPD_CMD_WRITE_RED_RAM = 0x19,
   EPD_CMD_GET_RED_RAM_CRC = 0x20,

   EPD_CMD_UPDATE_DISPLAY = 0x21,  // ask epd to show ram data
   
   EPD_CMD_READ_VERSION = 0x22,

};

#endif
 

// cc2640 received epd command frame from andoid app
// first byte is length, second is command, third and follow are command data if any
//static uint8_t epd_rx_frame[200];
//uint8_t rx_fram_len = 0;

//static uint8_t epd_resp_frame[6];
//uint8_t resp_fram_len = 1;

//EpdResponseCallback respCallback = NULL;

// Event used to control the EPD thread
Event_Struct EPDEvent;
Event_Handle hEPDEvent;

#define EPDTASK_EVENT_RX_REQUEST        Event_Id_00 
#define EPDTASK_EVENT_PERIODIC          Event_Id_01

#define EPDTASK_EVENT_ALL               ( EPDTASK_EVENT_RX_REQUEST | \
                                          EPDTASK_EVENT_PERIODIC )

static Clock_Struct periodicClock;

//static void handle_cmd();       
void TaskEPD_taskFxn(UArg a0, UArg a1);
//static void post_epd_response(uint8_t *buf, uint16_t len);

void TaskEPD_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = EPDTaskStack;
  taskParams.stackSize = EPD_TASK_STACK_SIZE;
  taskParams.priority = EPD_TASK_PRIORITY;

  Task_construct(&EPDTask, TaskEPD_taskFxn, &taskParams, NULL);
}

static void EPDTask_clockHandler(UArg arg)
{
    Event_post(hEPDEvent, arg);
}

void TaskEPD_taskInit(void)
{
    Util_constructClock(&periodicClock, EPDTask_clockHandler,
                        1000, 0, false, EPDTASK_EVENT_PERIODIC);
    epd_hw_init();
    Util_startClock(&periodicClock);
}

void TaskEPD_taskFxn(UArg a0, UArg a1)
{ 
    Event_Params evParams;
    Event_Params_init(&evParams);
    Event_construct(&EPDEvent, &evParams);
    hEPDEvent = Event_handle(&EPDEvent);
    
    TaskEPD_taskInit();
    
    while(1)
    {
        UInt events;
        events = Event_pend(hEPDEvent,Event_Id_NONE, EPDTASK_EVENT_ALL, BIOS_WAIT_FOREVER);
        
        if(events & EPDTASK_EVENT_RX_REQUEST) {
            //if (resp_fram_len) {
            //    post_epd_response(epd_resp_frame, resp_fram_len);
            //}
        
        }    

        if (events & EPDTASK_EVENT_PERIODIC) {
            Util_startClock(&periodicClock);

            EPD_Update(); 
        }
    }
}
