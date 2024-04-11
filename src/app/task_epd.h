#ifndef TASK_EPD_H
#define TASK_EPD_H

// used to call when epd processed request, and response is ready
typedef void (*EpdResponseCallback)(uint8_t event, uint8_t *buf, uint8_t len);
void EPDTask_RegisterResponseCallback(EpdResponseCallback callback);
void TaskEPD_createTask(void);

// api for event_post
void EPDTask_Update(void);

#endif