#ifndef TASK_EPD_H
#define TASK_EPD_H

#define IOID_12                 0x0000000C  // IO Id 12
#define IOID_13                 0x0000000D  // IO Id 13
#define IOID_3                  0x00000003  // IO Id 3
#define IOID_4                  0x00000004  // IO Id 4

// used to call when epd processed request, and response is ready
typedef void (*EpdResponseCallback)(uint8_t event, uint8_t *buf, uint8_t len);
void EPDTask_RegisterResponseCallback(EpdResponseCallback callback);
void TaskEPD_createTask(void);

// api for event_post
void EPDTask_Update(void);

#endif
