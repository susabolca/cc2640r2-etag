#ifndef _EPD_SERVICE_H_
#define _EPD_SERVICE_H_

#include <bcomdef.h>

// Service UUID
#define EPD_SERVICE_SERV_UUID       0xFFF0

// Unix Epoch (from 1970-1-1 00:00)
#define EPD_EPOCH_UUID              0xFFF1

// UTC local time offset 
#define EPD_UTC_OFFSET_UUID         0xFFF2

// battery
#define EPD_BATT_UUID               0xFFF3

// temperature
#define EPD_TEMP_UUID               0xFFF4

// RTC collaboration value 
#define EPD_RTC_COLLAB_UUID         0xFFF5

// EPD RXTX service
#define EPD_RXTX_UUID               0xFFFE

// Callback when a characteristic value has changed
typedef void (*EpdServiceChange_t)(uint16_t connHandle, uint8_t paramID,
                                   uint16_t len, uint8_t *pValue);

typedef struct {
    EpdServiceChange_t pfnChangeCb;          // Called when characteristic value changes
    EpdServiceChange_t pfnCfgChangeCb;       // Called when characteristic CCCD changes
} EpdServiceCBs_t;

extern bStatus_t EPDService_AddService(uint8_t rspTaskId);
extern bStatus_t EPDService_RegisterAppCBs(EpdServiceCBs_t *appCallbacks);
extern bStatus_t EPDService_SetParameter(uint8_t param, uint16_t len, void *value);
extern bStatus_t EPDService_GetParameter(uint8_t param, uint16_t *len, void *value);

#endif // _EPD_SERVICE_H_