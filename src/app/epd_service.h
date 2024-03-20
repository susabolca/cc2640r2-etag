#ifndef _EPD_SERVICE_H_
#define _EPD_SERVICE_H_

#include <bcomdef.h>

// Service UUID
#define EPD_SERVICE_SERV_UUID       0x1140

// Unix Epoch (from 1970-1-1 00:00)
#define EPD_EPOCH_ID                 0
#define EPD_EPOCH_UUID               0x1141
#define EPD_EPOCH_LEN                4
#define EPD_EPOCH_LEN_MIN            4

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