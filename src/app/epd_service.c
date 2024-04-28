
#include <string.h>

//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log
//#include <uartlog/UartLog.h>  // Comment out if using xdc Log
#include <icall.h>
#include "util.h"

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include <ti/sysbios/hal/Seconds.h> // Seconds_set

#include "epd_driver.h" // epd_battery, epd_temperature
#include "epd_service.h"
#include "task_epd.h"   // EPDTask_Update

// EPD_Service Service UUID
CONST uint8 EpdServiceUUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(EPD_SERVICE_SERV_UUID), HI_UINT16(EPD_SERVICE_SERV_UUID),
};

// Unix Epoch UUID
CONST uint8 EpdEpochUUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(EPD_EPOCH_UUID), HI_UINT16(EPD_EPOCH_UUID),
};

CONST uint8 EpdUtcOffsetUUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(EPD_UTC_OFFSET_UUID), HI_UINT16(EPD_UTC_OFFSET_UUID),
};

CONST uint8 EpdBattUUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(EPD_BATT_UUID), HI_UINT16(EPD_BATT_UUID),
};

CONST uint8 EpdTempUUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(EPD_TEMP_UUID), HI_UINT16(EPD_TEMP_UUID),
};

CONST uint8 EpdRtcCollabUUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(EPD_RTC_COLLAB_UUID), HI_UINT16(EPD_RTC_COLLAB_UUID),
};

CONST uint8 EpdRxTxUUID[ATT_BT_UUID_SIZE] = {
    LO_UINT16(EPD_RXTX_UUID), HI_UINT16(EPD_RXTX_UUID),
};

static EpdServiceCBs_t *pAppCBs = NULL;
static gattCharCfg_t *EpdDataConfig;

// Service declaration
static CONST gattAttrType_t EpdServiceDecl = { ATT_BT_UUID_SIZE, EpdServiceUUID };

//static uint8 EpochDesc[] = "Unix Epoch";
static uint8 EpochProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint32 EpochLastVal = 0; // last value of Unix Epoch

//static uint8 UtcOffDesc[] = "UTC Offset Mins";
static uint8 UtcOffProps = GATT_PROP_READ | GATT_PROP_WRITE;
//static int8  UtcOffVal[4] = {0};

//static uint8 BattDesc[] = "Battery mv";
static uint8 BattProps = GATT_PROP_READ;
//static uint8 BattVal[2] = {0};

//static uint8 TempDesc[] = "Temperature";
static uint8 TempProps = GATT_PROP_READ;
//static int8  TempVal[1] = {0};

// RTC Collaboration
static uint8 RtcCollabProps = GATT_PROP_READ | GATT_PROP_WRITE;
//static int8 RtcCollabVal[1] = {0};

// RxTx service
static uint8 RxTxProps = GATT_PROP_READ | GATT_PROP_WRITE;
//static uint8 RxTxBuf[64];

static gattAttribute_t EpdServiceAttrTbl[] =
{
    // EPD_Service Service Declaration
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)&EpdServiceDecl
    },

    // Epoch Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &EpochProps
    },
        // Epoch Characteristic Value
        {
            { ATT_BT_UUID_SIZE, EpdEpochUUID },
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            NULL 
        },

    // Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &UtcOffProps
    },
        // Characteristic Value
        {
            { ATT_BT_UUID_SIZE, EpdUtcOffsetUUID },
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            NULL
        },

    // Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &BattProps
    },
        // Characteristic Value
        {
            { ATT_BT_UUID_SIZE, EpdBattUUID },
            GATT_PERMIT_READ,
            0,
            NULL
        },

    // Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &TempProps
    },
        // Characteristic Value
        {
            { ATT_BT_UUID_SIZE, EpdTempUUID },
            GATT_PERMIT_READ,
            0,
            NULL
        },

    // Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &RtcCollabProps
    },
        // Characteristic Value
        {
            { ATT_BT_UUID_SIZE, EpdRtcCollabUUID },
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            NULL 
        },

    // Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &RxTxProps
    },
        // Characteristic Value
        {
            { ATT_BT_UUID_SIZE, EpdRxTxUUID },
            GATT_PERMIT_READ | GATT_PERMIT_WRITE,
            0,
            NULL
        },
};


static bStatus_t EPDService_ReadAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue,
                                        uint16_t *pLen,
                                        uint16_t offset,
                                        uint16_t maxLen,
                                        uint8_t method);
static bStatus_t EPDService_WriteAttrCB(uint16_t connHandle,
                                         gattAttribute_t *pAttr,
                                         uint8_t *pValue,
                                         uint16_t len,
                                         uint16_t offset,
                                         uint8_t method);


// Simple Profile Service Callbacks
CONST gattServiceCBs_t EpdServiceCBs =
{
    EPDService_ReadAttrCB,     // Read callback function pointer
    EPDService_WriteAttrCB,    // Write callback function pointer
    NULL                        // Authorization callback function pointer
};

bStatus_t EPDService_AddService(uint8_t rspTaskId)
{
    uint8_t status;

    // Allocate Client Characteristic Configuration table
    EpdDataConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) * linkDBNumConns);
    if (EpdDataConfig == NULL) {
        return (bleMemAllocError);
    }

    // Register with Link DB to receive link status change callback
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, EpdDataConfig);

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService(EpdServiceAttrTbl,
                                         GATT_NUM_ATTRS(EpdServiceAttrTbl),
                                         GATT_MAX_ENCRYPT_KEY_SIZE,
                                         &EpdServiceCBs);
    return status;
}

bStatus_t EPDService_RegisterAppCBs(EpdServiceCBs_t *appCallbacks)
{
    if(appCallbacks) {
        pAppCBs = appCallbacks;
        return SUCCESS;
    } else {
        return FAILURE;
    }
}

bStatus_t EPDService_SetParameter(uint8_t param, uint16_t len, void *value)
{
    bStatus_t ret = SUCCESS;

    switch (param) {

        default:
            return(INVALIDPARAMETER);
    }

#if 0
    // Check bounds, update value and send notification or indication if possible.
    if(len <= valMaxLen && len >= valMinLen) {
        memcpy(pAttrVal, value, len);
        *pValLen = len; // Update length for read and get.
    } else {
        ret = bleInvalidRange;
    }
#endif
    return ret;
}

bStatus_t EPDService_GetParameter(uint8_t param, uint16_t *len, void *value)
{
    bStatus_t ret = SUCCESS;
    switch (param) {

        default:
            ret = INVALIDPARAMETER;
            break;
    }
    return ret;
}

static bStatus_t utilExtractUuid16(gattAttribute_t *pAttr, uint16_t *pUuid)
{
  bStatus_t status = SUCCESS;

  if (pAttr->type.len == ATT_BT_UUID_SIZE) {
    // 16-bit UUID direct
    *pUuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
  } else if (pAttr->type.len == ATT_UUID_SIZE) {
    // 16-bit UUID extracted bytes 12 and 13
    *pUuid = BUILD_UINT16(pAttr->type.uuid[12], pAttr->type.uuid[13]);
  } else {
    *pUuid = 0xFFFF;
    status = FAILURE;
  }
  return status;
}

static bStatus_t EPDService_ReadAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen,
                                       uint16_t offset,
                                       uint16_t maxLen,
                                       uint8_t method)
{
    bStatus_t status = SUCCESS;

    if (offset > 0) {
        return ATT_ERR_ATTR_NOT_LONG;
    }

    uint16_t uuid;
    if (utilExtractUuid16(pAttr, &uuid) == FAILURE) {
        return ATT_ERR_INVALID_HANDLE;
    }

    switch(uuid) {
        case EPD_EPOCH_UUID: {
            uint32_t t = Seconds_get();
            *pLen = sizeof(t);
            memcpy(pValue, &t, *pLen);
            break;
        }

        case EPD_UTC_OFFSET_UUID: {
            int32_t t = utc_offset_mins;
            *pLen = sizeof(t);
            memcpy(pValue, &t, *pLen);
            break;
        }

        case EPD_BATT_UUID: {
            uint16_t v = INTFRAC2MV(epd_battery);
            *pLen = sizeof(v);
            memcpy(pValue, &v, *pLen);
            break;
        }
        
        case EPD_TEMP_UUID: {
            uint8_t v = epd_temperature;
            *pLen = sizeof(v);
            memcpy(pValue, &v, *pLen);
            break;
        }

        case EPD_RTC_COLLAB_UUID: {
            int8_t v = RTC_GetCollaborate();
            *pLen = sizeof(v);
            memcpy(pValue, &v, *pLen);
            break;   
        }

        case EPD_RXTX_UUID: {
            // send ble_data to BLE.
            uint8_t idx = ble_data_cur;
            if (idx >= ble_data_len) {
                return ATT_ERR_ATTR_NOT_FOUND;
            }
            uint8_t len = MIN(ble_data_len - idx, maxLen - 1);
            // index, data ...
            pValue[0] = idx;
            memcpy(&pValue[1], &ble_data[idx], len);
            *pLen = len + 1;
            break;
        }

        default:
            return ATT_ERR_ATTR_NOT_FOUND;
    }

    return status;
}

static bStatus_t EPDService_WriteAttrCB(uint16_t connHandle,
                                        gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len,
                                        uint16_t offset,
                                        uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint8_t changeParamID = 0xFF;
    uint16_t uuid; 

    // Find settings for the characteristic to be written.
    if (utilExtractUuid16(pAttr, &uuid) == FAILURE) {
        return ATT_ERR_INVALID_HANDLE;
    }

    switch (uuid) {
        case EPD_EPOCH_UUID: {
            if (len == 4) {
                uint32_t t = *(uint32_t*)pValue;
                Seconds_set(t);
                EpochLastVal = t;

                // notify EPD to refresh
                clock_last = -1;
                EPDTask_Update();
            }
            break;
        }

        case EPD_UTC_OFFSET_UUID: {
            if (len == 4) {
                int32_t t = *(int32_t*)pValue;
                if ((t >= (-12*60)) && (t <= (12*60))) { // +/- 12 hrs
                    utc_offset_mins = t;
                }
            }
            break;
        }

        case EPD_RTC_COLLAB_UUID: {
            if (len == 1) {
                int8_t v = *(int8_t*)pValue;
                RTC_SetCollaborate(v);
            }
            break;
        }

        case EPD_RXTX_UUID: {
            if (len < 64) {
                EPD_Command(pValue, len);
            }
            break;
        }

        case GATT_CLIENT_CHAR_CFG_UUID:
            status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len, offset, GATT_CLIENT_CFG_NOTIFY);
            break; 


        default:
            return ATT_ERR_ATTR_NOT_FOUND;
    }
    
    // Let the application know something changed (if it did) by using the
    // callback it registered earlier (if it did).
    if(changeParamID != 0xFF) {
        if(pAppCBs && pAppCBs->pfnChangeCb) {
            pAppCBs->pfnChangeCb(connHandle, changeParamID, len + offset, pValue); // Call app function from stack task context.
        }
    }
    return status;
}
