
#include <string.h>

//#include <xdc/runtime/Log.h> // Comment this in to use xdc.runtime.Log
//#include <uartlog/UartLog.h>  // Comment out if using xdc Log

#include <icall.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "epd_service.h"

// EPD_Service Service UUID
CONST uint8_t EpdServiceUUID[ATT_UUID_SIZE] =
{
    EPD_SERVICE_SERV_UUID_BASE128(EPD_SERVICE_SERV_UUID)
};

// Unix Epoch UUID
CONST uint8_t EpdUUID[ATT_UUID_SIZE] =
{
    EPD_EPOCH_UUID_BASE128(EPD_EPOCH_UUID)
};

static EpdServiceCBs_t *pAppCBs = NULL;

// Service declaration
static CONST gattAttrType_t EpdServiceDecl = { ATT_UUID_SIZE, EpdServiceUUID };

// Characteristic "Epoch" Properties (for declaration)
static uint8_t EpochProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

// Characteristic "Epoch" Value variable
static uint8_t EpochVal[EPD_EPOCH_LEN] = {0};

// Length of data in characteristic "Epoch" Value variable, initialized to minimal size.
static uint16_t EpochValLen = EPD_EPOCH_LEN_MIN;

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
        { ATT_UUID_SIZE, EpdUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        EpochVal
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

extern bStatus_t EPDService_AddService(uint8_t rspTaskId)
{
    uint8_t status;
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
    uint8_t  *pAttrVal;
    uint16_t *pValLen;
    uint16_t valMinLen;
    uint16_t valMaxLen;

    switch (param) {
        case EPD_EPOCH_ID:
            pAttrVal = EpochVal;
            pValLen = &EpochValLen;
            valMinLen = EPD_EPOCH_LEN_MIN;
            valMaxLen = EPD_EPOCH_LEN;
            break;

        default:
            return(INVALIDPARAMETER);
    }

    // Check bounds, update value and send notification or indication if possible.
    if(len <= valMaxLen && len >= valMinLen) {
        memcpy(pAttrVal, value, len);
        *pValLen = len; // Update length for read and get.
    } else {
        ret = bleInvalidRange;
    }

    return ret;
}

bStatus_t EPDService_GetParameter(uint8_t param, uint16_t *len, void *value)
{
    bStatus_t ret = SUCCESS;
    switch (param) {
        case EPD_EPOCH_ID:
            *len = MIN(*len, EpochValLen);
            memcpy(value, EpochVal, *len);
            break;

        default:
            ret = INVALIDPARAMETER;
            break;
    }
    return ret;
}

static uint8_t EPDService_findCharParamId(gattAttribute_t *pAttr)
{
    // Is this a Client Characteristic Configuration Descriptor?
    if(ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID ==
       *(uint16_t *)pAttr->type.uuid) {
        return (EPDService_findCharParamId(pAttr - 1)); // Assume the value attribute precedes CCCD and recurse
    
    // Is this attribute in "Epoch"?
    } else if (ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, EpdUUID, pAttr->type.len)) {
        return EPD_EPOCH_ID;
   
    } else {
        return 0xFF; // Not found. Return invalid.
    }
}

static bStatus_t EPDService_ReadAttrCB(uint16_t connHandle,
                                       gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen,
                                       uint16_t offset,
                                       uint16_t maxLen,
                                       uint8_t method)
{
    bStatus_t status = SUCCESS;
    uint16_t valueLen;
    uint8_t paramID = 0xFF;

    // Find settings for the characteristic to be read.
    paramID = EPDService_findCharParamId(pAttr);
    switch(paramID) {
        case EPD_EPOCH_ID:
            valueLen = EpochValLen;
            break;

        default:
            return ATT_ERR_ATTR_NOT_FOUND;
    }

    // Check bounds and return the value
    if(offset > valueLen) {
        status = ATT_ERR_INVALID_OFFSET;
    } else {
        *pLen = MIN(maxLen, valueLen - offset); // Transmit as much as possible
        memcpy(pValue, pAttr->pValue + offset, *pLen);
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
    uint8_t paramID = 0xFF;
    uint8_t changeParamID = 0xFF;
    uint16_t writeLenMin;
    uint16_t writeLenMax;
    uint16_t *pValueLenVar;

    // Find settings for the characteristic to be written.
    paramID = EPDService_findCharParamId(pAttr);
    switch(paramID) {
        case EPD_EPOCH_ID:
            writeLenMin = EPD_EPOCH_LEN_MIN;
            writeLenMax = EPD_EPOCH_LEN;
            pValueLenVar = &EpochValLen;

            /* Other considerations for Epoch can be inserted here */
            break;

        default:
            return ATT_ERR_ATTR_NOT_FOUND;
    }
    
    // Check whether the length is within bounds.
    if(offset >= writeLenMax) {
        status = ATT_ERR_INVALID_OFFSET;
    } else if(offset + len > writeLenMax) {
        status = ATT_ERR_INVALID_VALUE_SIZE;
    } else if(offset + len < writeLenMin &&
            (method == ATT_EXECUTE_WRITE_REQ || method == ATT_WRITE_REQ)) {
        // Refuse writes that are lower than minimum.
        // Note: Cannot determine if a Reliable Write (to several chars) is finished, so those will
        //       only be refused if this attribute is the last in the queue (method is execute).
        //       Otherwise, reliable writes are accepted and parsed piecemeal.
        status = ATT_ERR_INVALID_VALUE_SIZE;
    } else {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application and update length if enough data is written.
        //
        // Note: If reliable writes are used (meaning several attributes are written to using ATT PrepareWrite),
        //       the application will get a callback for every write with an offset + len larger than _LEN_MIN.
        // Note: For Long Writes (ATT Prepare + Execute towards only one attribute) only one callback will be issued,
        //       because the write fragments are concatenated before being sent here.
        if(offset + len >= writeLenMin) {
            changeParamID = paramID;
            *pValueLenVar = offset + len; // Update data length.
        }
    }

    // Let the application know something changed (if it did) by using the
    // callback it registered earlier (if it did).
    if(changeParamID != 0xFF) {
        if(pAppCBs && pAppCBs->pfnChangeCb) {
            pAppCBs->pfnChangeCb(connHandle, paramID, len + offset, pValue); // Call app function from stack task context.
        }
    }
    return status;
}
