#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#include <ti/sysbios/hal/Seconds.h> // Seconds_set
#include <xdc/runtime/System.h>     // snprintf

// profiles
#include "devinfoservice.h"
#include "epd_service.h"
#include "peripheral.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include "board.h"
#include "task_ble.h"

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use for automatic parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) for automatic parameter
// update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// After the connection is formed, the peripheral waits until the central
// device asks for its preferred connection parameters
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000

// Application specific event ID for HCI Connection Event End Events
#define SBP_HCI_CONN_EVT_END_EVT              0x0001

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Application events
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
//#define SBP_KEY_CHANGE_EVT                    0x0004

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
//#define SBP_PERIODIC_EVT                      Event_Id_00

// Bitwise OR of all events to pend on
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT    | \
                                               SBP_QUEUE_EVT)

// Row numbers for two-button menu
#define SBP_ROW_RESULT        TBM_ROW_APP
#define SBP_ROW_STATUS_1      (TBM_ROW_APP + 1)
#define SBP_ROW_STATUS_2      (TBM_ROW_APP + 2)
#define SBP_ROW_ROLESTATE     (TBM_ROW_APP + 3)
#define SBP_ROW_BDADDR        (TBM_ROW_APP + 4)

// App event passed from profiles.
typedef struct
{
    appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

// Display Interface
Display_Handle dispHandle = NULL;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
//static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// BLE mac address.
uint8_t mac_address[6];

// Scan response data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
    // complete name
    0x0b,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'C','2','6','_','0','0','0','0','0','0',

    // connection interval range
    0x05,   // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0       // 0dBm
};



// Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
    // Flags: this field sets the device to use general discoverable
    // mode (advertises indefinitely) instead of general
    // discoverable mode (advertise for 30 seconds at a time)
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,   // length of this data
    GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
    LO_UINT16(EPD_SERVICE_SERV_UUID),
    HI_UINT16(EPD_SERVICE_SERV_UUID)
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "CC2640R2_ETAG";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
//static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
//static void SimpleBLEPeripheral_performPeriodicTask(void);
//static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

// EPD Services 
static void BLETask_EpdService_ValueChangeCB(uint16_t connHandle,
                                                 uint8_t paramID,
                                                 uint16_t len,
                                                 uint8_t *pValue);
static void BLETask_EpdService_CfgChangeCB(uint16_t connHandle,
                                               uint8_t paramID,
                                               uint16_t len,
                                               uint8_t *pValue);

extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

// Peripheral GAPRole Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // GAPRole State Change Callbacks
};

// GAP Bond Manager Callbacks
// These are set to NULL since they are not needed. The application
// is set up to only perform justworks pairing.
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback
  NULL  // Pairing / Bonding state Callback
};

// Simple GATT Profile Callbacks
static EpdServiceCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  BLETask_EpdService_ValueChangeCB,
  BLETask_EpdService_CfgChangeCB,
};

void SimpleBLEPeripheral_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = sbpTaskStack;
    taskParams.stackSize = SBP_TASK_STACK_SIZE;
    taskParams.priority = SBP_TASK_PRIORITY;

    Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

static void SimpleBLEPeripheral_init(void)
{
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);
    
    // get mac address
    {
        uint32_t mac[2];
        mac[0] = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0);
        mac[1] = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);

        mac_address[5] = mac[0];
        mac_address[4] = mac[0]>>8;
        mac_address[3] = mac[0]>>16;
        mac_address[2] = mac[0]>>24;
        mac_address[1] = mac[1];
        mac_address[0] = mac[1]>>8;
    }

    // update scanRspData
    {
        char buf[8];
        System_snprintf(buf, 8, "%02x%02x%02x", mac_address[3], mac_address[4], mac_address[5]);
        memcpy(scanRspData+6, buf, 6);
    }

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

    // Create one-shot clocks for internal periodic events.
    //Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
    //                    SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);

    //dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);

    // Set GAP Parameters: After a connection was established, delay in seconds
    // before sending when GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE,...)
    // uses GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS or
    // GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS
    // For current defaults, this has no effect.
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

    // Setup the Peripheral GAPRole Profile. For more information see the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/gaprole.html
    {
        // Device starts advertising upon initialization of GAP
        uint8_t initialAdvertEnable = TRUE;

        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until re-enabled by the application
        uint16_t advertOffTime = 0;

        uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

        // Set the Peripheral GAPRole Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                            &initialAdvertEnable);
        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                            &advertOffTime);

        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                            scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

        GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                            &enableUpdateRequest);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                            &desiredMinInterval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                            &desiredMaxInterval);
        GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                            &desiredSlaveLatency);
        GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                            &desiredConnTimeout);
    }

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/gaprole.html
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Set GAP Parameters to set the advertising interval
    // For more information, see the GAP section of the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/gatt.html#gap-gatt-service-ggs
    {
        // Use the same interval for general and limited advertising.
        // Note that only general advertising will occur based on the above configuration
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    }

    // Setup the GAP Bond Manager. For more information see the section in the
    // User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/gapbondmngr.html#
    {
        // Hard code the passkey that will be used for pairing. The GAPBondMgr will
        // use this key instead of issuing a callback to the application. This only
        // works if both sides of the connection know to use this same key at
        // compile-time.
        uint32_t passkey = 0; // passkey "000000"
        // Don't send a pairing request after connecting; the peer device must
        // initiate pairing
        uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        // Use authenticated pairing: require passcode.
        uint8_t mitm = TRUE;
        // This device only has display capabilities. Therefore, it will display the
        // passcode during pairing. However, since the default passcode is being
        // used, there is no need to display anything.
        uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        // Request bonding (storing long-term keys for re-encryption upon subsequent
        // connections without repairing)
        uint8_t bonding = TRUE;

        GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                                &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
    DevInfo_AddService();                        // Device Information Service

    // EPD GATT service
    EPDService_AddService(0);                    // Simple GATT Profile

#ifdef IMAGE_INVALIDATE
    Reset_addService();
#endif //IMAGE_INVALIDATE

    // Setup the SimpleProfile Characteristic Values
    // For more information, see the sections in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/gatt.html#
    // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/gatt.html#gattservapp-module
#if 0  
    {
        uint8_t charValue1 = 1;
        uint8_t charValue2 = 2;
        uint8_t charValue3 = 3;
        uint8_t charValue4 = 4;
        uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

        EPDService_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                                &charValue1);
        EPDService_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                                &charValue2);
        EPDService_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                                &charValue3);
        EPDService_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                                &charValue4);
        EPDService_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                                charValue5);
    }
#endif
  
    // Register callback with SimpleGATTprofile
    EPDService_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);

    // Start Bond Manager and register callback
    VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

    // Register with GAP for HCI/Host messages. This is needed to receive HCI
    // events. For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/hci.html
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // Set default values for Data Length Extension
    // This should be included only if Extended Data Length Feature is enabled
    // in build_config.opt in stack project.
    {
        //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
        #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
        #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

        // This API is documented in hci.h
        // See BLE5-Stack User's Guide for information on using this command:
        // http://software-dl.ti.com/lprf/ble5stack-docs-latest/html/ble-stack/data-length-extensions.html
        // HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
    }

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
    // Initialize GATT Client
    GATT_InitClient();

    // This line masks the Resolvable Private Address Only (RPAO) Characteristic
    // in the GAP GATT Server from being detected by remote devices. This value
    // cannot be toggled without power cycling but should remain consistent across
    // power-cycles. Removing this command when Privacy is used will cause this
    // device to be treated in Network Privacy Mode by bonded devices - this means
    // that after disconnecting they will not respond to this device's PDUs which
    // contain its Identity Address.
    // Devices wanting to use Network Privacy Mode with other BT5 devices, this
    // line should be commented out.
    GGS_SetParamValue(GGS_DISABLE_RPAO_CHARACTERISTIC);
#endif // BLE_V42_FEATURES & PRIVACY_1_2_CFG

#if !defined (USE_LL_CONN_PARAM_UPDATE)
    // Get the currently set local supported LE features
    // The will result in a HCI_LE_READ_LOCAL_SUPPORTED_FEATURES event that
    // will get received in the main task processing loop. At this point,
    // feature bits can be set / cleared and the features can be updated.
    HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

    // Start the GAPRole
    VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
}

static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    SimpleBLEPeripheral_init();

    // Application main loop
    for (;;)
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                            ICALL_TIMEOUT_FOREVER);

        if (events) {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Fetch any available messages that might have been sent from the stack
            if (ICall_fetchServiceMsg(&src, &dest,
                                        (void **)&pMsg) == ICALL_ERRNO_SUCCESS) {
                uint8 safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity)) {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

                    // Check for BLE stack events first
                    if (pEvt->signature == 0xffff) {
                        // The GATT server might have returned a blePending as it was trying
                        // to process an ATT Response. Now that we finished with this
                        // connection event, let's try sending any remaining ATT Responses
                        // on the next connection event.
                        if (pEvt->event_flag & SBP_HCI_CONN_EVT_END_EVT) {
                            // Try to retransmit pending ATT Response (if any)
                            SimpleBLEPeripheral_sendAttRsp();
                        }
                    } else {
                        // Process inter-task message
                        safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
                    }
                }

                if (pMsg && safeToDealloc) {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            if (events & SBP_QUEUE_EVT) {
                while (!Queue_empty(appMsgQueue)) {
                    sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
                    if (pMsg) {
                        // Process message.
                        SimpleBLEPeripheral_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }
        }
    }
}

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE
              // The L2CAP Connection Parameter Update procedure is used to
              // support a delta between the minimum and maximum connection
              // intervals required by some iOS devices.

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // Get current feature set from received event (bits 1-9
                      // of the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          // LE Events
          case HCI_LE_EVENT_CODE:
            {
              hciEvt_BLEPhyUpdateComplete_t *pPUC
                = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

              // A Phy Update Has Completed or Failed
              if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
              {
                if (pPUC->status != SUCCESS)
                {
                  Display_print0(dispHandle, SBP_ROW_STATUS_1, 0,
                                 "PHY Change failure");
                }
                else
                {
                  Display_print0(dispHandle, SBP_ROW_STATUS_1, 0,
                                 "PHY Update Complete");
                  // Only symmetrical PHY is supported.
                  // rxPhy should be equal to txPhy.
                  Display_print1(dispHandle, SBP_ROW_STATUS_2, 0,
                                 "Current PHY: %s",
                                 (pPUC->rxPhy == HCI_PHY_1_MBPS) ? "1 Mbps" :

// Note: BLE_V50_FEATURES is always defined and long range phy (PHY_LR_CFG) is
//       defined in build_config.opt
#if (BLE_V50_FEATURES & PHY_LR_CFG)
                                   ((pPUC->rxPhy == HCI_PHY_2_MBPS) ? "2 Mbps" :
                                       "Coded:S2"));
#else  // !PHY_LR_CFG
                                   "2 Mbps");
#endif // PHY_LR_CFG
                }
              }
            }
            break;

          default:
            break;
        }
      }
      break;

      default:
        // do nothing
        break;

    }

  return (safeToDealloc);
}

static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_HCI_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, SBP_ROW_RESULT, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, SBP_ROW_RESULT, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      //SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        Display_print0(dispHandle, SBP_ROW_BDADDR, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Advertising");
      break;

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        //Util_startClock(&periodicClock);

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          Display_print1(dispHandle, SBP_ROW_ROLESTATE, 0, "Num Conns: %d", (uint16_t)numActive);
          Display_print0(dispHandle, SBP_ROW_STATUS_1, 0, Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Connected");
          Display_print0(dispHandle, SBP_ROW_STATUS_1, 0, Util_convertBdAddr2Str(peerAddress));
        }

      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      //Util_stopClock(&periodicClock);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Disconnected");

      // Clear remaining lines
      Display_clearLines(dispHandle, SBP_ROW_RESULT, SBP_ROW_STATUS_2);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Timed Out");


      // Clear remaining lines
      Display_clearLines(dispHandle, SBP_ROW_STATUS_1, SBP_ROW_STATUS_2);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif // PLUS_BROADCASTER
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Error");
      break;

    default:
      Display_clearLines(dispHandle, SBP_ROW_RESULT, SBP_ROW_STATUS_2);
      break;
  }

}

static void BLETask_EpdService_ValueChangeCB(uint16_t connHandle,
                                             uint8_t paramID,
                                             uint16_t len,
                                             uint8_t *pValue)
{
    if (len == 4) {
        Seconds_set(*((uint32_t*)pValue));
    }
}

static void BLETask_EpdService_CfgChangeCB(uint16_t connHandle,
                                           uint8_t paramID,
                                           uint16_t len,
                                           uint8_t *pValue)
{
    // no thing
}

#if 0
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}
#endif

static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, syncEvent, (uint8*)pMsg);
  }
}

// function to get AdbName in scan response.
void getBleAdvName(char *buf)
{
    memcpy(buf, scanRspData+2, 10);
    buf[11]='\0';
}