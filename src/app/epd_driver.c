// EPD Driver

#include <stdint.h>   // uint8_t 
#include <board.h>
#include <ti/drivers/PIN.h>
#include <ti/sysbios/knl/Clock.h>   // Clock_tickPeriod
#include <ti/sysbios/knl/Task.h>    // Task_sleep
#include <driverlib/cpu.h>  // CPUDelay

#include "epd_driver.h"
#include "task_epd.h"

// gpio setting
static PIN_Handle GPIOHandle = NULL;
static PIN_State  GPIOState;
static PIN_Config GPIOTable[] = {
  //EPD_POWER_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_BUSY_PIN  | PIN_GPIO_OUTPUT_DIS | PIN_INPUT_EN |  PIN_PULLUP,
  EPD_DC_PIN    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_RST_PIN   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_CS_PIN    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_SCL_PIN   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_SDA_PIN   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  PIN_TERMINATE
};

// epd frame buffer
uint8_t epd_buffer[EPD_BUF_MAX];

// debug only
int lut_size; 

// local time to UTC time offset, in minuts.
int32_t utc_offset_mins = 8 * 60;       // default is UTC+8

// battery voltage, in frac <3.8>
uint16_t epd_battery;

// in degree celcius, read from EPD.
int8_t epd_temperature;

// display mode (default is clock)
uint8_t epd_mode = EPD_MODE_CLOCK;

// rtc collaboration
int8_t epd_rtc_collab = 0;

// EPD BLE step
uint8_t epd_step = EPD_CMD_NC;
uint8_t epd_step_data[EPD_STEP_DATA_LEN];   // store parameters 

// the clock refesh on change
uint8_t clock_last = 0;

/*
 * Device APIs
 */
static inline void Util_delay_ms(uint16_t t)
{  
  Task_sleep( ((t) * 1000) / Clock_tickPeriod );
}

static inline void DEV_Digital_Write(uint32_t pin, uint8_t value)
{    
    PIN_setOutputValue(GPIOHandle, pin, value);    
}

static inline int DEV_Digital_Read(uint32_t pin)
{
    return PIN_getInputValue(pin);
}

static inline void DEV_Delay_ms(uint16_t t)
{
    Util_delay_ms(t);
}

static inline void DEV_Delay_us(uint16_t t)
{
    CPUdelay(t);
}

static inline void DEV_SPI_WriteByte(uint8_t byte)
{
    for (int i=0; i<8; i++) {
        DEV_Digital_Write(EPD_SCL_PIN, 0);
        if (byte & 0x80) {
            DEV_Digital_Write(EPD_SDA_PIN, 1);
        } else {
            DEV_Digital_Write(EPD_SDA_PIN, 0);
        }
        byte = (byte << 1);
        DEV_Delay_us(2);
        DEV_Digital_Write(EPD_SCL_PIN, 1);
        DEV_Delay_us(2);
    }
}

static inline uint8_t DEV_SPI_ReadByte()
{
    unsigned char i;
    uint8_t value = 0;

    for (i = 0; i < 8; i++) {
        DEV_Digital_Write(EPD_SCL_PIN, 0);
        DEV_Delay_us(10);
        DEV_Digital_Write(EPD_SCL_PIN, 1);
        DEV_Delay_us(10);
        value <<= 1;
        if (DEV_Digital_Read(EPD_SDA_PIN) != 0) {
            value |= 1;
        }
    }
    return value;
}

/*
 *  EPD_SSD APIs
 */
void EPD_SSD_Reset()
{    
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(20);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(20);
}

void EPD_SSD_SendCommand(uint8_t Reg)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
    //DEV_Digital_Write(EPD_DC_PIN, 0);
}

void EPD_SSD_SendData(uint8_t Data)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
    //DEV_Digital_Write(EPD_DC_PIN, 0);
}

uint8_t EPD_SSD_ReadData()
{
    uint8_t rc;
    // Change SDA Pin to input 
    PIN_setConfig(GPIOHandle, PIN_BM_ALL, EPD_SDA_PIN | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN | PIN_PULLUP);
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);

    // read bytes 
    rc = DEV_SPI_ReadByte();
    
    // change back to output
    PIN_setConfig(GPIOHandle, PIN_BM_ALL, EPD_SDA_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN);
    DEV_Digital_Write(EPD_CS_PIN, 1);

    return rc;
}

void EPD_SSD_ReadBytes(uint8_t* buf, uint8_t len)
{
    // Change SDA Pin to input 
    PIN_setConfig(GPIOHandle, PIN_BM_ALL, EPD_SDA_PIN | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN | PIN_PULLUP);
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);

    // read bytes 
    for (uint8_t i=0; i<len; i++) {
        buf[i] = DEV_SPI_ReadByte();
    }
    
    // change back to output
    PIN_setConfig(GPIOHandle, PIN_BM_ALL, EPD_SDA_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN);
    DEV_Digital_Write(EPD_CS_PIN, 1);
    //DEV_Digital_Write(EPD_DC_PIN, 0);
}

bool EPD_SSD_IsBusy()
{
    //LOW: idle, HIGH: busy
    return (DEV_Digital_Read(EPD_BUSY_PIN) == 1);
}

void EPD_SSD_WaitBusy(uint32_t ms)
{
    for (uint32_t i=0; i<ms; i+=10) {
        if (!EPD_SSD_IsBusy()) return;
        DEV_Delay_ms(10);
    }
}

// guess the LUT size
int EPD_SSD_LutDetect()
{
#define LUT_LEN_MAX    250
#define LUT_FILL       0xa5
    int i;
    EPD_SSD_SendCommand(0x32);
    for(i=0; i<LUT_LEN_MAX; i++) {
        EPD_SSD_SendData(LUT_FILL);
    }
    EPD_SSD_SendCommand(0x33);
    for(i=0; i<LUT_LEN_MAX; i++) {
        uint8_t c = EPD_SSD_ReadData();
        if (c != LUT_FILL) {
            break;
        }
    }
    return i;
}

// 0-100, for CR2450, 3000mv lithum battery.
uint8_t EPD_BATT_Percent(void)
{
    // Convert to from V to mV to avoid fractions.
    // Fractional part is in the lower 8 bits thus converting is done as follows:
    // (1/256)/(1/1000) = 1000/256 = 125/32
    // This is done most effectively by multiplying by 125 and then shifting
    // 5 bits to the right.
    uint32_t v = INTFRAC2MV(epd_battery);

    /* for cr2450, lithium battery,
     * 3000 (100%) - 2800 (60%)
     * 2800 ( 60%) - 2500 (20%)
     * 2500 ( 20%) - 2000 (0%)
     */
    if (v >= 3000) {
        return 100;
    } else if (v > 2800) {
        return (v-2800)*40/(3000-2800) + 60;
    } else if (v > 2500) {
        return (v-2500)*40/(2800-2500) + 20;
    } else if (v > 2000) {
        return (v-2000)*20/(2500-2000);
    }
    return 0;
}

/* RTC minor adjustment support.
 */
#if 1
#include <driverlib/aon_wuc.h>
#include <driverlib/../inc/hw_aux_wuc.h>

// collaborate rtc tick, slow down(<0), speed up(>0)
void RTC_SetCollaborate( int8_t rtc_collab )
{
    uint32_t subSecInc = (0x8000 + (int)rtc_collab) << 8;
    // Loading a new RTCSUBSECINC value is done in 5 steps:
    // 1. Write bit[15:0] of new SUBSECINC value to AUX_WUC_O_RTCSUBSECINC0
    // 2. Write bit[23:16] of new SUBSECINC value to AUX_WUC_O_RTCSUBSECINC1
    // 3. Set AUX_WUC_RTCSUBSECINCCTL_UPD_REQ
    // 4. Wait for AUX_WUC_RTCSUBSECINCCTL_UPD_ACK
    // 5. Clear AUX_WUC_RTCSUBSECINCCTL_UPD_REQ
    HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC0 ) = (( subSecInc       ) & 0xFFFF );
    HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1 ) = (( subSecInc >> 16 ) & 0xFF );

    HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL ) = 1;
    while( ! ( HWREGBITW( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL, AUX_WUC_RTCSUBSECINCCTL_UPD_ACK_BITN )));
    HWREG( AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL ) = 0;

    // save
    epd_rtc_collab = rtc_collab;
}

int8_t RTC_GetCollaborate( void ) 
{
    return epd_rtc_collab;
}
#endif

// Select a EPD
#if defined(EPD_2IN13_SSD1680)

#include "epd_2in13.c"

#elif defined(EPD_2IN9_SSD1680A)

#include "epd_2in9.c"

#else

#error "EPD not support."

#endif

// do command from BLE
void EPD_Command(const uint8_t *cmd, int cmd_len)
{
    // buffer recv position (max 64k)
    static uint16_t buf_cur = 0; 
    bool need_update = 0;

    switch (cmd[0]) {
        // clear screen 
        case EPD_CMD_CLR:
            need_update = 1;
            break;

        // display mode change
        case EPD_CMD_MODE:
            if (epd_mode != cmd[1]) {
                clock_last = 0;     // stop clock
                epd_mode = cmd[1];
                need_update = 1;
            }
            break;

        // recv epd_buffer
        case EPD_CMD_BUF:
            buf_cur = 0;
            // pass through
        case EPD_CMD_BUF_CONT: {
            uint16_t len = cmd_len - 1;
            len = MIN(len, (EPD_BUF_MAX - buf_cur));
            memcpy(&epd_buffer[buf_cur], &cmd[1], len);
            buf_cur += len;
            break;
        }

        // recv LUT
        case EPD_CMD_LUT:
            memcpy(lut_lite_ble, &cmd[1], sizeof(lut_lite_ble));
            break;

        // EPD Reset
        case EPD_CMD_RST:
            need_update = 1;
            break;

        // write BW ram
        case EPD_CMD_BW:
            need_update = 1;
            break;

        // write RED ram
        case EPD_CMD_RED:
            need_update = 1;
            break;

        // Display
        case EPD_CMD_DP:
            epd_step_data[0] = cmd[1];
            need_update = 1;
            break;

        // fill ram with color
        case EPD_CMD_FILL:
            epd_step_data[0] = cmd[1];
            need_update = 1;
            break;

        default:
            // ignore the command.
            return;
    }

    // save step
    epd_step = cmd[0];

    // notifiy EPDTask if needs
    if (need_update) {
        EPDTask_Update();
    }
}

// get EPD state
int EPD_State(uint8_t *buf, uint8_t size)
{
    return 0; 
}

// should be only called once!
void EPD_Init()
{
    GPIOHandle = PIN_open(&GPIOState, GPIOTable);      

    // test LUT size
    //lut_size = EPD_LUT_Detect();

    EPD_SSD_Init();
}

int EPD_Update()
{
    // update battery level
    epd_battery = AONBatMonBatteryVoltageGet(); 

    // update Display
    return EPD_SSD_Update();
}
