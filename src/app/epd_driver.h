#ifndef _EPD_DRIVER_H_
#define _EPD_DRIVER_H_

#include <stdint.h>     // uint8_t

// select a EPD device 
//#define EPD_2IN13_SSD1680
#define EPD_2IN9_SSD1680A

// pin configuration
#define EPD_BUSY_PIN            IOID_10
#define EPD_RST_PIN             IOID_9
#define EPD_DC_PIN              IOID_8
#define EPD_CS_PIN              IOID_7
#define EPD_SCL_PIN             IOID_6
#define EPD_SDA_PIN             IOID_5

// EPD Frame buffer MAX size
#define EPD_BUF_MAX (300 * 128 / 8)
extern uint8_t epd_buffer[EPD_BUF_MAX];

// local time to UTC offset, in +/- minutes.
extern int32_t utc_offset_mins;

// public sensor values
extern int8_t epd_temperature;          // in degree celsius, +/-127
extern uint16_t epd_battery;            // in (3.8) frac, minium value

// display mode 
extern uint8_t epd_mode;
#define EPD_MODE_CLOCK      0           // realtime clock 
#define EPD_MODE_IMG        1           // static image 

// TBD: split image display to steps in a dirty way.
extern uint8_t epd_step;
#define EPD_STEP_DATA_LEN       4
extern uint8_t epd_step_data[EPD_STEP_DATA_LEN];

// saved clock tick.
extern uint8_t clock_last;

// ble data buffer
#define BLE_DATA_MAX    256
extern uint8_t ble_data[BLE_DATA_MAX];  // ble data, a 256 bytes buffer
extern uint8_t ble_data_len;            // indicate the length of ble_data
extern uint8_t ble_data_cur;            // indicate the current position of ble_data

// EPD service commands
enum EPD_CMD {
    EPD_CMD_NC = 0,     // nothing

    // keep order
    EPD_CMD_CLR,        // clear screen
    EPD_CMD_MODE,       // set display mode
    EPD_CMD_BUF,        // first receive to epd_buffer
    EPD_CMD_BUF_CONT,   // continue write to epd_buffer 
    EPD_CMD_LUT,        // set lut
    EPD_CMD_RST,        // reset EPD
    EPD_CMD_BW,         // write EPD Black/White
    EPD_CMD_RED,        // write EPD Red
    EPD_CMD_DP,         // EPD display
    EPD_CMD_FILL,       // fill ram with color 
    
    EPD_CMD_BUF_PUT,    // put data to buffer 
    EPD_CMD_BUF_GET,    // get data from buffer 
    EPD_CMD_SNV_WRITE,  // write ble_data to snv 
    EPD_CMD_SNV_READ,   // read snv to ble_data 
    
    EPD_CMD_SAVE_CFG,   // save configuration to snv, rtc collaborate, utc offset, etc ...
    
    EPD_CMD_MAX
};

// CC2640r2 SNV user area, 0x80 - 0x8f
enum EPD_SNV {
    EPD_SNV_CFG = 0x80, // configuration
    EPD_SNV_LUT1,       // LUT1, fast bw
    EPD_SNV_LUT2,       // LUT2, gray bwr
    EPD_SNV_LUT3,       // LUT3

    EPD_SNV_MAX
};

// EPD configuration in SNV
struct epd_snv_cfg {
    union {
        struct {
            uint8_t mode;       // saved display mode
            int8_t  rtc_collab; // saved rtc collaborate
            int16_t utc_offset; // saved utc offset in minutes
        } cfg;
        uint8_t raw[32]; 
    } u;
};

/*
 * <int.frac> format size (3.8) bits.
 * int for 0-3 voltage
 * frac each means 1/256 of ONE voltage
 */
#define INTFRAC_V(x)    (x>>8)
#define INTFRAC_mV(x)   ((x&0xff)*125/32)
#define INTFRAC2MV(x)   (INTFRAC_mV(x)+(INTFRAC_V(x)*1000))

// Macro
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? a : b)
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? a : b)
#endif

// the driver public,
void EPD_Init();
int EPD_Update();
void EPD_SSD_Reset();
void EPD_SSD_SendCommand(uint8_t reg);
void EPD_SSD_SendData(uint8_t data);
uint8_t EPD_SSD_ReadData();
void EPD_SSD_ReadBytes(uint8_t* buf, uint8_t len);
bool EPD_SSD_IsBusy();
void EPD_SSD_WaitBusy(uint32_t ms);
uint8_t EPD_BATT_Percent();

// RTC
void RTC_SetCollaborate( int8_t rtc_collab );
int8_t RTC_GetCollaborate();

// SNV
int EPD_SNV_LoadCfg();
int EPD_SNV_SaveCfg();
int EPD_SNV_LoadLut(int index, uint8_t *lut, int len);
int EPD_SNV_SaveLut(int index, const uint8_t *lut, int len);

// API for EPD commands
void EPD_Command(const uint8_t *cmd, int cmd_len);

// for epd_inch.c shoule implement,
void EPD_SSD_Init();
int EPD_SSD_Upate();

#endif 
