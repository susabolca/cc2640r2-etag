// EPD 2in13 SSD1680 250x122

#define EPD_WIDTH       250 
#define EPD_HEIGHT      128
#define EPD_PAD_LEFT    46
#define EPD_PAD_TOP     8

#include <ti/drivers/PIN.h>
#include <ti/sysbios/hal/Seconds.h> // Seconds_get
//#include <ti/sysbios/knl/Task.h>    // System_sleep
#include <xdc/runtime/System.h>     // snprintf
#include <driverlib/aon_batmon.h>   // battery & temp
#include <util.h>

#include "epd_driver.h"

#include <time.h>       // time
#include <stdint.h>     // uint8_t 
#include <string.h>     // memset 

// OBD
#include "OneBitDisplay.h"
#include "font64.h"
#include "font24.h"
#include "font24zh.h"
#include "font16.h"

// One Bit Display
OBDISP obd = {0};

extern const uint8_t ucMirror[];

// https://www.mdpi.com/2072-666X/12/5/578
#define VSS  (0b00)
#define VH1  (0b01)
#define VSL  (0b10)
#define VH2  (0b11)
#define V(a,b,c,d)  (a<<6|b<<4|c<<2|d)

#define W___ V(VSL, VSS, VSS, VSS)
#define R___ V(VH2, VSS, VSS, VSS)
#define B___ V(VH1, VSS, VSS, VSS)
#define _B__ V(VSS, VH1, VSS, VSS)
#define _W__ V(VSS, VSL, VSS, VSS)
#define _R__ V(VSS, VH2, VSS, VSS)
#define BW__ V(VH1, VSL, VSS, VSS)
#define RW__ V(VH2, VSL, VSS, VSS)

// fast refresh for clock.
static const uint8_t lut_fast_bw[] = {
//  VS [0-11] phase [ABCD]
//     0     1     2     3     4     5     6     7     8     9    10    11
    B___, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT0 B
    _W__, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT1 W
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT2 R
    0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT3 NC?
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT4 VCOM
// 60: 
//  TP A, TP B, SRAB, TP C, TP D, SRCD,   RP
    0x05, 0x10, 0x06, 0x00, 0x00, 0x00, 0x00,   // 0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 1
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 2
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 3
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 4
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 5
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 6
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 9
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 10
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 11
// 144: FR  25-200Hz
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
// 150: XON
    0x00, 0x00, 0x00,
// 153:
//  EOPT    VGH     VSH1    VSH2    VSL     VCOM
//  3F      03      04                      2C
//  22      -20v    15v     3v      -15v
    0x22,   0x17,   0x41,   0x94,   0x32,   0x36    
};

// fast full refresh.
static const uint8_t lut_full_bwr[] = {
//  VS [0-11] phase [ABCD]
//     0     1     2     3     4     5     6     7     8     9    10    11
    W___, BW__, B___, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT0 B
    _B__, BW__, _W__, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT1 W
    _R__, RW__, _W__, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT2 R
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT3 NC?
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT4 VCOM
// 60: 
//  TP A, TP B, SRAB, TP C, TP D, SRCD,   RP
    0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x02,   // 0
    0x09, 0x09, 0x01, 0x00, 0x00, 0x00, 0x02,   // 1
    0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x02,   // 2
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 3
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 4
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 5
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 6
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 9
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 10
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 11
// 144: FR  25-200Hz
    0x22, 0x20, 0x00, 0x00, 0x00, 0x00,
// 150: XON
    0x00, 0x00, 0x00,
// 153:
//  EOPT    VGH     VSH1    VSH2    VSL     VCOM
//  3F      03      04                      2C
//  22      -20v    15v     3v      -15v
    0x22,   0x17,   0x41,   0x94,   0x32,   0x36    
};

static void EPD_2IN13_Lut(const unsigned char *lut)
{
    EPD_SSD_SendCommand(0x32);
    for(int i=0; i<153; i++) {
        EPD_SSD_SendData(lut[i]);
    }

    // gate voltage
    EPD_SSD_SendCommand(0x3F);
    EPD_SSD_SendData(*(lut+153));

    EPD_SSD_SendCommand(0x03);
    EPD_SSD_SendData(*(lut+154));

    // source voltage
    EPD_SSD_SendCommand(0x04);
    EPD_SSD_SendData(*(lut+155));    // VSH
    EPD_SSD_SendData(*(lut+156));    // VSH2
    EPD_SSD_SendData(*(lut+157));    // VSL

    EPD_SSD_SendCommand(0x2C);
    EPD_SSD_SendData(*(lut+158));
}

static void EPD_2IN13_Lut_ById(int id)
{
#if 0
    if (id == 1) {
        EPD_2IN13_Lut(lut_fast_bw);
    } 
#endif    
    EPD_2IN13_Lut(lut_full_bwr);
}

static void EPD_2IN13_SoftReset()
{
    EPD_SSD_SendCommand(0x12); // soft reset
    EPD_SSD_WaitBusy(100);
}

static int8_t EPD_2IN13_ReadTemp()
{
    int8_t rc;
    
    // soft reset
    //EPD_SSD_SendCommand(0x12);
    //EPD_SSD_WaitBusy(100);

    // Border Waveform
    //EPD_SSD_SendCommand(0x3C);
    //EPD_SSD_SendData(0x05);

    // Temperature sensor control
    EPD_SSD_SendCommand(0x18);
    EPD_SSD_SendData(0x80);       // 80: internal sensor 48: external sensor

    // Display update control
    EPD_SSD_SendCommand(0x22);
    EPD_SSD_SendData(0xb1);

    // Master Activation
    EPD_SSD_SendCommand(0x20);
    EPD_SSD_WaitBusy(100);

    // read temperature
    EPD_SSD_SendCommand(0x1b);
    rc = EPD_SSD_ReadData();

    return rc;
}

static void EPD_2IN13_LoadImage(uint8_t *image, int size, uint8_t cmd)
{
    EPD_SSD_SendCommand(cmd);
    for (int i = 0; i < size; i++) {
        EPD_SSD_SendData(image[i]);
    }
}

static void EPD_2IN13_BWR(int width, int height, int left, int top)
{
    // left up corner
    int w0 = EPD_PAD_LEFT + left;
    int h0 = EPD_PAD_TOP + top;
    
    // right bottom corner
    int w1 = w0 + width - 1;
    int h1 = h0 + height/8 - 1;
    
    // soft reset
    //EPD_SSD_SendCommand(0x12); 
    //EPD_SSD_WaitBusy();

    // Border Waveform
    EPD_SSD_SendCommand(0x3C); 
    EPD_SSD_SendData(0x05);

    // Driver output control
    EPD_SSD_SendCommand(0x01);
    EPD_SSD_SendData(0x27);  // mux=0x127(296)
    EPD_SSD_SendData(0x01);
    EPD_SSD_SendData(0x01);  // gd=0, sm=0, tb=1

    // Data entry mode    
    EPD_SSD_SendCommand(0x11);
    EPD_SSD_SendData(0x07);   // am=1, id=11

    // Set RAM X Address Start/End
    EPD_SSD_SendCommand(0x44);
    EPD_SSD_SendData(h0 & 0xff);
    EPD_SSD_SendData(h1 & 0xff);

    // Set RAM Y Address Start/End
    EPD_SSD_SendCommand(0x45); //set Ram-Y address start/end position
    EPD_SSD_SendData(w0 & 0xff);
    EPD_SSD_SendData(w0 >> 8);
    EPD_SSD_SendData(w1 & 0xff);
    EPD_SSD_SendData(w1 >> 8);
    
    // Temperature sensor control
    EPD_SSD_SendCommand(0x18);
    EPD_SSD_SendData(0x80);       // 80: internal sensor 48: external sensor
}

void EPD_2IN13_WriteRam(uint8_t *image, int width, int height, int left, int top, uint8_t is_red)
{
    // data size in bytes
    int size = width*height/8;

    // Set Ram X address
    uint8_t x = (top + 8) / 8;
    EPD_SSD_SendCommand(0x4E); 
    EPD_SSD_SendData(x);

    // Set Ram Y address
    uint16_t y = left + EPD_PAD_LEFT;
    EPD_SSD_SendCommand(0x4F); 
    EPD_SSD_SendData( y & 0xff);
    EPD_SSD_SendData( y >> 8);

    const uint8_t reg = is_red ? 0x26 : 0x24;     // BW: 0x24, Red: 0x26 
    if (image) {
        EPD_2IN13_LoadImage(image, size, reg);
    } else {
        EPD_SSD_SendCommand(reg);
        for (int i = 0; i < size; i++) {
            EPD_SSD_SendData(is_red ? 0x00 : 0xff);
        }
    }
}

void EPD_2IN13_Display(uint8_t reg)
{
    EPD_SSD_SendCommand(0x22);
    EPD_SSD_SendData(reg);
    EPD_SSD_SendCommand(0x20);
    //EPD_SSD_WaitBusy();
}

void EPD_2IN13_Sleep(void)
{
    EPD_SSD_SendCommand(0x10);    //enter deep sleep
    EPD_SSD_SendData(0x01);       // 01: mode 1, 11: mode 2
}

void EPD_2IN13_Clear(void)
{
    // wakeup EPD
    EPD_SSD_Reset();

    // do reset
    EPD_2IN13_SoftReset();

    // write white to ram
    EPD_2IN13_BWR(EPD_WIDTH, EPD_HEIGHT, 0, 0);
    EPD_2IN13_WriteRam(NULL, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);
    EPD_2IN13_WriteRam(NULL, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);

    // full display
    EPD_2IN13_Display(0xf7);

    // wait & sleep
    EPD_SSD_WaitBusy(15*1000);
    EPD_2IN13_Sleep();
}

void EPD_SSD_Update_Clock(void)
{
    time_t now;
    time(&now);

    // adjust TZ offset
    now += utc_offset_mins * 60;

    // get localtime
    struct tm *l = localtime(&now);

    if (clock_last == l->tm_min) {
        return;
    }
 
    //bool full_upd = (clock_last > 60 || (l->tm_hour== 0 && l->tm_min == 0)) ? true : false;
    char upd_type = 2;  // 0: full, 1: fast, 2: partial
    if (clock_last > 60 || (l->tm_hour== 0 && l->tm_min == 0)) {
        upd_type = 0;
    } 
    else if (l->tm_min == 0 || l->tm_min == 30) {
        upd_type = 1;
    }

    // clock started.
    clock_last = l->tm_min;

    // wakeup EPD
    EPD_SSD_Reset();

    char buf[32];
    obdCreateVirtualDisplay(&obd, EPD_WIDTH, EPD_HEIGHT, epd_buffer);
    obdFill(&obd, 0, 0);

#if 0
    // date
    const char *wstr[]={"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
    System_snprintf(buf, 32, "%u-%02u-%02u %s", 1900+l->tm_year, l->tm_mon+1, l->tm_mday, wstr[l->tm_wday]);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_24, 0, 24, buf, 1);
#endif

    // BLE dev name
#if 1    
    extern void getBleAdvName(char* buf);
    getBleAdvName(buf);
#else    
    extern uint8_t mac_address[6];
    System_snprintf(buf, 32, "%02x%02x%02x", mac_address[3], mac_address[4], mac_address[5]);
#endif    
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 0, 118, buf, 1);

    // temperature
    epd_temperature = EPD_2IN13_ReadTemp();
    char fmte[] = {'%', '3', 'u', 0xb0, 'c', '\0'};   // degrees celsius
    System_snprintf(buf, 32, fmte, epd_temperature);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 154, 118, buf, 1);

    // battery
    uint8_t v = EPD_BATT_Percent(); 
    System_snprintf(buf, 32, "%3u%%", v);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 200, 118, buf, 1);

    // time
    System_snprintf(buf, 32, "%02d:%02d", l->tm_hour, l->tm_min);
    obdWriteStringCustom(&obd, (GFXfont *)&DSEG7_Classic_Regular_64, 12, 28+70, buf, 1);

    // date
    System_snprintf(buf, 32, "%u-%02u-%02u", 1900+l->tm_year, l->tm_mon+1, l->tm_mday);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_24, 0, 24, buf, 1);

    // week
    char fmt[] = {0x37, 0x38, 0x30 + l->tm_wday, '\0'};   // chinese week day 
    obdWriteStringCustom(&obd, (GFXfont *)&Hei24pt, 158, 21, fmt, 1);

    // edian and invent 
    for (int i=0; i<sizeof(epd_buffer); i++) {
        uint8_t c = epd_buffer[i];
        epd_buffer[i] = ~ucMirror[c];
    }

    // full or fast update 
    EPD_2IN13_BWR(EPD_WIDTH, EPD_HEIGHT, 0, 0);
    if (upd_type == 2) 
        EPD_2IN13_Lut(lut_fast_bw);
    else if (upd_type == 1) 
        EPD_2IN13_Lut(lut_full_bwr);
    EPD_2IN13_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);
    EPD_2IN13_WriteRam(NULL, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);

    // show
    EPD_2IN13_Display(upd_type?0xc7:0xf7);    // c7: by REG  f7: by OTP   b1: no display 
    EPD_SSD_WaitBusy(15*1000);
    EPD_2IN13_Sleep();
    return;
}

void EPD_2IN13_Update_Image()
{
    //static uint8_t last_step = 0;
    uint8_t step = epd_step;

    switch (step) {
        case EPD_CMD_CLR:
            EPD_2IN13_Clear();
            break;

        case EPD_CMD_MODE:
            EPD_2IN13_Clear();
            break;

        case EPD_CMD_RST: // reset
            // wakeup EPD
            EPD_SSD_Reset();
            // soft reset
            EPD_2IN13_SoftReset();
            // ready BWR 
            EPD_2IN13_BWR(EPD_WIDTH, EPD_HEIGHT, 0, 0);
            break;
            
        case EPD_CMD_BW: // write BW ram
            EPD_2IN13_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);
            break;

        case EPD_CMD_RED: // write Red ram
            EPD_2IN13_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);
            break;

        case EPD_CMD_FILL: { // write ram with color
            uint8_t color = epd_step_data[0];
            if (color == 1) {   // red
                memset(epd_buffer, 0xff, EPD_WIDTH*EPD_HEIGHT/8);
                EPD_2IN13_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);
            } else {
                memset(epd_buffer, 0, EPD_WIDTH*EPD_HEIGHT/8);
                EPD_2IN13_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);
            }
            break;
        }

        case EPD_CMD_DP: { // master
            // display lut select
            // TBD: only full display.
            uint8_t dpm = 0; //epd_step_data[0];
            if (dpm == 1) { // lut1
                EPD_2IN13_Lut_ById(0);
            } else if (dpm == 2) {  // lut2
                EPD_2IN13_Lut_ById(1);
            } else if (dpm == 3) {  // lut3
                EPD_2IN13_Lut_ById(2);
            } else if (dpm == 0xff) {   // user ble lut
                EPD_2IN13_Lut(ble_data);
            }
            
            // otherwise using full lut.
            EPD_2IN13_Display(dpm ? 0xc7: 0xf7);    // fast display
            EPD_SSD_WaitBusy(15 * 1000);
            EPD_2IN13_Sleep();
            break;
        }
    }

    // done
    if (step == epd_step) {
        epd_step = EPD_CMD_NC;
    }
}
int EPD_SSD_Update(void)
{
    if (epd_mode == EPD_MODE_IMG) {
        EPD_2IN13_Update_Image();
        // stop tick clock
        return 0;
    }

    EPD_SSD_Update_Clock();
    return 1;
}

void EPD_SSD_Init(void)
{
    EPD_SSD_Reset();

    RTC_SetCollaborate(epd_rtc_collab);
}
