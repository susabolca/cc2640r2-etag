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

// OBD
#include "OneBitDisplay.h"
#include "font64.h"
#include "font24.h"
#include "font16.h"

// One Bit Display
OBDISP obd;

extern const uint8_t ucMirror[];

/*
 * <int.frac> format size (3.8) bits.
 * int for 0-3 voltage
 * frac each means 1/256 of ONE voltage
 */
#define INTFRAC_V(x)    (x>>8)
#define INTFRAC_mV(x)   ((x&0xff)*1000/256)
#define INTFRAC2MV(x)   (INTFRAC_mV(x)+(INTFRAC_V(x)*1000))

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
    0x44, 0x40, 0x00, 0x00, 0x00, 0x00,
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

static uint8_t EPD_2IN13_ReadTemp()
{
    uint8_t rc;
    
    // soft reset
    EPD_SSD_SendCommand(0x12);
    EPD_SSD_WaitBusy();

    // Border Waveform
    EPD_SSD_SendCommand(0x3C);
    EPD_SSD_SendData(0x05);

    // Temperature sensor control
    EPD_SSD_SendCommand(0x18);
    EPD_SSD_SendData(0x80);       // 80: internal sensor 48: external sensor

    // Display update control
    EPD_SSD_SendCommand(0x22);
    EPD_SSD_SendData(0xb1);

    // Master Activation
    EPD_SSD_SendCommand(0x20);
    EPD_SSD_WaitBusy();

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
    EPD_SSD_SendCommand(0x12); 
    EPD_SSD_WaitBusy();

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

void EPD_SSD_Update(void)
{
    static time_t last = 0;
    time_t now = time(NULL);
    if (last && ((now % 60) != 0)) {
        return;
    }
    last = now;
    // TBD: timezone +8
    now += (3600 * 8);
    struct tm *l = localtime(&now);

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

    // battery voltage
    uint32_t v = AONBatMonBatteryVoltageGet();
    uint8_t t = EPD_2IN13_ReadTemp();
    System_snprintf(buf, 32, "%3uc %u.%uv", t, INTFRAC_V(v), INTFRAC_mV(v)/100);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 164, 118, buf, 1);

    // time
    System_snprintf(buf, 32, "%02d:%02d", l->tm_hour, l->tm_min);
    obdWriteStringCustom(&obd, (GFXfont *)&DSEG7_Classic_Regular_64, 12, 28+70, buf, 1);

    // endian and invent
    for (int i=0; i<sizeof(epd_buffer); i++) {
        uint8_t c = epd_buffer[i];
        epd_buffer[i] = ~ucMirror[c];
    }

    // full update every 30 mins
    //bool full_upd = true;
    bool full_upd = (l->tm_min == 0 || l->tm_min == 30) ? true : false;
    EPD_2IN13_BWR(EPD_WIDTH, EPD_HEIGHT, 0, 0);
    if (!full_upd) EPD_2IN13_Lut(lut_full_bwr);
    EPD_2IN13_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);

    // red
    obdFill(&obd, 0, 0);

    // date
    const char *wstr[]={"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
    System_snprintf(buf, 32, "%u-%02u-%02u %s", 1900+l->tm_year, l->tm_mon+1, l->tm_mday, wstr[l->tm_wday]);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_24, 0, 24, buf, 1);
    for (int i=0; i<sizeof(epd_buffer); i++) {
        uint8_t c = epd_buffer[i];
        epd_buffer[i] = ucMirror[c];
    }

    EPD_2IN13_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);

    // show 
    EPD_2IN13_Display(full_upd ? 0xf7 : 0xc7);    // c7: by REG  f7: by OTP   b1: no display 
    EPD_SSD_WaitBusy();
    EPD_2IN13_Sleep();
    return;
}

void EPD_SSD_Init(void)
{

}
