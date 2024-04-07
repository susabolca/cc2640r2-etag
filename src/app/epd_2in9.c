// EPD 2in9 SSD1680A 296x128

#define EPD_SSD1680A
#define EPD_WIDTH   296
#define EPD_HEIGHT  128

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
#include "font16.h"
#include "font24.h"
#include "font80.h"

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
#define _VS(x) x<<6
#define VSS  _VS(0b00)
#define VSH1 _VS(0b01)
#define VSL  _VS(0b10)
#define VSH2 _VS(0b11)

static const uint8_t lut_full_bwr[] = {
//  0: LUTC x 7 
//  RP      A           B           C           D           SRAB    SRCD
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
//  56: LUTR x 7
    0x1,    VSL|0x2f,   0x0,        VSH2|0x3f,  0x0,        0x1,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
//  112: LUTW x 7 
    0x1,    VSL|0x3f,   0x0,        0x0,        0x0,        0x2,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
//  168: LUTB x 7
    0x1,    VSH1|0x2f,  0x0,        0x0,        0x0,        0x1,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,

// FR 
    0x04, // 2: 50hz, 3: 75Hz, 4: 100Hz, 5: 125Hz

// XON
    0x0, 0x0,

//  EOPT    VGH     VSH1    VSH2    VSL     VCOM
//  3F      03      04                      2C
//  22      -20v    15v     3v      -15v
    0x22,   0x17,   0x41,   0x94,   0x32,   0x36    
};

static void EPD_2IN9_Lut(const unsigned char *lut)
{
    EPD_SSD_SendCommand(0x32);
    for(int i=0; i<227; i++) {
        EPD_SSD_SendData(lut[i]);
    }

    // gate voltage
    EPD_SSD_SendCommand(0x3F);
    EPD_SSD_SendData(*(lut+227));

    EPD_SSD_SendCommand(0x03);
    EPD_SSD_SendData(*(lut+228));

    // source voltage
    EPD_SSD_SendCommand(0x04);
    EPD_SSD_SendData(*(lut+229));    // VSH
    EPD_SSD_SendData(*(lut+230));    // VSH2
    EPD_SSD_SendData(*(lut+231));    // VSL

    EPD_SSD_SendCommand(0x2C);
    EPD_SSD_SendData(*(lut+232));
}

static void EPD_2IN9_SoftReset()
{
    EPD_SSD_SendCommand(0x12); // soft reset
    EPD_SSD_WaitBusy(100);
}

static int8_t EPD_2IN9_ReadTemp()
{
    int8_t rc;
    
    //EPD_SSD_SendCommand(0x12); // soft reset
    //EPD_SSD_WaitBusy(100);

    // Border Waveform
    EPD_SSD_SendCommand(0x3C);
    EPD_SSD_SendData(0x05);

    // Temperature sensor control
    EPD_SSD_SendCommand(0x18);
    EPD_SSD_SendData(0x80);       // 80: internal sensor 48: external sensor

    // Display update control
    EPD_SSD_SendCommand(0x22);
    EPD_SSD_SendData(0xb1);   // full: 0xf7

    // Master Activation
    EPD_SSD_SendCommand(0x20);
    EPD_SSD_WaitBusy(100);

    // read temperature
    EPD_SSD_SendCommand(0x1b);
    rc = EPD_SSD_ReadData();

    return rc;
}

static void EPD_2IN9_LoadImage(uint8_t *image, int size, uint8_t cmd)
{
    EPD_SSD_SendCommand(cmd);
    for (int i = 0; i < size; i++) {
        EPD_SSD_SendData(image[i]);
    }
}

static void EPD_2IN9_BWR(int width, int height, int left, int top)
{
    // left up corner
    int w0 = left;
    int h0 = top/8;
    
    // right bottom corner
    int w1 = w0 + width - 1;
    int h1 = h0 + height/8 - 1;
    
    // soft reset
    //EPD_SSD_SendCommand(0x12); 
    //EPD_SSD_WaitBusy(100);

    // Border Waveform
    EPD_SSD_SendCommand(0x3C); 
    EPD_SSD_SendData(0x05);

    // Driver output control
    EPD_SSD_SendCommand(0x01);
    EPD_SSD_SendData(0x28);  // mux=0x128(296)
    EPD_SSD_SendData(0x01);
    EPD_SSD_SendData(0x01);  // gd=0, sm=0, tb=1
    
    EPD_SSD_SendCommand(0x11); //data entry mode
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

void EPD_2IN9_WriteRam(uint8_t *image, int width, int height, int left, int top, uint8_t is_red)
{
    // data size in bytes
    int size = width*height/8;

    // Set Ram X address
    EPD_SSD_SendCommand(0x4E); 
    EPD_SSD_SendData(top & 0xff);

    // Set Ram Y address
    EPD_SSD_SendCommand(0x4F); 
    EPD_SSD_SendData(left & 0xff);
    EPD_SSD_SendData(left >> 8);

    const uint8_t reg = is_red ? 0x26 : 0x24;     // BW: 0x24, Red: 0x26 
    if (image) {
        EPD_2IN9_LoadImage(image, size, reg);
    } else {
        EPD_SSD_SendCommand(reg);
        for (int i = 0; i < size; i++) {
            EPD_SSD_SendData(is_red ? 0x00 : 0xff);
        }
    }
}

void EPD_2IN9_Display(uint8_t reg)
{
    EPD_SSD_SendCommand(0x22);
    EPD_SSD_SendData(reg);
    EPD_SSD_SendCommand(0x20);
}

void EPD_2IN9_Sleep(void)
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
    now += utc_offset_mins * 60;
    struct tm *l = localtime(&now);

    // wakeup EPD
    EPD_SSD_Reset();

    // create obd
    char buf[32];
    obdCreateVirtualDisplay(&obd, EPD_WIDTH, EPD_HEIGHT, epd_buffer);
    obdFill(&obd, 0, 0);

    // soft reset
    EPD_2IN9_SoftReset();

    // BLE dev name
    extern void getBleAdvName(char* buf);
    getBleAdvName(buf);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 2, 128, buf, 1);

    // battery voltage
    uint8_t v = EPD_BATT_Percent(); 
    System_snprintf(buf, 32, "%3u%%", v);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 250, 128, buf, 1);

    // date
    const char *wstr[]={"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
    System_snprintf(buf, 32, "%u-%02u-%02u %s", 1900+l->tm_year, l->tm_mon+1, l->tm_mday, wstr[l->tm_wday]);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_24, 2, 22, buf, 1);

    // temp
    epd_temperature = EPD_2IN9_ReadTemp();
    const char fmt[] = {'%', '3', 'u', 0xb0, 'c', '\0'};   // degrees celsius
    System_snprintf(buf, 32, fmt, epd_temperature);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_24, 236, 22, buf, 1);

    // time
    System_snprintf(buf, 32, "%02d:%02d", l->tm_hour, l->tm_min);
    obdWriteStringCustom(&obd, (GFXfont *)&DSEG7_Classic_Regular_80, 8, 24+80+5, buf, 1);

    // endian and invent
    for (int i=0; i<sizeof(epd_buffer); i++) {
        uint8_t c = epd_buffer[i];
        epd_buffer[i] = ~ucMirror[c];
    }

    // full update every 30 mins
    bool full_upd = (l->tm_min == 0) ? true : false;
    EPD_2IN9_BWR(EPD_WIDTH, EPD_HEIGHT, 0, 0);
    if (!full_upd) EPD_2IN9_Lut(lut_full_bwr);
    EPD_2IN9_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);
    EPD_2IN9_WriteRam(NULL, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);

    // show
    EPD_2IN9_Display(full_upd ? 0xf7 : 0xc7);    // c7: by REG  f7: by OTP   b1: no display
    //need_sleep = 1;

    EPD_SSD_WaitBusy(15 * 1000);
    EPD_2IN9_Sleep();
    return;
}

void EPD_SSD_Init(void)
{
    EPD_SSD_Reset();
}

