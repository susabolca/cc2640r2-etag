// EPD 2in9 SSD1680 298x128 BW

#define EPD_WIDTH       296 
#define EPD_HEIGHT      128
#define EPD_PAD_LEFT    0 
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
#include "font16.h"
#include "font24.h"
#include "font24zh.h"
#include "font80.h"

// One Bit Display
OBDISP obd = {0};

extern const uint8_t ucMirror[];

// fast full refresh.
static const uint8_t lut_full_bw[] = {
	0x80,	0x4A,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x40,	0x4A,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x80,	0x4A,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x40,	0x4A,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,
	0xf,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0xf,	0x0,	0x0,	0x4,	0x0,	0x0,	0x0,					
	0xf,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,					
	0x22,	0x22,	0x22,	0x22,	0x22,	0x22,	0x0,	0x0,	0x0,			
	0x22,	0x17,	0x41,	0x0,	0x32,	0x36	
};

// https://www.mdpi.com/2072-666X/12/5/578
#define VSS  (0b00)
#define VH1  (0b01)
#define VSL  (0b10)
#define VH2  (0b11)
#define V(a,b,c,d)  (a<<6|b<<4|c<<2|d)

#define W___ V(VSL, VSS, VSS, VSS)  // 80
#define B___ V(VH1, VSS, VSS, VSS)  // 40
#define _W__ V(VSS, VSL, VSS, VSS)  // 20
#define _B__ V(VSS, VH1, VSS, VSS)  // 10
#define BW__ V(VH1, VSL, VSS, VSS)  // 60

#if 1
// fast refresh for clock.
static const uint8_t lut_part_bw[] = {
//  VS [0-11] phase [ABCD]
//     0     1     2     3     4     5     6     7     8     9    10    11
    0x00, B___, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT0 B
    W___, W___, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT1 W
    B___, B___, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT2 B
    0x00, W___, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT3 W
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,     // LUT4 VCOM
// 60: 
//  TP A, TP B, SRAB, TP C, TP D, SRCD,   RP
    0x0a, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,   // 0
    0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,   // 1
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
    0x44, 0x00, 0x00, 0x00, 0x00, 0x00,
// 150: XON
    0x00, 0x00, 0x00,
// 153:
//  EOPT    VGH     VSH1    VSH2    VSL     VCOM
//  3F      03      04                      2C
//  22      -20v    15v     0v      -15v
    0x22,   0x17,   0x2d,   0x00,   0x32,   0x36    
};

#else
static const uint8_t lut_part_bw[]= {
	0x00,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x80,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x40,0x40,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x00,0x80,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x00,0x00,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x06,0x00,0x3,0x0,0x0,0x0,0x0,  
	0x01,0x00,0x3,0x0,0x0,0x0,0x0,
	0x01,0x00,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x22,0x22,0x22,0x22,0x22,0x22,0x0,0x0,0x0,
	0x22,0x17,0x41,0x00,0x32,0x36,
};
#endif

static void EPD_2IN9_Lut(const unsigned char *lut)
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

static int8_t EPD_2IN9_ReadTemp()
{
    int8_t rc;
    
    // soft reset
    //EPD_SSD_SendCommand(0x12);
    //EPD_SSD_WaitBusy(100);

    // Border Waveform
    EPD_SSD_SendCommand(0x3C);
    EPD_SSD_SendData(0x80);

    // Temperature sensor control
    EPD_SSD_SendCommand(0x18);
    EPD_SSD_SendData(0x80);       // 80: internal sensor 48: external sensor

    // Display update control
    EPD_SSD_SendCommand(0x22);
    EPD_SSD_SendData(0xb9);

    // Master Activation
    //EPD_SSD_SendCommand(0x20);
    //EPD_SSD_WaitBusy(100);

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

static void EPD_2IN9_BW(int width, int height, int left, int top, bool is_full)
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
    EPD_SSD_SendData(0x80);

#if 1
    if (!is_full) {
        EPD_SSD_SendCommand(0x37);
        EPD_SSD_SendData(0x00);
        EPD_SSD_SendData(0x00);
        EPD_SSD_SendData(0x00);
        EPD_SSD_SendData(0x00);
        EPD_SSD_SendData(0x00);
        EPD_SSD_SendData(0x40);
        EPD_SSD_SendData(0x00);
        EPD_SSD_SendData(0x00);
        EPD_SSD_SendData(0x00);
        EPD_SSD_SendData(0x00);

        EPD_SSD_SendCommand(0x22);
        EPD_SSD_SendData(0xC0);
        EPD_SSD_SendCommand(0x20);
        EPD_SSD_WaitBusy(100);
    }
#endif

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
    //EPD_SSD_SendCommand(0x18);
    //EPD_SSD_SendData(0x80);       // 80: internal sensor 48: external sensor
}

void EPD_2IN9_WriteRam(uint8_t *image, int width, int height, int left, int top, uint8_t is_red)
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
        EPD_2IN9_LoadImage(image, size, reg);
    } else {
        EPD_SSD_SendCommand(reg);
        for (int i = 0; i < size; i++) {
            EPD_SSD_SendData(is_red?0x00:0xff);
        }
    }
}

void EPD_2IN9_Display(uint8_t reg)
{
    EPD_SSD_SendCommand(0x22);
    EPD_SSD_SendData(reg);
    EPD_SSD_SendCommand(0x20);
    //EPD_SSD_WaitBusy();
}

void EPD_2IN9_Sleep(void)
{
    EPD_SSD_SendCommand(0x10);    //enter deep sleep
    EPD_SSD_SendData(0x01);       // 01: mode 1, 11: mode 2
}

#if 0
void EPD_2IN9_Clear(void)
{
    // wakeup EPD
    EPD_SSD_Reset();

    // do reset
    EPD_2IN9_SoftReset();

    // write white to ram
    EPD_2IN9_BW(EPD_WIDTH, EPD_HEIGHT, 0, 0);
    EPD_2IN9_WriteRam(NULL, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);
    EDP_2IN9_WriteRam(NULL, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);

    // full display
    EPD_2IN9_Display(0xf7);

    // wait & sleep
    EPD_SSD_WaitBusy(15*1000);
    EPD_2IN9_Sleep();
}
#endif

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

    // BLE dev name
    extern void getBleAdvName(char* buf);
    getBleAdvName(buf);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 2, 128, buf, 1);

    // battery voltage
    uint8_t v = EPD_BATT_Percent(); 
    System_snprintf(buf, 32, "%3u%%", v);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 250, 128, buf, 1);

    // date
    System_snprintf(buf, 32, "%u-%02u-%02u", 1900+l->tm_year, l->tm_mon+1, l->tm_mday);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_24, 2, 22, buf, 1);

    // week
    {
        char fmt[] = {0x37, 0x38, 0x30 + l->tm_wday, '\0'};   // chinese week day 
        obdWriteStringCustom(&obd, (GFXfont *)&Hei24pt, 158, 20, fmt, 1);
    }

    // temp
    epd_temperature = EPD_2IN9_ReadTemp();
    const char fmt[] = {'%', '3', 'u', 0xb0, 'c', '\0'};   // degrees celsius
    System_snprintf(buf, 32, fmt, epd_temperature);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_24, 236, 22, buf, 1);

    // time
    System_snprintf(buf, 32, "%02d:%02d", l->tm_hour, l->tm_min);
    obdWriteStringCustom(&obd, (GFXfont *)&DSEG7_Classic_Regular_80, 8, 24+80+5, buf, 1);

    // edian and invent 
    for (int i=0; i<sizeof(epd_buffer); i++) {
        uint8_t c = epd_buffer[i];
        epd_buffer[i] = ~ucMirror[c];
    }

    // full or fast update 
    EPD_2IN9_BW(EPD_WIDTH, EPD_HEIGHT, 0, 0, upd_type == 0);
    if (upd_type == 1) {
        EPD_2IN9_Lut(lut_full_bw);
    } else if (upd_type == 2){
        EPD_2IN9_Lut(lut_part_bw);
    }

    EPD_2IN9_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);
    if (upd_type != 2) {
        EPD_2IN9_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);
    }

    // show
    EPD_2IN9_Display(upd_type==0?0xf7:0xcf);    // c7: by REG  f7: by OTP   b1: no display
    EPD_SSD_WaitBusy(15*1000);
    EPD_2IN9_Sleep();
    return;
}

void EPD_SSD_Update_Test(void)
{
    static uint8_t sec = 0;
    static uint8_t tick = 0;

    if (++tick < 3) {
        return;
    }
    tick = 0;
    
    char upd_type = 2;  // 0: full, 1: fast, 2: partial
    if (sec == 0) {
        upd_type = 0;
    }

    // wakeup EPD
    EPD_SSD_Reset();

    char buf[32];
    obdCreateVirtualDisplay(&obd, EPD_WIDTH, EPD_HEIGHT, epd_buffer);
    obdFill(&obd, 0, 0);

    // string
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 0, 22, "Only Test Don't Commit.", 1);
    
    // time
    System_snprintf(buf, 32, "%02d:%02d", sec, sec);
    obdWriteStringCustom(&obd, (GFXfont *)&DSEG7_Classic_Regular_80, 8, 24+80+5, buf, 1);
    sec = (sec + 1) % 60;
    
    // edian and invent 
    for (int i=0; i<sizeof(epd_buffer); i++) {
        uint8_t c = epd_buffer[i];
        epd_buffer[i] = ~ucMirror[c];
    }

    // full or fast update 
    EPD_2IN9_BW(EPD_WIDTH, EPD_HEIGHT, 0, 0, upd_type == 0);
#if 1
    if (upd_type == 2){
        EPD_2IN9_Lut(lut_part_bw);
    }
#endif
    
    EPD_2IN9_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 0);
    if (upd_type == 0) {
        EPD_2IN9_WriteRam(epd_buffer, EPD_WIDTH, EPD_HEIGHT, 0, 0, 1);
    }
    
    // show
    EPD_2IN9_Display(upd_type==0?0xf7:0xcf);    // c7: by REG  f7: by OTP   b1: no display 
    EPD_SSD_WaitBusy(15*1000);
    EPD_2IN9_Sleep();
    return;

}

int EPD_SSD_Update(void)
{
    EPD_SSD_Update_Clock();

    // as we only support clock for 2in13,
    // need to return 1 to keep the timer running
    return 1;
}

void EPD_SSD_Init(void)
{
    EPD_SSD_Reset();

    RTC_SetCollaborate(epd_rtc_collab);
}
