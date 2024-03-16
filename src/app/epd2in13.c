
#include "epd2in13.h"

#include <ti/sysbios/knl/Task.h>

#include "board.h"

#include "util.h"
#include <time.h>   // time
#include <ti/drivers/PIN.h>
#include <ti/sysbios/hal/Seconds.h> // Seconds_get
#include <xdc/runtime/System.h>     // snprintf

// OBD
#include "OneBitDisplay.h"
#include "font60.h"
#include "font16.h"

// battery & temp
#include <driverlib/aon_batmon.h>

extern const uint8_t ucMirror[];

// pin configuration
#define EPD_BUSY_PIN            IOID_10
#define EPD_RST_PIN             IOID_9
#define EPD_DC_PIN              IOID_8
#define EPD_CS_PIN              IOID_7
#define EPD_SCL_PIN             IOID_6
#define EPD_SDA_PIN             IOID_5

// gpio setting
static PIN_Handle GPIOHandle = NULL;
static PIN_State GPIOState;
static PIN_Config GPIOTable[] = {
  //EPD_POWER_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_BUSY_PIN  | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN |  PIN_PULLUP,
  EPD_DC_PIN    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_RST_PIN   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_CS_PIN    | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_SCL_PIN   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_SDA_PIN   | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  PIN_TERMINATE
};

extern uint8_t mac_address[6];

// epd frame buffer
uint8_t epd_temp[296*16];   // 296 x 128
OBDISP obd;

/*
 * <int.frac> format size (3.8) bits.
 * int for 0-3 voltage
 * frac each means 1/256 of ONE voltage
 */
#define INTFRAC2MV(x) (((x&0xff)*1000/256)+((x>>8)*1000))

// SSD1680A

// https://www.mdpi.com/2072-666X/12/5/578
#define _VS(x) x<<6
#define VSS  _VS(0b00)
#define VSH1 _VS(0b01)
#define VSL  _VS(0b10)
#define VSH2 _VS(0b11)

const uint8_t lut_full_bwr[] = {
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
    0x1,    VSL|0x1f,   0x0,        VSH2|0x3f,  0x0,        0x1,    0xa,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
//  112: LUTW x 7 
    0x1,    VSL|0x2f,   0x0,        0x0,        0x0,        0x1,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
    0x0,    0x0,        0x0,        0x0,        0x0,        0x0,    0x0,
//  168: LUTB x 7
    0x1,    VSH1|0x37,  0x0,        0x0,        0x0,        0x1,    0x0,
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

// quick and dirty way
// Task_sleep defined in <ti/sysbios/knl/Task.h>
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
    while(t--);
}

static void DEV_SPI_WriteByte(uint8_t byte)
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

static uint8_t DEV_SPI_ReadByte()
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

static void EPD_2IN13_Reset()
{    
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(10);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(10);
}

static void EPD_2IN13_SendCommand(uint8_t Reg)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
    //DEV_Digital_Write(EPD_DC_PIN, 0);
}

static void EPD_2IN13_SendData(uint8_t Data)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
    //DEV_Digital_Write(EPD_DC_PIN, 0);
}

static uint8_t EPD_2IN13_ReadData()
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

#if 0
static void EPD_2IN13_ReadBytes(uint8_t* buf, uint8_t len)
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

#endif

static inline int EPD_2IN13_IsBusy()
{
    //LOW: idle, HIGH: busy
    return (DEV_Digital_Read(EPD_BUSY_PIN) == 1);
}

static inline void EPD_2IN13_ReadBusy(void)
{
    while (EPD_2IN13_IsBusy()) {
        DEV_Delay_ms(10);
    }
}

static void EPD_Lut(const unsigned char *lut)
{
    EPD_2IN13_SendCommand(0x32);
    for(int i=0; i<227; i++) {
        EPD_2IN13_SendData(lut[i]);
    }

    // gate voltage
    EPD_2IN13_SendCommand(0x3F);
    EPD_2IN13_SendData(*(lut+227));

    EPD_2IN13_SendCommand(0x03);
    EPD_2IN13_SendData(*(lut+228));

    // source voltage
    EPD_2IN13_SendCommand(0x04);
    EPD_2IN13_SendData(*(lut+229));    // VSH
    EPD_2IN13_SendData(*(lut+230));    // VSH2
    EPD_2IN13_SendData(*(lut+231));    // VSL

    EPD_2IN13_SendCommand(0x2C);
    EPD_2IN13_SendData(*(lut+232));
}

#if 0
static void FixBuffer2(uint8_t *pSrc, uint8_t *pDst, uint16_t width, uint16_t height)
{
    int x, y;
    uint8_t *s, *d;
    for (y = 0; y < (height / 8); y++) { // byte rows
        d = &pDst[y * width];
        s = &pSrc[y * width];
        for (x = 0; x < width; x++) {
            //d[x * (height / 8)] = ~ucMirror[s[width - 1 - x]]; // invert and flip
            d[x] = ~ucMirror[s[x]];
        } // x
    } // y
}
#endif

static void EPD_2IN13_Init()
{
    EPD_2IN13_Reset();

    // clear
    //EPD_BWR(NULL, NULL, 296, 128, 0, 0);
}

static uint8_t EPD_ReadTemp()
{
    uint8_t rc;
    
    EPD_2IN13_SendCommand(0x12); // soft reset
    EPD_2IN13_ReadBusy();

    // Border Waveform
    EPD_2IN13_SendCommand(0x3C);
    EPD_2IN13_SendData(0x05);

    // Temperature sensor control
    EPD_2IN13_SendCommand(0x18);
    EPD_2IN13_SendData(0x80);       // 80: internal sensor 48: external sensor

    // Display update control
    EPD_2IN13_SendCommand(0x22);
    EPD_2IN13_SendData(0xb1);   // full: 0xf7

    // Master Activation
    EPD_2IN13_SendCommand(0x20);
    EPD_2IN13_ReadBusy();

    // read temperature
    EPD_2IN13_SendCommand(0x1b);
    rc = EPD_2IN13_ReadData();

    return rc;
}

static void EPD_2IN13_LoadImage(uint8_t *image, int size, uint8_t cmd)
{
    EPD_2IN13_SendCommand(cmd);
    for (int i = 0; i < size; i++) {
        EPD_2IN13_SendData(image[i]);
    }
}

static void EPD_BWR(int width, int height, int left, int top)
{
    EPD_2IN13_SendCommand(0x12); // soft reset
    
    // left up corner
    int w0 = left;
    int h0 = top;
    
    // right bottom corner
    int w1 = w0 + width - 1;
    int h1 = h0 + height/8 - 1;
    
    EPD_2IN13_ReadBusy();

    // Border Waveform
    EPD_2IN13_SendCommand(0x3C); 
    EPD_2IN13_SendData(0x05);

    // Driver output control
    EPD_2IN13_SendCommand(0x01);
    EPD_2IN13_SendData(0x28);  // mux=0x128(296)
    EPD_2IN13_SendData(0x01);
    EPD_2IN13_SendData(0x01);  // gd=0, sm=0, tb=1
    
    EPD_2IN13_SendCommand(0x11); //data entry mode
    EPD_2IN13_SendData(0x07);   // am=1, id=11

    // Set RAM X Address Start/End
    EPD_2IN13_SendCommand(0x44);
    EPD_2IN13_SendData(h0 & 0xff);
    EPD_2IN13_SendData(h1 & 0xff);

    // Set RAM Y Address Start/End
    EPD_2IN13_SendCommand(0x45); //set Ram-Y address start/end position
    EPD_2IN13_SendData(w0 & 0xff);
    EPD_2IN13_SendData(w0 >> 8);
    EPD_2IN13_SendData(w1 & 0xff);
    EPD_2IN13_SendData(w1 >> 8);

    // Temperature sensor control
    EPD_2IN13_SendCommand(0x18);
    EPD_2IN13_SendData(0x80);       // 80: internal sensor 48: external sensor

    // load lut
    EPD_Lut(lut_full_bwr);

#if 0
    // Set Ram X address
    EPD_2IN13_SendCommand(0x4E); 
    EPD_2IN13_SendData(h0 & 0xff);

    // Set Ram Y address
    EPD_2IN13_SendCommand(0x4F); 
    EPD_2IN13_SendData(w0 & 0xff);
    EPD_2IN13_SendData(w0 >> 8);

    if (image) {
        EPD_LoadImage(image, size, 0x24);
    } else {
        EPD_2IN13_SendCommand(0x24);
        for (int i = 0; i < size; i++) {
            EPD_2IN13_SendData(0xff);
        }
    }

    // Set Ram X address
    EPD_2IN13_SendCommand(0x4E); 
    EPD_2IN13_SendData(h0 & 0xff);

    // Set Ram Y address
    EPD_2IN13_SendCommand(0x4F); 
    EPD_2IN13_SendData(w0 & 0xff);
    EPD_2IN13_SendData(w0 >> 8);

    if (red_image) {
        EPD_LoadImage(red_image, size, 0x26);
    } else {
        EPD_2IN13_SendCommand(0x26);
        for (int i = 0; i < size; i++) {
            EPD_2IN13_SendData(0x00);
        }
    }
#endif

#if 0
    // Display update control
    EPD_2IN13_SendCommand(0x22);
    EPD_2IN13_SendData(0xc7);   // full: 0xf7

    // Master Activation
    EPD_2IN13_SendCommand(0x20);
#endif
}

void EPD_2IN13_WriteRam(uint8_t *image, int width, int height, int left, int top, uint8_t is_red)
{
    // data size in bytes
    int size = width*height/8;

    // Set Ram X address
    EPD_2IN13_SendCommand(0x4E); 
    EPD_2IN13_SendData(top & 0xff);

    // Set Ram Y address
    EPD_2IN13_SendCommand(0x4F); 
    EPD_2IN13_SendData(left & 0xff);
    EPD_2IN13_SendData(left >> 8);

    const uint8_t reg = is_red ? 0x26 : 0x24;     // BW: 0x24, Red: 0x26 

    if (image) {
        EPD_2IN13_LoadImage(image, size, reg);
    } else {
        EPD_2IN13_SendCommand(reg);
        for (int i = 0; i < size; i++) {
            EPD_2IN13_SendData(is_red ? 0x00 : 0xff);
        }
    }
}

void EPD_2IN13_Display(uint8_t reg)
{
    EPD_2IN13_SendCommand(0x22);
    EPD_2IN13_SendData(reg);
    EPD_2IN13_SendCommand(0x20);
    //EPD_2IN13_ReadBusy();
}

void EPD_2IN13_Sleep(void)
{
    EPD_2IN13_SendCommand(0x10);    //enter deep sleep
    EPD_2IN13_SendData(0x01);       // 01: mode 1, 11: mode 2
}

//static uint8_t need_sleep = 0;
void EPD_Update()
{
    static time_t last = 0;
    time_t now = time(NULL);
    if (last && ((now % 60) != 0)) {
        #if 0
        if (need_sleep && !EPD_2IN13_IsBusy()) { // put EPD deep sleep.
            EPD_2IN13_Sleep();
            need_sleep = 0;
        }
        #endif
        return;
    }
    last = now;
    // TBD: timezone +8
    now += (3600 * 8);
    struct tm *l = localtime(&now);

    // wakeup EPD
    EPD_2IN13_Reset();

    char buf[32];
    obdCreateVirtualDisplay(&obd, 296, 128, epd_temp);
    obdFill(&obd, 0, 0);

    // mac address
    System_snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_address[0],mac_address[1],mac_address[2],
            mac_address[3],mac_address[4],mac_address[5]);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 0, 16, buf, 1);

    // battery voltage
    System_snprintf(buf, 32, "%umv", INTFRAC2MV(AONBatMonBatteryVoltageGet()));
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 216, 16, buf, 1);

    // time
    System_snprintf(buf, 32, "%02d:%02d", l->tm_hour, l->tm_min);
    obdWriteStringCustom(&obd, (GFXfont *)&DSEG14_Classic_Mini_Regular_40, 70, 85, buf, 1);

    // date
    //const char wstr[]={"MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};
    System_snprintf(buf, 32, "%u-%02u-%02u %d", 1900+l->tm_year, l->tm_mon+1, l->tm_mday, l->tm_wday);
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 0, 122, buf, 1);

    // temperature
    System_snprintf(buf, 32, "%dC %dC", AONBatMonTemperatureGetDegC(), EPD_ReadTemp());
    obdWriteStringCustom(&obd, (GFXfont *)&Dialog_plain_16, 216, 122, buf, 1);

    // endian
    for (int i=0; i<sizeof(epd_temp); i++) {
        uint8_t c = epd_temp[i];
        epd_temp[i] = ~ucMirror[c];
    }

    //need_sleep = 1;
    EPD_BWR(296, 128, 0, 0);
    EPD_2IN13_WriteRam(epd_temp, 296, 128, 0, 0, 0);
    EPD_2IN13_WriteRam(NULL, 296, 128, 0, 0, 1);
    EPD_2IN13_Display(0xc7);    // c7: by REG  f7: by OTP   b1: no display 
    EPD_2IN13_ReadBusy();
    EPD_2IN13_Sleep();
    return;
}

// should be only called once!
void epd_hw_init()
{
    GPIOHandle = PIN_open(&GPIOState, GPIOTable);      

    // test LUT size
    //lut_size = EPD_LUT_detect();

    EPD_2IN13_Init();
}
