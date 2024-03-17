// EPD Driver

#include <stdint.h>   // uint8_t 
#include <board.h>
#include <ti/drivers/PIN.h>
#include <ti/sysbios/knl/Clock.h>   // Clock_tickPeriod
#include <ti/sysbios/knl/Task.h>    // Task_sleep

#include "epd_driver.h"

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
    while(t--);
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
    DEV_Delay_ms(10);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(10);
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

void EPD_SSD_WaitBusy(void)
{
    while (EPD_SSD_IsBusy()) {
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

// Select a EPD
#if defined(EPD_2IN13_SSD1680)

#include "epd_2in13.c"

#elif defined(EPD_2IN9_SSD1680A)

#include "epd_2in9.c"

#else

#error "EPD not support."

#endif

// should be only called once!
void EPD_Init()
{
    GPIOHandle = PIN_open(&GPIOState, GPIOTable);      

    // test LUT size
    //lut_size = EPD_LUT_Detect();

    EPD_SSD_Init();
}

void EPD_Update()
{
    EPD_SSD_Update();
}
