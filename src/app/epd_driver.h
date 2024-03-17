#ifndef _EPD_SPI_H_
#define _EPD_SPI_H_

#include <stdint.h>

// epd device 
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

// the driver public,
void EPD_Init();
void EPD_Update();
void EPD_SSD_Reset();
void EPD_SSD_SendCommand(uint8_t reg);
void EPD_SSD_SendData(uint8_t data);
uint8_t EPD_SSD_ReadData();
void EPD_SSD_ReadBytes(uint8_t* buf, uint8_t len);
bool EPD_SSD_IsBusy();
void EPD_SSD_WaitBusy();

// for epd_inch.c shoule implement,
void EPD_SSD_Init();
void EPD_SSD_Upate();

#endif 