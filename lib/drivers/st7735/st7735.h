#pragma once
#include <stdint.h>
#include "stm32f10x.h"

#define R5G6B5(r, g, b) (((r) % 32) << 11 | (((g) % 64) << 5) | (((b) % 32) >> 3))

typedef struct {
  GPIO_TypeDef* port;
  uint16_t      pin;
} PinInfo;

typedef struct {
  void (*delay)(int);
  PinInfo dc;
  PinInfo rst;
  PinInfo cs;
  DMA_Channel_TypeDef* dma;
  SPI_TypeDef* spi;
} ST7735_Config;

void ST7735_Init(ST7735_Config* config);
void ST7735_SendData8(uint8_t data);

void ST7735_StartTransmission();
void ST7735_FinishTransmission();

void ST7735_Reset();
void ST7735_Nop();
void ST7735_DispOff();
void ST7735_ColSet(uint8_t col_s, uint8_t col_e);
void ST7735_RowSet(uint8_t raw_s, uint8_t raw_e);
void ST7735_RamWrite();

void ST7735_FillScreen(uint16_t color);
void ST7735_DrawImage(const uint16_t img[][128], int rows, int cols);
void ST7735_DrawImageLinear(const uint16_t img[], int rows, int cols, int off);

void ST7735_SendRGBData(const uint16_t img[], uint8_t xs, uint8_t xe, uint8_t ys, uint8_t ye);


