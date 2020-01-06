#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "st7735.h"

#define DELAY 0x80

enum MADCTL_ARGS
{
  MADCTL_MY = 0x80,
  MADCTL_MX = 0x40,
  MADCTL_MV = 0x20,
  MADCTL_ML = 0x10,
  MADCTL_RGB = 0x00,
  MADCTL_BGR = 0x08,
  MADCTL_MH = 0x04
};

enum ST7735_CMD
{
  NOP     = 0,
  SWRESET = 0x1,
  RDDID   = 0x4,
  SLPOUT  = 0x11,
  INVOFF  = 0x20,
  INVON   = 0x21,
  CASET   = 0x2A,
  RASET   = 0x2B,
  RAMWR   = 0x2C,
  DISPOFF = 0x28,
  DISPON  = 0x29,
  NORON   = 0x13,
  COLMOD  = 0x3A,
  MADCTL  = 0x36,
  IDMOFF  = 0x38,
  IDMON   = 0x39,
  FRMCTR1 = 0xB1,
  FRMCTR2 = 0xB2,
  FRMCTR3 = 0xB3,
  INVCTR  = 0xB4,
  PWCTR1  = 0xC0,
  PWCTR2  = 0xC1,
  PWCTR3  = 0xC2,
  PWCTR4  = 0xC3,
  PWCTR5  = 0xC4,
  VMCTR1  = 0xC5,
  GMCTRP1 = 0xE0,
  GMCTRN1 = 0xE1,
};

// based on Adafruit ST7735 library for Arduino
static const uint8_t init_cmds[] =
{
  22,     // 21 commands in list:
  SWRESET,
  DELAY, //  1: Software reset, 0 args, w/delay
  150,   //     150 ms delay
  SLPOUT,
  DELAY, //  2: Out of sleep mode, 0 args, w/delay
  255,   //     255ms delay
  FRMCTR1,
  3,     //  3: Frame rate ctrl - normal mode, 3 args:
  0x01,
  0x2C,
  0x2D, //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
  FRMCTR2,
  3, //  4: Frame rate control - idle mode, 3 args:
  0x01,
  0x2C,
  0x2D, //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
  FRMCTR3,
  6, //  5: Frame rate ctrl - partial mode, 6 args:
  0x01,
  0x2C,
  0x2D, //     Dot inversion mode
  0x01,
  0x2C,
  0x2D, //     Line inversion mode
  INVCTR,
  1,    //  6: Display inversion ctrl, 1 arg, no delay:
  0x07, //     No inversion
  PWCTR1,
  3, //  7: Power control, 3 args, no delay:
  0xA2,
  0x02, //     -4.6V
  0x84, //     AUTO mode
  PWCTR2,
  1,    //  8: Power control, 1 arg, no delay:
  0xC5, //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
  PWCTR3,
  2,    //  9: Power control, 2 args, no delay:
  0x0A, //     Opamp current small
  0x00, //     Boost frequency
  PWCTR4,
  2,    // 10: Power control, 2 args, no delay:
  0x8A, //     BCLK/2, Opamp current small & Medium low
  0x2A,
  PWCTR5,
  2, // 11: Power control, 2 args, no delay:
  0x8A,
  0xEE,
  VMCTR1,
  1, // 12: Power control, 1 arg, no delay:
  0x0E,
  INVOFF,
  0, // 13: Don't invert display, no args, no delay
  MADCTL,
  1,                     // 14: Memory access control (directions), 1 arg:
  //MADCTL_ML, //     row addr/col addr, bottom to top refresh
  0,
  COLMOD,
  1,    // 15: set color mode, 1 arg, no delay:
  0x05, //     16-bit color
  CASET,
  4, //  1: Column addr set, 4 args, no delay:
  0x00,
  0x00, //     XSTART = 0
  0x00,
  0x7F, //     XEND = 127
  RASET,
  4, //  2: Row addr set, 4 args, no delay:
  0x00,
  0x00, //     XSTART = 0
  0x00,
  0x9F, //     XEND = 159
  GMCTRP1,
  16, //  1: Magical unicorn dust, 16 args, no delay:
  0x02,
  0x1c,
  0x07,
  0x12,
  0x37,
  0x32,
  0x29,
  0x2d,
  0x29,
  0x25,
  0x2B,
  0x39,
  0x00,
  0x01,
  0x03,
  0x10,
  GMCTRN1,
  16, //  2: Sparkles and rainbows, 16 args, no delay:
  0x03,
  0x1d,
  0x07,
  0x06,
  0x2E,
  0x2C,
  0x29,
  0x2D,
  0x2E,
  0x2E,
  0x37,
  0x3F,
  0x00,
  0x00,
  0x02,
  0x10,
  NORON,
  DELAY, //  3: Normal display on, no args, w/delay
  10,    //     10 ms delay
  DISPON,
  DELAY, //  4: Main screen turn on, no args w/delay
  100,
  IDMOFF,
  0
};

static ST7735_Config config;

static void spi_write(uint16_t data, int cmd)
{
  GPIO_WriteBit(config.dc.port, config.dc.pin, cmd ? Bit_RESET : Bit_SET);
  SPI_I2S_SendData(config.spi, data);
  while ((config.spi->SR & SPI_SR_TXE) == 0) {}
}

void ST7735_StartTransmission()
{
  GPIO_ResetBits(config.cs.port, config.cs.pin);
}

void ST7735_FinishTransmission()
{
  GPIO_SetBits(config.cs.port, config.cs.pin);
}

void ST7735_SendCmd(int op_id)
{
  spi_write(op_id, 1);
}

void ST7735_SendData8(uint8_t data)
{
  spi_write(data, 0);
}

void ST7735_SendData16(uint8_t data)
{
  spi_write(data, 0);
}

void ST7735_Reset()
{
  ST7735_SendCmd(SWRESET);
  config.delay(120);
}

void ST7735_ColSet(uint8_t col_s, uint8_t col_e)
{
  ST7735_SendCmd(CASET);
  ST7735_SendData8(0);
  ST7735_SendData8(col_s);
  ST7735_SendData8(0);
  ST7735_SendData8(col_e);
}

void ST7735_RowSet(uint8_t raw_s, uint8_t raw_e)
{
  ST7735_SendCmd(RASET);
  ST7735_SendData8(0);
  ST7735_SendData8(raw_s);
  ST7735_SendData8(0);
  ST7735_SendData8(raw_e);
}

void ST7735_RamWrite()
{
  ST7735_SendCmd(RAMWR);
}

void ST7735_DispOff()
{
  ST7735_SendCmd(DISPOFF);
}

void ST7735_Nop()
{
  ST7735_SendCmd(NOP);
}

void ST7735_Init(ST7735_Config* conf)
{
  config = *conf;
  GPIO_InitTypeDef pin_def = {
    .GPIO_Pin = conf->dc.pin,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_Mode = GPIO_Mode_Out_PP
  };
  GPIO_Init(conf->dc.port, &pin_def);

  pin_def.GPIO_Pin = conf->rst.pin;
  GPIO_Init(conf->rst.port, &pin_def);

  pin_def.GPIO_Pin = conf->cs.pin;
  GPIO_Init(conf->cs.port, &pin_def);

  // High RST for normal operational
  GPIO_SetBits(config.rst.port, config.rst.pin);

  // High CSX before transmision
  GPIO_SetBits(config.cs.port, config.cs.pin);

  int initCmdN = init_cmds[0];
  const uint8_t* cmdbuf = &init_cmds[1];
  ST7735_StartTransmission();
  while(initCmdN--)
  {
    ST7735_SendCmd(*cmdbuf++);
    uint8_t nargs = *cmdbuf++;
    if (nargs == DELAY)
    {
      ST7735_FinishTransmission();
      config.delay(*cmdbuf++);
      ST7735_StartTransmission();
    }
    else
    {
      while(nargs--)
        ST7735_SendData8(*cmdbuf++);
    }
  }
  ST7735_FinishTransmission();


  // Init DMA
}

void ST7735_FillScreen(uint16_t color)
{
  ST7735_StartTransmission();
  ST7735_ColSet(0, 127);
  ST7735_RowSet(0, 159);
  ST7735_FinishTransmission();

  ST7735_StartTransmission();
  ST7735_RamWrite();


  for (int y = 0; y <= 159; ++y)
  {
    for (int x = 0; x <= 127; ++x)
    {
      ST7735_SendData8(color >> 8);
      ST7735_SendData8(color);
    }
  }

  ST7735_FinishTransmission();
  ST7735_Nop();
}

void ST7735_DrawImage(const uint16_t img[][128], int rows, int cols)
{
  ST7735_StartTransmission();
  ST7735_ColSet(0, cols - 1);
  ST7735_RowSet(0, rows - 1);
  ST7735_FinishTransmission();

  ST7735_StartTransmission();
  ST7735_RamWrite();

  for (int y = 0; y < rows; ++y)
  {
    for (int x = 0; x < cols; ++x)
    {
      uint16_t color = img[x][y];
      ST7735_SendData8(color);
      ST7735_SendData8(color >> 8);
    }
  }

  ST7735_FinishTransmission();
  ST7735_Nop();
}

void ST7735_SendRGBData(const uint16_t img[], uint8_t xs, uint8_t xe, uint8_t ys, uint8_t ye)
{
  ST7735_StartTransmission();
  ST7735_ColSet(xs, xe);
  ST7735_RowSet(ys, ye);
  ST7735_FinishTransmission();

  ST7735_StartTransmission();
  ST7735_RamWrite();

  const int npixels = (xe - xs + 1) * (ye - ys + 1);

  // init DMA
  DMA_InitTypeDef dma_init = {
    .DMA_PeripheralBaseAddr = (uint32_t)(&config.spi->DR),
    .DMA_MemoryBaseAddr = (uint32_t)(img),
    .DMA_DIR = DMA_DIR_PeripheralDST,
    .DMA_BufferSize = npixels,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_High,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_Init(config.dma, &dma_init);
  DMA_ClearFlag(DMA1_FLAG_TC3);

  GPIO_SetBits(config.dc.port, config.dc.pin);

  // 16-bit data transfer
  SPI_DataSizeConfig(config.spi, SPI_DataSize_16b);

  // enable SPI DMA
  SPI_I2S_DMACmd(config.spi, SPI_I2S_DMAReq_Tx, ENABLE);

  // start DMA transfer
  DMA_Cmd(config.dma, ENABLE);
  while (config.dma->CNDTR > 0) {}
  DMA_Cmd(config.dma, DISABLE);

  SPI_Cmd(config.spi, DISABLE);
  SPI_DataSizeConfig(config.spi, SPI_DataSize_8b);
  SPI_I2S_DMACmd(config.spi, SPI_I2S_DMAReq_Tx, DISABLE);
  SPI_Cmd(config.spi, ENABLE);
  DMA_DeInit(config.dma);
  ST7735_FinishTransmission();
}

void ST7735_DrawImageLinear(const uint16_t img[], int rows, int cols, int off)
{
  ST7735_StartTransmission();
  ST7735_ColSet(0, cols - 1);
  ST7735_RowSet(0, rows - 1);
  ST7735_FinishTransmission();

  ST7735_StartTransmission();
  ST7735_RamWrite();

  int npixels = cols * rows;

  const uint16_t* data = &img[off];

  GPIO_SetBits(config.dc.port, config.dc.pin);
  DMA_InitTypeDef dma_init = {
    .DMA_PeripheralBaseAddr = (uint32_t)(&config.spi->DR),
    .DMA_MemoryBaseAddr = (uint32_t)(data),
    .DMA_DIR = DMA_DIR_PeripheralDST,
    .DMA_BufferSize = npixels - off,
    .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
    .DMA_MemoryInc = DMA_MemoryInc_Enable,
    .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    .DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
    .DMA_Mode = DMA_Mode_Normal,
    .DMA_Priority = DMA_Priority_High,
    .DMA_M2M = DMA_M2M_Disable
  };
  DMA_ClearFlag(DMA1_FLAG_TC3);

  SPI_DataSizeConfig(config.spi, SPI_DataSize_16b);
  SPI_I2S_DMACmd(config.spi, SPI_I2S_DMAReq_Tx, ENABLE);
  DMA_Init(config.dma, &dma_init);
  DMA_Cmd(config.dma, ENABLE);

  while (config.dma->CNDTR > 0) {}

  if (off > 0)
  {
    DMA_Cmd(config.dma, DISABLE);
    config.dma->CMAR = (uint32_t)(img);
    DMA_SetCurrDataCounter(config.dma, off);
    DMA_Cmd(config.dma, ENABLE);
    while (config.dma->CNDTR > 0) {}
  }

  SPI_Cmd(config.spi, DISABLE);
  SPI_DataSizeConfig(config.spi, SPI_DataSize_8b);
  SPI_I2S_DMACmd(config.spi, SPI_I2S_DMAReq_Tx, DISABLE);
  SPI_Cmd(config.spi, ENABLE);
  DMA_Cmd(config.dma, DISABLE);
  DMA_DeInit(config.dma);
  ST7735_FinishTransmission();
}
