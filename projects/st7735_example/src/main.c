#include "stm32f10x.h"
#include "core_cm3.h"
#include "st7735.h"
#include <math.h>
//#include "img.h"
//#include "girl_rgb565.h"

static volatile int tim2_hit = 0;
void TIM2_Delay(int delay_ms)
{
  TIM_Cmd(TIM2, DISABLE);

  TIM_PrescalerConfig(TIM2, 8000 - 1, TIM_PSCReloadMode_Update);
  TIM_SetAutoreload(TIM2, delay_ms);
  TIM_SetCounter(TIM2, 0);

  tim2_hit = 0;
  TIM_Cmd(TIM2, ENABLE);
  NVIC_EnableIRQ(TIM2_IRQn);

  while (tim2_hit == 0) {}

  TIM_Cmd(TIM2, DISABLE);
}

void TIM2_IRQHandler()
{
  if (TIM2->SR & TIM_SR_UIF)
  {
    tim2_hit = 1;
    CLEAR_BIT(TIM2->SR, TIM_SR_UIF);
  }
}

void TIM2_Init()
{
  TIM_TimeBaseInitTypeDef timInit = {
    .TIM_Prescaler = 0xFFFF,
    .TIM_CounterMode = TIM_CounterMode_Up,
    .TIM_Period = 59,
    .TIM_ClockDivision = TIM_CKD_DIV1
  };

  TIM_TimeBaseInit(TIM2, &timInit);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  NVIC_EnableIRQ(TIM2_IRQn);
}

void SPI1_Init()
{
  SPI1->CR2 = SPI_CR2_SSOE;
  SPI1->CR1 = SPI_CR1_MSTR; // chip is master SPI


  GPIO_InitTypeDef pinInit = {
    .GPIO_Pin = GPIO_Pin_4,
    .GPIO_Speed = GPIO_Speed_50MHz,
    .GPIO_Mode = GPIO_Mode_Out_PP
  };

  // PA4 NSS
  GPIO_Init(GPIOA, &pinInit);

  // PA5 SCK
  pinInit.GPIO_Pin = GPIO_Pin_5;
  pinInit.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &pinInit);

  // PA7 MOSI
  pinInit.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOA, &pinInit);

  SET_BIT(SPI1->CR1, SPI_CR1_SPE); // enable SPI1
}

void SystemInit()
{
  RCC_DeInit();

  RCC_APB2PeriphClockCmd(  RCC_APB2Periph_AFIO
                         | RCC_APB2Periph_GPIOA
                         | RCC_APB2Periph_GPIOB
                         | RCC_APB2Periph_GPIOC
                         | RCC_APB2Periph_SPI1
                         , ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_PWR, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // wait for HSI up
  while (!READ_BIT(RCC->CR, RCC_CR_HSIRDY)) {}

  // wait for HSE up
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_WaitForHSEStartUp();

  // wait for LSI up
  RCC_LSICmd(ENABLE);
  while (!READ_BIT(RCC->CSR, RCC_CSR_LSIRDY)) {}

  // HSE = 8 MHz. PLL = 8 * 3 = 24 Mhz.
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_3);

  RCC_HCLKConfig(RCC_SYSCLK_Div1);// AHB = PLL = 24 Mhz
  RCC_PCLK1Config(RCC_HCLK_Div2); // APB1 = AHB / 2
  RCC_PCLK2Config(RCC_HCLK_Div1); // APB2 = AHB

  // wait for PLL up
  RCC_PLLCmd(ENABLE);
  while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY)) {}

  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  RCC_MCOConfig(RCC_MCO_SYSCLK);

  // PA8 MCO pin
  GPIO_InitTypeDef pinInit = {
    .GPIO_Pin = GPIO_Pin_8,
    .GPIO_Speed = GPIO_Speed_50MHz,
    .GPIO_Mode = GPIO_Mode_AF_PP
  };
  GPIO_Init(GPIOA, &pinInit);

  // PC13 build-in LED pin
  pinInit.GPIO_Pin = GPIO_Pin_13;
  pinInit.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &pinInit);

  TIM2_Init();
  SPI1_Init();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP, DISABLE);

  ST7735_Config st7735_config = {
    .delay = TIM2_Delay,
    .dc = {
      .port = GPIOA,
      .pin  = GPIO_Pin_3
    },
    .rst = {
      .port = GPIOB,
      .pin  = GPIO_Pin_0
    },
    .cs = {
      .port = GPIOA,
      .pin  = GPIO_Pin_4
    },
    .dma = DMA1_Channel3,
    .spi = SPI1
  };
  ST7735_Init(&st7735_config);
}

uint16_t image_data[54*128];
float    sin_table[256];
float    cos_table[256];

int main(void)
{
  __enable_irq();

  GPIO_ResetBits(GPIOA, GPIO_Pin_8);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);

  ST7735_FillScreen(0);
  for (int i = 0; i < 256; ++i)
    sincosf(M_PI*2*i/255, &sin_table[i], &cos_table[i]);

  int c = 0;
  const int checkWidth = 15;
  for (;;)
  {
    ++c;
    int bufY = 0;
    int lastYs = 0;
    int angle = (c*1) % 255;
    float sinC = sin_table[angle];
    float cosC = cos_table[angle];
    for (int y = 0; y < 160; ++y, ++bufY)
    {
      for (int x = 0; x < 128; ++x)
      {
        int Xt = x*cosC + y*sinC;
        int Yt = -x*sinC + y*cosC;

        if (((Xt / checkWidth) & 0x1) == ((Yt / checkWidth) & 0x1))
          image_data[bufY*128 + x] = 0xFFFF;
        else
          image_data[bufY*128 + x] = 0;
      }

      if (bufY == 53)
      {
        ST7735_SendRGBData(image_data, 0, 127, lastYs, lastYs + bufY);
        lastYs += bufY;
        bufY = -1;
      }
    }
    ST7735_SendRGBData(image_data, 0, 127, lastYs, lastYs + bufY + 1);
    GPIO_WriteBit(GPIOC, GPIO_Pin_13,
                  GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == 0 ? Bit_SET
                                                                  : Bit_RESET);
  }
}
