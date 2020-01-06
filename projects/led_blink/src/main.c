#include <stm32f10x.h>
#include <stm32f10x_gpio.h>

#define LED_PIN   GPIO_Pin_13

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

void SystemInit()
{
  RCC_DeInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_PWR, ENABLE);

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

  // Configure GPIO
  GPIO_InitTypeDef ledPinInit = {
    .GPIO_Pin = LED_PIN,
    .GPIO_Speed = GPIO_Speed_2MHz,
    .GPIO_Mode = GPIO_Mode_Out_PP
  };
  GPIO_Init(GPIOC, &ledPinInit);
  TIM2_Init();
}

int main(void)
{
  __enable_irq();

  int led_state = 0;
  for (;;)
  {
    GPIO_WriteBit(GPIOC, LED_PIN, led_state);
    led_state ^= 1;
    TIM2_Delay(2000);
  }
}
