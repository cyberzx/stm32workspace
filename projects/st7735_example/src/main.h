#pragma once

#define PIN_BSRR(pin_id) (1 << pin_id)
#define PIN_BRR(pin_id) (1 << pin_id)

void TIM2_Delay(int delay_ms);
void TogglePin(int pin_id);
void SetPin(int pin_id);
void ClearPin(int pin_id);

