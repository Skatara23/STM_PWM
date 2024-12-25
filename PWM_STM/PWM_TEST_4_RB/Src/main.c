#include "stm32f4xx.h"

void delay_ms(uint32_t ms) {
    // Adjust this loop count based on your system clock frequency
    volatile uint32_t i;
    for (i = 0; i < (ms * 5000); i++) {
        // Delay function
    }
}

void TIM2_Configuration(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set prescaler to have 1us resolution
    TIM2->PSC = SystemCoreClock / 1000000 - 1;

    // Set auto-reload value (period)
    TIM2->ARR = 1000 - 1; // Period of 1000us (1ms)

    // Configure channel 1 in PWM mode
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // Preload enable

    // Enable output compare 1
    TIM2->CCER |= TIM_CCER_CC1E;

    // Enable TIM2
    TIM2->CR1 |= TIM_CR1_CEN;
}

void LED_PWM_SetDutyCycle(uint16_t dutyCycle) {
    // Limit duty cycle to be between 0 and 1000 (0% - 100%)
    if (dutyCycle > 1000) {
        dutyCycle = 1000;
    }

    // Set the duty cycle
    TIM2->CCR1 = dutyCycle;
}

int main(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA0 as alternate function mode
    GPIOA->MODER |= GPIO_MODER_MODER0_1; // Alternate function mode

    // Configure PA0 for TIM2_CH1
    GPIOA->AFR[0] |= (1 << 0 * 4); // AF1 for TIM2_CH1

    // Configure TIM2
    TIM2_Configuration();

    while (1) {
        // Vary duty cycle for LED brightness control
        for (uint16_t dutyCycle = 0; dutyCycle <= 1000; dutyCycle += 50) {
            LED_PWM_SetDutyCycle(dutyCycle);
            delay_ms(500); // Adjust delay for desired brightness change speed
        }
    }
}
