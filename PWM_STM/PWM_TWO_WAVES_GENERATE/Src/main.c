#include "stm32f4xx.h"

void delay_us(uint32_t us) {
    SysTick->LOAD = us * 84 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
        ;

    SysTick->CTRL = 0;
}

void TIM2_Config(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set the prescaler value
    TIM2->PSC = 83; // Assuming a 1MHz frequency (84MHz / (83 + 1) = 1MHz)

    // Set the auto-reload value for a 50 kHz period
    TIM2->ARR = 19999; // (1MHz / 50kHz) - 1

    // Configure Channel 1 in PWM mode
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;                    // Preload enable

    // Set the pulse value for a 50% duty cycle
    TIM2->CCR1 = 9999; // (1MHz / 50kHz) / 2 - 1

    // Enable the output compare 1 preload register on TIM2
    TIM2->CCER |= TIM_CCER_CC1E;

    // Enable the timer counter
    TIM2->CR1 |= TIM_CR1_CEN;
}

void GPIO_Config(void) {
    // Enable the GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA5 as an output
    GPIOA->MODER |= GPIO_MODER_MODER5_0; // General purpose output mode
}

int main(void) {
    GPIO_Config();
    //TIM2_Config();

    RCC->AHB1ENR|=(1U<<0);

    GPIOA->MODER|=(1U<<10);
    GPIOA->MODER&=~(1U<<11);

    while (1) {
        // Turn on the LED for the first 50% of the cycle
        GPIOA-> ODR |= (1U<<5);
        delay_us(5000); // 50% duty cycle

        // Turn off the LED for the second 50% of the cycle
        GPIOA-> ODR &= ~(1U<<5);
        delay_us(5000); // 50% duty cycle
    }
}
