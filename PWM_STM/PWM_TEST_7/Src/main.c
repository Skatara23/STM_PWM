#include "stm32f401xe.h"

// Define the system clock frequency
#define SYSTEM_CORE_CLOCK 16000000

// PWM parameters
#define PWM_FREQUENCY 50000 // 50 kHz
#define PWM_DUTY_CYCLE 50   // 50% duty cycle (adjust as needed)

void delay_ms(uint32_t ms);
void GPIO_Init(void);
void PWM_Init(void);

int main(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Initialize GPIO for the button and LED
    GPIO_Init();

    // Initialize PWM on PA6
    PWM_Init();

    while (1) {
        // Your main application code here

        // Start PWM generation (if required)
        TIM3->CR1 |= TIM_CR1_CEN;
    }
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        // Clear the interrupt flag
        EXTI->PR |= EXTI_PR_PR0;

        // Toggle the LED on PA5
        GPIOA->ODR ^= GPIO_ODR_OD5;

        // Add delay for debouncing (adjust as needed)
        delay_ms(100);
    }
}

void GPIO_Init(void) {
    // Configure PA0 as input mode (button)
    GPIOA->MODER &= ~GPIO_MODER_MODER0; // Input mode
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1; // Pull-down

    // Configure PA5 as output mode (LED)
    GPIOA->MODER |= GPIO_MODER_MODER5_0; // General-purpose output mode

    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Connect EXTI0 Line to PA0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // Clear EXTI0 bits
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

    // Configure EXTI0 to generate an interrupt on a rising or falling edge (whichever you prefer)
    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->RTSR |= EXTI_RTSR_TR0; // Rising edge
    // EXTI->FTSR |= EXTI_FTSR_TR0; // Falling edge (uncomment if you want falling edge)

    // Enable EXTI0 IRQ in NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 0); // Set priority to 0 (highest)
}

void delay_ms(uint32_t ms) {
    volatile uint32_t i;
    for (i = 0; i < (ms * (SYSTEM_CORE_CLOCK / 1000)); i++) {
        // Adjust this loop count based on your system clock frequency
    }
}

void PWM_Init(void) {
    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure PA6 as alternate function mode (AF2 for TIM3)
    GPIOA->MODER |= GPIO_MODER_MODER6_1; // Alternate function mode
    GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_2;  // AF2 (TIM3) for pin PA6

    // Configure TIM3 for PWM generation
    TIM3->PSC = (SYSTEM_CORE_CLOCK / PWM_FREQUENCY) - 1; // Set prescaler
    TIM3->ARR = (SYSTEM_CORE_CLOCK / PWM_FREQUENCY) - 1; // Set auto-reload value
    TIM3->CCR1 = ((SYSTEM_CORE_CLOCK / PWM_FREQUENCY) * PWM_DUTY_CYCLE) / 100 - 1; // Set duty cycle

    // Configure PWM mode 1 for channel 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

    // Enable the output compare 1 preload register
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    // NOTE: Do not enable the TIM3 counter here
    // TIM3->CR1 |= TIM_CR1_CEN;
}
