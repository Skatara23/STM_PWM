#include "stm32f4xx.h"


#define GPIOAEN 	(1U<<0)
#define PIN5		(1U<<5)
#define led_pin 		PIN5

#define GPIOBEN		(1U<<0)
#define PIN2		(1U<<2)
#define led_pin_1		PIN2

int main(void)
{
	RCC-> AHB1ENR |= GPIOAEN;
	GPIOA->MODER|=(1U<<10);
	GPIOA->MODER&=~(1U<<11);
	RCC-> AHB1ENR |= GPIOBEN;
	GPIOB->MODER|=(1U<<4);
	GPIOB->MODER|=(1U<<5);

	unsigned int i;

	while(1)
	{
		GPIOA->BSRR=led_pin;
		for(i=0; i<1000000; i++)
		{}

		GPIOA->BSRR= (1U<<21);
		for(i=0; i<1000000; i++)
		{}

		GPIOB->BSRR=led_pin_1;
				for(i=0; i<1000000; i++)
				{}

				GPIOB->BSRR= (1U<<18);
				for(i=0; i<1000000; i++)
				{}


	}

}
