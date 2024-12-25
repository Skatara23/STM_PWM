/*accroding to hex & binary formate interface led as on and off
 * port a:
 * pin no 5:
 */
#include "stm32f4xx.h"

#define PERIPH_BASE			(0x40000000UL)
#define AHB1PERIPH_OFFSET	(0X00020000UL)

//AHB1 BUS = 0x40000000 +0x00020000 = 0x40020000
#define AHB1PERIPH_BASE		(PERIPH_BASE + AHB1PERIPH_OFFSET)
#define GPIOA_OFFSET 		(0X0000U)		//0X00000000 8 DIGITS
#define GPIOA_BASE			(AHB1PERIPH_BASE + GPIOA_OFFSET)

#define RCC_OFFSET 			(0X3800UL)			//lsb bit of RCC
/*0X40020000+0X3800= 0X40023800		*/
#define RCC_BASE 			(AHB1PERIPH_BASE + RCC_OFFSET)

#define AHB1EN_R_OFFSET 		(0X30UL)			//Offset address of AHB1ENR
#define RCC_AHB1EN_R			(*(volatile unsigned int *)(RCC_BASE + AHB1EN_R_OFFSET))

#define MODE_R_OFFSET 			(0X00UL)
#define GPIOA_MODE_R			(*(volatile unsigned int *)(GPIOA_BASE + MODE_R_OFFSET))

/*0B 0000 0000 0000 0000 0000 0000 0000 0001 */

#define OD_R_OFFSET 	(0X14UL)
#define GPIOA_OD_R		(*(volatile unsigned int *)(GPIOA_BASE + OD_R_OFFSET))

#define GPIOAEN 		(1U<<0)

#define PIN5 			(1U<<5)
#define led5			 PIN5

int main(void)
{

	RCC_AHB1EN_R|=GPIOAEN;

	GPIOA_MODE_R|= (1U<<10);
	GPIOA_MODE_R&=~(1U<<11);

	while (1)
	{
		GPIOA_OD_R^=led5;
		for (int i =0; i<1000000; i++)
		{}
	}
}
