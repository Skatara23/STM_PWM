#include <stm32f4xx.h>


void uart_config(void)
{
	/*step to be followed
	1. enable the UART clock & gpio clock setting
	2. configure the UART pins for alternate functions
	3. enable the USART by writing the UE bit in USART_CR1 register to 1.
	4. program the M- bit in USART _CR1 to define the word length.
	5. select the desired baud rate using USART_BRR register.
	6. enable the Transmission/ reception setting the TE & RE bit in UART_CR1 register.
	7. we are using UART2 channel for PA2 & PA3 pin.
	*/

	//1. enable the clocks
	RCC-> AHB1ENR|= (1U<<0);				//for PortA, pin PA2 & PA3
	RCC-> APB1ENR|= (1U<<17); 				//eneable USART

	//2. setting up the gpio moder for PA2 & pa3 PIN for alternate function mode
	GPIOA->MODER|=(2U<<4);		//bits for 5:4 = 1:0 for pa2
	GPIOA->MODER|=(2U<<6);		//bits for 6:7 = 1:0 for pa3

//	GPIOA->MODER|=(1U<<4);		//bits for 5:4 = 1:0 for pa2
	//GPIOA->MODER|=(1U<<6);


	//GPIOA->MODER|=(1U<<10);
	//GPIOA->MODER&=~(1U<<11);

	//2.1 configure the output speed register for high speed data transfer
	GPIOA-> OSPEEDR|= (3U<<4)|(3U<<6);


	//2.2 select the alternate function on pg no - 151 AF7-0111
	GPIOA->AFR[0]|= (7U<<8);
	GPIOA->AFR[0]|=(7U<<12);

	//GPIOA->AFR[0]|=(1U<<9)|(1U<<10)|(1U<<8);		//  FOR PA2
	//GPIOA-> AFR[0]|= (1U<<12)|(1U<<13)|(1U<<14);		//FOR PA3 PIN

	//3.enable the UE bit from USART_CR1 register
	USART2-> CR1= 0X00; 		//clear all bit first
	USART2-> CR1|= (1U<<13); 			//set UE at 13 no

	//4. set M- bit for data length
	USART2->CR1&= ~(1U<<12); 			//for 8 bit data use

	//5. set the baud rate for USART in BRR register, set 115200 baudrate
	//USART2 -> BRR|=(13<<0)|(23<<4);
	//USART2 -> BRR = ((0x0068<<4) + 0x0003);
	USART2-> BRR=0x16d;

	//6. Enable the TE & RE bit from CR1 register
	USART2-> CR1|=(1U<<3); 		// for transmission
	USART2->CR1|=(1U<<2); 		//for reception
}


void UART2_sendstring(char *str)
{
	while(*str != '\0')
		{
			USART2 -> DR = *str;
			while (((USART2 -> SR) & (1U<<6))==0);
			str++;
		}
}

void USART2_transmit(unsigned char data)
{
	USART2 -> DR =data;
	while(((USART2 -> SR )&(1U<<6))==0);
	/* wait here till complete data is not sent*/
}

unsigned char USART2_receive()
{
	while(((USART2 -> SR)&(1U<<5))==0);
	/* wait here till complete data is not recieved */
	return (USART2 -> DR);
}



int main()
{

	uart_config();

	UART2_sendstring("\t reciever -> reciever data \n");
	char ch;
	while(1)
	{
		ch=  USART2_receive();
		USART2_transmit(ch);
	}
}
