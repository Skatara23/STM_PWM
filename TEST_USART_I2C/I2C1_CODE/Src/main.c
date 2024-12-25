// interfacing of i2c_1 with stm32

#include <stm32f4xx.h>

void I2C_config()
{
	/* step to be followed
	 * 1.  gpio and i2c clock enable
	 * 2.  configure the i2c pins for alternate function
	 * 2.1 select alternate function in moder function
	 * 2.2 select open drain output
	 * 2.3 select high speed for pins
	 * 2.4 select pull up for both the pins
	 * 2.5 configure the alternate function AFR function
	 * 3.  reset the i2c
	 * 4.  program the peripheral input the clock int I2C cr2 register
	 * 	   in order to gererate correct timing
	 * 5.  configure the clock control register
	 * 6. configure the rise time regsiter
	 * 7. programm the i2c _cr1 register to enable the pheripherals.
	 */
	//1. enable the I2C & GPIOB on PB8 AND PB9

	RCC -> AHB1ENR|=(1U<<1);		// enable the i2c on AHB1 bus
	RCC -> APB1ENR|=(1U<<21);		// enbale the GPIOB ON APB1 BUS

	// 2. CONFIGURE THE I2C PINS FOR ALTERNATE FUNCTION
	GPIOB -> MODER |= (2U<<16)|(2U<<18);

	// 2.1 SELECT THE OPEN DRAIN
	GPIOB ->OTYPER |=(1U<<8)|(1U<<9);  // OPEN DRAIN OUPUT SET ON PB8 AND PB9 PG 158

	// 2.2 SELECT THE VERY HIGH  SPEED FOR PINS
	GPIOB -> OSPEEDR |=(3U<<16)|(3U<<18);   // BITS (17:16) = 1:1 AND SAME AS (18:19)

	// 2.3 SELECT THE PULL UP REGISTER FOR BOTH PINS
	GPIOB -> PUPDR |=(1U<<16)|(1U<<18); 			// BITS (17:16 ) = 0:1 FOR PB8 SAME AS (18:19);

	//2.4 CONFIGURE THE ALTERNATE FUNCTION AFR REGISTER
	GPIOB -> AFR[1]=(4U<<0)|(4U<<4);   // BITS (3:2:1:0) = 0:1:0:0 FOR PB8 ,(7:6:5:4) = 0:1:0:0




}

