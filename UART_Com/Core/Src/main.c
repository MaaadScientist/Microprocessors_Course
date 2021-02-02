/*
 * Name : USART Task
 * Author : Quentin MONNIER
 * Date : 21 January 2021
 *
 * Sending and receiving character from computer using USART bus.
 */



#include "stm32f4xx.h"

#define setbit(reg,bit) ((reg) |= (1U << (bit)))
#define clearbit(reg,bit) ((reg) &= (~(1U << (bit))))
#define getbit(reg,bit) ((reg & (1U << bit)) >> bit)
#define togglebit(reg, bit) ((reg) ^= (1U << (bit)))
#define sendserial(log, reg, bit) ((log) ? setbit(reg,bit) : clearbit(reg,bit))


void sendChar (char ch)  {
  // wait till transmit register is empty 7th bit of SR
  while (!getbit(USART2->SR, 7));
  USART2->DR = ch;
}

char receiveChar(char * a) {
   // read from DR while receive register is not empty - 5th bit of SR
   if (getbit(USART2->SR, 5)) {
	   *a = USART2->DR;
	   return 1;
   }
   return 0;
}

void usart_tx(unsigned char ch)
{
	while(!getbit(USART2->SR,7));
	USART2->DR = ch;
}

int usart_rx (unsigned char *x)
{
	if(getbit(USART2->SR, 5)){
		*x = USART2->DR;
		return 1;
	}
	return 0;
}


/*
 * USART2 init
 * USART2 is connected to APB1 bus
 * USART2 uses PA2 as TX and PA3 as RX
 */
void serialInit(void){
	setbit(RCC->APB1ENR, 17);	// Enable APB1 clock
	setbit(USART2->CR1, 13);	// Enable USART2
	setbit(USART2->CR1, 2);		// USART2 - enable receive
	setbit(USART2->CR1, 3);		// USART2 - enable transmit
	USART2->BRR = (104 << 4); 	// USART2 baudrate, clock/(16*baudrate) should work -> 16*10^6/(16*9600) = 104
/*
  * STM32F401 HSI clock - 16 MHz
  * baudrate: 9600
  */


	setbit(RCC->AHB1ENR, 0);	// Enable AHB1 clock
	setbit(GPIOA->MODER, 5);	// PA2 alternate function, p. 157 of reference manual
	setbit(GPIOA->MODER, 7);	// PA3 alternate function

	GPIOA->AFR[0] |= 0x0700;	// alternate function, PA2 will be USART2_TX
								// see p. 162 of reference manual
								// see p. 45 of STM32F401xE datasheet
	GPIOA->AFR[0] |= 0x7000;	// alternate function, PA3
}

void sendString (char * str, int length)
{
	for (int i = 0; i < length; i++)
	{
		sendChar(str[i]);
	}
	sendChar('\n');
	sendChar('\r');
}

int main(void)
{
	char mess[20];
	char a;
	int i = 0;

	serialInit();
	sendString("hello\n\r",7); //notice on serial port when

	while (1)
	{
		if (usart_rx(&a)) // wait for USART register to written
		{
			if (a != '\r' && a != '\n')
				mess[i++] = a;
			else {
				sendString(mess, i); // send back what have been received.
				i = 0;
			}
		}

		//sendString("hello\n\r",7);
	}
}
