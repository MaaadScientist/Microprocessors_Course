#include "stm32f4xx.h"
#include "math.h"
/* Name : ADC Task sending potentiometer position on PC
 * Temperator sensor connected on the board
 * Potentiometer reading
 *
 */
#define setbit(reg, bit)      ((reg) |= (1U << (bit)))
#define clearbit(reg, bit)    ((reg) &= (~(1U << (bit))))
#define togglebit(reg, bit)   ((reg) ^= (1U << (bit)))
#define getbit(reg, bit)      (((reg) & (1U << (bit))) >> (bit))
#define	sendserial(log, reg, bit)   ((log) ? setbit(reg, bit) : clearbit(reg, bit))

#define BIT12
#define LOSAMP

void initGPIO()
{
    RCC->AHB1ENR |= 0x01;

    /* configure PA0 as ADC_IN0 */
    GPIOA->MODER   |=  0x03;     /* analog mode */
    GPIOA->OSPEEDR |=  0x03;     /* high speed */
}

void initADC1()
{
    // initialize the HSI clock
    setbit(RCC->CR, 0);         // enable HSI
    while (!getbit(RCC->CR, 1));// wait until HSI stable

    // initialize the ADC
    setbit(RCC->APB2ENR, 8);     // enable ADC1 peripheral clock
    ADC->CCR = 0;                 // disable temperature sensor, ADC prescaler = HSI/2
    clearbit(ADC1->SQR3, 0);    // 1st conversion in regular sequence will be from channel0
    clearbit(ADC1->SQR3, 1);    // reset state - all conversions from channel0 (PA_0)
    clearbit(ADC1->SQR3, 2);    // this is just an example
    clearbit(ADC1->SQR3, 3);
    clearbit(ADC1->SQR3, 4);
#ifdef BIT12
    clearbit(ADC1->CR1, 24);     // 12-bit resolution (Tconv = 15 ADCCLK cycles)
    clearbit(ADC1->CR1, 25);    // reset state
#endif
#ifdef BIT10
    setbit(ADC1->CR1, 24);        // 10-bit resolution (Tconv = 13 ADCCLK cycles)
#endif
#ifdef BIT8
    setbit(ADC1->CR1, 25);         // 8-bit resolution (Tconv =  11 ADCCLK cycles)
#endif
    clearbit(ADC1->CR2, 11);     // right alignment, reset state
#ifdef LOSAMP
    clearbit(ADC1->SMPR2, 0);    // channel0 sample rate: 3 cycles
    clearbit(ADC1->SMPR2, 1);    // reset state
    clearbit(ADC1->SMPR2, 2);
#endif
#ifdef HISAMP
    setbit(ADC1->SMPR2, 0);     // channel0 sample rate: 480 cycles
    setbit(ADC1->SMPR2, 1);
    setbit(ADC1->SMPR2, 2);
#endif
    setbit(ADC1->CR2, 0); // enable the ADC
}

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

void delay(int x)
{
    volatile int i;
    for (i = 0; i < 1000*x; i++);
}

void usart_tx(unsigned char ch)
{
	while(!readbit(USART2->SR,7));
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

float pos2vol(int pos)
{
	int n=8;
	#ifdef BIT10
    	   n=10;
	#endif
	#ifdef BIT12
    	   n=12;
	#endif

	float vol;
	vol=pos*3.3/(pow(2,n)-1);
	return vol;
}

int main()
{
    int i = 0;
    float vol;

    initGPIO();
    initADC1();
    char str[12];

    serialInit();
    sendString("ADC Test\n\r",10);


    while(1)
    {
        // single conversion mode, sec. 11.3.4 in RM (p. 841)
        setbit(ADC1->CR2, 30);             // software ADC start
        while (!getbit(ADC1->SR, 1));     // wait until conversion end
        i = ADC1->DR;
        sprintf(str,"%d", i);
        sendString(str, 12);
        vol=pos2vol(i);
        delay(10);
    }
}

/*
 * Voltage value = i/2^(n-1)*Vref
 * Vref = 3.3 V
 *
 */
