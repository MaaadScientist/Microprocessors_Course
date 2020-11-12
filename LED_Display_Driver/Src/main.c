/*
 * Name : LED Display Driver
 * Author : Quentin MONNIER
 * Date : 21 October 2020
 *
 * Trying to make a Stopwatch using the 7_segments LEDs
 *
 * Dictionary:
 * GPIO - general purpose input output
 * AHB1 - advanced high-perfomance bus
 * RCC  - reset and clock control
 * AHB1EN - AHB1 enable register
 * MODER - mode register
 * ODR - output data register
 *
 * Notes:
 * - address are in unsigned long format
 *
 * data -> GPIOA9
 * clockk -> GPIOA8
 * latch -> GPIOB5
 */
#include <stdio.h>
// useful macros to set/clear bit of the nuber at given address
#define setbit(reg,bit) ((reg) |= (1U << (bit)))
#define clearbit(reg,bit) ((reg) &= (~(1U << (bit))))
#define getbit(reg,bit) ((reg & (1U << bit)) >> bit)
#define togglebit(reg, bit) ((reg) ^= (1U << (bit)))
#define sendserial(log, reg, bit) ((log) ? setbit(reg,bit) : clearbit(reg,bit))
#define SYSTICK_BASE	0xE000E010

typedef struct
{
	unsigned long CTRL;      /* SYSTICK control and status register,       Address offset: 0x00 */
	unsigned long LOAD;      /* SYSTICK reload value register,             Address offset: 0x04 */
	unsigned long VAL;       /* SYSTICK current value register,            Address offset: 0x08 */
	unsigned long CALIB;     /* SYSTICK calibration value register,        Address offset: 0x0C */
} SYSTICK_type;

#define SYSTICK ((SYSTICK_type *) SYSTICK_BASE)

//bitfield

typedef struct {
	unsigned char B0:1;
	unsigned char B1:1;
	unsigned char B2:1;
	unsigned char B3:1;
	unsigned char B4:1;
	unsigned char B5:1;
	unsigned char B6:1;
	unsigned char B7:1;
} ODR_type;

typedef struct
{
	long sec1;
	long sec2;
	long min1;
	long min2;
} time;
/*
 * Wtf ?
 * struct char, char, int
 * char |  |  .  .  .
 * char |  |  .  .  .
 * int  |  |  |  |  |
 */

typedef struct
{
	unsigned int MODER;
	unsigned int TYPER;
	unsigned int OSPEEDR;
	unsigned int PUPDR;
	unsigned int IDR;
	ODR_type ODR;
	unsigned int BSSR;
	unsigned int BRR;
} GPIO;

/*
 * #define GPIOA = ((GPIO *)GPIOA_BASE)
 * setbit(GPIOA->ODR, 5); // |= (1 <<5)
 * clearbit(GPIOA->ODR, 5); // ~(1 <<5)
 *
 * GPIOA->ODR.B5 = 1;
 *
 * GPIOA->BSSR = 0b00100000; -> set 6th bit of ODR
 * BSSR - 32b - low 16 bits set ones, high 16 bits reset to 0
 * GPIOA->BSSR = 0b00100000 << 16;
 * GPIOA->BRR = 0b00100000; ->reset 6th bit of ODR
 *
 */

#define SYSTICK ((SYSTICK_type *) SYSTICK_BASE)

// boundary addresses of the peripherals (page. 38)
#define PERIPH_BASE     0x40000000
// stack is on the top of the (S)RAM
// adress of the SRAM base
#define SRAM_BASE       0x20000000
// F401 has 96kB of SRAM
#define SRAM_SIZE	1024*96
// top of the SRAM
#define SRAM_END	(SRAM_BASE + SRAM_SIZE)

// RCC base address (pp. 38)
#define RCC_BASE 	(PERIPH_BASE + 0x23800)
#define RCC_CR ((unsigned long *)(RCC_BASE))
#define RCC_CFGR ((unsigned long *)(RCC_BASE + 0x08))
// RCC_APB1ENR offset (pp. 118)
#define RCC_AHB1ENR	((unsigned long *)(RCC_BASE + 0x30))

// GPIOA base address
#define GPIOA_BASE	(PERIPH_BASE + 0x20000)
// MODER address
#define GPIOA_MODER ((unsigned long *)(GPIOA_BASE + 0x00))
// IDR address - input data register
#define GPIOA_OSPEEDR ((unsigned long *)(GPIOA_BASE + 0x08))
#define GPIOA_IDR   ((unsigned long *)(GPIOA_BASE + 0x10))
// ODR address - output data register
#define GPIOA_ODR 	((unsigned long *)(GPIOA_BASE + 0x14))


// GPIOB base address
#define GPIOB_BASE	(PERIPH_BASE + 0x20400)
// MODER address
#define GPIOB_MODER ((unsigned long *)(GPIOB_BASE + 0x00))
#define GPIOB_OSPEEDR ((unsigned long *)(GPIOB_BASE + 0x08))

// IDR address - input data register
#define GPIOB_IDR   ((unsigned long *)(GPIOB_BASE + 0x10))
// ODR address
#define GPIOB_ODR 	((unsigned long *)(GPIOB_BASE + 0x14))

union integer
{
	int A;
	unsigned char B[4];
};

/*
 * union intger var; // declaration
 * var.A = 10;		// | 0x0a | 0x00 | 0x00 | 0x00 |
 *
 *printf("%i\n", var.B[0]);  // -> 10
 *printf("%i\n", var.B[1]);  // -> 0
 *var.B[1] = 1;
 *var.B[1] printf("%i\n", var.A); // -> 266 ~ 10 + 1*266
 *
 *union f
 *{
 *	float f;
 *	struct i {
 *		int sign:1
 *		int mantisa:23
 *		int exponent:23
 *}
 */

static unsigned char SEGMENT_MAP[10] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0X80,0X90}; // byte maps for numbers 1 to 9
static unsigned char SEGMENT_SELECT[4] = {0xF1,0xF2,0xF4,0xF8}; // byte maps to select digit 1 to 4
static unsigned int a = 0;

void systick_handler(void);
void init_systick(unsigned long s, unsigned char en);
void delay_ms(volatile unsigned long s);
int main();
void digit(unsigned char data, unsigned char segment);
void display(unsigned char s0, unsigned char s1, unsigned char s2, unsigned char s3);
int bin2bcd(unsigned int val);
void num2time(time* timer1, long t);
volatile unsigned long seconds = 0;

unsigned long *vector_table[] __attribute__((section(".isr_vector"))) = {
    (unsigned long *)SRAM_END,   	// 0 initial stack pointer
    (unsigned long *)main,       	// 1 main as Reset_Handler
	0,								// 2 NMI
	0,								// 3 Hard Fault
	0,								// 4 Memory management fault
	0, 								// 5 Bus fault
	0,								// 6 Usage fault
	0,								// 7 Reserved
	0,								// 8 .
	0,								// 9 .
	0,								// 10 Reserved
	0,								// 11 SVCall
	0,								// 12 Reserved for debug
	0,								// 13 Reserved
	0,								// 14 PendSV
	(unsigned long *)systick_handler// 15 Systick
};


int main()
{
	printf("init section\n");
	*RCC_AHB1ENR = 0x3; //
	setbit(*RCC_CR, 16);
	setbit(*RCC_AHB1ENR, 0);
	setbit(*RCC_AHB1ENR, 1);
	setbit(*GPIOA_MODER, 16);
	setbit(*GPIOA_MODER, 18);
	setbit(*GPIOB_MODER, 10);
	setbit(*GPIOA_OSPEEDR, 16);
	setbit(*GPIOA_OSPEEDR, 17);
	setbit(*GPIOA_OSPEEDR, 18);
	setbit(*GPIOA_OSPEEDR, 19);
	setbit(*GPIOB_OSPEEDR, 10);
	setbit(*GPIOB_OSPEEDR, 11);

	init_systick(1000, 0);

	unsigned int x;

	x = bin2bcd(a);
	display(x >> 12 & 0x0f, x >> 8 & 0x0f, x >> 4 & 0x0f, x & 0x0f);

	time timer1 ={0,0,0,0};
	//time *timer1 = NULL;
	long t = 0;
	while(1)
	{
		while(getbit(*GPIOA_IDR, 4) == 0)
		{
			delay_ms(1000);
			t++;

			if(t >= 6039)
			{
				t =0;
			}

			num2time(&timer1,t);
			display(timer1.min2, timer1.min1,timer1.sec2, timer1.sec1);
		}

		if(getbit(*GPIOA_IDR, 1) == 1)
		{
			t = 0;
		}

		num2time(&timer1,t);
		display(timer1.min2, timer1.min1,timer1.sec2, timer1.sec1);
		/*int u=a;


		while(getbit(*GPIOA_IDR, 4) == 0)
		{
			u++;
			if(u==255)
			{
				u = 1;
			}
			x= bin2bcd(u);
			display(x >> 12 & 0x0f, x >> 8 & 0x0f, x >> 4 & 0x0f, x & 0x0f);
			delay_ms(1000);
		}

		if(getbit(*GPIOA_IDR, 1) == 1)
		{
			u = 0;

		}

		x= bin2bcd(u);
		display(x >> 12 & 0x0f, x >> 8 & 0x0f, x >> 4 & 0x0f, x & 0x0f);
		//x = bin2bcd(a);
		//display(x >> 12 & 0x0f, x >> 8 & 0x0f, x >> 4 & 0x0f, x & 0x0f);
		 *
		 *
		 */
		//display(1,2,3,4);
	}





}

void systick_handler(void) {
	printf("seconds = %lu\n", seconds++);
	togglebit(*GPIOA_ODR, 6);
}

void init_systick(unsigned long s, unsigned char en)
{
	// Systick - 24b counter - 8388608
	// Main clock source is running with HSI by default which is at 16 Mhz.
	// SysTick clock source can be set with CTRL register (Bit 2)
	// 0: AHB/8 -> (2 MHz)
	// 1: Processor clock (AHB) -> (16 MHz)
	SYSTICK->CTRL |= 0x00000; // Currently set to run at 2 Mhz -> 5e-7s
	// Enable callback
	SYSTICK->CTRL |= (en << 1); // if argument 'en' is 1, then interrupt is enabled
	// Load the reload value
	SYSTICK->LOAD = s;
	// Set the current value to 0
	SYSTICK->VAL = 0;
	// Enable SysTick
	SYSTICK->CTRL |= (1 << 0);		// setbit(SYSTICK->CTRL, 1)
}

void delay_ms(volatile unsigned long s)
{
	for(s; s>0; s--){
		while(!(SYSTICK->CTRL & (1 << 16))); // Wait until COUNTFLAG is 1
	}
}

void digit(unsigned char data, unsigned char segment)
{
	clearbit (*GPIOB_ODR, 5);

	for (int i = 7; i>=0; --i)
	{
		clearbit(*GPIOA_ODR,8);
		sendserial((SEGMENT_MAP[data] >> i) & 1, *GPIOA_ODR, 9);
		setbit(*GPIOA_ODR, 8);
	}

	for(int i = 7; i >=0; --i)
	{
		clearbit(*GPIOA_ODR, 8);
		sendserial((SEGMENT_SELECT[segment] >> i) & 1, *GPIOA_ODR, 9);
		setbit(*GPIOA_ODR, 8);
	}

	setbit(*GPIOB_ODR, 5);
}

void display(unsigned char s0, unsigned char s1, unsigned char s2, unsigned char s3)
{
	digit(s0,0);
	digit(s1,1);
	digit(s2,2);
	digit(s3,3);
}


int bin2bcd(unsigned int val)
{
	int s = 0;
	int BCD = 0;
	while (val>0)
	{
		BCD += (val%10) << s;
		s+=4;
		val/=10;
	}
	return BCD;
}

void num2time(time* timer1, long t)
{
	long min=t/60;
	long sec=t%60;

	timer1->min2 = min/10;
	timer1->min1 = min%10;
	timer1->sec2 = sec/10;
	timer1->sec1 = sec%10;
}
