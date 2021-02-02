#include "stm32f4xx.h"

#define setbit(reg, bit) ((reg) |= (1U << (bit)))
#define clearbit(reg, bit) ((reg) &= (~(1U << (bit))))
#define togglebit(reg, bit) ((reg) ^= (1U << (bit)))
#define getbit(reg, bit) ((reg & (1U << bit)) >> bit)

void EXTI15_10_IRQHandler()
{
    // change PR (pending interrupt) to 1, if is 0
    setbit(EXTI->PR, 13);
    // change of system LED
    togglebit(GPIOA->ODR, 5);
}

void buttonConfig(void)
{
    setbit(RCC->AHB1ENR, 2);
    // RCC->AHB1ENR |= 0x05; -- two GPIO's at once
    // activate pull-up
    setbit(GPIOC->PUPDR, 26);

    // System configuration controller clock enable, page 121
    setbit(RCC->APB2ENR, 14);

    // mask
    setbit(EXTI->IMR, 13);

    // IRQ on falling / rising
    setbit(EXTI->FTSR, 13);

    // GPIOC 13 will be source of IRQ
    setbit(SYSCFG->EXTICR[3], 5);

    // enable IRQ
    NVIC_EnableIRQ(EXTI15_10_IRQn); // 40 ~ EXTI15_10_IRQn

    // the same !
    setbit(NVIC->ISER[40/32], 40%32);
}

void ledConfig()
{
    // minimal config
    setbit(RCC->AHB1ENR, 0);
    setbit(GPIOA->MODER, 10);
}

int main(void)
{
    ledConfig();
    buttonConfig();

    while (1)
    {
    	//everything is supposed to be handled by interrutps
    	// boring stuff here
    }
}
