#include <stdint.h>
//#include <stm32f10x.h> //Not used when we use libopencm3
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		//__NOP();
		__asm volatile("nop");
	}
}

int __attribute((noreturn)) main(void) {
	#if 0
	// Enable clock for AFIO
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	// Enable PC13 push-pull mode
	GPIOC->CRH &= ~GPIO_CRH_CNF13; //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0; //Max speed = 10Mhz

    while (1) {
	    GPIOC->ODR |= (1U<<13U); //U -- unsigned suffix (to avoid syntax warnings in IDE)
		delay(1000000);
	    GPIOC->ODR &= ~(1U<<13U);
	    delay(1000000);
    }
	#endif
	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN; //TODO: check for rcc_ function to enable gpio clock

	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	while (1) {
		gpio_toggle(GPIOC, GPIO13);
		delay(1000000);
	}
}
