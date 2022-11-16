#include <stdint.h>
#include <stm32f10x.h> //Not used when we use libopencm3
#include <stdbool.h>
#include <system_stm32f10x.h>
#include <utils.h>
#include <spi.h>
#include <gmg12864.h>

void HardFault_Handler(void) {
	while (1) {
		;
	}
}

int __attribute((noreturn)) main(void) {
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0;
	SPI_init();
	//Init additional lines (RS/A0 and RST)
	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF4|GPIO_CRL_MODE4 |
		GPIO_CRL_CNF3|GPIO_CRL_MODE3) |
		GPIO_CRL_MODE3_1|GPIO_CRL_MODE3_0|
		GPIO_CRL_MODE4_1|GPIO_CRL_MODE4_0;

	// enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
	/* Init display */
	displayInitHw();

	displayInit();

	displayClear();

	uint32_t shift = 0;
	while (1) {
		/* test SPI code */
		GPIOC->ODR |= GPIO_ODR_ODR13;
		clearPixBuf();
		for (int i=0; i<DISPLAY_W; i+=4) {
			for (int j=0; j<DISPLAY_H; j+=4) {
				int i_new = (i + shift) % DISPLAY_W;
				int j_new = (j + shift) % DISPLAY_H;
				pixbufSetPixel(i_new, j_new, true);
			}
		}
		if (shift == 0)
			displayBlitScreen();
		else
			displayBlitRect(0, 0, 16, 16);
		GPIOC->ODR &= ~GPIO_ODR_ODR13;
		delay_us(250000);
		shift = (shift + 1) % 4;
	}
}
