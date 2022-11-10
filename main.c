#include <stdint.h>
#include <stm32f10x.h> //Not used when we use libopencm3
#include <stdbool.h>
/*
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencmsis/core_cm3.h>
*/
#include <system_stm32f10x.h>

void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		__asm volatile("nop");
	}
}

void delay_us(uint32_t us) { //8 ticks/iteration
	__asm volatile (
		"push {r0}\r\n"
		"mov R0, %0\r\n"
		"_loop:\r\n" //approx. 6ticks/iteration
			"cmp R0, #0\r\n"     //1
			"beq _exit\r\n"      //1 or 1+P (when condition is True)
			"sub R0, R0, #1\r\n" //1
			"nop\r\n" //1 allignment
			"b _loop\r\n" //1+P (pipeline refill) ~4 cycle
		"_exit:\r\n"
		"pop {r0}\r\n"
		:: "r"(9 * us) //for 72Mhz
	);
}

void HardFault_Handler(void) {
	while (1) {
		;
	}
}

/*
//If we use CMSIC interrupts table, we do not want have them redefined in libopencmsis
//Otherwise we should use only libopencm3 irq table and it's startup
#undef TIM2_IRQHandler
//This is a workaround and should be fixed in future
void TIM2_IRQHandler(void) {
	gpio_toggle(GPIOC, GPIO13);
	timer_clear_flag(TIM2, TIM_SR_UIF);
}
*/

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {
		uint32_t _gpios = GPIOC->ODR;
		GPIOC->BSRR = ((_gpios & (1<<13)) << 16) | 
			(~_gpios & (1<<13));
		TIM2->SR &= ~TIM_SR_UIF; //Clear Interrupt flag
	}
}

//#define LED_PORT GPIOC
//#define LED_PIN  GPIO13

//#define BTN_PORT GPIOB
//#define BTN_PIN  GPIO6

void SPI_send(SPI_TypeDef *spi, uint16_t data) {
	while (!(spi->SR & SPI_SR_TXE)) ;
	spi->DR = data;
}

uint16_t SPI_read(SPI_TypeDef *spi) {
	while (!(spi->SR & SPI_SR_RXNE)) ; //wait not empty
	uint16_t data = spi->DR;
	return data;
}

void SPI_waitBusy(SPI_TypeDef *spi) {
	while (spi->SR & SPI_SR_BSY) ;
}

void SPI_select() {
	GPIOA->BSRR = (1<<2) << 16U; //reset GPIOA pin
	//GPIOA->ODR &= ~GPIO_ODR_ODR2; 
}

void SPI_deselect() {
	GPIOA->BSRR = (1<<2); //set GPIOA pin 2
	//GPIOA->ODR |= GPIO_ODR_ODR2;
}

//Init SPI1 with necessary GPIO modes
void SPI_init() {
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	//PA6 -- MISO
	//PA7 -- MOSI
	//PA5 -- SCK
	//PA2 -- NSS
	//clear all modes 
	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF5|GPIO_CRL_MODE5 |
		GPIO_CRL_CNF7|GPIO_CRL_MODE7|
		GPIO_CRL_CNF6|GPIO_CRL_MODE6|
		GPIO_CRL_CNF2|GPIO_CRL_MODE2);
	// set modes
	GPIOA->CRL |= GPIO_CRL_CNF7_1|GPIO_CRL_MODE7_1 |
		GPIO_CRL_CNF5_1|GPIO_CRL_MODE5_1|GPIO_CRL_MODE5_0; //alternate output push-pull 50Mhz
	GPIOA->CRL |= GPIO_CRL_CNF6_0; //input floating
	GPIOA->CRL |= GPIO_CRL_MODE2_1|GPIO_CRL_MODE2_0; // output push-pull 50Mhz
	SPI_deselect();
	
	SPI1->CR2 = 0;
	SPI1->CR1 = SPI_CR1_BR_0|SPI_CR1_BR_1|SPI_CR1_BR_2|
		SPI_CR1_SSM |
		SPI_CR1_SSI | SPI_CR1_MSTR;
}

bool SPI_echo_test() {
	SPI_select();
	SPI_send(SPI1, 0xAA);
	uint16_t read1 = SPI_read(SPI1);
	SPI_send(SPI1, 55);
	uint16_t read2 = SPI_read(SPI1);
	SPI_waitBusy(SPI1);
	SPI_deselect();
	return (read1 == 0xAA && read2 == 55);
}

int __attribute((noreturn)) main(void) {
	/* Timer blinking example */
	#if 0
	//RCC_APB2ENR |= RCC_APB2ENR_IOPCEN; //TODO: check for rcc_ function to enable gpio clock
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	/* Timer blinking example */
	uint32_t tim_pre = 1024;
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_reset_pulse(RST_TIM2);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM2, tim_pre);
	timer_disable_preload(TIM2);
	timer_set_period(TIM2, 30000);
	timer_enable_irq(TIM2, TIM_DIER_UIE); //update IRQ
	
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_clear_pending_irq(NVIC_TIM2_IRQ);
	timer_enable_counter(TIM2);

	while (1) {
		__asm volatile ("nop");
	}
	#endif 
	/* Multiple blinking example */
	#if 0
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);

	int nOutputLines = 4;
	uint32_t ports[] = {GPIOC, GPIOB, GPIOB, GPIOB};
	uint16_t pins[] = {GPIO13, GPIO9, GPIO8, GPIO7};
	uint32_t periods[] = {4000000, 1500000, 2000000, 1500000};
	/* Configure ports */
	for (int i=0; i<nOutputLines; i++) {
		gpio_set_mode(ports[i], GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pins[i]);
	}

	uint32_t phases[nOutputLines]; // delay to next state change
	// set initial phases
	for (int i=0; i<nOutputLines; i++) {
		phases[i] = periods[i];
	}
	phases[3] = 2 * periods[3];
	while (1) {
		/* choose min. time to next state change */
		uint32_t tau = phases[0];
		for (int i=0; i<nOutputLines; i++) {
			if (phases[i] < tau)
				tau = phases[i];
		}
		delay(tau);
		for (int i=0; i<nOutputLines; i++) {
			phases[i] -= tau;
			if (phases[i] == 0) { //It's time to change state
				gpio_toggle(ports[i], pins[i]);
				phases[i] = periods[i]; // reload period value
			}
		}
	}
	#endif
	/* Start/stop blinking via button */
	#if 0 //libopencm3 version
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB); 

	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO6);
	gpio_set(GPIOB, GPIO6);

	uint32_t btnPeriod = 10000;
	uint32_t ledPeriod = 1000000;
	uint32_t btnPhase = btnPeriod;
	uint32_t ledPhase = ledPeriod;
	bool ledEnabled = true;

	bool buttonPrevState = gpio_get(GPIOB, GPIO6);

	while (1) {
		uint32_t tau = btnPhase;
		if (ledPhase < tau)
			tau = ledPhase;
		delay(tau);
		ledPhase -= tau;
		btnPhase -= tau;
		if (btnPhase == 0) { //It's time to check button state
			btnPhase = btnPeriod;
			bool buttonNewState = gpio_get(GPIOB, GPIO6);
			GPIOB->IDR;
			if (!buttonNewState && buttonPrevState) { // button line change level 1->0 
				ledEnabled = !ledEnabled; // Logical NOT operation
			}
			buttonPrevState = buttonNewState;
		}
		if (ledPhase == 0) { //It's time to change led state
			if (ledEnabled)
				gpio_toggle(GPIOC, GPIO13);
			ledPhase = ledPeriod;
		}
	}
	#endif

	#if 0 // CMSIS version
	RCC->APB2ENR = RCC_APB2ENR_IOPBEN|RCC_APB2ENR_IOPCEN;
	// push-pull mode GPIOC pin 13
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0;
	// input pull up mode GPIOB pin 6
	GPIOB->CRL = GPIOB->CRL & ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6) | GPIO_CRL_CNF6_1;
	// enable pull up
	GPIOB->ODR |= (1 << 6);

	uint32_t btnPeriod = 10000;
	uint32_t ledPeriod = 1000000;
	uint32_t btnPhase = btnPeriod;
	uint32_t ledPhase = ledPeriod;
	bool ledEnabled = true;

	bool buttonPrevState = GPIOB->IDR & (1 << 6); // check bit 6 corresponding to pin 6

	while (1) {
		uint32_t tau = btnPhase;
		if (ledPhase < tau)
			tau = ledPhase;
		delay_us(tau);
		ledPhase -= tau;
		btnPhase -= tau;
		if (btnPhase == 0) { //It's time to check button state
			btnPhase = btnPeriod;
			bool buttonNewState = GPIOB->IDR & (1 << 6);
			if (!buttonNewState && buttonPrevState) { // button line change level 1->0 
				ledEnabled = !ledEnabled; // Logical NOT operation
			}
			buttonPrevState = buttonNewState;
		}
		if (ledPhase == 0) { //It's time to change led state
			if (ledEnabled) {
				// toggle with BSRR register
				uint32_t _gpios = GPIOC->ODR;
				GPIOC->BSRR = ((_gpios & (1<<13)) << 16) | (~_gpios & (1<<13));
			}
			ledPhase = ledPeriod;
		}
	}
	#endif
	#if 0
	/* CMSIS Simple timer example */
	// Init GPIO
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0;
	// Init timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

	TIM2->PSC = 36 * 1000 - 1;
	TIM2->ARR = 100;
	TIM2->DIER |= TIM_DIER_UIE; // Enable Update Interrupt in Timer
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn); // Enable IRQ in NVIC
	TIM2->CR1 |= TIM_CR1_CEN; // Start timer
	while (1) {
		__asm volatile ("nop");
	}
	#endif
	/* CMSIS Timer PWM example */
	// Init timer
	#if 0
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;

	//TIM2_CH2
	// Init GPIO in Alternative function push-pull mode
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1) | GPIO_CRL_CNF1_1 | GPIO_CRL_MODE1_0;

	TIM2->PSC = 36 - 1;
	TIM2->ARR = 1024;
	TIM2->CCR2 = 10;
	TIM2->CR1 |= TIM_CR1_ARPE;
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2; //set PWM mode for channel
	TIM2->CCER |= TIM_CCER_CC2E; //Enable timer channel out
	TIM2->CR1 |= TIM_CR1_CEN; // Start timer
	while (1) {
		__asm volatile ("nop");
	}
	#endif

	#if 0
	/* Simple timer example: blink on port PC13 */
	// Init GPIO PC13
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF13|GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0;
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF14|GPIO_CRH_MODE14) | GPIO_CRH_CNF14_1;
	GPIOC->ODR |= GPIO_ODR_ODR14; //pullup
	// TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	// reset TIMER
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
	TIM2->PSC = 36000 - 1; //1tick = 1ms
	TIM2->ARR = 1000;
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
	bool btn_prev = false;
	while (1) {
		bool btn_state = ! (GPIOC->IDR & (1<<14));
		if (btn_state && !btn_prev) { //button is pressed
			if (TIM2->CR1 & TIM_CR1_CEN) {
				TIM2->CR1 &= ~TIM_CR1_CEN; //stop
			} else {
				TIM2->CR1 |= TIM_CR1_CEN; //start
			}
		}
		btn_prev = btn_state;
		delay_us(10000); //10ms
	}
	#endif
	
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0;
	SPI_init();

	// enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;

	while (1) {
		/* test SPI code */

		GPIOC->ODR |= GPIO_ODR_ODR13;
		delay_us(250000);
		GPIOC->ODR &= ~GPIO_ODR_ODR13;
		delay_us(250000);
	}
}
