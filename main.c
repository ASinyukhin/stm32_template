#include <stdint.h>
//#include <stm32f10x.h> //Not used when we use libopencm3
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencmsis/core_cm3.h>

void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		__asm volatile("nop");
	}
}

void HardFault_Handler(void) {
	while (1) {
		;
	}
}


void TIM2_IRQHandler(void) {
//void tim2_isr(void) {
	gpio_toggle(GPIOC, GPIO13);
	timer_clear_flag(TIM2, TIM_SR_UIF);
}

int __attribute((noreturn)) main(void) {

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	//RCC_APB2ENR |= RCC_APB2ENR_IOPCEN; //TODO: check for rcc_ function to enable gpio clock
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART2);

	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	/* Timer blinking example */
	uint32_t tim_pre = 1024;
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_reset_pulse(RST_TIM2);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM2, tim_pre);
	timer_disable_preload(TIM2);
	timer_set_period(TIM2, 20000);
	timer_enable_irq(TIM2, TIM_DIER_UIE); //update IRQ
	
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_clear_pending_irq(NVIC_TIM2_IRQ);
	timer_enable_counter(TIM2);

	while (1) {
		__asm volatile ("nop");
	}
}
