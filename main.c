#include <stdint.h>
#include <stm32f10x.h>
#include <stdbool.h>

void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		__NOP();
	}
}

#define MIN(a,b) (((a)<=(b))?(a):(b))
#define MAX(a,b) (((a)>=(b))?(a):(b))

//delay в микросекундах. 1 итерация -- 8 тактов, 9 итераций -- 8*9*1/72 = 1мкс
void delay_us(uint32_t us) {
	__asm volatile (
		"push {r0}\r\n"
		"mov R0, %0\r\n"
		"_loop:\r\n" //approx. 8ticks/iteration
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

//задержка в ms. Для больших задержек, учитываем вместимость типа.
//Если us слишком большое, то делаем несколько задержек
void delay_ms(uint32_t ms) {
	const uint32_t maxMs = UINT32_MAX/1000;
	while (ms > 0) {
		uint32_t msToDelay = MIN(ms, maxMs);
		delay_us(msToDelay * 1000);
		ms -= msToDelay;
	}
	//Упрощённо: delay_us(ms * 1000);
}

#define LED_PIN_NO 13


//Port Number: A,B,C,...
//Pin number (line)
void gpioToggle(GPIO_TypeDef* port, uint32_t lineNo) {
	const uint32_t mask = (1<<lineNo);
	uint16_t gpio = port->ODR & mask;
	port->BSRR = (gpio << 16) | (~gpio & mask);
	//BSRR: [31:16] -- reset, [15:0] -- set
}

static volatile uint32_t Counter = 0;

void SysTick_Handler() {
	//Do some periodic action
	Counter++;
}

//getter
uint32_t getSystemCounter() {
	return Counter;
}


/*
void TIM2_IRQHandler() {
	if (TIM2->SR & TIM_SR_CC1IF) {
		gpioToggle(GPIOC, 13);
		TIM2->SR &= ~TIM_SR_CC1IF;
	}
	if (TIM2->SR & TIM_SR_UIF) {
		gpioToggle(GPIOC, 13);
		TIM2->SR &= ~TIM_SR_UIF;
	}
}
*/
//#define BUTTON_MASK (GPIO_IDR_IDR3|GPIO_IDR_IDR4|GPIO_IDR_IDR5)

void TIM2_IRQHandler(void) {
	//SR -- Status Register -- какое событие произошло
	//CC1IF -- флаг события Compare (сравнение)
	if (TIM2->SR & TIM_SR_CC1IF) {
		gpioToggle(GPIOC, 13);
		TIM2->SR &= ~TIM_SR_CC1IF;
	}
	if (TIM2->SR & TIM_SR_UIF != 0) {
		gpioToggle(GPIOC, 13);
		TIM2->SR &= ~TIM_SR_UIF;
	}
}

int __attribute((noreturn)) main(void) {
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	//CRL: пины 0-7, CRH: пины 8-15
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0; //PC13 = output
	//включаем тактирование порта A
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	//конфигурируем GPIOA 3-5 пины на вход
	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF3|GPIO_CRL_CNF4|GPIO_CRL_CNF5 |
	  GPIO_CRL_MODE3|GPIO_CRL_MODE4|GPIO_CRL_MODE5) |
	  GPIO_CRL_CNF3_1|GPIO_CRL_CNF4_1|GPIO_CRL_CNF5_1;
	//включаем подтяжку к питанию
	GPIOA->ODR |= GPIO_ODR_ODR3|GPIO_ODR_ODR4|GPIO_ODR_ODR5;

#if 0
	const uint32_t t1 = 500; //led switch period
	const uint32_t t2 = 50;  //button check period

	uint32_t p[2] = {t1, t2};
	//У кого-то это кнопка PB9
	bool prevButtonState = GPIOA->IDR & GPIO_IDR_IDR5;

	bool ledBlink = true;

	while (1) {
		uint32_t tau = MIN(p[0], p[1]);
		delay_ms(tau);
		for (int i=0; i<2; i++) {
			p[i] -= tau;
		}
		if (p[0] == 0) {
			if (ledBlink)
				gpioToggle(GPIOC, 13);
			p[0] = t1;
		}
		if (p[1] == 0) {
			//PB9 кнопка
			bool newState = GPIOA->IDR & GPIO_IDR_IDR5;
			if (!newState && prevButtonState)
				ledBlink = !ledBlink; //switch led blink
			prevButtonState = newState;
			p[1] = t2;
		}
	}
#endif
	//100us -- 1timer tick 
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = SystemCoreClock/10000;
	TIM2->ARR = 10000;
	TIM2->CCR1 = 1000;
	TIM2->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
	//вкл. прервывание в ядре
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN; //вкл.

	while (1) {
		;
	}
}
