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


int __attribute((noreturn)) main(void) {
#if 0 //Простейшая программа "Blink"
	// Enable clock for AFIO
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	// Enable PC13 push-pull mode
	GPIOC->CRH &= ~GPIO_CRH_CNF13; //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0; //Max speed = 10Mhz

    while (1) {
	    GPIOC->ODR |= (1U<<13U); //U -- unsigned suffix (to avoid syntax warnings in IDE)
		delay(5000000);
	    GPIOC->ODR &= ~(1U<<13U);
	    delay(1000000);
    }
#endif
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

	while (1) {
		//Переключение состояния светодиода
		uint16_t gpio = GPIOC->ODR & GPIO_ODR_ODR13;
		GPIOC->BSRR = (gpio << 16) | (~gpio & GPIO_ODR_ODR13);
		//BSRR: [31:16] -- reset, [15:0] -- set
		delay_ms(100);
	}
}
