#include <stdint.h>
#include <stdbool.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencmsis/core_cm3.h>
//#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <string.h>

void hard_fault_handler() {
	while (1) {
		;
	}
}



void delay(uint32_t ticks) {
	for (int i=0; i<ticks; i++) {
		;
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

void cmd(uint8_t command) {
	//gpio_set(); //RS
	//spi_send();
	//
}

static TaskHandle_t BlinkTaskHandle = NULL;
static TaskHandle_t ControlHandle = NULL;

static SemaphoreHandle_t Semphr = NULL;

//bool Command = false;
//bool LedState = false;

#define BUFFER_SIZE 128

#define NONE               0
#define COMMAND            1
#define ERROR_BUF_OVERFLOW 2

#define TOGGLE_COMMAND     3


static char Buffer[BUFFER_SIZE] = {0};
static int pos = 0;
//static MsgLen = 0;

//ISR -- Interrupt Service Routine
//Во FreeRTOS приоритет преррывания не выше приоритета планировщика!
void usart1_isr (void) {
	BaseType_t woken = false;
	if (USART_SR(USART1) & USART_SR_RXNE) {
		uint8_t byte = usart_recv(USART1);
		Buffer[pos] = byte;
		pos++;
		if (pos >= BUFFER_SIZE) {
			xTaskNotifyFromISR(ControlHandle, ERROR_BUF_OVERFLOW, eSetValueWithOverwrite, 
				&woken);
		}
		if (byte == '\r' || byte == '\n') {
			xTaskNotifyFromISR(ControlHandle, COMMAND, eSetValueWithOverwrite, 
				&woken);
		}
		portYIELD_FROM_ISR(woken);
	}

	/*
	if (USART_SR(USART1) & USART_SR_RXNE) //RXNE -- not empty 
	//т.е. пришёл байт
	{
		uint8_t byte = usart_recv(USART1);
		switch (byte) {
			case 'E':
			case 'e':
				LedState = true;
				Command = true;
				break;
			case 'D':
			case 'd':
				LedState = false;
				Command = true;
				break;
			default:
				;
		}
	}*/

}

void usart_print(const char *str) {
	while (*str != '\0') {
		usart_send_blocking(USART1, *str);
		str++;
	}
}

//1. Очереди. Queue_t, xQueueHandle_t.
//2. Оповещения Notification. uint32_t. 
//3. Семафоры.
//arg -- параметр задаче (который нам нужен)
void taskBlink(void *arg) {
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, 
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	uint32_t command = 0;
	bool blinkEnabled = false;
	while (1) {
		command = ulTaskNotifyTake(pdTRUE, 0);
		if (command == TOGGLE_COMMAND)
			blinkEnabled = !blinkEnabled;

		if (blinkEnabled) {
			gpio_toggle(GPIOC, GPIO13);
		}
		vTaskDelay(100);
	}
}

void taskControl(void *arg) {
	//USART1
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, 
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9); //PA9 -- TX
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT, GPIO10); //PA10 -- RX
	
	usart_set_baudrate(USART1, 9600);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	//usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	//прерывание в периферии вкл.
	usart_enable_rx_interrupt(USART1);
	//NVIC -- в ядре MCU
	nvic_set_priority(NVIC_USART1_IRQ, 0xb0);
	nvic_enable_irq(NVIC_USART1_IRQ);
	usart_enable(USART1);

	const char *toggleCmd = "toggle";

	while (1) {
		uint32_t value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (value == ERROR_BUF_OVERFLOW) {
			usart_print("ERROR: Buffer overflow\r\n");
		} else if (value == COMMAND) {
			//Parse command
			uint32_t commandType = 0;
			if (strncmp(Buffer, toggleCmd, strlen(toggleCmd)) == 0) {
				commandType = TOGGLE_COMMAND;
			}
			pos = 0;
			if (commandType != 0) {
				xTaskNotify(BlinkTaskHandle, commandType, eSetValueWithOverwrite);
				usart_print("OK\r\n");
			}
		}
		/*
		//Управление кнопками
		uint16_t state = 
			gpio_get(GPIOA, GPIO3|GPIO4|GPIO5);
		if (prevState & ~state ) { //high->low
			uint32_t command = 1;
			//BaseType_t result = xQueueSend(queue, &command, 1);
			//xTaskNotify(BlinkTaskHandle, command, eSetValueWithOverwrite);
			xSemaphoreGive(Semphr);
		}
		if (state & GPIO4) {
			;
		}
		if (state & GPIO5) {
			;
		}
		prevState = state;
		vTaskDelay(20); //20ms
		*/
	}
}

typedef struct {
	int arg1;
	int arg2;
	char str[32];
	QueueHandle_t q;
}TaskArguments_T;

int main(void) {
	rcc_clock_setup_pll (&rcc_hse_configs [RCC_CLOCK_HSE8_72MHZ ]);

#if 0
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, 
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	//PA 3,4,5 -- на вход!
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3|GPIO4|GPIO5);
	//подтягивающие резисторы
	gpio_set(GPIOA, GPIO3|GPIO4|GPIO5);

	const uint32_t t1 = 500; //led switch period
	const uint32_t t2 = 50;  //button check period
	uint32_t p[2] = {t1, t2};
	
	bool prevButtonState = gpio_get(GPIOA, GPIO5);
	bool ledBlink = true;
	while (1) {
		uint32_t tau = MIN(p[0], p[1]);
		delay_ms(tau);
		for (int i=0; i<2; i++) {
			p[i] -= tau;
		}
		if (p[0] == 0) {
			if (ledBlink)
				gpio_toggle(GPIOC, GPIO13);
			p[0] = t1;
		}
		if (p[1] == 0) {
			bool newState = gpio_get(GPIOA, GPIO5);
			if (!newState && prevButtonState)
				ledBlink = !ledBlink; //switch led blink
			prevButtonState = newState;
			p[1] = t2;
		}
	}
#endif
/*
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_SPI1);
	//MISO, MOSI, CLK, NSS, RS, RSE
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
	 GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
	 GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5);
	//CS -- Chip select
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
	 GPIO_CNF_OUTPUT_PUSHPULL, GPIO1|GPIO2|GPIO3);

	//SPI
	spi_init_master(SPI1, SPI_CR1_BR_FPCLK_DIV_64, 
		SPI_CR1_CPOL, SPI_CR1_CPHA, SPI_CR1_DFF_8BIT,
		SPI_CR1_MSBFIRST );
	spi_enable(SPI1);
*/
#if 0	
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, 
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); //PC13 LED
	//USART1
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, 
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9); //PA9 -- TX
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT, GPIO10); //PA10 -- RX
	
	usart_set_baudrate(USART1, 9600);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	//usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	//прерывание в периферии вкл.
	usart_enable_rx_interrupt(USART1);
	//NVIC -- в ядре MCU
	nvic_set_priority(NVIC_USART1_IRQ, 0);
	nvic_enable_irq(NVIC_USART1_IRQ);
	usart_enable(USART1);

	while (1) {
		if (Command) {
			if (LedState)
				gpio_set(GPIOC, GPIO13);
			else
				gpio_clear(GPIOC, GPIO13);
			Command = false;
			usart_print("OK\r\n");
		}
	}
#endif
	QueueHandle_t queue = xQueueCreate(10, sizeof(uint32_t));
	Semphr = xSemaphoreCreateBinary();

	//Task -- задача
	//создаём таск
	xTaskCreate(taskBlink, "blink", 256, queue, 0, &BlinkTaskHandle);

	xTaskCreate(taskControl, "control", 256, queue, 0, &ControlHandle);

	//handle -- "ручка" управления таском
	//передаём управление пранировщику задач
	vTaskStartScheduler();
}
