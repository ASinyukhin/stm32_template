#include <stdint.h>
#include <stddef.h>
#include <stm32f10x.h> //Not used when we use libopencm3
#include <stdbool.h>
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

/************************************************************/

#define DISPLAY_H 64
#define DISPLAY_W 130

#define BUF_SIZE ((DISPLAY_H/8) * DISPLAY_W)

static uint8_t PixBuff[BUF_SIZE] = {0};

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
	SPI1->CR1 = SPI_CR1_BR_0| // SPI_CR1_BR_2| // SPI_CR1_BR_0|SPI_CR1_BR_1|SPI_CR1_BR_2|
		SPI_CR1_SSM |
		SPI_CR1_SSI | SPI_CR1_MSTR;
	SPI1->CR1 |= SPI_CR1_CPOL|SPI_CR1_CPHA;
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

#define RS_LINE_NO  (4)
#define RST_LINE_NO (3)

#define GPIO_WRITE_BIT(bit, level) do {\
	GPIOA->BSRR = ((~(level) & 1) << ((bit) + 16)) | (((level) & 1) << (bit));\
} while (0)

#define DISPLAY_SET_RS(level) GPIO_WRITE_BIT(RS_LINE_NO, level)
#define DISPLAY_RESET(level)  GPIO_WRITE_BIT(RST_LINE_NO, level)

#define DISPLAY_ON            ((uint8_t) 0b10101111)
#define DIPLAY_SET_START_LINE ((uint8_t) 0b01000000)
#define DISPLAY_SET_PAGE_ADDR ((uint8_t) 0b10110000)

void displaySendCmd(uint8_t cmd) {
	DISPLAY_SET_RS(false); //command
	SPI_send(SPI1, cmd);
	SPI_waitBusy(SPI1);
}

void displaySendData(uint8_t data) {
	DISPLAY_SET_RS(true); //data
	SPI_send(SPI1, data);
	SPI_waitBusy(SPI1);
}

void displaySendMultiple(uint8_t *data, size_t count) {
	DISPLAY_SET_RS(true);
	for (int i=0; i<count; i++) {
		SPI_send(SPI1, data[i]);
	}
	SPI_waitBusy(SPI1);
}

static void clearPixBuf() {
	for (int i=0; i<BUF_SIZE; i++)
		PixBuff[i] = 0;
}

// Fast copying from pixbuf into a DDRAM
void displayBlitScreen() {
	SPI_select();
	for (int page=0; page<8; page++) {
		displaySendCmd(0xB0 | page);
		displaySendMultiple(&PixBuff[page * DISPLAY_W], DISPLAY_W);
		displaySendCmd(0xEE);
	}
	SPI_deselect();
}

// Fast blit rectangle onto display DDRAM
void displayBlitRect(int x, int y, int w, int h) {
	SPI_select();
	uint8_t page_start = (y / 8) & 0x0F;
	uint8_t page_end = ((y + h) / 8);
	if (page_end >= 7)
		page_end = 7;

	uint8_t msb = (x >> 4) & 0x0F;
	uint8_t lsb = x & 0x0F;

	for (int page=page_start; page <= page_end; page++) {
		displaySendCmd(0xB0 | page);
		displaySendCmd(0b00010000 | msb);
		displaySendCmd(0b00000000 | lsb);
		displaySendMultiple(&PixBuff[page * DISPLAY_W], w);
		displaySendCmd(0xEE);
	}
	SPI_deselect();
}

void displayClear() {
	SPI_select();
	displaySendCmd(0x40 | 0x00); // Set start line address (Lines 0x00...0x3F)
	for(int k=0; k<=7; k++) { // Clear DRAM
    	displaySendCmd(0xB0 | k); // Set Page 0 (Pages 0x00...0x0F)
    	for(int i=0; i<DISPLAY_W; i++)
			displaySendData(0x0); //displaySendData(0b01010101);
    	displaySendCmd(0xEE); // End writing to the page, return the page address back
  	}
	clearPixBuf();
	SPI_deselect();
}

void pixbufSetPixel(int x, int y, bool color) {
	uint8_t safe_x = x % DISPLAY_W;
	uint8_t page = (y / 8) & 0x0F;
	int place = page * DISPLAY_W + safe_x;
	uint8_t data = PixBuff[place]; //one small column
	// clear pixel and set new value
	uint8_t new_data = data & ~(1 << (y % 8)) | (color << (y % 8));
	PixBuff[place] = new_data;
}

// Put Pixel into DRAM. Slow implementation
void displayPutPixel(int x, int y, bool color) { //x in [0..64], y in [0..130]
	uint8_t safe_x = x % DISPLAY_W;
	SPI_select();
	uint8_t page = (y / 8) & 0x0F;
	uint8_t msb = (x >> 4) & 0x0F;
	uint8_t lsb = x & 0x0F;
	//set page address
	displaySendCmd(0b10110000 | page);
	//Column address set msb
	displaySendCmd(0b00010000 | msb);
	//Column address set lsb
	displaySendCmd(0b00000000 | lsb);
	int place = page * DISPLAY_W + safe_x;
	uint8_t data = PixBuff[place]; //one small column
	// clear pixel and set new value
	uint8_t new_data = data & ~(1 << (y % 8)) | (color << (y % 8));
	displaySendData(new_data);
	PixBuff[place] = new_data;
	SPI_deselect();
}

void displayInit() {
	SPI_select();
	DISPLAY_RESET(false);
	delay_us(10000); //10ms
	DISPLAY_RESET(true);
	delay_us(1000);
	/* LCD bias setting (11) */
	displaySendCmd(0b10100011); //1/7 bias
	/* ADC selection */
	displaySendCmd(0b10100000);
	/* Common output mode selection */
	displaySendCmd(0b11000000);
	/* Power control mode */
	displaySendCmd(0x28 | 0b111);  //0b111
	/* Vo regulator resistor ratio (check) */
	uint8_t resRatio = 0x04;
	displaySendCmd(0b00100000 | resRatio);
  	displaySendCmd(0xA6); // Normal color, A7 = inverse color
  	displaySendCmd(0xAF); // Display on
	SPI_deselect();
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
