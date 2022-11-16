#include <gmg12864.h>
#include <spi.h>
#include <utils.h>

#define BUF_SIZE ((DISPLAY_H/8) * DISPLAY_W)

static uint8_t PixBuff[BUF_SIZE] = {0};

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

void clearPixBuf() {
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
	uint8_t page_end = (y + h) / 8;
	if (page_end >= 7)
		page_end = 7;
    //column address
	uint8_t msb = (x >> 4) & 0x0F;
	uint8_t lsb = x & 0x0F;

	for (int page=page_start; page <= page_end; page++) {
		displaySendCmd(0xB0 | page);
		displaySendCmd(0b00010000 | msb);
		displaySendCmd(0b00000000 | lsb);
		displaySendMultiple(&PixBuff[page * DISPLAY_W + x], w);
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

void displayInitHw() {
    SPI_init();
    //Init additional lines (RS/A0 and RST)
	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF4|GPIO_CRL_MODE4 |
		GPIO_CRL_CNF3|GPIO_CRL_MODE3) |
		GPIO_CRL_MODE3_1|GPIO_CRL_MODE3_0|
		GPIO_CRL_MODE4_1|GPIO_CRL_MODE4_0;
    // enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
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
