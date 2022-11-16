#include <spi.h>
#include <stddef.h>

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
	GPIOA->BSRR = (1 << CS_LINE_NO) << 16U; // reset bit
}

void SPI_deselect() {
	GPIOA->BSRR = (1 << CS_LINE_NO); //set bit
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
/*
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
*/