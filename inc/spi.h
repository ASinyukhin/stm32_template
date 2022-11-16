#ifndef __SPI_CMSIS_H__
#define __SPI_CMSIS_H__

#define CS_LINE_NO  (2)
#define RS_LINE_NO  (4)
#define RST_LINE_NO (3)

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>

void SPI_send(SPI_TypeDef *spi, uint16_t data);

uint16_t SPI_read(SPI_TypeDef *spi);

void SPI_waitBusy(SPI_TypeDef *spi);

void SPI_select();

void SPI_deselect();

void SPI_init();

#endif
