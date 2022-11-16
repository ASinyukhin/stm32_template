#ifndef __UTILS_H__
#define __UTILS_H__
#include <stdint.h>

#define MAX(x,y) ( ((x)>(y))?(x):(y) )
#define MIN(x,y) ( ((x)<(y))?(x):(y) )

void delay(uint32_t ticks) ;

void delay_us(uint32_t us);

#endif
