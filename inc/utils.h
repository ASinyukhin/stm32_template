#ifndef __UTILS_H__
#define __UTILS_H__
#include <stdint.h>

#define MAX(x,y) ( ((x)>(y))?(x):(y) )
#define MIN(x,y) ( ((x)<(y))?(x):(y) )
#define SIGN(x) ( ((x)>=0) ? (1) : (-1) )
#define ABS(x) ( ((x)>=0)?(x):(-(x)) )
#define SQR(x) ((x)*(x))

#define SWAP(a,b,type) do{type _tmp = a; a = b; b = _tmp; }while(0)

void delay(uint32_t ticks) ;

void delay_us(uint32_t us);

#endif
