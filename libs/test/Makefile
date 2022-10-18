CC ?= gcc
AR ?= ar

OPT ?=-O2

TARGETS ?= stm32/f1

all: test.a

src.o:
	echo $(OPT)
	echo $(TARGETS)
	$(CC) $(OPT) -c src.c -o src.o

test.a: src.o
	$(AR) rcv libtest.a src.o 
	
clean:
	rm -f *.o *.a
