obj-m += kernel/irblaster.o

all: rpi3 example

.PHONY: example
example:
	$(CC) example/test.c -o test

rpi0: rpi1

rpi1:
	make CFLAGS_MODULE=-DBCM_P_BASE=0x20000000 -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

rpi2: rpi3

rpi3:
	make CFLAGS_MODULE=-DBCM_P_BASE=0x3F000000 -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

rpi4:
	make CFLAGS_MODULE=-DBCM_P_BASE=0x7E000000 -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm test