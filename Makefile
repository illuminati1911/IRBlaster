obj-m += kernel/irblaster.o

all: kernel example

.PHONY: example
example:
	$(CC) example/test.c -o test

.PHONY: kernel
kernel:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm test