ifneq ($(KERNELRELEASE),)
	obj-m := mySignalLED.o myTempSensor.o myTempSensor_comp.o 
else
	KERNELDIR := $(EC535)/bbb/stock/stock-linux-4.19.82-ti-rt-r33
	PWD := $(shell pwd)
	ARCH := arm
	CROSS := arm-linux-gnueabihf-
	CC := $(CROSS)gcc

default: modules

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS) modules

ul_program: ul_program.c
	$(CC) -o ul_program ul_program.c -lpthread -Wall -Wextra

ul_program_comp: ul_program_comp.c
	$(CC) -o ul_program_comp ul_program_comp.c -lpthread -Wall -Wextra

all: modules ul_program ul_program_comp

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) ARCH=$(ARCH) clean
	rm -f ul_program ul_program_comp

endif