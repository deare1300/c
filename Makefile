#local
ifneq ($(KERNELRELEASE),)  
obj-m :=rs232_serial.o  
else  
KERNELDIR ?= /lib/modules/$(shell uname -r)/build  
PWD := $(shell pwd)  
default:  
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules  
endif

clean:
	rm -rf *.o *.ko *.mod.c
