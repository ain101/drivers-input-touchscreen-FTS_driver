obj-m = ft5x06.o 

INC=-I $(PWD)

ccflags-y := -std=gnu99 $(INC)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

