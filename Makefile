
EXTRA_CFLAGS=-I$(PWD)/include

obj-m += lib1000.o

all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean


