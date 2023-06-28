obj-m := cc2531.o

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile

else
# normal makefile
KDIR ?= /lib/modules/$(shell uname -r)/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

modules_install: default
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

.PHOHY: clean modules_install

endif
