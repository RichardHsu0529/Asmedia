CONFIG_MODULE_SIG=n
obj-m := Gpio-asm28xx.o
KVERSION := $(shell uname -r)

all:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(shell pwd) modules

clean:
	$(MAKE) -C /lib/modules/$(KVERSION)/build M=$(shell pwd) clean
