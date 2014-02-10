CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

all:	lpcspifi_init.bin lpcspifi_write.bin

%.bin:	../../flashstub/%.S
	$(CC) -nostdlib -Wl,-Ttext,0x10080000 $<
	$(OBJCOPY) -O binary a.out $@

clean:
	-rm *.bin
