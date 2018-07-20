EK_DIR=/opt/stellaris/ek-lm3s1968
CC=arm-none-eabi-gcc
CFLAGS=-I$(EK_DIR) -Wall -O99 -mcpu=cortex-m3 -mthumb -nostdlib -nostartfiles -ffreestanding  -g
OCDFLAGS=-f /usr/share/openocd/scripts/interface/ftdi/luminary.cfg -f /usr/share/openocd/scripts/target/stellaris.cfg
LDFLAGS=-T blinky_codered.ld -L$(EK_DIR)/driverlib -L$(EK_DIR)/utils -lutils -ldrivers


out.bin:  out.elf
	arm-none-eabi-objcopy out.elf out.bin -O binary

blinky.s: 
	$(CC) $(CFLAGS) blinky.c -S

out.elf: startup_codered.c blinky.c blinky_codered.ld
	$(CC) $(CFLAGS) startup_codered.c blinky.c -o out.elf $(LDFLAGS)

install: out.bin
	openocd $(OCDFLAGS) -c 'program out.bin'

clean:
	rm -f out.bin out.elf
