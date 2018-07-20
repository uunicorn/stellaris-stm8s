# Location where ek-lm3s1968.zip was unzipped
EK_DIR=/opt/stellaris/ek-lm3s1968

CC=arm-none-eabi-gcc
CFLAGS= \
    -I$(EK_DIR)/third_party/lwip-1.3.1/apps -I$(EK_DIR)/third_party -I$(EK_DIR)/third_party/lwip-1.3.1/src/include  \
    -I$(EK_DIR) -Wall -O99 -mcpu=cortex-m3 -mthumb -nostdlib -nostartfiles -ffreestanding  -g -Dgcc
OCDFLAGS=-f /usr/share/openocd/scripts/interface/ftdi/luminary.cfg -f /usr/share/openocd/scripts/target/stellaris.cfg
LDFLAGS=-T blinky_codered.ld -L. -lutils -ldrivers

UTILS_OBJS= cmdline.o cpu_usage.o flash_pb.o isqrt.o ringbuf.o sine.o uartstdio.o ustdlib.o
DRIVERS_OBJS= adc.o can.o comp.o cpu.o epi.o ethernet.o flash.o gpio.o hibernate.o i2c.o i2s.o interrupt.o mpu.o pwm.o qei.o ssi.o sysctl.o systick.o timer.o uart.o udma.o usb.o watchdog.o
LIBS=libutils.a libdrivers.a
OBJS=startup_codered.o blinky.o

VPATH=$(EK_DIR)/utils:$(EK_DIR)/driverlib

all: out.elf

libutils.a: $(UTILS_OBJS)
	$(AR) rs libutils.a $(UTILS_OBJS)

libdrivers.a: $(DRIVERS_OBJS)
	$(AR) rs libdrivers.a $(DRIVERS_OBJS)

out.bin:  out.elf
	arm-none-eabi-objcopy out.elf out.bin -O binary

out.elf: $(OBJS) $(LIBS)
	$(CC) $(CFLAGS) $(OBJS) -o out.elf $(LDFLAGS)

install: out.bin
	openocd $(OCDFLAGS) -c 'program out.bin'

clean:
	rm -f out.bin out.elf $(OBJS) $(LIBS) $(UTILS_OBJS) $(DRIVERS_OBJS)
