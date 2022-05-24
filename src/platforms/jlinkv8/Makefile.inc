CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

OPT_FLAGS = -Os
CFLAGS += -mcpu=arm7tdmi -mthumb-interwork

LDFLAGS = -nostartfiles -lc -lnosys -Wl,-Map=mapfile -mthumb-interwork -mcpu=arm7tdmi -Wl,-gc-sections

ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

SRC +=	serialno.c	\
		timing.c	\
		timing_jlinkv8.c

all:	blackmagic.bin

host_clean:
	-$(Q)$(RM) *.bin *elf *hex
