# tinyAVR 1-series Firmware Makefile
# Supports: ATtiny816, ATtiny1616, ATtiny3216, etc.

# ============ CONFIGURATION ============
MCU      = attiny1616
F_CPU    = 20000000
TARGET   = main

# Serial port for UPDI programmer
PORT     = /dev/ttyUSB0
BAUD     = 230400

# ============ TOOLCHAIN ============
CC       = avr-gcc
OBJCOPY  = avr-objcopy
SIZE     = avr-size
AVRDUDE  = avrdude

# ATtiny Device Family Pack (provides headers for tinyAVR 1-series)
DFP      = ./dfp
DFP_URL  = https://packs.download.microchip.com/Microchip.ATtiny_DFP.3.1.260.atpack

CFLAGS   = -mmcu=$(MCU) -B $(DFP)/gcc/dev/$(MCU) -I $(DFP)/include \
           -DF_CPU=$(F_CPU)UL -Os -Wall -std=gnu11
LDFLAGS  = -mmcu=$(MCU) -B $(DFP)/gcc/dev/$(MCU)

# ============ TARGETS ============
all: $(TARGET).hex size

$(TARGET).elf: $(TARGET).c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

size: $(TARGET).elf
	$(SIZE) --mcu=$(MCU) -C $<

flash: $(TARGET).hex
	$(AVRDUDE) -c serialupdi -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$<:i

fuses:
	$(AVRDUDE) -c serialupdi -p $(MCU) -P $(PORT) -b $(BAUD) -U fuse2:w:0x01:m

info:
	$(AVRDUDE) -c serialupdi -p $(MCU) -P $(PORT) -b $(BAUD) -v

# ============ SETUP ============
setup:
	@if [ ! -d "$(DFP)/include" ]; then \
		echo "Downloading ATtiny DFP (~30MB)..."; \
		curl -L -o dfp.zip "$(DFP_URL)"; \
		mkdir -p $(DFP); \
		python3 -c "import zipfile; zipfile.ZipFile('dfp.zip').extractall('$(DFP)')"; \
		rm dfp.zip; \
		echo "DFP installed to $(DFP)/"; \
	else \
		echo "DFP already exists at $(DFP)/"; \
	fi

clean:
	rm -f $(TARGET).elf $(TARGET).hex

distclean: clean
	rm -rf $(DFP)

.PHONY: all flash fuses info setup clean distclean size
