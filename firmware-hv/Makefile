# tinyAVR 1-series Firmware Makefile
# Supports: ATtiny816, ATtiny1616, ATtiny3216, etc.

# ============ CONFIGURATION ============
MCU      = attiny1616
F_CPU    = 20000000
TARGET   = main

# AVR writer (web API)
WRITER   = http://ws-octopus.local:8081

# ============ TOOLCHAIN ============
CC       = avr-gcc
OBJCOPY  = avr-objcopy
SIZE     = avr-size

# ATtiny Device Family Pack (provides headers for tinyAVR 1-series)
DFP      = ./dfp
DFP_URL  = https://packs.download.microchip.com/Microchip.ATtiny_DFP.3.1.260.atpack

CFLAGS   = -mmcu=$(MCU) -B $(DFP)/gcc/dev/$(MCU) -I $(DFP)/include \
           -DF_CPU=$(F_CPU)UL -O2 -Wall -std=gnu11
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
	curl -F part=$(MCU) -F file=@$< $(WRITER)/flash

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

.PHONY: all flash setup clean distclean size
