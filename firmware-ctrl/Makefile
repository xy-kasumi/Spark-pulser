# ============ CONFIGURATION ============
MCU      = attiny1616
TARGET   = main

# ============ TOOLCHAIN ============
CC       = avr-gcc
OBJCOPY  = avr-objcopy
SIZE     = avr-size

# ATtiny Device Family Pack (provides headers for tinyAVR 1-series)
DFP      = ./dfp
DFP_URL  = https://packs.download.microchip.com/Microchip.ATtiny_DFP.3.1.260.atpack

CFLAGS   = -mmcu=$(MCU) -B $(DFP)/gcc/dev/$(MCU) -I $(DFP)/include \
           -O2 -Wall -std=gnu11
LDFLAGS  = -mmcu=$(MCU) -B $(DFP)/gcc/dev/$(MCU)

# ============ TARGETS ============
all: $(TARGET).hex size

$(TARGET).elf: $(TARGET).c config.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

size: $(TARGET).elf
	$(SIZE) --mcu=$(MCU) -C $<

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

format:
	clang-format -i $(TARGET).c config.h

clean:
	rm -f $(TARGET).elf $(TARGET).hex

distclean: clean
	rm -rf $(DFP)

.PHONY: all setup format clean distclean size
