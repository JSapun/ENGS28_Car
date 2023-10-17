# MAIN is the name of the top-level program in your project (without the .c extension)
MAIN = motor

# LIBRARIES are the libraries in your LIBDIR folder that the current project uses
LIBRARIES = ioE28 USARTE28 tb6612_T0 ADC i2c SevenSeg


TOPDIR = .

# LIBDIR is the path to the folder (relative to your project folder) that contains 
# additional libraries that your project may use.
LIBDIR      = ${TOPDIR}/mylib
LIBDIRS = $(addprefix $(LIBDIR)/, $(LIBRARIES))
LIBSOURCES = $(addsuffix .c, $(LIBDIRS))


# These are Airlift shield specific
AIRLIFT    = AdafruitIODrv wifiDrv wifiSpiDrv
AIRLIFT_DIR = ${TOPDIR}/airlift
AIRLIFT_DIRS = $(addprefix $(AIRLIFT_DIR)/, $(AIRLIFT))
AIRLIFT_SRCS = $(addsuffix .c, $(AIRLIFT_DIRS))


# automatically add paths and extensions to libraries

# SOURCES is the name of all the source files in your project (including $(MAIN))
# The first version automatically includes source files from your library directory
#SOURCES = $(wildcard $(MAIN).c $(LIBDIR)/*.c)
# The second version only includes those files in your library directory that you 
# selected in LIBRARIES.
SOURCES = $(MAIN).c $(LIBSOURCES) $(AIRLIFT_SRCS)
# No libraries version
#SOURCES = $(MAIN).c

# For MacOS & Linux:
# Update this line with the correct number for /dev/tty.usbmodem* for your setup
# (type ls /dev/tty.usbmodem*).
# This will likely change every time you plug the Arduino into your computer.
#PROGRAMMER = -c arduino -b 115200 -P /dev/tty.usbmodem*

# For Windows:
PROGRAMMER = -c arduino -b 115200 -P COM3

# Shouldn't have to change these unless you decide to target a different Arduino
# than the UNO.
DEVICE     = atmega328p
CLOCK      = 16000000
FUSES      = -U hfuse:w:0xde:m -U lfuse:w:0xff:m -U efuse:w:0x05:m

# Find header and object files corresponding to your sources
OBJECTS=$(SOURCES:.c=.o)
HEADERS=$(SOURCES:.c=.h)

# Tune the lines below only if you know what you are doing:
# -std=c99 imposes C99 language standard, as recommended by Barr and JPL for embedded; 
#  default is -std=gnu11 (gnu C 2011)
# -Wall enables all warnings about constructions that some users consider questionable
#  and are easily avoided 
# -O optimizes for code size and execution time without increasing compilation time; 
# -Os optimizes only for code size
# -DF_CPU passes the clock speed down to units that need it, e.g., delay functions
CFLAGS = -std=c99 -Wall -O -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -I. -I$(LIBDIR) -I$(AIRLIFT_DIR)
COMPILE = avr-gcc $(CFLAGS)
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)

# symbolic targets:
all:	$(MAIN).hex

.c.o: $(HEADERS) Makefile
	 $(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:$(MAIN).hex:i

fuse:
	$(AVRDUDE) $(FUSES)

## Xcode uses the Makefile targets "", "clean" and "install"
#install: flash fuse

## if you use a bootloader, change the command below appropriately:
#load: all
#	bootloadHID main.hex

clean:
	rm -f $(MAIN).hex $(MAIN).elf $(MAIN).o

cleanall: clean
	rm -f $(OBJECTS)

clobber: cleanall
	@rm -f *.hex
	@rm -f *.elf
	@rm -f *.o

# file targets:
$(MAIN).elf: $(OBJECTS)
	$(COMPILE) $^ $(LDLIBS) -o $@

$(MAIN).hex: $(MAIN).elf
	rm -f $(MAIN).hex
	avr-objcopy -j .text -j .data -O ihex $(MAIN).elf $(MAIN).hex
	avr-size --format=avr --mcu=$(DEVICE) $(MAIN).elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	$(MAIN).elf
	avr-objdump -d $(MAIN).elf

cpp:
	$(COMPILE) -E $(SOURCES)

# All targets - not super useful, but good for build regression
testBuild : 
	make MAIN=AirLift_blinky
	make MAIN=AirLift_TestWifi
	make MAIN=AdafruitIOTestLEDOnOffWithLibrary
	make MAIN=AdafruitIOTestGetDataWithLibrary
	make MAIN=AdafruitIOTestPostDataWithLibrary

