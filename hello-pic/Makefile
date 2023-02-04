CWD := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
XC8 := /Applications/microchip/xc8/v2.40/bin/xc8
PP3_DIR := ~/workspace/github/Arduino-PIC-Programmer
PORT := /dev/tty.usbmodem1444301
PIC := 18F47Q43

all: upload

hello_pic.hex: hello_pic.c
	$(XC8) --chip=$(PIC) hello_pic.c

upload: hello_pic.hex
	cd $(PP3_DIR); ./pp3 -c $(PORT) -s 1700 -v 2 -t $(PIC) $(CWD)/hello_pic.hex

clean::
	rm -f *.as
	rm -f *.p1 *.d *.hex *.pre *.lst *.cmf *.hxl *.sdb *.obj *.sym *.rlf *.elf
