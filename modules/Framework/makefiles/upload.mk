#
# Rules and tools for uploading firmware to various DP boards.
#

UPLOADER		 = $(DP_BASE)/Tools/px_uploader.py

SYSTYPE			:= $(shell uname -s)

#
# Serial port defaults.
#
# XXX The uploader should be smarter than this.
#
ifeq ($(SYSTYPE),Darwin)
SERIAL_PORTS		?= "/dev/tty.usbmodemPX*,/dev/tty.usbmodem*"
endif
ifeq ($(SYSTYPE),Linux)
SERIAL_PORTS="/dev/ttyACM*,/dev/serial/by-id/*_PX4_*,/dev/serial/by-id/usb-3D_Robotics*,/dev/serial/by-id/usb-The_Autopilot*,/dev/serial/by-id/usb-Bitcraze*,/dev/serial/by-id/pci-Bitcraze*,/dev/serial/by-id/usb-Gumstix*,"
endif
ifeq ($(SERIAL_PORTS),)
SERIAL_PORTS		 = "COM32,COM31,COM30,COM29,COM28,COM27,COM26,COM25,COM24,COM23,COM22,COM21,COM20,COM19,COM18,COM17,COM16,COM15,COM14,COM13,COM12,COM11,COM10,COM9,COM8,COM7,COM6,COM5,COM4,COM3,COM2,COM1,COM0"
endif

.PHONY:	all upload-$(METHOD)-$(BOARD)
all:	upload-$(METHOD)-$(BOARD)

upload-serial-uavrs-v1:	$(BUNDLE) $(UPLOADER)
	$(Q) $(PYTHON) -u $(UPLOADER) --port $(SERIAL_PORTS) $(BUNDLE)
	
upload-serial-uavrs-v2:	$(BUNDLE) $(UPLOADER)
	$(Q) $(PYTHON) -u $(UPLOADER) --port $(SERIAL_PORTS) $(BUNDLE)