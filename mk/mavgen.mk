# mavlink header generation
MAVLINK_DIR := $(SKETCHBOOK)/modules/Mavlink/
MESSAGE_DEFINITIONS := $(SKETCHBOOK)/modules/Mavlink/message_definitions/v1.0
MAVLINK_HEADERS := $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/$(MAVLINK_SUBDIR)/ardupilotmega/mavlink.h $(wildcard $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/$(MAVLINK_SUBDIR)/,*.h) $(wildcard $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/$(MAVLINK_SUBDIR)/ardupilotmega,*.h)
MAVLINK_OUTPUT_DIR := $(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink/$(MAVLINK_SUBDIR)

#$(info MAVLINK_DIR:$(MAVLINK_DIR))
#$(info MESSAGE_DEFINITIONS:$(MESSAGE_DEFINITIONS))
#$(info MAVLINK_HEADERS:$(MAVLINK_HEADERS))
#$(info MAVLINK_OUTPUT_DIR:$(MAVLINK_OUTPUT_DIR))

$(MAVLINK_HEADERS): $(MESSAGE_DEFINITIONS)/ardupilotmega.xml $(MESSAGE_DEFINITIONS)/common.xml
	echo "Generating MAVLink headers..."
	#goto mavlink module directory and run the most recent generator script
	echo "Generating C code using mavgen.py located at" $(SKETCHBOOK)/modules/Mavlink/
	PYTHONPATH=$(MAVLINK_DIR) python $(MAVLINK_DIR)/pymavlink/tools/mavgen.py --lang=C --wire-protocol=$(MAVLINK_WIRE_PROTOCOL) --output=$(MAVLINK_OUTPUT_DIR) $(MESSAGE_DEFINITIONS)/ardupilotmega.xml; if [ $$? -le 0 -o $$? -gt 128 ]; then echo "mavgen: success"; exit 0; else echo "mavgen: failed"; exit 1; fi
