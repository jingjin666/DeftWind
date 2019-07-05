#
# Makefile for the uavrs-v1_DeftWind configuration
#

#
# DeftWind modules
#
MODULES		+= $(DEFTWIND_MODULE_DIR)

#
# Board drivers
#
MODULES			+= drivers/boards/uavrs-v1
MODULES			+= drivers/device
MODULES			+= drivers/stm32
MODULES			+= drivers/stm32/adc
MODULES			+= drivers/led
MODULES			+= drivers/fmu

#
# System commands
#
MODULES			+= systemcmds/bl_update
MODULES			+= systemcmds/mixer
MODULES			+= systemcmds/perf
MODULES			+= systemcmds/reboot
MODULES			+= systemcmds/top
MODULES			+= systemcmds/nshterm
MODULES   		+= systemcmds/mtd
MODULES   		+= systemcmds/ver
MODULES         += systemcmds/usb_connected
MODULES			+= systemcmds/tests

#
# Examples modules
#
MODULES		+= examples/hello_world
MODULES		+= examples/hello_world_cxx

#
# Board drivers
#
MODULES		+= drivers/boards/uavrs-v1

#
# Transitional support - add commands from the NuttX export archive.
#
# In general, these should move to modules over time.
#
# Each entry here is <command>.<priority>.<stacksize>.<entrypoint> but we use a helper macro
# to make the table a bit more readable.
#
define _B
	$(strip $1).$(or $(strip $2),SCHED_PRIORITY_DEFAULT).$(or $(strip $3),CONFIG_PTHREAD_STACK_DEFAULT).$(strip $4)
endef

#                  command                 priority                   stack  entrypoint
BUILTIN_COMMANDS := \
	$(call _B, sercon,                 ,                          2048,  sercon_main                ) \
	$(call _B, serdis,                 ,                          2048,  serdis_main                )
