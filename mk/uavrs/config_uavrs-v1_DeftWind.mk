#
# Makefile for the uavrs-v1_DeftWind configuration
#

#
# DeftWind modules
#
MODULES		+= $(DEFTWIND_MODULE_DIR)

#
# Examples modules
#
MODULES		+= examples/hello_world

#
# Library modules
#
#MODULES		+= modules/uORB

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
