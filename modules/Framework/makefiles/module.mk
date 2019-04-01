ifeq (MODULE_MK,)
$(error No module makefile specified)
endif
#$(info %% MODULE_MK           = $(MODULE_MK))

#
# Get the board/toolchain config
#

include $(BOARD_FILE)

#
# Get the module's config
#
include $(MODULE_MK)
MODULE_SRC		:= $(dir $(MODULE_MK))
#$(info %  MODULE_NAME         = $(MODULE_NAME))
#$(info %  MODULE_SRC          = $(MODULE_SRC))
#$(info %  MODULE_OBJ          = $(MODULE_OBJ))
#$(info %  MODULE_WORK_DIR     = $(MODULE_WORK_DIR))

#
# Things that, if they change, might affect everything
#
GLOBAL_DEPS		+= $(MAKEFILE_LIST)

################################################################################
# Builtin command definitions
################################################################################
#$(info --MODULE_COMMAND:$(MODULE_COMMAND))
ifneq ($(MODULE_COMMAND),)
MODULE_ENTRYPOINT	?= $(MODULE_COMMAND)_main
MODULE_STACKSIZE	?= CONFIG_PTHREAD_STACK_DEFAULT
MODULE_PRIORITY		?= SCHED_PRIORITY_DEFAULT
MODULE_COMMANDS		+= $(MODULE_COMMAND).$(MODULE_PRIORITY).$(MODULE_STACKSIZE).$(MODULE_ENTRYPOINT)
CXXFLAGS		+= -DDP_MAIN=$(MODULE_COMMAND)_app_main
endif
#$(info --MODULE_COMMANDS:$(MODULE_COMMANDS))
ifneq ($(MODULE_COMMANDS),)
MODULE_COMMAND_FILES	:= $(addprefix $(WORK_DIR)/builtin_commands/COMMAND.,$(MODULE_COMMANDS))
#$(info --MODULE_COMMAND_FILES:$(MODULE_COMMAND_FILES))
# Create the command files
# Ensure that there is only one entry for each command
#
.PHONY: $(MODULE_COMMAND_FILES)
$(MODULE_COMMAND_FILES): command = $(word 2,$(subst ., ,$(notdir $(@))))
$(MODULE_COMMAND_FILES): exclude = $(dir $@)COMMAND.$(command).*
$(MODULE_COMMAND_FILES): $(GLOBAL_DEPS)
	@$(REMOVE) -f $(exclude)
	@$(MKDIR) -p $(dir $@)
	@$(ECHO) "CMD:     $(command)"
	$(Q) $(TOUCH) $@
endif

################################################################################
# Adjust compilation flags to implement EXPORT
################################################################################

ifeq ($(DEFAULT_VISIBILITY),)
DEFAULT_VISIBILITY = hidden
else
DEFAULT_VISIBILITY = default
endif

CFLAGS		+= -fvisibility=$(DEFAULT_VISIBILITY) -include $(FRAMEWORK_INCLUDE_DIR)visibility.h
CXXFLAGS	+= -fvisibility=$(DEFAULT_VISIBILITY) -include $(FRAMEWORK_INCLUDE_DIR)visibility.h	

OBJS	 = $(addsuffix .o,$(SRCS))

#$(info OBJS:$(OBJS))
$(OBJS): $(GLOBAL_DEPS)

vpath %.c $(MODULE_SRC)
$(filter %.c.o,$(OBJS)): %.c.o: %.c $(GLOBAL_DEPS)
	$(call COMPILE,$<,$@)

vpath %.cpp $(MODULE_SRC)
$(filter %.cpp.o,$(OBJS)): %.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

vpath %.S $(MODULE_SRC)
$(filter %.S.o,$(OBJS)): %.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

$(MODULE_OBJ):		$(OBJS) $(GLOBAL_DEPS)
	$(call PRELINK,$@,$(OBJS))

clean:
	$(Q) $(REMOVE) $(MODULE_PRELINK) $(OBJS)
		
#
# What we're going to build
#
module: $(MODULE_OBJ) $(MODULE_COMMAND_FILES)


	
