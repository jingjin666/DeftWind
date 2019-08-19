#####################
# 获取setup.mk
#####################
MK_DIR ?= $(dir $(lastword $(MAKEFILE_LIST)))
include $(MK_DIR)setup.mk

#####################
# 引入CONFIG_FILE
# CONFIG = 【xxx_DeftWind】
# CONFIG_FILE = 【~/DeftWind/modules/Framework/makefiles/nuttx/config_xxx_DeftWind.mk】
#####################
ifeq ($(CONFIG),)
$(error Missing configuration name or file (specify with CONFIG=<config>))
endif
export CONFIG
CONFIG_FILE = $(wildcard $(DP_MK_DIR)$(DP_TARGET_OS)/config_$(CONFIG).mk)
include $(CONFIG_FILE)
#$(info %  CONFIG			= $(CONFIG))
#$(info %  CONFIG_FILE	= $(CONFIG_FILE))

#####################
# 引入BOARD_FILE
# BOARD = 【xxx】,由上层传入
# BOARD_FILE = 【~/DeftWind/modules/Framework/makefiles/nuttx/board_xxx.mk】
# 导入toolchain_gnu-arm-eabi.mk
#####################
# The board config in turn will fetch the toolchain configuration.
#
ifeq ($(BOARD),)
BOARD			:= $(firstword $(subst _, ,$(CONFIG)))
endif
BOARD_FILE		:= $(wildcard $(DP_MK_DIR)$(DP_TARGET_OS)/board_$(BOARD).mk)
ifeq ($(BOARD_FILE),)
$(error Config $(CONFIG) references board $(BOARD), but no board definition file found)
endif
export BOARD
export BOARD_FILE
include $(BOARD_FILE)
#$(info %  BOARD               = $(BOARD))
#$(info %  BOARD_FILE      = $(BOARD_FILE))

#####################
# MAKEFILE全局依赖
#####################
GLOBAL_DEPS	+= $(MAKEFILE_LIST)

#####################
# 添加头文件包含 Firmware,BSP驱动
#####################
INCLUDE_DIRS		+= $(DP_MODULE_SRC)drivers/boards/$(BOARD)
INCLUDE_DIRS		+= $(DP_MODULE_SRC)lib/matrix

#####################
# 引入OS编译 【~/DeftWind/modules/Framework/makefiles/nuttx.mk】
# 引入OS头文件,库
#####################
include $(DP_MK_DIR)$(DP_TARGET_OS).mk

#####################
# 编译Modules
# 从目录【~/DeftWind/modules/Framework/Build/uavrs-v2_DeftWind.build/  ~/DeftWind/modules/Framework/src/】搜索可编译module
# 搜索原理为从搜索目录里查找有module.mk文件的目录
# MODULE_SEARCH_DIRS = 所有包含module.mk的文件路径
# MODULES = 排序MODULE_SEARCH_DIRS
# MODULE_MKFILES = 所有的module.mk
#####################
# where to look for modules
MODULE_SEARCH_DIRS	+= $(WORK_DIR) $(MODULE_SRC) $(DP_MODULE_SRC)
#$(info MODULE_SEARCH_DIRS:$(MODULE_SEARCH_DIRS))
# sort and unique the modules list
MODULES			:= $(sort $(MODULES))
#$(info MODULES:$(MODULES))
# locate the first instance of a module by full path or by looking on the
# module search path
define MODULE_SEARCH
	$(firstword $(abspath $(wildcard $(1)/module.mk)) \
		$(abspath $(foreach search_dir,$(MODULE_SEARCH_DIRS),$(wildcard $(search_dir)/$(1)/module.mk))) \
		MISSING_$1)
endef

# make a list of module makefiles and check that we found them all
MODULE_MKFILES		:= $(foreach module,$(MODULES),$(call MODULE_SEARCH,$(module)))
#$(info MODULE_MKFILES:$(MODULE_MKFILES))
MISSING_MODULES		:= $(subst MISSING_,,$(filter MISSING_%,$(MODULE_MKFILES)))
ifneq ($(MISSING_MODULES),)
$(error Cant find module(s): $(MISSING_MODULES))
endif

#####################
# 编译MODULE_OBJS
# 引入【~/DeftWind/modules/Framework/makefiles/modules.mk】进行编译
#####################
MODULE_OBJS		:= $(foreach path,$(dir $(MODULE_MKFILES)),$(WORK_DIR)$(path)module.pre.o)
#$(info MODULE_OBJS:$(MODULE_OBJS))
# rules to build module objects
.PHONY: $(MODULE_OBJS)
$(MODULE_OBJS):		relpath = $(patsubst $(WORK_DIR)%,%,$@)
$(MODULE_OBJS):		mkfile = $(patsubst %module.pre.o,%module.mk,$(relpath))
$(MODULE_OBJS):		workdir = $(@D)
$(MODULE_OBJS):		$(GLOBAL_DEPS) $(NUTTX_CONFIG_HEADER)
	@echo "%%%% Building MODULE_OBJS"
	$(Q) $(MKDIR) -p $(workdir)
	$(Q) $(MAKE) $(MQUIET) -r -f $(DP_MK_DIR)module.mk \
		-C $(workdir) \
		MODULE_WORK_DIR=$(workdir) \
		MODULE_OBJ=$@ \
		MODULE_MK=$(mkfile) \
		MODULE_NAME=$(lastword $(subst /, ,$(workdir))) \
		module

# make a list of phony clean targets for modules
MODULE_CLEANS		:= $(foreach path,$(dir $(MODULE_MKFILES)),$(WORK_DIR)$(path)/clean)

# rules to clean modules
.PHONY: $(MODULE_CLEANS)
$(MODULE_CLEANS):	relpath = $(patsubst $(WORK_DIR)%,%,$@)
$(MODULE_CLEANS):	mkfile = $(patsubst %clean,%module.mk,$(relpath))
$(MODULE_CLEANS):
	@$(ECHO) %% cleaning using $(mkfile)
	$(Q)+ $(MAKE) -r -f $(DP_MK_DIR)module.mk \
	MODULE_WORK_DIR=$(dir $@) \
	MODULE_MK=$(mkfile) \
	clean

#####################
# 编译ROMFS
# ROMFS_ROOT : 【~/DeftWind/mk/ROMFS】
#####################
ROMFS_ROOT	 = $(DP_BASE)/ROMFS
ifneq ($(ROMFS_ROOT),)
ifeq ($(wildcard $(ROMFS_ROOT)),)
$(error ROMFS_ROOT specifies a directory that does not exist)
endif

# Add dependencies on anything in the ROMFS root directory
ROMFS_FILES		+= $(wildcard \
			     $(ROMFS_ROOT)/* \
			     $(ROMFS_ROOT)/*/* \
			     $(ROMFS_ROOT)/*/*/* \
			     $(ROMFS_ROOT)/*/*/*/* \
			     $(ROMFS_ROOT)/*/*/*/*/* \
			     $(ROMFS_ROOT)/*/*/*/*/*/*)
ifeq ($(ROMFS_FILES),)
$(error ROMFS_ROOT $(ROMFS_ROOT) specifies a directory containing no files)
endif
ROMFS_DEPS		+= $(ROMFS_FILES)

# Extra files that may be copied into the ROMFS /extras directory
# ROMFS_EXTRA_FILES are required, ROMFS_OPTIONAL_FILES are optional
ROMFS_EXTRA_FILES	+= $(wildcard $(ROMFS_OPTIONAL_FILES))
ROMFS_DEPS		+= $(ROMFS_EXTRA_FILES)

ROMFS_IMG		 = romfs.img
ROMFS_SCRATCH		 = romfs_scratch
ROMFS_CSRC		 = $(ROMFS_IMG:.img=.c)
ROMFS_OBJ		 = $(ROMFS_CSRC:.c=.o)
LIBS			+= $(ROMFS_OBJ)
LINK_DEPS		+= $(ROMFS_OBJ)

# Remove all comments from startup and mixer files
ROMFS_PRUNER	 = $(DP_BASE)/Tools/px_romfs_pruner.py

# Turn the ROMFS image into an object file
$(ROMFS_OBJ): $(ROMFS_IMG) $(GLOBAL_DEPS)
	$(call BIN_TO_OBJ,$<,$@,romfs_img)

# Generate the ROMFS image from the root
$(ROMFS_IMG): $(ROMFS_SCRATCH) $(ROMFS_DEPS) $(GLOBAL_DEPS)
	@$(ECHO) "ROMFS:   $@"
	$(GENROMFS) -f $@ -d $(ROMFS_SCRATCH) -V "NSHInitVol"

# Construct the ROMFS scratch root from the canonical root
$(ROMFS_SCRATCH): $(ROMFS_DEPS) $(GLOBAL_DEPS)
	$(Q) $(MKDIR) -p $(ROMFS_SCRATCH)
	$(Q) $(COPYDIR) $(ROMFS_ROOT)/* $(ROMFS_SCRATCH)
# delete all files in ROMFS_SCRATCH which start with a . or end with a ~
	$(Q) $(RM) $(ROMFS_SCRATCH)/*/.[!.]* $(ROMFS_SCRATCH)/*/*~
ifneq ($(ROMFS_EXTRA_FILES),)
	$(Q) $(MKDIR) -p $(ROMFS_SCRATCH)/extras
	$(Q) $(COPY) $(ROMFS_EXTRA_FILES) $(ROMFS_SCRATCH)/extras
endif
	$(Q) $(PYTHON) -u $(ROMFS_PRUNER) --folder $(ROMFS_SCRATCH)

EXTRA_CLEANS		+= $(ROMGS_OBJ) $(ROMFS_IMG)

endif


################################################################################
# Builtin command list generation
################################################################################

BUILTIN_CSRC = $(WORK_DIR)builtin_commands.c

MODULE_COMMANDS		 = $(subst COMMAND.,,$(notdir $(wildcard $(WORK_DIR)builtin_commands/COMMAND.*)))

#ifneq ($(BUILTIN_COMMANDS),)
#$(info BUILTIN_COMMANDS:$(BUILTIN_COMMANDS))

# (BUILTIN_PROTO,<cmdspec>,<outputfile>)
define BUILTIN_PROTO
	$(ECHO) 'extern int $(word 4,$1)(int argc, char *argv[]);' >> $2;
endef

# (BUILTIN_DEF,<cmdspec>,<outputfile>)
define BUILTIN_DEF
	$(ECHO) '    {"$(word 1,$1)", $(word 2,$1), $(word 3,$1), $(word 4,$1)},' >> $2;
endef

# Don't generate until modules have updated their command files
$(BUILTIN_CSRC):	$(GLOBAL_DEPS) $(MODULE_OBJS) $(MODULE_MKFILES) $(BUILTIN_COMMAND_FILES)
	@$(ECHO) "CMDS:    $@"
	$(Q) $(ECHO) '/* builtin command list - automatically generated, do not edit */' > $@
	$(Q) $(ECHO) '#include <nuttx/config.h>' >> $@
	$(Q) $(ECHO) '#include <nuttx/binfmt/builtin.h>' >> $@
	$(Q) $(foreach spec,$(BUILTIN_COMMANDS),$(call BUILTIN_PROTO,$(subst ., ,$(spec)),$@))
	$(Q) $(foreach spec,$(MODULE_COMMANDS),$(call BUILTIN_PROTO,$(subst ., ,$(spec)),$@))
	$(Q) $(ECHO) 'const struct builtin_s g_builtins[] = {' >> $@
	$(Q) $(foreach spec,$(BUILTIN_COMMANDS),$(call BUILTIN_DEF,$(subst ., ,$(spec)),$@))
	$(Q) $(foreach spec,$(MODULE_COMMANDS),$(call BUILTIN_DEF,$(subst ., ,$(spec)),$@))
	$(Q) $(ECHO) '    {NULL, 0, 0, NULL}' >> $@
	$(Q) $(ECHO) '};' >> $@
	$(Q) $(ECHO) 'const int g_builtin_count = $(words $(BUILTIN_COMMANDS) $(MODULE_COMMANDS));' >> $@
	
SRCS += $(BUILTIN_CSRC)
#$(info ----------------------------------------------SRCS:$(SRCS))
#endif

all: firmware

PRODUCT_BUNDLE = $(WORK_DIR)firmware.dp
PRODUCT_BIN	 = $(WORK_DIR)firmware.bin
PRODUCT_HEX = $(WORK_DIR)firmware.hex
PRODUCT_ELF	= $(WORK_DIR)firmware.elf

.PHONY: firmware
firmware: $(PRODUCT_BUNDLE)

#
# Object files we will generate from sources
#
OBJS	 := $(foreach src,$(SRCS),$(WORK_DIR)$(src).o)
#$(info ----------------------------------------------OBJS:$(OBJS))

$(OBJS): $(GLOBAL_DEPS)

$(filter %.c.o,$(OBJS)): $(WORK_DIR)%.c.o: %.c $(GLOBAL_DEPS)
	$(call COMPILE,$<,$@)

$(filter %.cpp.o,$(OBJS)): $(WORK_DIR)%.cpp.o: %.cpp $(GLOBAL_DEPS)
	$(call COMPILEXX,$<,$@)

$(filter %.S.o,$(OBJS)): $(WORK_DIR)%.S.o: %.S $(GLOBAL_DEPS)
	$(call ASSEMBLE,$<,$@)

#
# Built product rules
#
$(WORK_DIR)firmware.dp: $(PRODUCT_BIN) $(PRODUCT_HEX)
#	@echo "PRODUCT_BUNDLE: $(PRODUCT_BUNDLE)"
	@echo %% Generating $@
	$(Q) $(MKFW) --prototype $(IMAGE_DIR)/$(BOARD).prototype \
	    --git_identity $(SKETCHBOOK) \
		--image $< > $@

$(PRODUCT_HEX): $(PRODUCT_ELF)
#	@echo "PRODUCT_HEX: $(PRODUCT_HEX)"
	$(call SYM_TO_HEX,$<,$@)

$(PRODUCT_BIN): $(PRODUCT_ELF)
#	@echo "PRODUCT_BIN: $(PRODUCT_BIN)"
	$(call SYM_TO_BIN,$<,$@)

$(PRODUCT_ELF): $(OBJS) $(MODULE_OBJS) $(GLOBAL_DEPS) $(LINK_DEPS) $(MODULE_MKFILES)
#	@echo "PRODUCT_ELF: $(PRODUCT_ELF)"
	$(call LINK,$@,$(OBJS) $(MODULE_OBJS))

-include $(DEP_INCLUDES)
