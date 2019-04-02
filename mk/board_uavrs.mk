#####################
# 获取DP_ROOT = 【~/DeftWind/modules/Framework】
# 获取NUTTX_ROOT = 【~/DeftWind/modules/NuttX】
# 获取NUTTX_SRC = 【~/DeftWind/modules/NuttX/nuttx】
#####################
DP_DIRECTORY = $(SKETCHBOOK)/modules/Framework
DP_ROOT = $(shell cd $(DP_DIRECTORY) && pwd)
NUTTX_DIRECTORY = $(SKETCHBOOK)/modules/NuttX
NUTTX_ROOT = $(shell cd $(NUTTX_DIRECTORY) && pwd)
NUTTX_SRC = $(NUTTX_ROOT)/nuttx
#$(info DP_DIRECTORY::$(DP_DIRECTORY))
#$(info NUTTX_DIRECTORY::$(NUTTX_DIRECTORY))
#$(info DP_ROOT::$(DP_ROOT))
#$(info NUTTX_ROOT::$(NUTTX_ROOT))
#$(info NUTTX_SRC::$(NUTTX_SRC))

#####################
# 获取板级编译文件xxx_CONFIG_FILE = 【~/DeftWind/mk/uavrs/config_xxx_DeftWind.mk】
#####################
UAVRS_V1_CONFIG_FILE = $(MK_DIR)/uavrs/config_uavrs-v1_DeftWind.mk
#$(info UAVRS_V1_CONFIG_FILE::$(UAVRS_V1_CONFIG_FILE))

UAVRS_V2_CONFIG_FILE = $(MK_DIR)/uavrs/config_uavrs-v2_DeftWind.mk
#$(info UAVRS_V2_CONFIG_FILE::$(UAVRS_V2_CONFIG_FILE))

#####################
# C/C++编译选项
#####################
EXTRAFLAGS += -DUAVCAN=1
EXTRAFLAGS += -D__STDC_FORMAT_MACROS

# Add missing parts from libc and libstdc++
EXTRAFLAGS += -DHAVE_STD_NULLPTR_T=0
EXTRAFLAGS += -DHAVE_ENDIAN_H=0
EXTRAFLAGS += -DHAVE_BYTESWAP_H=0
EXTRAFLAGS += -DHAVE_OCLOEXEC=0

EXTRAFLAGS += -I$(BUILDROOT)/libraries/GCS_MAVLink/include/mavlink
# Add missing parts from libc and libstdc++ for all boards
EXTRAFLAGS += -I$(SKETCHBOOK)/libraries/AP_Common/missing


DEFINES        +=   -DCONFIG_HAL_BOARD=$(HAL_BOARD) -DCONFIG_HAL_BOARD_SUBTYPE=$(HAL_BOARD_SUBTYPE)

CCACHE = /usr/bin/ccache
SKETCHFLAGS = $(SKETCHLIBINCLUDES) -DCONFIG_HAL_BOARD=HAL_BOARD_UAVRS -DSKETCHNAME="\\\"$(SKETCH)\\\"" -DSKETCH_MAIN=DeftWind_main
WARNFLAGS = -Wall -Wextra -Wlogical-op -Werror -Wno-unknown-pragmas -Wno-redundant-decls -Wno-psabi -Wno-packed -Wno-error=double-promotion -Wno-error=unused-variable -Wno-error=reorder -Wno-error=float-equal -Wno-error=pmf-conversions -Wno-error=missing-declarations -Wno-error=unused-function -Wno-sign-compare -Wreorder -Wno-shadow
OPTFLAGS = -fsingle-precision-constant

#####################
# 编译xxx_DeftWind, 指定【~/DeftWind/modules/Framework/Makefile.make】进行make
#####################
DP_MAKE = $(MAKE) --no-print-directory -C $(SKETCHBOOK) -f $(DP_ROOT)/Makefile.make EXTRADEFINES="$(SKETCHFLAGS) $(WARNFLAGS) $(OPTFLAGS) "'$(EXTRAFLAGS)' DEFTWIND_MODULE_DIR=$(SKETCHBOOK) SKETCHBOOK=$(SKETCHBOOK) CCACHE=$(CCACHE) DP_ROOT=$(DP_ROOT) NUTTX_SRC=$(NUTTX_SRC) MAXOPTIMIZATION="-Os"

#####################
# 编译archives, 指定【~/DeftWind/modules/Framework/Makefile.make】进行make
#####################
DP_MAKE_ARCHIVES = $(MAKE) -C $(DP_ROOT) -f $(DP_ROOT)/Makefile.make NUTTX_SRC=$(NUTTX_SRC) CCACHE=$(CCACHE) archives MAXOPTIMIZATION="-Os"
#$(info DP_MAKE_ARCHIVES::$(DP_MAKE_ARCHIVES))

#####################
# 编译固件
# xxx依赖于【~/DeftWind/modules/Framework/Archives/xxx.export 和 ~/DeftWind/module_mk】
# 拷贝板级编译文件到【~/DeftWind/modules/Framework/makefiles/nuttx】,firmware.mk编译时需要
# 开始make xxx_DeftWind
#####################
uavrs-v1: $(MAVLINK_HEADERS) $(DP_ROOT)/Archives/uavrs-v1.export module_mk
	@echo "%%%% Building uavrs-v1"
	@ cp $(UAVRS_V1_CONFIG_FILE) $(DP_ROOT)/makefiles/nuttx/
	@ $(DP_MAKE) uavrs-v1_DeftWind
	@ arm-none-eabi-size $(DP_ROOT)/Build/uavrs-v1_DeftWind.build/firmware.elf
	@ cp $(DP_ROOT)/Images/uavrs-v1_DeftWind.dp $(BUILDROOT)/$(SKETCH)-v1.dp

uavrs-v2: $(MAVLINK_HEADERS) $(DP_ROOT)/Archives/uavrs-v2.export module_mk
	@echo "%%%% Building uavrs-v2"
	@ cp $(UAVRS_V2_CONFIG_FILE) $(DP_ROOT)/makefiles/nuttx/
	@ $(DP_MAKE) uavrs-v2_DeftWind
	@ arm-none-eabi-size $(DP_ROOT)/Build/uavrs-v2_DeftWind.build/firmware.elf
	@ cp $(DP_ROOT)/Images/uavrs-v2_DeftWind.dp $(BUILDROOT)/$(SKETCH)-v2.dp
	
#####################
# xxx.export 为编译出的NuttX库
# 开始编译archives,配置BOARDS=xxx
#####################
$(DP_ROOT)/Archives/uavrs-v1.export: 
	$(DP_MAKE_ARCHIVES) BOARDS=uavrs-v1
	
$(DP_ROOT)/Archives/uavrs-v2.export: 
	$(DP_MAKE_ARCHIVES) BOARDS=uavrs-v2

#####################
# 伪目标uavrs-v1-clean
# 删除xxx.export和相关Builds
#####################
.PHONY: uavrs-v1-clean
uavrs-v1-clean: 
	@echo "%%%% Cleaning target"
	rm -rf $(DP_ROOT)/Archives
	rm -rf $(DP_ROOT)/Build $(DP_ROOT)/Images/*.dp $(DP_ROOT)/Images/*.bin $(DP_ROOT)/Images/*.hex
	rm -f $(SRCROOT)/*.o
		
#####################
# 伪目标uavrs-v2-clean
# 删除xxx.export和相关Builds
#####################
.PHONY: uavrs-v2-clean
uavrs-v2-clean: 
	@echo "%%%% Cleaning target"
	rm -rf $(DP_ROOT)/Archives
	rm -rf $(DP_ROOT)/Build $(DP_ROOT)/Images/*.dp $(DP_ROOT)/Images/*.bin $(DP_ROOT)/Images/*.hex
	rm -f $(SRCROOT)/*.o

#####################
# 生成【~/DeftWind/module.mk】
# module.mk自动生成App层下的所有源文件SRCS,包括SRCROOT和LIBRARY
# 导出的MODULE_COMMAND=DeftWind
#####################
.PHONY: module_mk
module_mk: 
	@echo "%%%% Building $(SKETCHBOOK)/module.mk"
	@echo "# Auto-generated file - do not edit" > $(SKETCHBOOK)/module.mk.new
	@echo  "MODULE_COMMAND = DeftWind" >> $(SKETCHBOOK)/module.mk.new
	@echo "SRCS = $(wildcard $(SRCROOT)/*.cpp) $(SKETCHLIBSRCSRELATIVE)" >> $(SKETCHBOOK)/module.mk.new
	@echo "MODULE_STACKSIZE = 4096" >> $(SKETCHBOOK)/module.mk.new
	@echo "EXTRACXXFLAGS = -Wframe-larger-than=1300" >> $(SKETCHBOOK)/module.mk.new
	@ cmp $(SKETCHBOOK)/module.mk $(SKETCHBOOK)/module.mk.new 2>/dev/null || mv $(SKETCHBOOK)/module.mk.new $(SKETCHBOOK)/module.mk
	@ rm -f $(SKETCHBOOK)/module.mk.new
