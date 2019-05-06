#####################
# 获取Framework的绝对路径【~/DeftWind/modules/Framework/】并导出
# 提供给setup.mk
#####################
export DP_BASE := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
include $(DP_BASE)/makefiles/setup.mk

#####################
# 通过拷贝过来的板级配置文件,判断有几个可输出的固件
# KNOWN_CONFIGS = 【uavrs-v1_DeftWind uavrs-v2_DeftWind】
#####################
KNOWN_CONFIGS		:= $(subst config_,,$(basename $(notdir $(wildcard $(DP_MK_DIR)$(DP_TARGET_OS)/config_*.mk))))
CONFIGS			?= $(KNOWN_CONFIGS)
#$(info KNOWN_CONFIGS:$(KNOWN_CONFIGS))
#$(info CONFIGS:$(CONFIGS))

#####################
# 编译xxx_DeftWind
# 当MAKECMDGOALS为xxx_DeftWind时过滤出所需要的EXPLICIT_CONFIGS = 【xxx_DeftWind】,并修改CONFIGS 为 EXPLICIT_CONFIGS 
# 当MAKECMDGOALS为archives时,EXPLICIT_CONFIGS为空
# EXPLICIT_CONFIGS 不为空时依赖于all
#####################
#$(info %%  MAKECMDGOALS:$(MAKECMDGOALS))
EXPLICIT_CONFIGS	:= $(filter $(CONFIGS),$(MAKECMDGOALS))
#$(info EXPLICIT_CONFIGS:$(EXPLICIT_CONFIGS))
ifneq ($(EXPLICIT_CONFIGS),)
CONFIGS := $(EXPLICIT_CONFIGS)
.PHONY: $(EXPLICIT_CONFIGS)
$(EXPLICIT_CONFIGS):	all
endif

#####################
# 输出印象文件
# 起初的目标固件DESIRED_FIRMWARES = 【~/DeftWind/modules/Framework/Images/xxx_DeftWind.dp】
# 所有的目标固件STAGED_FIRMWARES,包括起初的目标固件DESIRED_FIRMWARES
# 临时编译生成的目标固件FIRMWARES = 【~/DeftWind/modules/Framework/Build/xxx_DeftWind.build/firmware.dp】
#####################
DESIRED_FIRMWARES = $(foreach config,$(CONFIGS),$(IMAGE_DIR)$(config).dp)
#$(info DESIRED_FIRMWARES:$(DESIRED_FIRMWARES))
STAGED_FIRMWARES = $(foreach config,$(KNOWN_CONFIGS),$(IMAGE_DIR)$(config).dp)
#$(info STAGED_FIRMWARES:$(STAGED_FIRMWARES))
FIRMWARES = $(foreach config,$(KNOWN_CONFIGS),$(BUILD_DIR)$(config).build/firmware.dp)
#$(info FIRMWARES:$(FIRMWARES))

#####################
# STAGED_FIRMWARES依赖【~/DeftWind/modules/Framework/Build/xxx_DeftWind.build/firmware.dp】
# 拷贝至STAGED_FIRMWARES
#####################
$(STAGED_FIRMWARES): $(IMAGE_DIR)%.dp: $(BUILD_DIR)%.build/firmware.dp
	@$(ECHO) %% Copying $@
	$(Q) $(COPY) $< $@
	$(Q) $(COPY) $(patsubst %.dp,%.bin,$<) $(patsubst %.dp,%.bin,$@)
	$(Q) $(COPY) $(patsubst %.dp,%.hex,$<) $(patsubst %.dp,%.hex,$@)

#####################
# FIRMWARES = 【~/DeftWind/modules/Framework/Build/xxx_DeftWind.build/firmware.dp】
# 进入firmware.mk 开始编译FIRMWARE_GOAL = 【firmware】
#####################
FIRMWARE_GOAL = firmware
.PHONY: $(FIRMWARES)
$(BUILD_DIR)%.build/firmware.dp: config   = $(patsubst $(BUILD_DIR)%.build/firmware.dp,%,$@)
$(BUILD_DIR)%.build/firmware.dp: work_dir = $(BUILD_DIR)$(config).build/
$(FIRMWARES): $(BUILD_DIR)%.build/firmware.dp: generateuorbtopicheaders
	@echo %%%% Building $(config) in $(work_dir)
	$(Q) $(MKDIR) -p $(work_dir)
	$(Q) $(MAKE) $(MQUIET) -r -C $(work_dir) -f $(DP_MK_DIR)firmware.mk CONFIG=$(config) WORK_DIR=$(work_dir) $(FIRMWARE_GOAL)

#####################
# all依赖DESIRED_FIRMWARES<=STAGED_FIRMWARES依赖FIRMWARES
#####################
all: $(DESIRED_FIRMWARES)

#####################
# 根据deftwind.mk传入的BOARDS生成NUTTX_ARCHIVES = 【~/DeftWind/modules/Framework/Archives/xxx.export】
# 进入~/DeftWind/modules/NuttX/nuttx/tools
# make distclean
# 拷贝BSP【~/DeftWind/modules/Framework/nuttx-configs/xxx】到【~/DeftWind/modules/NuttX/nuttx/nuttx-configs】
# 配置nsh
# make export生成nuttx-export.zip
# 拷贝nuttx-export.zip至【~/DeftWind/modules/Framework/Archives/xxx.export】
# 删除NuttX中的BSP,保持Nuttx清洁
#####################
NUTTX_ARCHIVES = $(foreach board, $(BOARDS), $(ARCHIVE_DIR)$(board).export)
#$(info NUTTX_ARCHIVES:$(NUTTX_ARCHIVES))
.PHONY: archives
archives: $(NUTTX_ARCHIVES)
$(ARCHIVE_DIR)%.export:	board = $(notdir $(basename $@))
$(ARCHIVE_DIR)%.export:	configuration = nsh
$(NUTTX_ARCHIVES): $(ARCHIVE_DIR)%.export: $(NUTTX_SRC)
	@echo Configuring NuttX for $(board)
	cd $(NUTTX_SRC) && $(RMDIR) nuttx-export
ifneq (, $(wildcard $(NUTTX_SRC)/.config))
	$(MAKE) -C $(NUTTX_SRC) -r $(MQUIET) distclean
endif
	cd $(NUTTX_SRC)/configs && $(COPYDIR) $(DP_BASE)/nuttx-configs/$(board) .
	cd $(NUTTX_SRC)/tools && ./configure.sh $(board)/$(configuration)
	@echo Exporting NuttX for $(board)
	$(MAKE) -C $(NUTTX_SRC) -r $(MQUIET) CONFIG_ARCH_BOARD=$(board) export
	$(MKDIR) -p $(dir $@)
	$(COPY) $(NUTTX_SRC)/nuttx-export.zip $@
	cd $(NUTTX_SRC)/configs && $(RMDIR) $(board)

#####################
# uorb msg 生成器
#####################	
MSG_DIR = $(DP_BASE)/msg
UORB_TEMPLATE_DIR = $(DP_BASE)/msg/templates/uorb
MULTIPLATFORM_TEMPLATE_DIR = $(DP_BASE)/msg/templates/dp/uorb
TOPICS_DIR = $(DP_BASE)/src/modules/uORB/topics
MULTIPLATFORM_HEADER_DIR = $(DP_BASE)/src/platforms/$(DP_TARGET_OS)/dp_messages
MULTIPLATFORM_PREFIX = dp_
TOPICHEADER_TEMP_DIR = $(BUILD_DIR)topics_temporary
GENMSG_PYTHONPATH = $(DP_BASE)/Tools/genmsg/src
GENCPP_PYTHONPATH = $(DP_BASE)/Tools/gencpp/src

#####################
# generateuorbtopicheaders
#####################
.PHONY: generateuorbtopicheaders
generateuorbtopicheaders:
	@echo "Generating uORB topic headers"
	$(Q) (PYTHONPATH=$(GENMSG_PYTHONPATH):$(GENCPP_PYTHONPATH):$(PYTHONPATH) $(PYTHON) \
		$(DP_BASE)/Tools/px_generate_uorb_topic_headers.py \
		-d $(MSG_DIR) -o $(TOPICS_DIR) -e $(UORB_TEMPLATE_DIR) -t $(TOPICHEADER_TEMP_DIR))
	@$(ECHO) "Generating multiplatform uORB topic wrapper headers"
	$(Q) (rm -r $(TOPICHEADER_TEMP_DIR))
	$(Q) (PYTHONPATH=$(GENMSG_PYTHONPATH):$(GENCPP_PYTHONPATH):$(PYTHONPATH) $(PYTHON) \
		$(DP_BASE)/Tools/px_generate_uorb_topic_headers.py \
		-d $(MSG_DIR) -o $(MULTIPLATFORM_HEADER_DIR) -e $(MULTIPLATFORM_TEMPLATE_DIR) -t $(TOPICHEADER_TEMP_DIR) -p $(MULTIPLATFORM_PREFIX))
	$(Q) (rm -r $(TOPICHEADER_TEMP_DIR))