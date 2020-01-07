#DeftWind Makefile

#####################
# make configure时判断TARGET是否定义
# 导入TARGET变量,在正式编译固件时
#####################
ifneq ($(MAKECMDGOALS),configure)
ifneq (mk/configure.mk, $(wildcard mk/configure.mk))
$(error "Please make configure board=xxx")
endif
include  mk/configure.mk
else
ifeq ($(board),)
$(error "Configure board cannot be null")
endif
endif

ifneq ($(upload), )
ifeq ($(upload), y)
UPLOAD = -upload
else
$(error ""Please make upload=y)
endif
endif

#####################
# 编译固件
# 进入DeftCopter目录下进行make
#####################
.PHONY: copter
copter:
	@echo "%%%% Building TARGET:$(TARGET)$(UPLOAD)"
	@ $(MAKE) -C DeftCopter $(TARGET)$(UPLOAD)
	
.PHONY: copter-test
copter-test:
	@echo "%%%% Building TARGET:$(TARGET)$(UPLOAD)"
	@ $(MAKE) -C DeftCopter $(TARGET)$(UPLOAD) TEST=DRIVER_TEST

#####################
# 配置硬件平台,每次切换硬件平台时必须重新配置,再编译固件
# 根据board值重新输出配置信息到configure.mk文件
#####################
.PHONY: configure
configure:
	@echo "Building mk/configure.mk"
	@echo "# Auto-generated file - do not edit" > mk/configure.mk.new
	@echo "TARGET = $(board)" >> mk/configure.mk.new
	@echo "export TARGET" >> mk/configure.mk.new
	cmp mk/configure.mk mk/configure.mk.new 2>/dev/null || mv mk/configure.mk.new mk/configure.mk
	rm -f mk/configure.mk.new

#####################
# 清除目标文件,但不会重置board
# clean依赖于copter,相关的清除操作会在deftwind.mk下执行
#####################
ifneq ($(TARGET),)
$(info TARGET=$(TARGET))
board:=$(TARGET)
.PHONY: clean
clean: TARGET=$(board)-clean
clean: copter
endif
