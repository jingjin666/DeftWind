#####################
# ��ȡhost����,Ŀǰֻ֧��Linux
#####################
SYSTYPE	:=	$(shell uname)
#$(info SYSTYPE::$(SYSTYPE))

ifneq ($(SYSTYPE), Linux)
$(error ERROR: $(SYSTYPE) cannot support)
endif

#####################
# ��ȡMK_DIR = ��~/DeftWind/mk��
#####################
MK_DIR := $(realpath $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST)))))
#$(info MK_DIR::$(MK_DIR))

#####################
# ��ȡSRCROOT = ��~/DeftWind/DeftPlane��
#####################
#$(info 1::$(MAKEFILE_LIST))
#$(info 2::$(firstword $(MAKEFILE_LIST)))
#$(info 3::$(dir $(firstword $(MAKEFILE_LIST))))
#$(info 4::$(realpath $(dir $(firstword $(MAKEFILE_LIST)))))
SRCROOT = $(realpath $(dir $(firstword $(MAKEFILE_LIST))))
#$(info SRCROOT::$(SRCROOT))

#####################
# ��ȡSKETCH = ��DeftPlane��
#####################
#$(info 5::$(subst /, ,$(SRCROOT)))
SKETCH =	$(lastword $(subst /, ,$(SRCROOT)))
#$(info SKETCH::$(SKETCH))

#####################
# ��ȡSKETCHBOOK = ��~/DeftWind��
#####################
SKETCHBOOK = $(shell cd $(SRCROOT)/.. && pwd)
#$(info SKETCHBOOK::$(SKETCHBOOK))

#####################
# ��ȡBUILDROOT = ��~/DeftWind/Build.DeftPlane��
#####################
ifneq ($(findstring uavrs, $(MAKECMDGOALS)),)
BUILDROOT = $(SKETCHBOOK)/Build.$(SKETCH)
endif
#$(info BUILDROOT::$(BUILDROOT))

#####################
# ����mavlink python������ = ��~/DeftWind/mk/mavgen.mk��
#####################
ifneq ($(findstring mavlink1, $(MAKECMDGOALS)),)
EXTRAFLAGS += -DMAVLINK_PROTOCOL_VERSION=1
MAVLINK_SUBDIR=v1.0
MAVLINK_WIRE_PROTOCOL=1.0
else
EXTRAFLAGS += -DMAVLINK_PROTOCOL_VERSION=2
MAVLINK_SUBDIR=v2.0
MAVLINK_WIRE_PROTOCOL=2.0
endif
include $(MK_DIR)/mavgen.mk

#####################
# ����uavcan python������ = ��~/DeftWind/mk/uavcangen.mk��
#####################
include $(MK_DIR)/uavcangen.mk

#####################
# ����ros,msg����������
#####################
PYTHONPATH=$(SKETCHBOOK)/mk/Tools/genmsg/src:$(SKETCHBOOK)/mk/Tools/gencpp/src
export PYTHONPATH

#####################
# ��ȡ��������
#####################
# handle target based overrides for board type
ifneq ($(findstring uavrs, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_UAVRS
endif

ifneq ($(findstring sitl, $(MAKECMDGOALS)),)
HAL_BOARD = HAL_BOARD_SITL
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE
endif

# default to SITL
ifeq ($(HAL_BOARD),)
HAL_BOARD = HAL_BOARD_SITL
HAL_BOARD_SUBTYPE = HAL_BOARD_SUBTYPE_NONE
endif

#####################
# ��ȡLibraries
#####################
#
#
# Find sketchbook libraries referenced by the sketch.
#
# Include paths for sketch libraries 
#
SRCSUFFIXES = *.cpp

ifneq ($(TEST), DRIVER_TEST)
MAKE_INC=$(wildcard $(SRCROOT)/make.inc)
ifeq (,$(MAKE_INC))
$(error You must have a make.inc file to list library dependencies)
else
include $(MAKE_INC)
endif

GLOBAL_MAKE_INC=$(wildcard $(SKETCHBOOK)/mk/make.inc)
ifeq (,$(GLOBAL_MAKE_INC))
$(error You must have a make.inc in mk/ directory)
else
include $(GLOBAL_MAKE_INC)
endif
else
GLOBAL_TESTS_MAKE_INC=$(wildcard $(SKETCHBOOK)/test/make.inc)
include $(GLOBAL_TESTS_MAKE_INC)
endif
LIBTOKENS := $(LIBRARIES)
# HAL and board specific libraries are included here.
LIBTOKENS += \
	AP_HAL \
	AP_HAL_Empty

ifeq ($(HAL_BOARD),HAL_BOARD_UAVRS)
LIBTOKENS += \
	AP_HAL_UAVRS
endif

ifeq ($(HAL_BOARD),HAL_BOARD_SITL)
LIBTOKENS += \
	AP_HAL_SITL \
	SITL
endif

SKETCHLIBS		:=	$(wildcard $(addprefix $(SKETCHBOOK)/libraries/,$(LIBTOKENS)))
SKETCHLIBNAMES		:=	$(notdir $(SKETCHLIBS))
SKETCHLIBSRCDIRS	:=	$(SKETCHLIBS) $(addsuffix /utility,$(SKETCHLIBS))
SKETCHLIBSRCS		:=	$(wildcard $(foreach suffix,$(SRCSUFFIXES),$(addsuffix /$(suffix),$(SKETCHLIBSRCDIRS))))
SKETCHLIBOBJS		:=	$(addsuffix .o,$(basename $(subst $(SKETCHBOOK),$(BUILDROOT),$(SKETCHLIBSRCS))))
SKETCHLIBINCLUDES	:=	-I$(SKETCHBOOK)/libraries/ -I$(BUILDROOT)/libraries/ -I$(BUILDROOT)/libraries/GCS_MAVLink/
SKETCHLIBSRCSRELATIVE	:=	$(subst $(SKETCHBOOK)/,,$(SKETCHLIBSRCS))
#$(info SKETCHLIBSRCSRELATIVE:$(SKETCHLIBSRCSRELATIVE))

# uavcan includes
include $(SKETCHBOOK)/modules/uavcan/libuavcan/include.mk

# board specific includes
ifeq ($(HAL_BOARD),HAL_BOARD_SITL)
include $(MK_DIR)/board_sitl.mk
endif

ifeq ($(HAL_BOARD),HAL_BOARD_UAVRS)
include $(MK_DIR)/board_uavrs.mk
endif