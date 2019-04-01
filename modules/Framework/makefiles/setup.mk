#
# Some useful paths.
#
export DP_INCLUDE_DIR	 = $(abspath $(DP_BASE)/src/include)/
export DP_MODULE_SRC	 = $(abspath $(DP_BASE)/src)/
export DP_LIB_DIR	 = $(abspath $(DP_BASE)/src/lib)/
export DP_PLATFORMS_DIR = $(abspath $(DP_BASE)/src/platforms)/
export DP_MK_DIR	 = $(abspath $(DP_BASE)/makefiles)/
export NUTTX_SRC	 = $(abspath $(DP_BASE)/NuttX/nuttx)/
export NUTTX_APP_SRC	 = $(abspath $(DP_BASE)/NuttX/apps)/
export MAVLINK_SRC	 = $(abspath $(DP_BASE)/mavlink)/
export UAVCAN_DIR	 = $(abspath $(DP_BASE)/uavcan)/
export ROMFS_SRC	 = $(abspath $(DP_BASE)/ROMFS)/
export IMAGE_DIR	 = $(abspath $(DP_BASE)/Images)/
export BUILD_DIR	 = $(abspath $(DP_BASE)/Build)/
export ARCHIVE_DIR	 = $(abspath $(DP_BASE)/Archives)/

#
# Default include paths
#
export INCLUDE_DIRS	:= $(DP_MODULE_SRC) \
			   $(DP_MODULE_SRC)modules/ \
			   $(DP_INCLUDE_DIR) \
			   $(DP_LIB_DIR) \
			   $(DP_PLATFORMS_DIR)

#
# Tools
#
export MKFW		 = $(DP_BASE)/Tools/px_mkfw.py
export UPLOADER		 = $(DP_BASE)/Tools/px_uploader.py
export COPY		 = cp
export COPYDIR		 = cp -Rf
export REMOVE		 = rm -f
export RMDIR		 = rm -rf
export GENROMFS		 = genromfs
export TOUCH		 = touch
export MKDIR		 = mkdir
export FIND		 = find
export ECHO		 = echo
export UNZIP_CMD	 = unzip
export PYTHON		 = python
export OPENOCD		 = openocd
export GREP		 = grep

#
# Debugging
#
export MQUIET			 = --no-print-directory

#
# Host-specific paths, hacks and fixups
#
export SYSTYPE		:= $(shell uname -s)

ifeq ($(SYSTYPE),Darwin)
# Eclipse may not have the toolchain on its path.
export PATH		:= $(PATH):/usr/local/bin
endif

#
# Top-level Makefile for building DP firmware images.
#
export DP_TARGET_OS =  nuttx

#
# Makefile debugging.
#
export Q		:= $(if $(V),,@)