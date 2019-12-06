#
# IMXRT USB CDC VCOM driver
#

MODULE_COMMAND	= usb_cdcacm

SRCS		= circular_buf.c usb_cdcacm.c usb_device_cdc_acm.c usb_device_ch9.c usb_device_class.c usb_device_descriptor.c usb_device_dci.c usb_device_ehci.c usb_osa_bm.c usb_phy.c

INCLUDE_DIRS	+= $(NUTTX_SRC)/arch/arm/src/imxrt $(NUTTX_SRC)/arch/arm/src/common

MAXOPTIMIZATION	 = -Os
