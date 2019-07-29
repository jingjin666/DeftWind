####
链接文件应该遵循如下规则
ifeq ($(CONFIG_ARMV7M_ITCM),y)
  ifeq ($(CONFIG_ARCH_RAMVECTORS),y)
	ifeq ($(CONFIG_ARMV7M_DTCM),y)
	  LDSCRIPT = ld_nor_itcm_dtcm.script
	else
	  LDSCRIPT = ld_nor_itcm_ocram.script
	endif
  else
    $(error ITCM && RAMVECTORS should be)
  endif
else
  ifeq ($(CONFIG_ARMV7M_DTCM),y)
    LDSCRIPT = ld_nor_dtcm.script
  else
    LDSCRIPT = ld_nor_ocram.script
  endif
endif