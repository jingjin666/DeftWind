#DeftWind Makefile

#####################
# make configureʱ�ж�TARGET�Ƿ���
# ����TARGET����,����ʽ����̼�ʱ
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

#####################
# ����̼�
# ����DeftPlaneĿ¼�½���make
#####################
.PHONY: plane
plane:
	@echo "%%%% Building TARGET:$(TARGET)"
	@ $(MAKE) -C DeftPlane $(TARGET)

#####################
# ����Ӳ��ƽ̨,ÿ���л�Ӳ��ƽ̨ʱ������������,�ٱ���̼�
# ����boardֵ�������������Ϣ��configure.mk�ļ�
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
# ���Ŀ���ļ�,����������board
# clean������plane,��ص������������deftwind.mk��ִ��
#####################
ifneq ($(TARGET),)
$(info TARGET=$(TARGET))
board:=$(TARGET)
.PHONY: clean
clean: TARGET=$(board)-clean
clean: plane
endif
