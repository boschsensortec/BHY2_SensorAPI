# Root of COINES installation folder
# Can be C:/COINES/v2.0 , $(HOME)/COINES/v2.0 ,etc.,
# Don't use '\' 
COINES_INSTALL_PATH ?= ../../..

EXAMPLE_FILE = euler.c

C_SRCS += ../common/common.c
C_SRCS += ../../bhy2.c
C_SRCS += ../../bhy2_hif.c
C_SRCS += ../../bhy2_parse.c

INCLUDEPATHS += ../common
INCLUDEPATHS += ../..
INCLUDEPATHS += ../../firmware

include $(COINES_INSTALL_PATH)/coines.mk