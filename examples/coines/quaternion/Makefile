ifeq ($(OS),Windows_NT)
HOME=C:
endif

COINES_INSTALL_PATH ?= $(HOME)/coines

EXAMPLE_FILE ?= quaternion.c

C_SRCS += ../common/common.c
C_SRCS += ../../../bhy2.c
C_SRCS += ../../../bhy2_hif.c
C_SRCS += ../../../bhy2_parse.c
INCLUDEPATHS += ../common
INCLUDEPATHS += ../../..
INCLUDEPATHS += ../../../firmware

include $(COINES_INSTALL_PATH)/coines.mk