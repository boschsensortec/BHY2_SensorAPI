ifndef COINES_INSTALL_PATH
$(error COINES_INSTALL_PATH is not defined)
endif

EXAMPLE_FILE ?= bhy2cli.c

C_SRCS += parse.c
C_SRCS += ../common/common.c
C_SRCS += ../../../bhy2.c
C_SRCS += ../../../bhy2_hif.c
C_SRCS += ../../../bhy2_parse.c
INCLUDEPATHS += ../common
INCLUDEPATHS += ../../..
INCLUDEPATHS += ../../../firmware

# Override other target definitions
TARGET = PC

OPT = -Ofast

include $(COINES_INSTALL_PATH)/coines.mk
