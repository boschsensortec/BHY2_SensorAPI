COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bhy2cli.c

BHY2_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

C_SRCS += dinject.c
C_SRCS += parse.c
C_SRCS += logbin.c
C_SRCS += cli.c
C_SRCS += bhy2cli_callbacks.c
C_SRCS += common_callbacks.c
C_SRCS += verbose.c
C_SRCS += ../common/common.c
C_SRCS += ../../bhy2.c
C_SRCS += ../../bhy2_hif.c
C_SRCS += ../../bhy2_parse.c
C_SRCS += ../../bhy2_klio.c
C_SRCS += ../../bhy2_pdr.c
C_SRCS += ../../bhy2_swim.c
C_SRCS += ../../bhy2_bsec.c
INCLUDEPATHS += .
INCLUDEPATHS += ../common
INCLUDEPATHS += ../..
INCLUDEPATHS += ../../firmware

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHY2_INTF), I2C)
CFLAGS += -DBHY2_USE_I2C
endif

# Override other target definitions
#TARGET = MCU_APP30
#LOCATION = FLASH

include $(COINES_INSTALL_PATH)/coines.mk