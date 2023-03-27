COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bhy2cli.c

BHY2_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

API_LOCATION ?= ../..

COMMON_LOCATION ?= ..

C_SRCS += \
post_mortem.c \
dinject.c \
parse.c \
logbin.c \
cli.c \
bhy2cli_callbacks.c \
common_callbacks.c \
verbose.c \
$(COMMON_LOCATION)/common/common.c \
$(API_LOCATION)/source/bhy2.c \
$(API_LOCATION)/source/bhy2_hif.c \
$(API_LOCATION)/source/bhy2_parse.c \
$(API_LOCATION)/source/bhy2_klio.c \
$(API_LOCATION)/source/bhy2_pdr.c \
$(API_LOCATION)/source/bhy2_swim.c \
$(API_LOCATION)/source/bhy2_bsec.c \
$(API_LOCATION)/source/bhi3.c \
$(API_LOCATION)/source/bhi3_multi_tap.c

INCLUDEPATHS += . \
$(COMMON_LOCATION)/common \
$(API_LOCATION)/source \
$(API_LOCATION)/firmware

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHY2_INTF), I2C)
CFLAGS += -DBHY2_USE_I2C
endif

# Override other target definitions
#TARGET = MCU_APP30
#LOCATION = FLASH

include $(COINES_INSTALL_PATH)/coines.mk