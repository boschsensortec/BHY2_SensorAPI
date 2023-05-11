COINES_INSTALL_PATH ?= ../../../..

EXAMPLE_FILE ?= bhy2cli.c

BHY2_INTF ?= 

COINES_BACKEND ?= COINES_BRIDGE

API_LOCATION ?= ../..

FW_LOCATION ?= ../..

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
$(API_LOCATION)/bhy2.c \
$(API_LOCATION)/bhy2_hif.c \
$(API_LOCATION)/bhy2_parse.c \
$(API_LOCATION)/bhy2_klio.c \
$(API_LOCATION)/bhy2_swim.c \
$(API_LOCATION)/bhy2_bsec.c \
$(API_LOCATION)/bhi3.c \
$(API_LOCATION)/bhi3_multi_tap.c \
$(API_LOCATION)/bhy2_head_tracker.c

INCLUDEPATHS += . \
$(COMMON_LOCATION)/common \
$(API_LOCATION) \
$(FW_LOCATION)/firmware

# By default, the sensor is connected over SPI. Define this to change to I2C
ifeq ($(BHY2_INTF), I2C)
CFLAGS += -DBHY2_USE_I2C
endif

# Override other target definitions
#TARGET = MCU_APP30
#LOCATION = FLASH

include $(COINES_INSTALL_PATH)/coines.mk