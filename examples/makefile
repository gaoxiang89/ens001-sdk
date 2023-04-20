# SDK 根目录
SDK_ROOT = ../sdk

# Program file
TESTNAME	= ces

# 设置gcc，可以使用绝对路径, 最后需要加 ’/’
CROSS_COMPILE_PATH ?= ~/opt/gcc-arm-none-eabi-10.3-2021.07/bin/
CROSS_COMPILE ?= $(CROSS_COMPILE_PATH)arm-none-eabi-


CC := $(CROSS_COMPILE)gcc
CXX := $(CROSS_COMPILE)g++
AS := $(CROSS_COMPILE)gcc
NM := $(CROSS_COMPILE)nm
OBJCOPY := $(CROSS_COMPILE)objcopy
OBJDUMP := $(CROSS_COMPILE)objdump
SIZE := $(CROSS_COMPILE)size
GDB ?= $(CROSS_COMPILE)gdb

# 编译存放的文件夹
BUILD_DIR	?= ./build

# 启动代码
CMSIS_DIR    = $(SDK_ROOT)/CMSIS
CORE_DIR     = $(CMSIS_DIR)/Include
DEVICE_DIR   = $(CMSIS_DIR)/Device/NanoChip/ENS001
STARTUP_FILE = $(DEVICE_DIR)/Source/GCC/startup_ens001.s
SYSTEM_FILE  = $(DEVICE_DIR)/Source/system_ens001.c

# 链接脚本，Flash分区
LINKER_SCRIPT_PATH = $(SDK_ROOT)/ldscripts
LINKER_SCRIPT = $(LINKER_SCRIPT_PATH)/cmsdk_cm0.ld

# GCC 编译参数
GNU_CC_FLAGS = -g -Os -mthumb -mcpu=cortex-m0
GNU_CC_FLAGS +=	-ffunction-sections
GNU_CC_FLAGS +=	-fdata-sections
# 使用newlib-nano，减少几K空间
GNU_CC_FLAGS +=	--specs=nano.specs

# 
SRC_AMS += $(STARTUP_FILE)

# 添加需要的C文件
SRC_C += $(SDK_ROOT)/retarget/retarget.c
SRC_C += $(SDK_ROOT)/retarget/uart_stdout.c
SRC_C += $(SYSTEM_FILE)
SRC_C += main.c
SRC_C += $(SDK_ROOT)/lib/delay_systick.c
SRC_C += $(SDK_ROOT)/Peripherals/Source/ens001_gpio.c


# 添加需要的头文件目录
INC += -I$(CORE_DIR)
INC += -I$(DEVICE_DIR)/Include
INC += -I$(SDK_ROOT)/retarget
INC += -I$(SDK_ROOT)/Peripherals/Include
INC += -I$(SDK_ROOT)/lib


# 添加全局宏定义
USER_DEFINE += -D__STACK_SIZE=0x200
USER_DEFINE += -D__HEAP_SIZE=0x1000

# ---------------------------------------------------------------------------------------
all: ${BUILD_DIR}
	@$(CC) $(GNU_CC_FLAGS) ${SRC_AMS} ${SRC_C} ${INC} \
		${USER_DEFINE} \
		-L $(LINKER_SCRIPT_PATH) \
		-T $(LINKER_SCRIPT) -o ${BUILD_DIR}/$(TESTNAME).elf
	@# Generate hex file
	@$(OBJCOPY) -S ${BUILD_DIR}/$(TESTNAME).elf -O ihex ${BUILD_DIR}/$(TESTNAME).hex
	@# Generate size of elf
	@${SIZE} ${BUILD_DIR}/$(TESTNAME).elf

$(BUILD_DIR):
	$(CMD_ECHO) mkdir -p $(BUILD_DIR)

flash: all
	JLinkExe ./jlink_flash.conf -nogui 1

# ---------------------------------------------------------------------------------------
.PHONY: clean
clean :
	@rm -rf build
