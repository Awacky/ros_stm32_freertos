
TARGET = starsRobot

DEBUG = 1

OPT = -Og

BUILD_DIR = build

C_SOURCES = \
APP/canLink_Task.c \
APP/decManage_Task.c \
APP/ErrorManage_Task.c \
APP/hcBle_Task.c \
APP/LedBeep_Task.c \
APP/log_Task.c \
APP/terminal_Task.c \
APP/timeout.c \
APP/uart1Link_Task.c \
APP/uart3Link_Task.c \
APP/uart4Link_Task.c \
APP/uart5Link_Task.c \
APP/usbLink_Task.c \
APP/userMain.c \
APP/ws2812Run_Task.c \
Middlewares/FreeRTOS/croutine.c \
Middlewares/FreeRTOS/event_groups.c \
Middlewares/FreeRTOS/list.c \
Middlewares/FreeRTOS/queue.c \
Middlewares/FreeRTOS/tasks.c \
Middlewares/FreeRTOS/timers.c \
Middlewares/FreeRTOS/portable/MemMang/heap_4.c \
Middlewares/FreeRTOS/portable/GCC/ARM_CM4F/port.c \
Middlewares/AppSioSendProcessing.c \
Middlewares/canMotor.c \
Middlewares/CustomPlot.c \
Middlewares/delay.c \
Middlewares/iap.c \
Middlewares/motor.c \
Middlewares/syscalls.c \
Middlewares/terminal.c \
Middlewares/w25qxx.c \
Middlewares/CRC_CALC/crc_calc.c \
Middlewares/Modbus_RTU/Modbus.c \
Middlewares/PSTwo/pstwo.c \
Middlewares/SBUS/sbus.c \
Middlewares/SEGGER/RTT/SEGGER_RTT_printf.c \
Middlewares/SEGGER/RTT/SEGGER_RTT.c \
Middlewares/WS2812/WS2812.c \
Public/config_param.c \
Public/led.c \
Public/log_handle.c \
Public/sonar.c \
STM32Bsp/Drive/can.c \
STM32Bsp/Drive/device_storage.c \
STM32Bsp/Drive/eeprom.c \
STM32Bsp/Drive/hw_adc.c \
STM32Bsp/Drive/hw_spi.c \
STM32Bsp/HwConfig/hw_STM32F40x.c \
STM32Bsp/SioInterface/uart4.c \
STM32Bsp/SioInterface/uart5.c \
STM32Bsp/SioInterface/usart1.c \
STM32Bsp/SioInterface/usart2.c \
STM32Bsp/SioInterface/usart3.c \
STM32Bsp/Stm32libs/FWLIB/src/misc.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_adc.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_can.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_crc.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_cryp_aes.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_cryp_des.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_cryp_tdes.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_cryp.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_dac.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_dbgmcu.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_dcmi.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_dma.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_dma2d.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_exti.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_flash_ramfunc.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_flash.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_fsmc.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_gpio.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_hash_md5.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_hash_sha1.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_hash.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_i2c.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_iwdg.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_ltdc.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_pwr.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_rcc.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_rng.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_sai.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_sdio.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_spi.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_syscfg.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_tim.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_usart.c \
STM32Bsp/Stm32libs/FWLIB/src/stm32f4xx_wwdg.c \
STM32Bsp/Stm32libs/ST/stm32f4xx_it.c \
STM32Bsp/Stm32libs/ST/system_stm32f4xx.c \
STM32Bsp/USB/STM32_USB_Device_Library/Core/src/usbd_core.c \
STM32Bsp/USB/STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
STM32Bsp/USB/STM32_USB_Device_Library/Core/src/usbd_req.c \
STM32Bsp/USB/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c \
STM32Bsp/USB/STM32_USB_OTG_Driver/src/usb_core.c \
STM32Bsp/USB/STM32_USB_OTG_Driver/src/usb_dcd.c \
STM32Bsp/USB/STM32_USB_OTG_Driver/src/usb_dcd_int.c \
STM32Bsp/USB/USB_APP/src/usb_bsp.c \
STM32Bsp/USB/USB_APP/src/usbd_cdc_vcp.c \
STM32Bsp/USB/USB_APP/src/usbd_desc.c \
STM32Bsp/USB/USB_APP/src/usbd_usr.c \

CXX_SOURCES	= \
APP/ros_node.cpp \
APP/moveBase_Task.cpp \
APP/OledShow_Task.cpp \
Middlewares/OLED/oled.cpp \
Middlewares/PID/PID.cpp \
Middlewares/SoftwareIIC.cpp \
Middlewares/AT24Cxx.cpp \
Middlewares/WireBase.cpp \
Middlewares/IMU/fake_mag.cpp \
Middlewares/IMU/gy85.cpp \
Middlewares/IMU/mpu6050.cpp \
Middlewares/IMU/MPU6500.cpp \
Middlewares/IMU/MPU9250.cpp \
Middlewares/Servo/servo.cpp \
Middlewares/RosLibs/duration.cpp \
Middlewares/RosLibs/time.cpp

C_INCLUDES = \
-IAPP \
-IConfig \
-IMiddlewares \
-IMiddlewares/CRC_CALC \
-IMiddlewares/IMU \
-IMiddlewares/Modbus_RTU \
-IMiddlewares/OLED \
-IMiddlewares/PID \
-IMiddlewares/PSTwo \
-IMiddlewares/SBUS \
-IMiddlewares/SEGGER/RTT \
-IMiddlewares/Servo \
-IMiddlewares/WS2812 \
-IMiddlewares/FreeRTOS/include \
-IMiddlewares/FreeRTOS/portable/GCC/ARM_CM4F \
-IPrivate \
-IPublic \
-ISTM32Bsp/Drive \
-ISTM32Bsp/HwConfig \
-ISTM32Bsp/SioInterface \
-ISTM32Bsp/Stm32libs/CoreSupport \
-ISTM32Bsp/Stm32libs/CoreSupport/gcc \
-ISTM32Bsp/Stm32libs/FWLIB/inc \
-ISTM32Bsp/Stm32libs/ST \
-IMiddlewares/RosLibs \
-ISTM32Bsp/USB/STM32_USB_Device_Library/Class/cdc/inc \
-ISTM32Bsp/USB/STM32_USB_Device_Library/Core/inc \
-ISTM32Bsp/USB/STM32_USB_OTG_Driver/inc \
-ISTM32Bsp/USB/USB_APP/interface \

AS_DEFS = 

C_DEFS =  \
-DSTM32F40_41xxx \
-DUSE_STDPERIPH_DRIVER \
-DUSE_USB_OTG_FS \
-DMicroros

AS_INCLUDES =  \
-ISTM32Bsp/Stm32libs/CoreSupport/gcc

ASM_SOURCES = \
STM32Bsp/Stm32libs/CoreSupport/gcc/startup_stm32f407xx.s
LDSCRIPT = \
STM32Bsp/Stm32libs/CoreSupport/gcc/STM32F407VETx_FLASH.ld

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)


# compile gcc flags
# ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CXXFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
CXXFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script


# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,-lstdc++  -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# micro-ROS addons
#######################################
LDFLAGS += Middlewares/MicroRos/lib/cortex-m4/libmicroros.a
LDFLAGS += Private/libstarsRobot.a
C_INCLUDES += -IMiddlewares/MicroRos/include
C_INCLUDES += -IMiddlewares/MicroRos
C_INCLUDES += -IMiddlewares/MicroRos/msgs
C_INCLUDES += -IMiddlewares/MicroRos/lib/cortex-m4

# Add micro-ROS utils
C_SOURCES += Middlewares/MicroRos/custom_memory_manager.c
C_SOURCES += Middlewares/MicroRos/microros_allocators.c
C_SOURCES += Middlewares/MicroRos/microros_time.c
C_SOURCES += Middlewares/MicroRos/microros_tasks.c

# Set here the custom transport implementation
C_SOURCES += Middlewares/MicroRos/dma_transport.c

print_cflags:
	@echo $(CFLAGS)

	
#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CXX) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

# $(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
# 	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
# 	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***


###
## CUSTOM COMMANDS
###

# Flash with J-Link
# Configure device name, everything else should remain the same
jflash: $(BUILD_DIR)/jflash
	JLinkExe -commanderscript $<
# Change to yours
device = STM32F407VG
# First create a file with all commands
$(BUILD_DIR)/jflash: $(BUILD_DIR)/$(TARGET).bin
	@touch $@
	@echo device $(device) > $@
	@echo -e si 1'\n'speed 4000 >> $@
	@echo loadbin $< 0x8010000 >> $@
	@echo -e r'\n'g'\n'qc >> $@

# FLash with ST-LINK
stflash: $(BUILD_DIR)/$(TARGET).bin
	st-flash --reset write $< 0x8000000

# Flash with UART module
# If you have problem with flashing but it does connect,
# remove '-e 0' so that it will erase flash contents and
# flash firmware fresh
uflash: $(BUILD_DIR)/$(TARGET).bin
	# This one is used if you have UART module with RTS and DTR pins
	stm32flash -b 115200 -e 0 -R -i rts,dtr,-rts:rts,-dtr,-rts -v -w $< $(PORT)
	# Else use this one and manualy set your MCU to bootloader mode
	#stm32flash -b 115200 -e 0 -v -w $< $(PORT)
