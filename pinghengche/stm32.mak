#This file is generated by VisualGDB.
#It contains GCC settings automatically derived from the board support package (BSP).
#DO NOT EDIT MANUALLY. THE FILE WILL BE OVERWRITTEN. 
#Use VisualGDB Project Properties dialog or modify Makefile or per-configuration .mak files instead.

#VisualGDB provides BSP_ROOT and TOOLCHAIN_ROOT via environment when running Make. The line below will only be active if GNU Make is started manually.
BSP_ROOT ?= $(LOCALAPPDATA)/VisualGDB/EmbeddedBSPs/arm-eabi/com.sysprogs.arm.stm32
EFP_BASE ?= $(LOCALAPPDATA)/VisualGDB/EmbeddedEFPs
TESTFW_BASE ?= $(LOCALAPPDATA)/VisualGDB/TestFrameworks
TOOLCHAIN_ROOT ?= C:/SysGCC/arm-eabi
#Embedded toolchain
CC := $(TOOLCHAIN_ROOT)/bin/arm-eabi-gcc.exe
CXX := $(TOOLCHAIN_ROOT)/bin/arm-eabi-g++.exe
LD := $(CXX)
AR := $(TOOLCHAIN_ROOT)/bin/arm-eabi-ar.exe
OBJCOPY := $(TOOLCHAIN_ROOT)/bin/arm-eabi-objcopy.exe

#Additional flags
PREPROCESSOR_MACROS += ARM_MATH_CM3 STM32F103C8 flash_layout STM32F10X_MD
INCLUDE_DIRS += . $(BSP_ROOT)/STM32F1xxxx/STM32F10x_StdPeriph_Driver/inc $(BSP_ROOT)/STM32F1xxxx/CMSIS_StdPeriph/CM3/CoreSupport $(BSP_ROOT)/STM32F1xxxx/CMSIS_StdPeriph/CM3/DeviceSupport/ST/STM32F10x
LIBRARY_DIRS += 
LIBRARY_NAMES += compactcpp
ADDITIONAL_LINKER_INPUTS += 
MACOS_FRAMEWORKS += 
LINUX_PACKAGES += 

CFLAGS += 
CXXFLAGS += 
ASFLAGS += 
LDFLAGS += --specs=nano.specs --specs=nosys.specs
COMMONFLAGS += -mcpu=cortex-m3 -mthumb
LINKER_SCRIPT := $(BSP_ROOT)/STM32F1xxxx/LinkerScripts/STM32F103C8_flash.lds

