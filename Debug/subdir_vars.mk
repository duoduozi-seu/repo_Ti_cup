################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../mspm0-modules.syscfg 

C_SRCS += \
../PTZ_controller.c \
../main.c \
../motor.c \
./ti_msp_dl_config.c \
/Applications/ti/mspm0_sdk_2_05_00_05/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../uart_handler.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./PTZ_controller.d \
./main.d \
./motor.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./uart_handler.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./PTZ_controller.o \
./main.o \
./motor.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./uart_handler.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"PTZ_controller.o" \
"main.o" \
"motor.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"uart_handler.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"PTZ_controller.d" \
"main.d" \
"motor.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"uart_handler.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../PTZ_controller.c" \
"../main.c" \
"../motor.c" \
"./ti_msp_dl_config.c" \
"/Applications/ti/mspm0_sdk_2_05_00_05/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../uart_handler.c" 

SYSCFG_SRCS__QUOTED += \
"../mspm0-modules.syscfg" 


