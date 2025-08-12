################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Drivers/LSM6DSV16X/%.o: ../Drivers/LSM6DSV16X/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"/Applications/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/IMU660RB/Fusion" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/IMU660RB" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/LSM6DSV16X" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/VL53L0X" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/WIT" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/BNO08X_UART_RVC" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/Ultrasonic_GPIO" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/Ultrasonic_Capture" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/OLED_Hardware_I2C" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/OLED_Hardware_SPI" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/OLED_Software_I2C" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/OLED_Software_SPI" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/MPU6050" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Debug" -I"/Applications/ti/mspm0_sdk_2_05_00_05/source/third_party/CMSIS/Core/Include" -I"/Applications/ti/mspm0_sdk_2_05_00_05/source" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/Drivers/MSPM0" -I"/Users/zhouyiduo/workspace_ccstheia/mspm0-modules/4_4_KEY" -DMOTION_DRIVER_TARGET_MSPM0 -DMPU6050 -D__MSPM0G3507__ -gdwarf-3 -MMD -MP -MF"Drivers/LSM6DSV16X/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


