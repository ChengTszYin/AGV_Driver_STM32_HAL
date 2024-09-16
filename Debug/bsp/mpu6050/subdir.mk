################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/mpu6050/mpu6050.c 

OBJS += \
./bsp/mpu6050/mpu6050.o 

C_DEPS += \
./bsp/mpu6050/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/mpu6050/%.o bsp/mpu6050/%.su bsp/mpu6050/%.cyclo: ../bsp/mpu6050/%.c bsp/mpu6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../bsp/ddsm115 -I../bsp/mpu6050 -I../bsp/sr04 -I../app/util -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bsp-2f-mpu6050

clean-bsp-2f-mpu6050:
	-$(RM) ./bsp/mpu6050/mpu6050.cyclo ./bsp/mpu6050/mpu6050.d ./bsp/mpu6050/mpu6050.o ./bsp/mpu6050/mpu6050.su

.PHONY: clean-bsp-2f-mpu6050

