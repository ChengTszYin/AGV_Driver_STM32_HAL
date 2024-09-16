################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/sr04/SR04.c 

OBJS += \
./bsp/sr04/SR04.o 

C_DEPS += \
./bsp/sr04/SR04.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/sr04/%.o bsp/sr04/%.su bsp/sr04/%.cyclo: ../bsp/sr04/%.c bsp/sr04/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../bsp/ddsm115 -I../bsp/mpu6050 -I../bsp/sr04 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bsp-2f-sr04

clean-bsp-2f-sr04:
	-$(RM) ./bsp/sr04/SR04.cyclo ./bsp/sr04/SR04.d ./bsp/sr04/SR04.o ./bsp/sr04/SR04.su

.PHONY: clean-bsp-2f-sr04

