################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/ddsm115/DDSMLib.c 

OBJS += \
./bsp/ddsm115/DDSMLib.o 

C_DEPS += \
./bsp/ddsm115/DDSMLib.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/ddsm115/%.o bsp/ddsm115/%.su bsp/ddsm115/%.cyclo: ../bsp/ddsm115/%.c bsp/ddsm115/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../bsp/ddsm115 -I../bsp/mpu6050 -I../bsp/sr04 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-bsp-2f-ddsm115

clean-bsp-2f-ddsm115:
	-$(RM) ./bsp/ddsm115/DDSMLib.cyclo ./bsp/ddsm115/DDSMLib.d ./bsp/ddsm115/DDSMLib.o ./bsp/ddsm115/DDSMLib.su

.PHONY: clean-bsp-2f-ddsm115

