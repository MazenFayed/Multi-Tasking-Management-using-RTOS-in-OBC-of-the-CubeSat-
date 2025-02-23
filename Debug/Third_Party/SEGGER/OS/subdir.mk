################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.c 

OBJS += \
./Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.o 

C_DEPS += \
./Third_Party/SEGGER/OS/SEGGER_SYSVIEW_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
Third_Party/SEGGER/OS/%.o: ../Third_Party/SEGGER/OS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/drivers/inc" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/Config" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/OS" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/SEGGER" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Config" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/FreeRTOS/org/Source/include" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


