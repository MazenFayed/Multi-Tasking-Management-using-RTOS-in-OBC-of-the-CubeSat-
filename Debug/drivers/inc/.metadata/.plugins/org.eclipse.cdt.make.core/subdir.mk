################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/inc/.metadata/.plugins/org.eclipse.cdt.make.core/specs.c 

OBJS += \
./drivers/inc/.metadata/.plugins/org.eclipse.cdt.make.core/specs.o 

C_DEPS += \
./drivers/inc/.metadata/.plugins/org.eclipse.cdt.make.core/specs.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/inc/.metadata/.plugins/org.eclipse.cdt.make.core/%.o: ../drivers/inc/.metadata/.plugins/org.eclipse.cdt.make.core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/drivers/inc" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/Config" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/OS" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/SEGGER" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Config" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/FreeRTOS/org/Source/include" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


