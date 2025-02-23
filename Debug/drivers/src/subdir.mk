################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/MPU6050.c \
../drivers/src/MadgwickAHRS.c \
../drivers/src/stm32f446xx_I2C.c \
../drivers/src/stm32f446xx_adc.c \
../drivers/src/stm32f446xx_dac.c \
../drivers/src/stm32f446xx_gpio_driver.c \
../drivers/src/stm32f446xx_rcc.c \
../drivers/src/stm32f446xx_spi_driver.c \
../drivers/src/stm32f446xx_timer.c \
../drivers/src/stm32f446xx_usart.c 

OBJS += \
./drivers/src/MPU6050.o \
./drivers/src/MadgwickAHRS.o \
./drivers/src/stm32f446xx_I2C.o \
./drivers/src/stm32f446xx_adc.o \
./drivers/src/stm32f446xx_dac.o \
./drivers/src/stm32f446xx_gpio_driver.o \
./drivers/src/stm32f446xx_rcc.o \
./drivers/src/stm32f446xx_spi_driver.o \
./drivers/src/stm32f446xx_timer.o \
./drivers/src/stm32f446xx_usart.o 

C_DEPS += \
./drivers/src/MPU6050.d \
./drivers/src/MadgwickAHRS.d \
./drivers/src/stm32f446xx_I2C.d \
./drivers/src/stm32f446xx_adc.d \
./drivers/src/stm32f446xx_dac.d \
./drivers/src/stm32f446xx_gpio_driver.d \
./drivers/src/stm32f446xx_rcc.d \
./drivers/src/stm32f446xx_spi_driver.d \
./drivers/src/stm32f446xx_timer.d \
./drivers/src/stm32f446xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -DDEBUG -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/drivers/inc" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/Config" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/OS" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/SEGGER/SEGGER" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Config" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"C:/Users/comp 59/Desktop/rtos_project/stm32f446xx_drivers/Third_Party/FreeRTOS/org/Source/include" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


