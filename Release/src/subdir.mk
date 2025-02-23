################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/001led_toggle.c \
../src/002led_button.c \
../src/004gpio_freq.c \
../src/006spi_tx_testing.c \
../src/007+SPI_txonly_arduino_generic.c \
../src/007spi_txonly_arduino.c \
../src/008SpicmdHandling.c \
../src/009spi_cmd_handling_it.c \
../src/010+I2c_master_tx_testing.c \
../src/010i2c_master_tx_testing.c \
../src/011+i2c_master_rx_testing2.c \
../src/011i2c_master_rx_testing.c \
../src/012i2c_master_rx_testingIT.c \
../src/IMU.c 

OBJS += \
./src/001led_toggle.o \
./src/002led_button.o \
./src/004gpio_freq.o \
./src/006spi_tx_testing.o \
./src/007+SPI_txonly_arduino_generic.o \
./src/007spi_txonly_arduino.o \
./src/008SpicmdHandling.o \
./src/009spi_cmd_handling_it.o \
./src/010+I2c_master_tx_testing.o \
./src/010i2c_master_tx_testing.o \
./src/011+i2c_master_rx_testing2.o \
./src/011i2c_master_rx_testing.o \
./src/012i2c_master_rx_testingIT.o \
./src/IMU.o 

C_DEPS += \
./src/001led_toggle.d \
./src/002led_button.d \
./src/004gpio_freq.d \
./src/006spi_tx_testing.d \
./src/007+SPI_txonly_arduino_generic.d \
./src/007spi_txonly_arduino.d \
./src/008SpicmdHandling.d \
./src/009spi_cmd_handling_it.d \
./src/010+I2c_master_tx_testing.d \
./src/010i2c_master_tx_testing.d \
./src/011+i2c_master_rx_testing2.d \
./src/011i2c_master_rx_testing.d \
./src/012i2c_master_rx_testingIT.d \
./src/IMU.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -O3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


