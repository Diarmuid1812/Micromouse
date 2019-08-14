################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/MPU6050.c \
../src/delay.c \
../src/main.c \
../src/syscalls.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/MPU6050.o \
./src/delay.o \
./src/main.o \
./src/syscalls.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/MPU6050.d \
./src/delay.d \
./src/main.d \
./src/syscalls.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F411RETx -DNUCLEO_F411RE -DDEBUG -DSTM32F411xE -DUSE_STDPERIPH_DRIVER -I"C:/Users/Wojtek/Documents/STM32_workspace/PID/StdPeriph_Driver/inc" -I"C:/Users/Wojtek/Documents/STM32_workspace/PID/inc" -I"C:/Users/Wojtek/Documents/STM32_workspace/PID/CMSIS/device" -I"C:/Users/Wojtek/Documents/STM32_workspace/PID/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


