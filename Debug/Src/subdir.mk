################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/uart_it.c 

OBJS += \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/uart_it.o 

C_DEPS += \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/uart_it.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"E:/Github/git_repository/STM32_Drivers/STM32_uart_application/STM32-Driver-Layer/Inc/Stm32f446xx_Driver_Layer" -I"E:/Github/git_repository/STM32_Drivers/STM32_uart_application/STM32-Driver-Layer/Inc" -I"E:/Github/git_repository/STM32_Drivers/STM32_uart_application/STM32-Driver-Layer/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/uart_it.cyclo ./Src/uart_it.d ./Src/uart_it.o ./Src/uart_it.su

.PHONY: clean-Src

