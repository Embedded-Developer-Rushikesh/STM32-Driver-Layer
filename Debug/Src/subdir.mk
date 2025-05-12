################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio_intrupt.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/gpio_intrupt.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/gpio_intrupt.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/gpio_intrupt.o: ../Src/gpio_intrupt.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"E:/Github/git_repository/STM32_Drivers/STM32_spi_application/STM32-Driver-Layer/Inc/Stm32f446xx_Driver_Layer" -I"E:/Github/git_repository/STM32_Drivers/STM32_spi_application/STM32-Driver-Layer/Inc" -I"E:/Github/git_repository/STM32_Drivers/STM32_spi_application/STM32-Driver-Layer/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"E:/Github/git_repository/STM32_Drivers/STM32_spi_application/STM32-Driver-Layer/Inc/Stm32f446xx_Driver_Layer" -I"E:/Github/git_repository/STM32_Drivers/STM32_spi_application/STM32-Driver-Layer/Inc" -I"E:/Github/git_repository/STM32_Drivers/STM32_spi_application/STM32-Driver-Layer/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/gpio_intrupt.cyclo ./Src/gpio_intrupt.d ./Src/gpio_intrupt.o ./Src/gpio_intrupt.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

