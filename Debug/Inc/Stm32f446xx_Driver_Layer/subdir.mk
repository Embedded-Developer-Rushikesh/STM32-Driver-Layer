################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/Stm32f446xx_Driver_Layer/STM32F446xx_rcc_driver.c \
../Inc/Stm32f446xx_Driver_Layer/STM32F446xx_uart_driver.c \
../Inc/Stm32f446xx_Driver_Layer/stm32f446xx_SysTick_driver.c \
../Inc/Stm32f446xx_Driver_Layer/stm32f446xx_gpio_driver.c \
../Inc/Stm32f446xx_Driver_Layer/stm32f446xx_spi_driver.c 

OBJS += \
./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_rcc_driver.o \
./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_uart_driver.o \
./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_SysTick_driver.o \
./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_gpio_driver.o \
./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_spi_driver.o 

C_DEPS += \
./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_rcc_driver.d \
./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_uart_driver.d \
./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_SysTick_driver.d \
./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_gpio_driver.d \
./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Inc/Stm32f446xx_Driver_Layer/%.o Inc/Stm32f446xx_Driver_Layer/%.su Inc/Stm32f446xx_Driver_Layer/%.cyclo: ../Inc/Stm32f446xx_Driver_Layer/%.c Inc/Stm32f446xx_Driver_Layer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -DNUCLEO_F446RE -c -I../Inc -I"E:/Github/git_repository/STM32_Drivers/STM32_spi_application/STM32-Driver-Layer/Inc" -I"E:/Github/git_repository/STM32_Drivers/STM32_spi_application/STM32-Driver-Layer/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Inc-2f-Stm32f446xx_Driver_Layer

clean-Inc-2f-Stm32f446xx_Driver_Layer:
	-$(RM) ./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_rcc_driver.cyclo ./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_rcc_driver.d ./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_rcc_driver.o ./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_rcc_driver.su ./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_uart_driver.cyclo ./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_uart_driver.d ./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_uart_driver.o ./Inc/Stm32f446xx_Driver_Layer/STM32F446xx_uart_driver.su ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_SysTick_driver.cyclo ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_SysTick_driver.d ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_SysTick_driver.o ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_SysTick_driver.su ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_gpio_driver.cyclo ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_gpio_driver.d ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_gpio_driver.o ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_gpio_driver.su ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_spi_driver.cyclo ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_spi_driver.d ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_spi_driver.o ./Inc/Stm32f446xx_Driver_Layer/stm32f446xx_spi_driver.su

.PHONY: clean-Inc-2f-Stm32f446xx_Driver_Layer

