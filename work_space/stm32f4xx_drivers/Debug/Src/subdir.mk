################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/006spi_tx.c 

OBJS += \
./Src/006spi_tx.o 

C_DEPS += \
./Src/006spi_tx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/006spi_tx.o: ../Src/006spi_tx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"E:/stm32_cude_ide/work_space/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/006spi_tx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

