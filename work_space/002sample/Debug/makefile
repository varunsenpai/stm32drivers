################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
002sample.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
002sample.list \

OBJCOPY_BIN += \
002sample.bin \


# All Target
all: 002sample.elf secondary-outputs

# Tool invocations
002sample.elf: $(OBJS) $(USER_OBJS) E:\stm32_cude_ide\work_space\002sample\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "002sample.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"E:\stm32_cude_ide\work_space\002sample\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="002sample.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

002sample.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "002sample.list"
	@echo 'Finished building: $@'
	@echo ' '

002sample.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "002sample.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
