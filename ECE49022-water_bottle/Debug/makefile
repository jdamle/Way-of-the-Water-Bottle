################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include StdPeriph_Driver/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: ECE49022-water_bottle.elf

# Tool invocations
ECE49022-water_bottle.elf: $(OBJS) $(USER_OBJS) C:\Users\jaide\Documents\GitHub\Way-of-the-Water-Bottle\ECE49022-water_bottle/LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -T"C:/Users/jaide/Documents/GitHub/Way-of-the-Water-Bottle/ECE49022-water_bottle/LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "ECE49022-water_bottle.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "ECE49022-water_bottle.elf" "ECE49022-water_bottle.bin"
	arm-none-eabi-size "ECE49022-water_bottle.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
