################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := CMSIS_SunRiseClock
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
CMSIS_SunRiseClock.elf \

MAP_FILES += \
CMSIS_SunRiseClock.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CMSIS_SunRiseClock.list \

OBJCOPY_HEX += \
CMSIS_SunRiseClock.hex \

OBJCOPY_BIN += \
CMSIS_SunRiseClock.bin \


# All Target
all: main-build

# Main-build Target
main-build: CMSIS_SunRiseClock.elf secondary-outputs

# Tool invocations
CMSIS_SunRiseClock.elf CMSIS_SunRiseClock.map: $(OBJS) $(USER_OBJS) C:\Users\kalichak.y\Documents\budylnyk-svitanok-by-kalichak-yurii\STM32L010K4TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "CMSIS_SunRiseClock.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\kalichak.y\Documents\budylnyk-svitanok-by-kalichak-yurii\STM32L010K4TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="CMSIS_SunRiseClock.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CMSIS_SunRiseClock.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CMSIS_SunRiseClock.list"
	@echo 'Finished building: $@'
	@echo ' '

CMSIS_SunRiseClock.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "CMSIS_SunRiseClock.hex"
	@echo 'Finished building: $@'
	@echo ' '

CMSIS_SunRiseClock.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "CMSIS_SunRiseClock.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) CMSIS_SunRiseClock.bin CMSIS_SunRiseClock.elf CMSIS_SunRiseClock.hex CMSIS_SunRiseClock.list CMSIS_SunRiseClock.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
