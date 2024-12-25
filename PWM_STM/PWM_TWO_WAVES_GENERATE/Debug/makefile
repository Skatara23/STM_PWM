################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
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


BUILD_ARTIFACT_NAME := PWM_TWO_WAVES_GENERATE
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
PWM_TWO_WAVES_GENERATE.elf \

MAP_FILES += \
PWM_TWO_WAVES_GENERATE.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
PWM_TWO_WAVES_GENERATE.list \

OBJCOPY_HEX += \
PWM_TWO_WAVES_GENERATE.hex \


# All Target
all: main-build

# Main-build Target
main-build: PWM_TWO_WAVES_GENERATE.elf secondary-outputs

# Tool invocations
PWM_TWO_WAVES_GENERATE.elf PWM_TWO_WAVES_GENERATE.map: $(OBJS) $(USER_OBJS) E:\STM32_prgrm\PWM_TWO_WAVES_GENERATE\STM32F401RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "PWM_TWO_WAVES_GENERATE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"E:\STM32_prgrm\PWM_TWO_WAVES_GENERATE\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="PWM_TWO_WAVES_GENERATE.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

PWM_TWO_WAVES_GENERATE.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "PWM_TWO_WAVES_GENERATE.list"
	@echo 'Finished building: $@'
	@echo ' '

PWM_TWO_WAVES_GENERATE.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "PWM_TWO_WAVES_GENERATE.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) PWM_TWO_WAVES_GENERATE.elf PWM_TWO_WAVES_GENERATE.hex PWM_TWO_WAVES_GENERATE.list PWM_TWO_WAVES_GENERATE.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
