################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include MPU6050/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
MM_peripherials_config.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MM_peripherials_config.list \

OBJCOPY_BIN += \
MM_peripherials_config.bin \


# All Target
all: MM_peripherials_config.elf secondary-outputs

# Tool invocations
MM_peripherials_config.elf: $(OBJS) $(USER_OBJS) C:\Users\szust\STM32CubeIDE\workspace_1.3.0\MicromouseHAL\MM_peripherials_config\STM32F411RETX_FLASH.ld
	arm-none-eabi-gcc -o "MM_peripherials_config.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\szust\STM32CubeIDE\workspace_1.3.0\MicromouseHAL\MM_peripherials_config\STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MM_peripherials_config.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MM_peripherials_config.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MM_peripherials_config.list"
	@echo 'Finished building: $@'
	@echo ' '

MM_peripherials_config.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MM_peripherials_config.bin"
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
