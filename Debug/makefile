################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
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
sansan_H7MainBoard_firmware.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
sansan_H7MainBoard_firmware.list \

OBJCOPY_BIN += \
sansan_H7MainBoard_firmware.bin \


# All Target
all: sansan_H7MainBoard_firmware.elf secondary-outputs

# Tool invocations
sansan_H7MainBoard_firmware.elf: $(OBJS) $(USER_OBJS) C:\Users\Akito\OneDrive\ -\ stn.nagaokaut.ac.jp\SSL\cubeIDEprojects\sansan_H7MainBoard_firmware\STM32H753VITX_FLASH.ld
	arm-none-eabi-gcc -o "sansan_H7MainBoard_firmware.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"C:\Users\Akito\OneDrive - stn.nagaokaut.ac.jp\SSL\cubeIDEprojects\sansan_H7MainBoard_firmware\STM32H753VITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="sansan_H7MainBoard_firmware.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

sansan_H7MainBoard_firmware.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "sansan_H7MainBoard_firmware.list"
	@echo 'Finished building: $@'
	@echo ' '

sansan_H7MainBoard_firmware.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "sansan_H7MainBoard_firmware.bin"
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
