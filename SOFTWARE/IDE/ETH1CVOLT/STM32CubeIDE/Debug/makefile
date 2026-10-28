################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/ThreadX/RTOS/ThreadX/TraceX\ support/subdir.mk
-include Middlewares/ThreadX/RTOS/ThreadX/PerformanceInfo/subdir.mk
-include Middlewares/ThreadX/RTOS/ThreadX/Low\ Power\ support/subdir.mk
-include Middlewares/ThreadX/RTOS/ThreadX/Core/subdir.mk
-include Middlewares/NetXDuo/Network/NetXDuo/NX\ Core/subdir.mk
-include Middlewares/NetXDuo/Network/NetXDuo/Addons\ mDNS/subdir.mk
-include Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/subdir.mk
-include Middlewares/NetXDuo/Network/NetXDuo/Addons\ DHCP\ Client/subdir.mk
-include Middlewares/Interfaces/Network/Ethernet\ Phy\ Interface/LAN8742_Phy_Interface/subdir.mk
-include Middlewares/Interfaces/Network/Ethernet\ Interface/subdir.mk
-include Middlewares/FileX/File\ System/FileX/Core/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/Components/subdir.mk
-include Application/User/libscpi/src/subdir.mk
-include Application/User/Startup/subdir.mk
-include Application/User/NetXDuo/App/subdir.mk
-include Application/User/HiSLIP/subdir.mk
-include Application/User/FileX/App/subdir.mk
-include Application/User/Core/subdir.mk
-include Application/User/AZURE_RTOS/App/subdir.mk
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


BUILD_ARTIFACT_NAME := ETH1CVOLT
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
ETH1CVOLT.elf \

MAP_FILES += \
ETH1CVOLT.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ETH1CVOLT.list \


# All Target
all: main-build

# Main-build Target
main-build: ETH1CVOLT.elf secondary-outputs

# Tool invocations
ETH1CVOLT.elf ETH1CVOLT.map: $(OBJS) $(USER_OBJS) /home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/STM32H743VITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "ETH1CVOLT.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/STM32H743VITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ETH1CVOLT.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ETH1CVOLT.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ETH1CVOLT.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) ETH1CVOLT.elf ETH1CVOLT.list ETH1CVOLT.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
