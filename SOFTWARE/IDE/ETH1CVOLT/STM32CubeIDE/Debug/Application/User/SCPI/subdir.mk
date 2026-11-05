################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/SCPI/printf.c \
../Application/User/SCPI/scpi_def.c \
../Application/User/SCPI/scpi_server.c 

OBJS += \
./Application/User/SCPI/printf.o \
./Application/User/SCPI/scpi_def.o \
./Application/User/SCPI/scpi_server.o 

C_DEPS += \
./Application/User/SCPI/printf.d \
./Application/User/SCPI/scpi_def.d \
./Application/User/SCPI/scpi_server.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/SCPI/%.o Application/User/SCPI/%.su Application/User/SCPI/%.cyclo: ../Application/User/SCPI/%.c Application/User/SCPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_PWR_LDO_SUPPLY -DTX_INCLUDE_USER_DEFINE_FILE -DFX_INCLUDE_USER_DEFINE_FILE -DNX_INCLUDE_USER_DEFINE_FILE -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../../Core/Inc -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/BSD" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/SCPI" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/HiSLIP" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/libscpi/inc" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/libscpi/inc/scpi" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/libscpi/src" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/libscpi" -I../../AZURE_RTOS/App -I../../FileX/App -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/filex/common/inc/ -I../../Middlewares/ST/filex/ports/generic/inc/ -I../../Drivers/BSP/Components/lan8742/ -I../../Middlewares/ST/netxduo/common/drivers/ethernet/ -I../../Middlewares/ST/netxduo/addons/dhcp/ -I../../Middlewares/ST/netxduo/common/inc/ -I../../Middlewares/ST/netxduo/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/netxduo/addons/mdns/ -I../../Middlewares/ST/netxduo/addons/web/ -I../../Middlewares/ST/threadx/common/inc/ -I../../Middlewares/ST/threadx/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/threadx/utility/low_power/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-SCPI

clean-Application-2f-User-2f-SCPI:
	-$(RM) ./Application/User/SCPI/printf.cyclo ./Application/User/SCPI/printf.d ./Application/User/SCPI/printf.o ./Application/User/SCPI/printf.su ./Application/User/SCPI/scpi_def.cyclo ./Application/User/SCPI/scpi_def.d ./Application/User/SCPI/scpi_def.o ./Application/User/SCPI/scpi_def.su ./Application/User/SCPI/scpi_server.cyclo ./Application/User/SCPI/scpi_server.d ./Application/User/SCPI/scpi_server.o ./Application/User/SCPI/scpi_server.su

.PHONY: clean-Application-2f-User-2f-SCPI

