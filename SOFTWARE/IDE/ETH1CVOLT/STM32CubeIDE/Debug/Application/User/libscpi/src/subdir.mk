################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/libscpi/src/error.c \
../Application/User/libscpi/src/expression.c \
../Application/User/libscpi/src/fifo.c \
../Application/User/libscpi/src/ieee488.c \
../Application/User/libscpi/src/lexer.c \
../Application/User/libscpi/src/minimal.c \
../Application/User/libscpi/src/parser.c \
../Application/User/libscpi/src/units.c \
../Application/User/libscpi/src/utils.c 

OBJS += \
./Application/User/libscpi/src/error.o \
./Application/User/libscpi/src/expression.o \
./Application/User/libscpi/src/fifo.o \
./Application/User/libscpi/src/ieee488.o \
./Application/User/libscpi/src/lexer.o \
./Application/User/libscpi/src/minimal.o \
./Application/User/libscpi/src/parser.o \
./Application/User/libscpi/src/units.o \
./Application/User/libscpi/src/utils.o 

C_DEPS += \
./Application/User/libscpi/src/error.d \
./Application/User/libscpi/src/expression.d \
./Application/User/libscpi/src/fifo.d \
./Application/User/libscpi/src/ieee488.d \
./Application/User/libscpi/src/lexer.d \
./Application/User/libscpi/src/minimal.d \
./Application/User/libscpi/src/parser.d \
./Application/User/libscpi/src/units.d \
./Application/User/libscpi/src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/libscpi/src/%.o Application/User/libscpi/src/%.su Application/User/libscpi/src/%.cyclo: ../Application/User/libscpi/src/%.c Application/User/libscpi/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_PWR_LDO_SUPPLY -DTX_INCLUDE_USER_DEFINE_FILE -DFX_INCLUDE_USER_DEFINE_FILE -DNX_INCLUDE_USER_DEFINE_FILE -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../../Core/Inc -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/HiSLIP" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/libscpi/inc" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/libscpi/inc/scpi" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/libscpi/src" -I"/home/grzegorz/git/ETH1CVOLT/SOFTWARE/IDE/ETH1CVOLT/STM32CubeIDE/Application/User/libscpi" -I../../AZURE_RTOS/App -I../../FileX/App -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/filex/common/inc/ -I../../Middlewares/ST/filex/ports/generic/inc/ -I../../Drivers/BSP/Components/lan8742/ -I../../Middlewares/ST/netxduo/common/drivers/ethernet/ -I../../Middlewares/ST/netxduo/addons/dhcp/ -I../../Middlewares/ST/netxduo/common/inc/ -I../../Middlewares/ST/netxduo/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/netxduo/addons/mdns/ -I../../Middlewares/ST/netxduo/addons/web/ -I../../Middlewares/ST/threadx/common/inc/ -I../../Middlewares/ST/threadx/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/threadx/utility/low_power/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-libscpi-2f-src

clean-Application-2f-User-2f-libscpi-2f-src:
	-$(RM) ./Application/User/libscpi/src/error.cyclo ./Application/User/libscpi/src/error.d ./Application/User/libscpi/src/error.o ./Application/User/libscpi/src/error.su ./Application/User/libscpi/src/expression.cyclo ./Application/User/libscpi/src/expression.d ./Application/User/libscpi/src/expression.o ./Application/User/libscpi/src/expression.su ./Application/User/libscpi/src/fifo.cyclo ./Application/User/libscpi/src/fifo.d ./Application/User/libscpi/src/fifo.o ./Application/User/libscpi/src/fifo.su ./Application/User/libscpi/src/ieee488.cyclo ./Application/User/libscpi/src/ieee488.d ./Application/User/libscpi/src/ieee488.o ./Application/User/libscpi/src/ieee488.su ./Application/User/libscpi/src/lexer.cyclo ./Application/User/libscpi/src/lexer.d ./Application/User/libscpi/src/lexer.o ./Application/User/libscpi/src/lexer.su ./Application/User/libscpi/src/minimal.cyclo ./Application/User/libscpi/src/minimal.d ./Application/User/libscpi/src/minimal.o ./Application/User/libscpi/src/minimal.su ./Application/User/libscpi/src/parser.cyclo ./Application/User/libscpi/src/parser.d ./Application/User/libscpi/src/parser.o ./Application/User/libscpi/src/parser.su ./Application/User/libscpi/src/units.cyclo ./Application/User/libscpi/src/units.d ./Application/User/libscpi/src/units.o ./Application/User/libscpi/src/units.su ./Application/User/libscpi/src/utils.cyclo ./Application/User/libscpi/src/utils.d ./Application/User/libscpi/src/utils.o ./Application/User/libscpi/src/utils.su

.PHONY: clean-Application-2f-User-2f-libscpi-2f-src

