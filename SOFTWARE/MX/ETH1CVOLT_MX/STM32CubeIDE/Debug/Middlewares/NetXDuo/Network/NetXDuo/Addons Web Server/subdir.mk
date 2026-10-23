################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/grzegorz/git/ETH1CVOLT/SOFTWARE/MX/ETH1CVOLT_MX/Middlewares/ST/netxduo/addons/web/nx_tcpserver.c \
/home/grzegorz/git/ETH1CVOLT/SOFTWARE/MX/ETH1CVOLT_MX/Middlewares/ST/netxduo/addons/web/nx_web_http_server.c 

OBJS += \
./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_tcpserver.o \
./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_web_http_server.o 

C_DEPS += \
./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_tcpserver.d \
./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_web_http_server.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_tcpserver.o: /home/grzegorz/git/ETH1CVOLT/SOFTWARE/MX/ETH1CVOLT_MX/Middlewares/ST/netxduo/addons/web/nx_tcpserver.c Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_PWR_LDO_SUPPLY -DTX_INCLUDE_USER_DEFINE_FILE -DFX_INCLUDE_USER_DEFINE_FILE -DNX_INCLUDE_USER_DEFINE_FILE -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../../Core/Inc -I../../AZURE_RTOS/App -I../../FileX/App -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/filex/common/inc/ -I../../Middlewares/ST/filex/ports/generic/inc/ -I../../Drivers/BSP/Components/lan8742/ -I../../Middlewares/ST/netxduo/common/drivers/ethernet/ -I../../Middlewares/ST/netxduo/addons/dhcp/ -I../../Middlewares/ST/netxduo/common/inc/ -I../../Middlewares/ST/netxduo/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/netxduo/addons/mdns/ -I../../Middlewares/ST/netxduo/addons/web/ -I../../Middlewares/ST/threadx/common/inc/ -I../../Middlewares/ST/threadx/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/threadx/utility/low_power/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/NetXDuo/Network/NetXDuo/Addons Web Server/nx_tcpserver.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_web_http_server.o: /home/grzegorz/git/ETH1CVOLT/SOFTWARE/MX/ETH1CVOLT_MX/Middlewares/ST/netxduo/addons/web/nx_web_http_server.c Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_FULL_LL_DRIVER -DUSE_PWR_LDO_SUPPLY -DTX_INCLUDE_USER_DEFINE_FILE -DFX_INCLUDE_USER_DEFINE_FILE -DNX_INCLUDE_USER_DEFINE_FILE -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../../Core/Inc -I../../AZURE_RTOS/App -I../../FileX/App -I../../NetXDuo/App -I../../NetXDuo/Target -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/ST/filex/common/inc/ -I../../Middlewares/ST/filex/ports/generic/inc/ -I../../Drivers/BSP/Components/lan8742/ -I../../Middlewares/ST/netxduo/common/drivers/ethernet/ -I../../Middlewares/ST/netxduo/addons/dhcp/ -I../../Middlewares/ST/netxduo/common/inc/ -I../../Middlewares/ST/netxduo/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/netxduo/addons/mdns/ -I../../Middlewares/ST/netxduo/addons/web/ -I../../Middlewares/ST/threadx/common/inc/ -I../../Middlewares/ST/threadx/ports/cortex_m7/gnu/inc/ -I../../Middlewares/ST/threadx/utility/low_power/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Middlewares/NetXDuo/Network/NetXDuo/Addons Web Server/nx_web_http_server.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-NetXDuo-2f-Network-2f-NetXDuo-2f-Addons-20-Web-20-Server

clean-Middlewares-2f-NetXDuo-2f-Network-2f-NetXDuo-2f-Addons-20-Web-20-Server:
	-$(RM) ./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_tcpserver.cyclo ./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_tcpserver.d ./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_tcpserver.o ./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_tcpserver.su ./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_web_http_server.cyclo ./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_web_http_server.d ./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_web_http_server.o ./Middlewares/NetXDuo/Network/NetXDuo/Addons\ Web\ Server/nx_web_http_server.su

.PHONY: clean-Middlewares-2f-NetXDuo-2f-Network-2f-NetXDuo-2f-Addons-20-Web-20-Server

