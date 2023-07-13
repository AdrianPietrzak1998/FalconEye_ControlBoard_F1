################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/1wire.c \
../Core/Src/Eeprom_backup.c \
../Core/Src/GFX_BW.c \
../Core/Src/M24Cxx.c \
../Core/Src/adc.c \
../Core/Src/button.c \
../Core/Src/dma.c \
../Core/Src/ds18b20.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/led_blink.c \
../Core/Src/main.c \
../Core/Src/menu.c \
../Core/Src/menu_out_set.c \
../Core/Src/parser_complex.c \
../Core/Src/ring_buffer.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/utils.c 

OBJS += \
./Core/Src/1wire.o \
./Core/Src/Eeprom_backup.o \
./Core/Src/GFX_BW.o \
./Core/Src/M24Cxx.o \
./Core/Src/adc.o \
./Core/Src/button.o \
./Core/Src/dma.o \
./Core/Src/ds18b20.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/led_blink.o \
./Core/Src/main.o \
./Core/Src/menu.o \
./Core/Src/menu_out_set.o \
./Core/Src/parser_complex.o \
./Core/Src/ring_buffer.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/utils.o 

C_DEPS += \
./Core/Src/1wire.d \
./Core/Src/Eeprom_backup.d \
./Core/Src/GFX_BW.d \
./Core/Src/M24Cxx.d \
./Core/Src/adc.d \
./Core/Src/button.d \
./Core/Src/dma.d \
./Core/Src/ds18b20.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/led_blink.d \
./Core/Src/main.d \
./Core/Src/menu.d \
./Core/Src/menu_out_set.d \
./Core/Src/parser_complex.d \
./Core/Src/ring_buffer.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/1wire.cyclo ./Core/Src/1wire.d ./Core/Src/1wire.o ./Core/Src/1wire.su ./Core/Src/Eeprom_backup.cyclo ./Core/Src/Eeprom_backup.d ./Core/Src/Eeprom_backup.o ./Core/Src/Eeprom_backup.su ./Core/Src/GFX_BW.cyclo ./Core/Src/GFX_BW.d ./Core/Src/GFX_BW.o ./Core/Src/GFX_BW.su ./Core/Src/M24Cxx.cyclo ./Core/Src/M24Cxx.d ./Core/Src/M24Cxx.o ./Core/Src/M24Cxx.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/button.cyclo ./Core/Src/button.d ./Core/Src/button.o ./Core/Src/button.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/ds18b20.cyclo ./Core/Src/ds18b20.d ./Core/Src/ds18b20.o ./Core/Src/ds18b20.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/led_blink.cyclo ./Core/Src/led_blink.d ./Core/Src/led_blink.o ./Core/Src/led_blink.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/menu.cyclo ./Core/Src/menu.d ./Core/Src/menu.o ./Core/Src/menu.su ./Core/Src/menu_out_set.cyclo ./Core/Src/menu_out_set.d ./Core/Src/menu_out_set.o ./Core/Src/menu_out_set.su ./Core/Src/parser_complex.cyclo ./Core/Src/parser_complex.d ./Core/Src/parser_complex.o ./Core/Src/parser_complex.su ./Core/Src/ring_buffer.cyclo ./Core/Src/ring_buffer.d ./Core/Src/ring_buffer.o ./Core/Src/ring_buffer.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/utils.cyclo ./Core/Src/utils.d ./Core/Src/utils.o ./Core/Src/utils.su

.PHONY: clean-Core-2f-Src

