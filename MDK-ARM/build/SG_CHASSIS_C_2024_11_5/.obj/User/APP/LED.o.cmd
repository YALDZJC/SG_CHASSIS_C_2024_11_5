"D:\Keil5\ARM\ARMCLANG\bin\armclang.exe" --target=arm-arm-none-eabi -c -xc++ -std=c++17 -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../../../AppData/Local/Arm/Packs/ARM/CMSIS-DSP/1.15.0/Include -I../Middlewares/ST/ARM/DSP/Inc -IUser/APP -IUser/BSP -IUser/Algorithm -IUser/HAL -IUser/Task -I.cmsis/include -IRTE/_SG_CHASSIS_C_2024_11_5 -DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4 -D__FPU_PRESENT -mfloat-abi=hard -mfpu=fpv4-sp-d16 -mcpu=cortex-m4 -mlittle-endian -O3 -ffunction-sections -funsigned-char -fshort-enums -fshort-wchar -fno-rtti -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -g -o ./build/SG_CHASSIS_C_2024_11_5/.obj/User/APP/LED.o -MMD ./User/APP/LED.cpp