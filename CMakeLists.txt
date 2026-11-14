#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.26.4)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(h7_imu C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM7;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING;__FPU_PRESENT=1)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-O0 -g)
endif ()

#set(USER_OPTIMIZATION_LEVEL "-O3 -Ofast -finline-functions")
set(USER_OPTIMIZATION_LEVEL "-Ofast")
#set(USER_OPTIMIZATION_LEVEL "-O0")
#set(USER_OPTIMIZATION_LEVEL "-Og")

set(USER_SRC_FILES_DIR "User/*/*/*.*" "User/*/")

include_directories(Core/Inc USB_DEVICE/App USB_DEVICE/Target Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F Middlewares/ST/STM32_USB_Device_Library/Core/Inc Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include)
include_directories(Middlewares/ST/ARM/DSP/Inc)

include_directories(User/Algorithm/Alg_Queue User/Algorithm/Alg_Matrix User/Algorithm/Alg_Ekf_Quaternion User/Algorithm/Alg_Quaternion User/Algorithm/Alg_ButterWorthFilter User/Algorithm/Alg_Slide_Statistics
                    User/BSP/Buzzer User/BSP/LED User/BSP/USB User/BSP/IMU User/BSP/VBAT User/BSP/Heater User/BSP/Time_Stamp User/BSP/SPI User/BSP/Reset User/BSP/FLASH User/BSP/FDCAN
                    User/BSP
                    User/Callback/Timer_Callback User/Callback/GPIO_Callback User/Callback/FDCAN_Callback
                    User/Components/Component_BMI088
                    User/User_Main)

add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32H723xx)

file(GLOB_RECURSE SOURCES "Library/*.*" "Core/*.*" "Middlewares/*.*" "USB_DEVICE/*.*" "Drivers/*.*"
                ${USER_SRC_FILES_DIR})

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32H723VGTX_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

# add compile options to user src files
#set(USER_Optimize_DIR "User/Algorithm/*" "User/Callback/*" "User/Components/*")
set(USER_Optimize_DIR "User/Algorithm/*" "User/Components/*")
file(GLOB_RECURSE USER_SRC_FILES ${USER_Optimize_DIR}/*.cpp ${USER_Optimize_DIR}/*.c)
set_source_files_properties(
        ${USER_SRC_FILES}
        PROPERTIES COMPILE_FLAGS ${USER_OPTIMIZATION_LEVEL}
)

# link math library:: arm_cortexM7lfsp_math
# means Cortex-M7, Little endian, Single Precision Floating Point Unit
target_link_libraries(${PROJECT_NAME}.elf "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfsp_math.a")

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
