cmake_minimum_required (VERSION 2.8)

include_directories(../stm32)

set(SOURCES_BOOTLOADER
../stm32/dfu_f1.c
../stm32/dfucore.c
usbdfu.c
)

#*******************************************************************************
# Build the bootloader without Ttext
#*******************************************************************************
add_executable(bootloader ${SOURCES_BOOTLOADER})
target_link_libraries(bootloader ${OPENCM3_LIBRARIES})
embedded_framework_binhex(bootloader)
