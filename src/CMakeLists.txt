cmake_minimum_required (VERSION 2.8)

# where to look for libOpenCM3
set(CMAKE_MODULE_PATH ../libopencm3)

find_package(OpenCM3 REQUIRED QUIET)
include_directories(${OPENCM3_INCLUDE_DIRS})
add_definitions(${OPENCM3_DEFINITIONS})
link_directories(${OPENCM3_LINK_DIRECTORIES})

add_definitions(-DFIRMWARE_VERSION=\"666\")

include_directories(.)
include_directories(include)
include_directories(platforms/roblink)
include_directories(platforms/stm32)
include_directories(platforms/common)

add_definitions(-DBUILDDATE=\"2015-07-09\" -DVERSION_SUFFIX=\"CMAKE\")

add_definitions(-DSTM32F1)

set(SOURCES
adiv5.c
adiv5_jtagdp.c
adiv5_swdp.c
command.c
cortexm.c
crc32.c
exception.c
gdb_main.c
gdb_packet.c
hex_utils.c
jtag_scan.c
jtagtap_generic.c
kinetis.c
lmi.c
lpc11xx.c
lpc43xx.c
lpc_common.c
main.c
morse.c
nrf51.c
sam3x.c
samd.c
stm32f1.c
stm32f4.c
stm32l0.c
target.c
)

set(SOURCES_PLATFORM
platforms/stm32/gdb_if.c
platforms/stm32/jtagtap.c
platforms/stm32/swdptap.c
platforms/stm32/timing.c
platforms/stm32/serialno.c
#platforms/stm32/cdcacm.c
platforms/stm32/traceswo.c
platforms/common/cdcacm.c
platforms/roblink/usbuart.c
platforms/roblink/platform.c
)

set(SOURCES_BOOTLOADER
platforms/stm32/dfu_f1.c
platforms/stm32/dfucore.c
platforms/roblink/usbdfu.c
)

#*******************************************************************************
# Build BlackMagic
#*******************************************************************************
add_executable(blackmagic ${SOURCES} ${SOURCES_PLATFORM})
target_link_libraries(blackmagic ${OPENCM3_LIBRARIES})
embedded_framework_binhex(blackmagic)

#set_target_properties(blackmagic PROPERTIES LINK_FLAGS_DEBUG "-Wl,-Ttext=0x8002000")
#set_target_properties(blackmagic PROPERTIES LINK_FLAGS_RELEASE "-Wl,-Ttext=0x8002000")

add_custom_target(fwupdate DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/blackmagic.bin COMMAND dfu-util -s 0x08002000 -D ${CMAKE_CURRENT_BINARY_DIR}/blackmagic.bin COMMENT "DFU Upload")

#*******************************************************************************
# Build the bootloader without Ttext
#*******************************************************************************
add_executable(bootloader ${SOURCES_BOOTLOADER})
target_link_libraries(bootloader ${OPENCM3_LIBRARIES})
embedded_framework_binhex(bootloader)

