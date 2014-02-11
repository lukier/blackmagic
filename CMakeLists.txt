cmake_minimum_required (VERSION 2.8)

#*******************************************************************************
# Project name
#*******************************************************************************
project(BlackMagic)

# ----------------------------------------------------------------------
# Display build information
# ----------------------------------------------------------------------
message("Project: ${PROJECT_NAME} build on ${CMAKE_HOST_SYSTEM_NAME} for ${CMAKE_SYSTEM_NAME}, Type: ${CMAKE_BUILD_TYPE}")

# ---------------------------------------------
# Enable CMake testing and assembly
# ---------------------------------------------
enable_testing()
enable_language(ASM)

# ---------------------------------------------
# Set default build type
# ---------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
elseif(${CMAKE_BUILD_TYPE} MATCHES "None")
    set(CMAKE_BUILD_TYPE Debug)
endif()

#*******************************************************************************
# Configure the compiler
#*******************************************************************************
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    # ---------------------------------------------
    # Limited warning suppressions
    # ---------------------------------------------
    set(WARN_SUPPR "-Wno-unused-parameter -Wno-sign-compare")

    # ---------------------------------------------
    # C/C++ standard
    # ---------------------------------------------
    set(STD_CC gnu11)
    set(STD_CXX gnu++11)

    if(NOT CMAKE_CROSSCOMPILING)
        set(X86_ARCH "-march=native")
        set(PC_SIMULATION ON)
    endif()

    # ---------------------------------------------
    # Default flags 
    # ---------------------------------------------
    set(CMAKE_C_FLAGS_DEBUG            "-O0 -g -Wall -Wextra ${WARN_SUPPR} ${X86_ARCH} -std=${STD_CC}")
    set(CMAKE_CXX_FLAGS_DEBUG          "-O0 -g -Wall -Wextra ${WARN_SUPPR} ${X86_ARCH} -std=${STD_CXX}")
    set(CMAKE_C_FLAGS_RELEASE          "-Os -g -Wall -Wextra ${WARN_SUPPR} ${X86_ARCH} -std=${STD_CC}")
    set(CMAKE_CXX_FLAGS_RELEASE        "-Os -g -Wall -Wextra ${WARN_SUPPR} ${X86_ARCH} -std=${STD_CXX}")
endif()

#*******************************************************************************
# Embedded flags and settings
#*******************************************************************************
# ---------------------------------------------
# General embedded configuration
# ---------------------------------------------
set(CMAKE_EXECUTABLE_SUFFIX .elf) 

# ---------------------------------------------
# Very embedded specific
# ---------------------------------------------

# ---------------------------------------------
# Turn off heavy stuff
# ---------------------------------------------
set(EXTRA_CXXFLAGS "${EXTRA_CXXFLAGS} -fno-exceptions -fno-non-call-exceptions")

# ---------------------------------------------
# Enable C++11 features (chrono, thread/mutex/future are heavy and useless (and require gthr))
# ---------------------------------------------
add_definitions(-D_POSIX_MONOTONIC_CLOCK)
add_definitions(-D_GLIBCXX_USE_NANOSLEEP)
add_definitions(-D_GLIBCXX_USE_GETTIMEOFDAY)
add_definitions(-D_GLIBCXX_USE_CLOCK_REALTIME)
add_definitions(-D_GLIBCXX_USE_CLOCK_MONOTONIC)
add_definitions(-D_POSIX_TIMERS)

# ---------------------------------------------
# Other options
# ---------------------------------------------
set(EXTRA_CFLAGS "${EXTRA_CFLAGS} -fno-common -fno-strict-aliasing -fdata-sections -ffunction-sections")
set(EXTRA_CXXFLAGS "${EXTRA_CXXFLAGS} -fno-common -fno-strict-aliasing -fdata-sections -ffunction-sections")

# ---------------------------------------------
# C compiler flags
# ---------------------------------------------
set(CMAKE_C_FLAGS_DEBUG   "${CMAKE_C_FLAGS_DEBUG} ${EXTRA_CFLAGS}")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${EXTRA_CFLAGS}")

# ---------------------------------------------
# C++ compiler flags
# ---------------------------------------------
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} ${EXTRA_CXXFLAGS} -fno-rtti -Winline -fno-threadsafe-statics")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${EXTRA_CXXFLAGS} -fno-rtti -Winline -fno-threadsafe-statics")

# ---------------------------------------------
# Assembler flags
# ---------------------------------------------
set(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} ${EXTRA_AFLAGS} -Wa,-mapcs-32")
set(CMAKE_ASM_FLAGS_DEBUG   "${CMAKE_ASM_FLAGS_DEBUG} ${EXTRA_AFLAGS} -Wa,-mapcs-32 -gdwarf-2")

# ---------------------------------------------
# Executable linker flags
# ---------------------------------------------
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_CFLAGS} ${EXTRA_CXXFLAGS} ${EXTRA_LDFLAGS} -static -Wl,--gc-sections -nostartfiles -nostdlib -Wl,--demangle")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_CFLAGS} ${EXTRA_CXXFLAGS} ${EXTRA_LDFLAGS} -static -Wl,--gc-sections -nostartfiles -nostdlib -Wl,--demangle")

# ---------------------------------------------
# Compiler options
# ---------------------------------------------
add_definitions(-mcpu=cortex-m3 -mthumb)

# ---------------------------------------------
# Linker script
# ---------------------------------------------
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/platforms/roblink)
set(EMBEDDED_LDSCRIPT roblink.ld)

# ---------------------------------------------
# Append linker flags
# ---------------------------------------------
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${EMBEDDED_LDSCRIPT}") 
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${EMBEDDED_LDSCRIPT}")

# ---------------------------------------------
# Linker paths and libraries
# --------------------------------------------- 
link_directories(/usr/${TOOLCHAIN_PREFIX}/${TOOLCHAIN_PREFIX}/lib/armv7-m)
link_directories(/usr/${TOOLCHAIN_PREFIX}/lib/gcc/${TOOLCHAIN_PREFIX}/${TOOLCHAIN_VERSION}/armv7-m)
set(STDLIB_COMMAND "-lc_s -lnosys -lgcc")

string(REPLACE "<LINK_LIBRARIES>" "-Wl,--start-group <LINK_LIBRARIES> ${STDLIB_COMMAND} -Wl,--end-group" CMAKE_C_LINK_EXECUTABLE         "${CMAKE_C_LINK_EXECUTABLE}")
string(REPLACE "<LINK_LIBRARIES>" "-Wl,--start-group <LINK_LIBRARIES> ${STDLIB_COMMAND} -Wl,--end-group" CMAKE_C_CREATE_SHARED_LIBRARY   "${CMAKE_C_CREATE_SHARED_LIBRARY}")
string(REPLACE "<LINK_LIBRARIES>" "-Wl,--start-group <LINK_LIBRARIES> ${STDLIB_COMMAND} -Wl,--end-group" CMAKE_C_CREATE_SHARED_MODULE    "${CMAKE_C_CREATE_SHARED_MODULE}")
string(REPLACE "<LINK_LIBRARIES>" "-Wl,--start-group <LINK_LIBRARIES> ${STDLIB_COMMAND} -Wl,--end-group" CMAKE_CXX_LINK_EXECUTABLE       "${CMAKE_CXX_LINK_EXECUTABLE}")
string(REPLACE "<LINK_LIBRARIES>" "-Wl,--start-group <LINK_LIBRARIES> ${STDLIB_COMMAND} -Wl,--end-group" CMAKE_CXX_CREATE_SHARED_LIBRARY "${CMAKE_CXX_CREATE_SHARED_LIBRARY}")
string(REPLACE "<LINK_LIBRARIES>" "-Wl,--start-group <LINK_LIBRARIES> ${STDLIB_COMMAND} -Wl,--end-group" CMAKE_CXX_CREATE_SHARED_MODULE  "${CMAKE_CXX_CREATE_SHARED_MODULE}")

# ---------------------------------------------
# Generate bin/hex targets function
# ---------------------------------------------
function(embedded_framework_binhex target)
    set_target_properties(${target} PROPERTIES LINK_FLAGS "-Wl,-Map,${target}.elf.map,--cref")
    add_custom_command(OUTPUT ${target}.bin COMMAND ${CMAKE_OBJCOPY} ${CMAKE_OCFLAGS} -O binary ${CMAKE_CURRENT_BINARY_DIR}/${target}.elf ${CMAKE_CURRENT_BINARY_DIR}/${target}.bin DEPENDS ${target} COMMENT "Generate flat binary")
    add_custom_command(OUTPUT ${target}.hex COMMAND ${CMAKE_OBJCOPY} ${CMAKE_OCFLAGS} -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${target}.elf ${CMAKE_CURRENT_BINARY_DIR}/${target}.hex DEPENDS ${target} COMMENT "Generate hex file")
    add_custom_target(${target}_all ALL DEPENDS ${target} ${target}.bin ${target}.hex COMMAND /usr/arm-none-eabi/bin/arm-none-eabi-size ${CMAKE_CURRENT_BINARY_DIR}/${target}.elf COMMENT "Binary size info")
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${CMAKE_CURRENT_BINARY_DIR}/${target}.bin;${CMAKE_CURRENT_BINARY_DIR}/${target}.hex;${CMAKE_CURRENT_BINARY_DIR}/${target}.elf.map")
endfunction()

#*******************************************************************************
# Build
#*******************************************************************************
add_definitions(-DMAIN_STACK_SIZE=4096)
set(EMBEDDED_CPU_NAME STM32F103)
add_subdirectory(libopencm3)
add_subdirectory(src)