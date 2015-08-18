include(CMakeForceCompiler)

# ---------------------------------------------
# this one is important
# ---------------------------------------------
set(CMAKE_SYSTEM_NAME ARM)
set(CMAKE_SYSTEM_PROCESSOR ARM_Embedded)

# ---------------------------------------------
# Toolchain prefix
# ---------------------------------------------
set(TOOLCHAIN_PREFIX arm-none-eabi)

# ---------------------------------------------
# Toolchain version
# ---------------------------------------------
set(TOOLCHAIN_VERSION 4.9.3)

# ---------------------------------------------
# Where is the target environment 
# ---------------------------------------------
set(CMAKE_FIND_ROOT_PATH /usr/${TOOLCHAIN_PREFIX})

# ---------------------------------------------
# Specify the cross compiler tools
# ---------------------------------------------
find_program(ARM_CC ${TOOLCHAIN_PREFIX}-gcc HINTS ${CMAKE_FIND_ROOT_PATH}/bin)
find_program(ARM_CXX ${TOOLCHAIN_PREFIX}-g++ HINTS ${CMAKE_FIND_ROOT_PATH}/bin)
CMAKE_FORCE_C_COMPILER(${ARM_CC} GNU)
CMAKE_FORCE_CXX_COMPILER(${ARM_CXX} GNU)
find_program(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc HINTS ${CMAKE_FIND_ROOT_PATH}/bin)
find_program(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar HINTS ${CMAKE_FIND_ROOT_PATH}/bin)
find_program(CMAKE_NM ${TOOLCHAIN_PREFIX}-nm HINTS ${CMAKE_FIND_ROOT_PATH}/bin)
find_program(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld HINTS ${CMAKE_FIND_ROOT_PATH}/bin)
find_program(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-ranlib HINTS ${CMAKE_FIND_ROOT_PATH}/bin)
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy HINTS ${CMAKE_FIND_ROOT_PATH}/bin)
find_program(GDB ${TOOLCHAIN_PREFIX}-gdb HINTS ${CMAKE_FIND_ROOT_PATH}/bin)

# ---------------------------------------------
# search for programs in the build host directories
# ---------------------------------------------
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# ---------------------------------------------
# for libraries and headers in the target directories
# ---------------------------------------------
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
