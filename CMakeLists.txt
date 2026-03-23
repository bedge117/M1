# See COPYING.txt for license details.

cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME M1_v0800_C3.9)

# C3 fork revision — must match M1_C3_REVISION in m1_fw_update_bl.h
set(M1_C3_REVISION 9)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Resolve toolchain utilities to full paths so post-build steps work
# regardless of PATH at build time (bare names from toolchain file fail
# inside nested cmd.exe / ninja subprocesses)
get_filename_component(_TC_BIN "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(TOOL_SIZE    ${TOOLCHAIN_PREFIX}size    HINTS "${_TC_BIN}" REQUIRED)
find_program(TOOL_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy HINTS "${_TC_BIN}" REQUIRED)
find_program(TOOL_OBJDUMP ${TOOLCHAIN_PREFIX}objdump HINTS "${_TC_BIN}" REQUIRED)
set(CMAKE_SIZE    "${TOOL_SIZE}")
set(CMAKE_OBJCOPY "${TOOL_OBJCOPY}")
set(CMAKE_OBJDUMP "${TOOL_OBJDUMP}")

# Python is needed for the CRC injection post-build step
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/m1_01)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    m1_01

    # Add user defined libraries
)

# Specify the output ELF file
set(TARGET_ELF "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf")
# Specify the output BIN file
set(TARGET_BIN "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin")
# Command to display memory usage
add_custom_command(
    COMMAND ${CMAKE_COMMAND} -E echo "Memory usage:"    
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_SIZE} ${TARGET_ELF}
    #COMMENT "Memory usage:"
)

# Command to generate .list file
add_custom_command(
    COMMAND ${CMAKE_COMMAND} -E echo "Generating list file ${CMAKE_PROJECT_NAME}.list"
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJDUMP}
-h -S ${TARGET_ELF} > ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.list
    #COMMENT "Generating .list file"
)

# Command to generate .bin file
add_custom_command(
    COMMAND ${CMAKE_COMMAND} -E echo "Generating bin file ${CMAKE_PROJECT_NAME}.bin"
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
-O binary ${TARGET_ELF} ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    #COMMENT "Generating .bin file"
)

# Command to generate .hex file
add_custom_command(
    COMMAND ${CMAKE_COMMAND} -E echo "Generating hex file ${CMAKE_PROJECT_NAME}.hex"
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
-O ihex ${TARGET_ELF} ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    #COMMENT "Generating .hex file"
)

# Command to inject CRC32 + C3 metadata into the .bin file
# Also generates a stock-compatible SD card binary (_sdcard.bin)
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Injecting CRC + C3 metadata into ${CMAKE_PROJECT_NAME}_wCRC.bin"
    COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_SOURCE_DIR}/tools/append_crc32.py
        ${TARGET_BIN}
        --output ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}_wCRC.bin
        --c3-revision ${M1_C3_REVISION}
        --verbose
)