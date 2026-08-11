cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# CubeMX has its own CNakeLists at:
set(ARM_TOOLCHAIN_BIN
        "/home/leoxu/.local/arm-gnu-toolchain/arm-gnu-toolchain-15.3.rel1-x86_64-arm-none-eabi/bin"
)

set(ENV{PATH} "${ARM_TOOLCHAIN_BIN}:$ENV{PATH}")

set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake"
)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME leo-foc)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Application/Src/application.c
        Application/Inc/application.h
        Application/Src/ws2812.c
        Application/Inc/ws2812.h
        Application/Src/utils.c
        Application/Inc/utils.h
        Application/Src/mt6835.c
        Application/Inc/mt6835.h
        Application/Inc/mt6835.h)

#file(GLOB application_sources Application/Src/*.c)
#file(GLOB application_include Application/Inc/*.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

file(GLOB application_sources CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/Application/Src/*.c"
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
        "${CMAKE_CURRENT_SOURCE_DIR}/Application/Inc"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${application_sources}

)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        "${CMAKE_CURRENT_SOURCE_DIR}/Application/Inc"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols

)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
