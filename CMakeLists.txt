cmake_minimum_required(VERSION 3.10)
project(nes_emulator C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add MSYS2 mingw64 paths for Windows
if(WIN32)
    list(APPEND CMAKE_PREFIX_PATH "C:/msys64/mingw64")
    include_directories("C:/msys64/mingw64/include")
endif()

# Find SDL2 package
find_package(SDL2 REQUIRED)

# Include directories
include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Add source files
add_executable(nes_emu
    src/main.c
    src/nes.c
    src/memory.c
    src/loader.c
)

# Link SDL2
target_link_libraries(nes_emu ${SDL2_LIBRARIES}) 