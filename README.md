# NES Emulator

A Nintendo Entertainment System (NES) emulator written in C using SDL2 for display and input handling.

## Project Structure

```
.
├── CMakeLists.txt    # CMake build configuration
├── src/
│   ├── main.c       # Main entry point
│   ├── nes.h        # Core NES definitions and structures
│   ├── nes.c        # Core NES implementation
│   ├── memory.h     # Memory management interface
│   ├── memory.c     # Memory management implementation
│   ├── loader.h     # ROM loader interface
│   └── loader.c     # ROM loader implementation (iNES format)
```

## Dependencies

- CMake 3.10 or higher
- SDL2 development libraries
- C compiler with C11 support

## Building

### Windows with MSYS2/MinGW

1. Install MSYS2 from https://www.msys2.org/
2. Open MSYS2 MINGW64 terminal and install dependencies:
```bash
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-SDL2
```

3. Build the project:
```bash
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make
```

### Linux

1. Install dependencies:
```bash
# Ubuntu/Debian
sudo apt-get install cmake gcc libsdl2-dev

# Fedora
sudo dnf install cmake gcc SDL2-devel

# Arch Linux
sudo pacman -S cmake gcc sdl2
```

2. Build the project:
```bash
mkdir build
cd build
cmake ..
make
```

## Usage

```bash
./nes_emu <rom_file>
```

## Current Features

- Basic project structure
- iNES format ROM loading
- Memory management system
- SDL2 window and rendering setup

## TODO

- [ ] CPU implementation
- [ ] PPU implementation
- [ ] APU implementation
- [ ] Controller input
- [ ] Mapper support
- [ ] Save states
- [ ] Audio output

## Contributing

Feel free to contribute to this project by submitting issues or pull requests.

## License

This project is open source and available under the MIT License.
