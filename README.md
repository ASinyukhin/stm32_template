# Template for starting development for stm32f1x.

Based on CMSIS, uses cmake+make bulding system.

## Requirements:
* Arm Toolchain: arm-none-eabi-gcc
* gdb-multiarch or arm-none-eabi-gdb debugger
* cmake >= 3.0
* make
* build-essential
* st-link utility for Linux
* git (recommended for clonning project)

## Usage:

Clone project via git or http download (not recommended).

Open terminal, cd to root project directory.

Run `cmake .` to generate Makefile, then run `make` or `make bin` or `make hex` depending on what format of firmware you want.

Flash firmware with command `make flash`.

To clean build artifacts run `make clean`. Also after modifying CMakeLists.txt you need to clean cmake cache via deleting CMakeCache.txt file (but NOT CMakeLists!)

## Usage in VScode:

Required VS extensions:
* CMake
* Cmake Tools
* C/C++ (IntelliSense, debugging, and code browsing)
* Cortex-Debug
* C/C++ Themes (optional)

Firstly, open working directory and setup toochain.

Visit https://code.visualstudio.com/docs/cpp/cmake-linux for details.

Setup variant: Debug or Release.

To build project run: "Terminal->Run Build task->Cmake: build"
Project can be rebuild with "Cmake: clean rebuild"

Also you can build artifacts (bin/hex/elf) from CMake extension tab.
