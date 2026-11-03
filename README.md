# inverted-pendulum-stabilizer
Project of Group-9 for Gebze Technical University CSE396 class which aims to balance a rod while it is upright, from the initial state looking downwards using RL and LQR approaches selectable from a PC GUI control panel.  

## Project layout
- `src/firmware/` — control loop on the microcontroller (separate team, own toolchain)
- `src/gui/` — Dear ImGui + ImPlot desktop app (C++20, GLFW + OpenGL 3):
  - `app/main.cpp` — the whole application for now (target `pendulum-gui`)
  - `libs/` — vendored Dear ImGui, ImPlot, glad and KHR headers, so no system
    packages are needed for them
- `src/models/` — `environment.yml` plus the `rl/` and `lqr/` model work

## Setup for development and build
### GUI
For GUI development only GLFW 3, OpenGL, CMake (>= 3.25), Ninja and a C/C++
compiler are required. Dear ImGui, ImPlot, glad and the KHR headers are vendored
inside `src/gui/libs/`, so no extra GUI toolkit or graphics API SDK has to be
installed.
#### Arch
`sudo pacman -S base-devel cmake ninja glfw`

#### Ubuntu
`sudo apt-get update && sudo apt-get install -y build-essential cmake ninja-build libglfw3-dev libgl1-mesa-dev`

#### Windows
Use MSYS2 with the MinGW64 environment:
`pacman -S --needed mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake mingw-w64-x86_64-ninja mingw-w64-x86_64-glfw`

Add `C:\msys64\mingw64\bin` to `PATH` before building and running, because the
executable looks for its DLLs (for example `glfw3.dll`) there.

### Models
Run `conda env create -f src/models/environment.yml` to create the conda environment.

### Firmware

## Building
Every part is an independent project with its own build. There is no top-level
CMake file on purpose: the firmware, the GUI and the models never share compiled
code or toolchains.

### GUI
Needs CMake (>= 3.25) and Ninja, because the presets use it.
```bash
cd src/gui
cmake --preset debug          # configure into src/gui/build/debug
cmake --build --preset debug  # build
```
The `release` preset does the same for an optimized build:
```bash
cd src/gui
cmake --preset release
cmake --build --preset release
```
`src/gui/build.sh` and `src/gui/build.bat` do the configure and build steps of
the `debug` preset for you.
Without presets: `cmake -S . -B build && cmake --build build`.

### Models
```bash
conda env create -f src/models/environment.yml
conda activate inverted-pendulum
cd src/models && pytest
```

### Firmware



