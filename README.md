# inverted-pendulum-stabilizer
Project of Group-9 for Gebze Technical University CSE396 class which aims to balance a rod while it is upright, from the initial state looking downwards using RL and LQR approaches selectable from a PC GUI control panel.  

## Project layout
- `src/firmware/` — control loop on the microcontroller (separate team, own toolchain)
- `src/gui/` — Qt6 desktop app, split into modules:
  - `core/` — data, units, parsing, interfaces (`pendulum_gui_core`, testable without a display)
  - `app/` — executable, wires sources to views
  - `twin/`, `plots/`, `control_panel/` — view modules (to be added; they may depend on `core/` only)
- `src/models/` — `environment.yml` plus the `rl/` and `lqr/` model work

## Setup for development and build
### GUI
For gui development, Qt6 libraries and vulkan headers must be installed on your system
#### Arch
`sudo pacman -S base-devel cmake vulkan-headers qt6-base qt6-serialport qt6-3d qt6-declarative`

#### Ubuntu
`sudo apt-get update && sudo apt-get install -y cmake libvulkan-dev qt6-base-dev libqt6serialport6-dev qt6-3d-dev qt6-declarative-dev`

#### Windows
1. Download the Qt Online Installer from the official Qt website (Open Source version).
2. Run the installer and select the latest Qt 6.x version.
3. Ensure you check the boxes for Desktop C++ (MSVC or MinGW for Windows) and Qt Serial Port.
4. Add the Qt installation path to your system's Environment Variables so CMake can find it.

### Models
Run `conda env create -f src/models/environment.yml` to create the conda environment.

### Firmware

## Building
Every part is an independent project with its own build. There is no top-level
CMake file on purpose: the firmware, the GUI and the models never share compiled
code or toolchains.

### GUI
Needs Ninja, because the presets use it.
```bash
cd src/gui
cmake --preset debug          # configure into src/gui/build/debug
cmake --build --preset debug  # build
ctest --preset debug          # run unit tests
```
`src/gui/build.sh` and `src/gui/build.bat` do the first two steps for you.
Without presets: `cmake -S . -B build && cmake --build build`.

### Models
```bash
conda env create -f src/models/environment.yml
conda activate inverted-pendulum
cd src/models && pytest
```

### Firmware




