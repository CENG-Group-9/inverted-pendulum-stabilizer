# inverted-pendulum-stabilizer
Project of Group-9 for Gebze Technical University CSE396 class which aims to balance a rod while it is upright, from the initial state looking downwards using RL and LQR approaches selectable from a PC GUI control panel.  

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
Run `conda env create -f environment.yml` to create a conda environment with the required pacakges installed. In your system.


