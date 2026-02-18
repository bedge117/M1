<!-- See COPYING.txt for license details. -->

# M1 Build Tool

The M1 firmware can be built with STM32CubeIDE or Visual Studio Code.

## Installation  

Clone the repository. Install the required software and extensions:

* For STM32CubeIDE users:

- The code has been built with STM32CubeIDE 1.17.0.
Download: https://www.st.com/en/development-tools/stm32cubeide.html (STM32CubeIDE 1.17.0, GNU Tools for STM32 12.3.rel1),
and https://www.st.com/en/development-tools/stm32cubeprog.html.

* For Visual Studio Code users:

- The code has been built with arm-none-eabi-gcc 14.2.Rel1.
Download: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads (14.2.Rel1)

* For Linux users:

- Install ARM GCC toolchain (e.g., `arm-none-eabi-gcc`) and Ninja
- On Debian/Ubuntu: `sudo apt install gcc-arm-none-eabi ninja-build`

- VSCode download: https://code.visualstudio.com/download

Extensions installation:
+ CMake Tools
+ Cortex-Debug
+ Ninja (follow steps below)

- Chocolatey (package manager) installation:
Download: https://sourceforge.net/projects/chocolatey.mirror/

Prerequisites:
Before running the installation command, ensure that you have Chocolatey itself installed. You can find official installation instructions on the Chocolatey website. 

Installation Command:
Open an elevated Command Prompt or PowerShell session (right-click and select "Run as administrator") and execute the following command: 
powershell
choco install ninja

Verification:
After the installation is complete, you can verify that Ninja is installed correctly and added to your system's PATH environment variable by opening a new terminal window and running: 
powershell
ninja --version

## Build

* For Linux users:
```bash
make
```
Output: `./artifacts/` (MonstaTek_M1_v0800.elf, .bin, .hex)

* For Visual Studio Code users:
- Select build type from PROJECT STATUS - Configure (gcc-14_2_build-release, gcc-14_2_build-debug).
- Click the Build icon.

* For STM32CubeIDE users:
Build in the IDE.

## Build directories

* For Linux users:
Firmware is built in the folder `./build/` with output copied to `./artifacts/`.

* For Visual Studio Code users:
Firmware is built in the folder ./out/build/gcc-14_2_build-release for release configuration and ./out/build/gcc-14_2_build-debug for debug configuration.

* For STM32CubeIDE users:
Firmware is built in the folder ./Release for release configuration and ./Debug for debug configuration.

## Debug

* For Visual Studio Code users:
- Select debugger from RUN AND DEBUG (Debug with ST-Link, Debug with J-Link).
- Debugger setting can be changed in ./.vscode/launch.json.
- J-Link software download: https://www.segger.com/downloads/jlink/

* For STM32CubeIDE users:
Debug in the IDE.