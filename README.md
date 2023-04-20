# ens001-sdk
早期接触到 NanoChip 的 ENS001 芯片， 官方 SDK 比较乱，有很多测试芯片代码。影响用户使用。
所以准备写一个SDK

### TODO List

- [x] Build System
    - [x] Makefile
- [x] Flash
    - [x] Jlink
- [ ] Debug
    - [ ] Jlink GDB
- [ ] Peripherals
    - [ ] GPIO
    - [ ] UART
    - [ ] Waveform
- [ ] RTOS
    - [ ] FreeRTOS
- [ ] Bootloader

#### Build System

- Setup
  - 下载解压 arm-none-eabi-gcc 工具链 `gcc-arm-none-eabi-10.3-2021.07`
  - 配置gcc
    - 在bashrc或者zshrc中添加  `export PATH=~/opt/gcc-arm-none-eabi-10.3-2021.07/bin:"$PATH"`
    - 或者在makefile中添加路径 `CROSS_COMPILE_PATH ?= ~/opt/gcc-arm-none-eabi-10.3-2021.07/bin/`
- Build： `make`

#### Flash
ENS001是一颗新芯片，不再Jlink默认支持清单中。因此需要我们手动添加到清单中。
详细方法见《如何给Jlink添加新芯片》

烧录： `make flash`