# 独立编译与 ST-LINK 烧录

适用设备：STM32F407VETx，512 KiB Flash，ST-LINK V2/SWD。无需 CubeIDE。所有命令均在 mdp21_8 目录执行。

## 已完成的验证（2026-10-07，新加坡时间）

使用 Arm GNU Toolchain 15.3.Rel1（GCC 15.3.1）与 OpenOCD 0.12.0，完成 68 个源文件的编译、链接及 ELF/BIN/HEX 生成。构建沿用 CubeIDE Debug 参数：Cortex-M4、FPv4-SP-D16、hard float ABI、O0、nano/ nosys specs，以及原 Flash 链接脚本。

通过 ST-LINK V2J46S7 读取 device id 0x101f6413、512 KiB Flash，供电约 3.21 V。首次烧录前已备份完整 Flash 到 `backups/pre-flash-20261007.bin`（524288 字节）。新固件写入、校验成功（Verified OK），并复位运行。随后读取 CFSR/HFSR 均为零，并恢复 CPU 运行。当前 ACTIVE_TEST 保持 ASSESSMENT_COMMAND_INTERFACE；未进行 UART 命令或运动功能测试。

固件大小：text 85336，data 140，bss 75668 字节。Flash 占用为 text+data，共 85476 字节。构建产生原有源码的两个警告（未使用变量、OLED 字符串指针符号差异），新链接器另报告 RWX LOAD 段；均未阻止构建。本次保留业务源码及原链接脚本。

## 工具安装

本机已完成安装。另一台 Apple Silicon Mac 可运行：

```sh
brew install open-ocd
./scripts/setup-toolchain.sh
```

编译器位于项目 `.tools/`，安装脚本从 Arm 官方下载并核验固定 SHA-256。Makefile 自动使用该编译器；已有其他完整 Arm 工具链时，可通过 `TOOLCHAIN_DIR=/path/to/bin` 指定。

## 一键编译烧录

在项目目录运行 `./flash.sh`，或从任意目录用绝对路径运行该脚本。脚本检查工具、编译、备份完整 Flash、写入并校验，最后复位运行；任一步失败即停止。每次备份使用时间戳及进程号命名，日志保存到 `build/`。首次使用缺少工具时，按上方安装步骤操作。

`./flash.sh --build-only` 只编译，不连接设备。可用 `JOBS=8 ./flash.sh` 指定并行编译数量；`TOOLCHAIN_DIR` 和 `OPENOCD` 环境变量可指定自定义工具路径。

## 日常操作

```sh
make -j8 build       # 生成 build/mdp21_8.elf、.bin、.hex
make probe          # 检查 ST-LINK 和目标芯片
make backup         # 按时间戳备份完整 Flash
make flash          # 编译、写入、校验、复位运行
make verify         # 单独校验当前 ELF，再复位运行
make clean          # 清理独立 build 目录
```

源文件清单在 `sources.mk`，与现有 CubeIDE Debug 配置一致。增加或删除源文件时同步维护清单。头文件变化会触发依赖重编译。独立产物放在 `build/`，原 `Debug/` 配置保持可用。`make flash` 会覆盖固件所占用的 Flash 扇区，烧录前可运行 `make backup`。

## 恢复首次烧录前的固件

确认需要恢复后，在本目录运行：

```sh
openocd -f openocd.cfg -c "program backups/pre-flash-20261007.bin 0x08000000 verify reset exit"
```

## 调试与连接故障

`make debug` 启动 OpenOCD，GDB 端口为 3333。在另一个终端运行 `.tools/arm-gnu-toolchain-15.3.rel1-darwin-arm64-arm-none-eabi/bin/arm-none-eabi-gdb build/mdp21_8.elf`，再执行 `target extended-remote localhost:3333`。

连接失败时检查目标板供电、SWDIO/SWCLK/GND 接线、USB 连接，以及是否有其他调试程序占用 ST-LINK。当前配置使用 SWD、初始适配器频率 1 MHz；STM32F4 的 OpenOCD 复位事件会临时调整频率。若需要复位下连接，将 NRST 连接后在 `openocd.cfg` 添加 `reset_config srst_only srst_nogate connect_assert_srst`。

本次日志：`build/flash.log`、`build/runtime-check.log`；完整首次编译日志：`build-first.log`。`.tools/`、`build/`、`backups/` 已加入忽略规则。
