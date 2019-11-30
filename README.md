project for testing a lot of things with STM32F030:

1. PlatformIO STM32 platform without framework (realy with CubeMX framework but PIO have outdated STM32F0 HAL/LL/CMSIS)
2. CubeMX for initialization code generation
3. STM32 UART serial communication with interrupts (single direction, only output)
4. STM32 IC2 slave with interrupts, sequential streams and restarts
5. Black magic probe (on BluePill) for uploading code/debuging/serial communication

I2C master for testing I2C implementation in STM32F030 with Arduino can be found [here](https://github.com/devegied/i2c_test_ArduinoMaster)

for upload/debug/serial communication one needs to ajust Black Magic Probe serial ports in `platformio.ini` file (options upload_port, debug_port and monitor_port) and for quick reset task in `.vscode/tasks.json` file