project for testing a lot of things with STM32F030:

1. PlatformIO STM32 platform without framework (realy with CubeMX framework but PIO have outdated STM32F0 HAL/LL/CMSIS)
2. CubeMX for initialization code generation
3. STM32 UART serial communication with interrupts (single direction, only output)
4. STM32 IC2 slave with interrupts, sequential streams and restarts
5. Black magic probe (on BluePill) for uploading code/debuging/serial communication

I2C master for testing I2C implementation in STM32F030 with Arduino can be found [here](https://github.com/devegied/i2c_test_ArduinoMaster)

for upload/debug/serial communication one needs to ajust Black Magic Probe serial ports in `platformio.ini` file (options upload_port, debug_port and monitor_port) and for quick reset task in `.vscode/tasks.json` file

###### Used STM32 HAL API

```
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);

void HAL_Delay(uint32_t Delay);

void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

__HAL_UART_DISABLE_IT(__HANDLE__, __INTERRUPT__)
__HAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)
__HAL_UART_GET_FLAG(__HANDLE__, __FLAG__)
__HAL_UART_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__)
```