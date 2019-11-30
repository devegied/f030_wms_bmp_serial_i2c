/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
project for testing a lot of things:
1. Black magic probe:
1a. uploading code
1b. debuging
1c. serial communication
2. STM32 UART serial communication with interrupts (single direction, only output)
3. STM32 IC2 slave with interrupts and sequential streams
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  I2C_SLAVE_STOP        = 0x00U,
  I2C_SLAVE_WAIT        = 0x01U,
  I2C_SLAVE_WAIT4ADDR   = 0x02U,
  I2C_SLAVE_SENDSTAT    = 0x03U,
  I2C_SLAVE_WRITEREG    = 0x04U,
  I2C_SLAVE_READREG     = 0x05U,
  I2C_SLAVE_HAVEADDR    = 0x06U
} I2C_StatusTypeDef; // I2C communication protocol status
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t aTxBuffer[10] = "  10 char ";//buffer for UART output
uint8_t aRxBuffer[10]; //buffer to receive data from I2C bus
uint8_t i2cTxBuffer[10] = "  10 char "; //buffer to send data to I2C bus
uint8_t hexBuf[16] = "0123456789ABCDEF"; //buffer used for integer to readable hexadecimal conversion
uartTypeDef uart1; // structure variable holding UART handle and fifo for buffering output data
uint8_t uart1TxBuff[128]; //buffer for UART output data
FIFO txFifo; // fifo structure for UART output data
__IO I2C_StatusTypeDef i2cState = I2C_SLAVE_STOP; //I2C protocal state
__IO uint8_t rv = 22U; // dummy register value
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*"Slave transfer completed" interupt calback*/
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
  if(i2cState==I2C_SLAVE_READREG){//we are sending register values to the master
    uart_buffer("DR",2,&uart1);
    uart_buffer_byte(hexBuf[(rv&0xF0U)>>4],&uart1);
    uart_buffer_byte(hexBuf[rv&0x0FU],&uart1);
    i2cTxBuffer[0]=rv;
    if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2cTxBuffer, 1, I2C_FIRST_FRAME/*there will be no more restarts, but this is not the last frame*/) != HAL_OK) {
      uart_write("\nFATAL\n",7,&uart1);
      Error_Handler();
    }else{
      uart_write("\n",1,&uart1);
    }
    rv++;
  }else if(i2cState==I2C_SLAVE_SENDSTAT){
    uart_buffer("SR",2,&uart1);
    if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2cTxBuffer, 2, I2C_FIRST_FRAME/*there will be no more restarts, but this is not the last frame*/) != HAL_OK) {
      uart_write("\nFATAL\n",7,&uart1);
      Error_Handler();
    }else{
      uart_write("\n",1,&uart1);
    }
  }else{
    uart_buffer("TxUS",4,&uart1);
    uart_buffer_byte(hexBuf[(i2cState&0xF0U)>>4],&uart1);
    uart_buffer_byte(hexBuf[i2cState&0x0FU],&uart1);
    uart_write("\n",1,&uart1);
  }
}
/*"Slave receive completed" interupt calback*/
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
  if(i2cState==I2C_SLAVE_WAIT4ADDR){//first byte after address match, we got register number in aRxBuffer[0]
    uart_buffer("R",1,&uart1);
    uart_buffer_byte(hexBuf[(aRxBuffer[0]&0xF0U)>>4],&uart1);
    uart_buffer_byte(hexBuf[aRxBuffer[0]&0x0FU],&uart1);
    i2cState=I2C_SLAVE_HAVEADDR;//we have address, master will restart with read op or will send next byte for write op
    uart_write("\n",1,&uart1);
    if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, aRxBuffer+1, 1, I2C_FIRST_AND_NEXT_FRAME/*not finish automaticaly after frame and allow restart*/) != HAL_OK) {//lets wait for next byte if it will be write op
      uart_write("\nFATAL\n",7,&uart1);
      Error_Handler();
    }
  }else if(i2cState==I2C_SLAVE_HAVEADDR||i2cState==I2C_SLAVE_WRITEREG){//we already have register number and now we got first or next data byte
    uart_buffer("DW",1,&uart1);
    uart_buffer_byte(hexBuf[(aRxBuffer[1]&0xF0U)>>4],&uart1);
    uart_buffer_byte(hexBuf[aRxBuffer[1]&0x0FU],&uart1);
    i2cState=I2C_SLAVE_WRITEREG;
    uart_write("\n",1,&uart1);
    if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, aRxBuffer+1, 1, I2C_FIRST_FRAME/*not finish automaticaly after frame*/) != HAL_OK) {//lets try to get one more byte
      uart_write("\nFATAL\n",7,&uart1);
			Error_Handler();
    }
  }else{
    uart_buffer("RxUS",4,&uart1);
    uart_buffer_byte(hexBuf[(i2cState&0xF0U)>>4],&uart1);
    uart_buffer_byte(hexBuf[i2cState&0x0FU],&uart1);
    uart_write("\n",1,&uart1);
  }
}
/*"Slave address match and send/receive completed" interupt calback*/
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
  uart_buffer("EL",2,&uart1);
  uart_buffer_byte(hexBuf[(i2cState&0xF0U)>>4],&uart1);
  uart_buffer_byte(hexBuf[i2cState&0x0FU],&uart1);
  uart_write("\n",1,&uart1);
  if(HAL_I2C_EnableListen_IT(hi2c) != HAL_OK){// Restart listening
    uart_write("\nFATAL\n",7,&uart1);
    Error_Handler();
  }else{
    i2cState=I2C_SLAVE_WAIT;
  }
}
/*"I2C error" interupt calback*/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){
  if(i2cState==I2C_SLAVE_WAIT4ADDR && hi2c->ErrorCode==HAL_I2C_ERROR_AF){//we were in receive mode waiting for register and got NACK - master ended communication - typical situation for I2C bus scan
    uart_write("busScan\n",8,&uart1);
  }else if((i2cState==I2C_SLAVE_READREG||//we were sending bytes from register
            i2cState==I2C_SLAVE_WRITEREG||//we were geting bytes to register
            i2cState==I2C_SLAVE_SENDSTAT) //we were sending status bytes
            && 
            hi2c->ErrorCode==HAL_I2C_ERROR_AF){//got NACK - master ended communication. Two bytes that we sent should be lost in the bus
    uart_write("MasterNACK\n",11,&uart1);
  }else{ // other error
    uart_buffer_byte('E',&uart1);
    uart_buffer_byte((hi2c->ErrorCode&HAL_I2C_ERROR_BERR)?'B':'_',&uart1);//HAL_I2C_ERROR_BERR (0x0001U) - BERR: Bus error - START/STOP condition in wrong i2c protocol place
    uart_buffer_byte((hi2c->ErrorCode&HAL_I2C_ERROR_ARLO)?'A':'_',&uart1);//HAL_I2C_ERROR_ARLO (0x0002U) - ARLO: arbitration lost - slave got unexpected level from i2c bus
    uart_buffer_byte((hi2c->ErrorCode&HAL_I2C_ERROR_AF)?'N':'_',&uart1);//HAL_I2C_ERROR_AF   (0x0004U) - NACKF: masted did not acknowledge written data byte
    uart_buffer_byte((hi2c->ErrorCode&HAL_I2C_ERROR_OVR)?'O':'_',&uart1);//HAL_I2C_ERROR_OVR  (0x0008U) - OVR: Overrun/underrun, slave is too slow
    uart_buffer_byte((hi2c->ErrorCode&HAL_I2C_ERROR_SIZE)?'S':'_',&uart1);//HAL_I2C_ERROR_SIZE (0x0040U) - Size Management error
    uart_buffer_byte('/',&uart1);
    uart_buffer_byte(hexBuf[(i2cState&0xF0U)>>4],&uart1);
    uart_buffer_byte(hexBuf[i2cState&0x0FU],&uart1);
    uart_write("\n",1,&uart1);
  }
}
/*"I2C address match" interupt calback*/
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
  UNUSED(AddrMatchCode);//we have only one address so do not check it
  uart_buffer_byte('A',&uart1);
  if(i2cState==I2C_SLAVE_WAIT){//first transaction step
    if(TransferDirection==I2C_DIRECTION_TRANSMIT/*from Master point of view*/){//master wants to write address of register
      uart_buffer("WR",2,&uart1);
      i2cState=I2C_SLAVE_WAIT4ADDR;
      if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, aRxBuffer, 1, I2C_NEXT_FRAME/*allow restart after received byte*/) != HAL_OK) {//read the registernumber  byte
        uart_write("\nFATAL\n",7,&uart1);
			  Error_Handler();
      }else{
        uart_write("\n",1,&uart1);
      }
    }else{//master wants to read from the beggining - report status
      uart_buffer("RS",2,&uart1);
      i2cState=I2C_SLAVE_SENDSTAT;/*master reads status - we allways try to give all of what we have at the moment*/
      if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2cTxBuffer, 2, I2C_FIRST_FRAME/*there will be no restarts*/) != HAL_OK) {// send status bytes
			  uart_write("\nFATAL\n",7,&uart1);
        Error_Handler();
      }else{
        uart_write("\n",1,&uart1);
      }
    }
  }else{//not the first match in one transaction, so we got our address after restart - we should already have register address
    if(i2cState==I2C_SLAVE_HAVEADDR){
      if(TransferDirection==I2C_DIRECTION_TRANSMIT/*from Master point of view*/){//very strange, direction before restart was incoming, and after restart also incoming
        uart_buffer("RW",2,&uart1);
        i2cState=I2C_SLAVE_WRITEREG;
        if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, aRxBuffer+1, 1, I2C_FIRST_FRAME/*allow more incoming bytes after this one*/) != HAL_OK) {//lets try to get byte from master
			    uart_write("\nFATAL\n",7,&uart1);
          Error_Handler();
        }else{
          uart_write("\n",1,&uart1);
        }
      }else{//direction change after restart, we sending register data to the master
        uart_buffer("DR",2,&uart1);
        uart_buffer_byte(hexBuf[(rv&0xF0U)>>4],&uart1);
        uart_buffer_byte(hexBuf[rv&0x0FU],&uart1);
        i2cState=I2C_SLAVE_READREG;
        i2cTxBuffer[0]=rv;
        if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2cTxBuffer, 1, I2C_FIRST_FRAME/*no more restarts, but we can send  more bytes after this one*/) != HAL_OK) {
          uart_write("\nFATAL\n",7,&uart1);
          Error_Handler();
        }else{
          uart_write("\n",1,&uart1);
        }
        rv++;
      }
    }else{//unknown situation
      uart_buffer("ARUS",4,&uart1);
      uart_buffer_byte(hexBuf[(i2cState&0xF0U)>>4],&uart1);
      uart_buffer_byte(hexBuf[i2cState&0x0FU],&uart1);
      uart_write("\n",1,&uart1);
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //add ring buffer to the UART
  uart1.huart = &huart1;
  uart1.tx = &txFifo;
	fifo_init(uart1.tx,128,uart1TxBuff);
  
  uart_write("\nProject f030_wms_bmp_serial_i2c is ongoing\n", 44,&uart1);
  // listen for address match on I2C bus
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK){
    Error_Handler();
  }
  i2cState=I2C_SLAVE_WAIT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(333);
    HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30A02E3E;
  hi2c1.Init.OwnAddress1 = 102;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : RED_LED_Pin BLUE_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  //indicate error by turing red led on
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
