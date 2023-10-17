/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#include "basicApp.h"
#include "24lc01Eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _1_SECOND 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/***************************************************************************************************/
// for led blink
uint32_t period_of_led_blink = _1_SECOND;
uint32_t last_time = 0;
/***************************************************************************************************/


/***************************************************************************************************/
// for Uart Rx Irq
char    uartRxData[64];		                     //Receive Data Buffer
uint8_t uartRxIntData[64];	                     //Receive Data Interrupt Buffer (temp)
volatile bool isDataReceived = false;		 //Callback tamamlandi mi?, kontrol flag
/***************************************************************************************************/


/***************************************************************************************************/
// for CanA comm
FDCAN_TxHeaderTypeDef bufTxHdr_A;
FDCAN_RxHeaderTypeDef bufRxHdr_A;
uint8_t bufRx_A[64];

// for CanB comm
FDCAN_TxHeaderTypeDef bufTxHdr_B;
FDCAN_RxHeaderTypeDef bufRxHdr_B;
uint8_t bufRx_B[64];
/***************************************************************************************************/



/***************************************************************************************************/
extern Can_Route_Values_t routeOne;
extern Can_Route_Values_t routeTwo;
/***************************************************************************************************/



/***************************************************************************************************/
torkCanMsg tempCanMsg_A;    //canA Rx den alınan mesajları Ringbuffera atmak icin kullanılır
torkCanMsg tempCanMsg_B;    //canB Rx den alınan mesajları Ringbuffera atmak icin kullanılır

torkCanMsg canMsg;

uint8_t result_One = 0;
uint8_t result_Two = 0;
/***************************************************************************************************/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
/* USER CODE BEGIN PFP */
void isPressedBtn();
void heartBeat();
void isDataReceivedFromUart();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t *data = "tork robotik\n";
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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  /***************************************************************************************************/
	debugPrint("/* USER CODE BEGIN 2 */\n");
	Init_Basic_App();

	HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartRxIntData, sizeof(uartRxIntData));   //...test icin

	debugPrint("/* USER CODE  END  2 */\n");
  /***************************************************************************************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
     /**
     * rota1
     * yön canA -> canB
     ****************************************************************************************************/
		result_One = canMsgRingBufferPop(&routeOne.Route_Ring_Buf ,&canMsg);

		while(result_One == CAN_OK)
		{
			debugPrint("TX, route1-> canB->\n");

			//... gönderilmeden mesjlar değişim geçirme fonk. eklenecek

			bufTxHdr_B.Identifier          = canMsg.Identifier;
			bufTxHdr_B.IdType              = canMsg.IdType;
			bufTxHdr_B.TxFrameType         = canMsg.FrameType;
			bufTxHdr_B.DataLength          = canMsg.DataLength;
			bufTxHdr_B.FDFormat            = canMsg.FDFormat;
			bufTxHdr_B.BitRateSwitch       = canMsg.BitRateSwitch;
			bufTxHdr_B.ErrorStateIndicator = canMsg.ErrorStateIndicator;

			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &bufTxHdr_B, canMsg.Payload);  //canB den gönder

			result_One = canMsgRingBufferPop(&routeOne.Route_Ring_Buf ,&canMsg);
		}

     /***************************************************************************************************/


     /**
     * rota2
     * yön canB -> canA
     ****************************************************************************************************/
		result_Two = canMsgRingBufferPop(&routeTwo.Route_Ring_Buf ,&canMsg);

		while(result_Two == CAN_OK)
		{
			debugPrint("TX, route2-> canA-> \n");
			//... gönderilmeden mesjlar değişim geçirme fonk. eklenecek

			bufTxHdr_B.Identifier          = canMsg.Identifier;
			bufTxHdr_B.IdType              = canMsg.IdType;
			bufTxHdr_B.TxFrameType         = canMsg.FrameType;
			bufTxHdr_B.DataLength          = canMsg.DataLength;
			bufTxHdr_B.FDFormat            = canMsg.FDFormat;
			bufTxHdr_B.BitRateSwitch       = canMsg.BitRateSwitch;
			bufTxHdr_B.ErrorStateIndicator = canMsg.ErrorStateIndicator;

			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &bufTxHdr_B, canMsg.Payload);  //canA den gönder

			result_Two = canMsgRingBufferPop(&routeTwo.Route_Ring_Buf ,&canMsg);
		}
     /***************************************************************************************************/


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		isPressedBtn();
		heartBeat();
		isDataReceivedFromUart();
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 28;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 27;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 3;
  hfdcan2.Init.NominalTimeSeg1 = 28;
  hfdcan2.Init.NominalTimeSeg2 = 3;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 4;
  hfdcan2.Init.DataTimeSeg1 = 27;
  hfdcan2.Init.DataTimeSeg2 = 4;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

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
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = 0;
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
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_A_Pin|LED_B_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_A_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_A_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLINK_Pin */
  GPIO_InitStruct.Pin = LED_BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLINK_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * function   :  isPressedBtn1(): switch 2 basılmasını algılar
 * 				 Butona basmayı algılaması icin loop icerisinde sürekli cagrılmalıdır!
 *
 * parameters :  void
 *
 * return     :  void
 **/
void isPressedBtn()
{
	static int msCount = 0;

	if (!HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin))
	{
		msCount++;
		if (msCount > 1e3)
		{

			//***********************************
			// do something
			//***********************************
			debugPrint("btn\n");


			//***********************************

			msCount = -3e6;
		}
	}
	else
	{
		msCount = 0;
	}
}

/**
 * function   :   heartBeat():  cihazın calisma durumunu gösterir
 * 				  				1 sn  aralıklarla yanması      NORMAL
 * 				  				200ms aralıklarla yanması      ARIZA  //... farklı arıza durumlarında farklı period koy
 * 				  				loop icerisinde cagrılmalıdır. "period_of_led_blink" degiskeni ile frekansı belirlenir.
 * parameters :   void
 * return     :   void
 */
void heartBeat()
{
	if (HAL_GetTick() - last_time > period_of_led_blink)
	{
		last_time = HAL_GetTick();
		HAL_GPIO_TogglePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin);
	}
}

/**
 * CAN_A 'a gelen mesajlarda hfdcan1'ın FIFO0'ına konur  line0 kesmesi devreye girer
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &bufRxHdr_A, tempCanMsg_A.Payload) == HAL_OK)
		{
			//dbgPrint("canA new RxFifo0 :"); dbgDumpHex(payloadRx_A, 8); dbgPrint("\n");
			//dbgPrintf("RxHeader.Identifier: %x\n", bufRxHdr_A.Identifier);

			HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin,GPIO_PIN_SET);    //veri aldıgında ısıgı söndür

			tempCanMsg_A.Identifier          = bufRxHdr_A.Identifier;
			tempCanMsg_A.IdType              = bufRxHdr_A.IdType;
			tempCanMsg_A.FrameType           = bufRxHdr_A.RxFrameType;
			tempCanMsg_A.DataLength          = bufRxHdr_A.DataLength;
			tempCanMsg_A.FDFormat            = bufRxHdr_A.FDFormat;
		    tempCanMsg_A.ErrorStateIndicator = bufRxHdr_A.ErrorStateIndicator;
			tempCanMsg_A.BitRateSwitch       = bufRxHdr_A.BitRateSwitch;

			canMsgRingBufferPush(&routeOne.Route_Ring_Buf, tempCanMsg_A);

			debugPrint("RX ->canA  ->route1\n");
		}
		//dbgPrint("ERROR: Fifo0Callback HAL_FDCAN_GetRxMessage\n");
		//Error_Handler();
	}
}

/**
 *  CAN_B 'a gelen mesajlarda hfdcan2'ın FIFO1'ına konur  line1 kesmesi devreye girer
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &bufRxHdr_B, tempCanMsg_B.Payload) == HAL_OK)
		{
			//dbgPrint("canB new RxFifo1 :"); dbgDumpHex(tempCanMsg_B.Payload, 8); dbgPrint("\n");
			//dbgPrintf("RxHeader.Identifier: %x\n", bufRxHdr_B.Identifier);

			HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin,GPIO_PIN_SET);    //veri aldıgında ısıgı söndür

			tempCanMsg_B.Identifier          = bufRxHdr_B.Identifier;
			tempCanMsg_B.IdType              = bufRxHdr_B.IdType;
			tempCanMsg_B.FrameType           = bufRxHdr_B.RxFrameType;
			tempCanMsg_B.DataLength          = bufRxHdr_B.DataLength;
			tempCanMsg_B.FDFormat            = bufRxHdr_B.FDFormat;
			tempCanMsg_B.ErrorStateIndicator = bufRxHdr_B.ErrorStateIndicator;
			tempCanMsg_B.BitRateSwitch       = bufRxHdr_B.BitRateSwitch;

			canMsgRingBufferPush(&routeTwo.Route_Ring_Buf, tempCanMsg_B);

			debugPrint("RX, ->canB  ->route2\n");
		}
		//dbgPrint("ERROR: Fifo0Callback HAL_FDCAN_GetRxMessage\n");
		//Error_Handler();
	}
}



/***********************************************************
 *  Name        :HAL_UARTEx_RxEventCallback
 *  Parameters  :@huart, UART_HandleTypeDef
 *  				@Size,	Buffer receive size
 *  Returns     :@void Interrupt Callback fonksiyonu.
 *  Function    :HAL_UARTEx_ReceiveToIdle_IT sayesinde ToIdle_IT fonksiyonu sayesinde bilinmeyen RxData
 *  				frame leri uzerinden data aktarimi yapilabilmektedir.
 *--------------------------------------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, uartRxIntData, sizeof(uartRxIntData));   //interrupt kurulumu

	memcpy(uartRxData, uartRxIntData, sizeof(uartRxData));			   //RxIntData -> RxData kopyalanmasini sagliyor

	memset(uartRxIntData, 0, sizeof(uartRxIntData));

	//memset(TxData,0,TX_DATA_SIZE);				     	   //memset ile TxData ve RxData sifirlanmasini sagliyor

	isDataReceived = true;								   //Data geldi comm. basarili flag aktif hale geldi
}

/**
 * test icin olusturuldu
 */
void isDataReceivedFromUart()
{
	if(isDataReceived)
	{
		isDataReceived = 0;

		if (!strncmp("classic_can", uartRxData, 3))
		{
			debugPrint("classic_can mesaji gonderiliyor...");
		}
		else if(!strncmp("_canfd", uartRxData, 6))
		{
			debugPrint("_canfd mesaji gonderiliyor...");
		}
		else if(!strncmp("canfd_brs", uartRxData, 9))
		{
			debugPrint("canfd_brs mesaji gonderiliyor...");
		}

		memset(uartRxData,0,sizeof(uartRxData));
	  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
		static long count = 0;
		count++;
		while (count == 1e5)
		{
			HAL_GPIO_TogglePin(LED_BLINK_GPIO_Port, LED_BLINK_Pin);
			HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
			HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

			count = 0;
		}

		static long count2 = 0;
		count2++;
		while (count2 == 1e6)
		{
			HAL_NVIC_SystemReset();
		}
	}
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
