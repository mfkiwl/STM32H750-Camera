/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "quadspi.h"
#include "sdmmc.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "H750 ILI9488.h"
#include "ov2640.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* DMA로 이미지(FrameBuffer)를 LCD로 보낼 때는 480x320=153,600으로 64K가 넘으므로
 * 한 번에 보낼 수 없다.  따라서 DCMI에서 한 프레임이 전달되었을 때 보낼 수 있는
 * 480x136(=65,280)만큼 2번을 보내고 나머지 480x48(=23,040)을 보낸다.
 * 즉, 전체를 3부분으로 나누어 보내기로 한다.
 */

uint16_t FrameBuffer[480 * 320];	// 카메라에서 보내온 영상이 담길 프레임 버퍼
uint32_t RemaindImage1 = (uint32_t)FrameBuffer + (480 * 136) * 2; //  2번째 보낼 선두 위치. RGB565 2Byte이므로 곱하기 2
uint32_t RemaindImage2 = (uint32_t)FrameBuffer + (480 * 136 * 2) * 2;	// 3번째 보낼 선두 위치
uint32_t Remainder = 0;						// 3번 나누어 보내는 영상의 어느 부분인지 표시
uint32_t CameraInit = 0;					// 카메라가 초기화 되었슴.
uint32_t UpdateCameraFrame = 0;		// 카메라에서 한 프레임을 보내 왔슴.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// LCD로 보낼 DMA의 완료 콜백함수로 등록할 것이다.
static void TransferComplete(DMA_HandleTypeDef *hdma_memtomem_dma1_stream0);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback(void)
{
	static int cnt = -1000;

	cnt++;
	if(cnt > 500)
	{
		cnt = 0;

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);		// 메인보드의 LED를 점멸하여 동작중임을 표시하자.
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
  MX_DMA_Init();
  MX_QUADSPI_Init();
  MX_SDMMC1_SD_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DCMI_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();

  volatile uint32_t id = LCD_Read_ID();
  (void)id;		// 컴파일러 경고를 없애기 위해

  LCD_Direction(1);
  LCD_Clear(RED);
  HAL_Delay(500);
  LCD_Clear(GREEN);
  HAL_Delay(500);
  LCD_Clear(BLUE);
  HAL_Delay(500);


  volatile uint32_t idc = ov2640_ReadID(CAMERA_I2C_ADDRESS);
  (void)idc;

//  ov2640_Init(CAMERA_I2C_ADDRESS, CAMERA_R480x272);
  ov2640_Init(CAMERA_I2C_ADDRESS, CAMERA_R480x320);
//  ov2640_Config(CAMERA_I2C_ADDRESS, uint32_t feature, uint32_t value, uint32_t brightness_value)

  CameraInit = 1;
	UpdateCameraFrame = 0;

	// DMA로 이미지를 LCD에 전송할 때, 3개로 나눈 다음 부분을 보내기 위해 CPTL 콜백함수 등록
  HAL_DMA_RegisterCallback(&hdma_memtomem_dma1_stream0, HAL_DMA_XFER_CPLT_CB_ID, TransferComplete);

  // 디지탈 카메라 전송 시작. DCMI의 DMA Mode를 Circula로 하여 계속 이미지를 읽어오게 하였다.
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)(&FrameBuffer), (480 * 320) / 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  	if(UpdateCameraFrame)		// 디지탈 카메라로부터 1프레임 들어왔슴.
  	{
  		UpdateCameraFrame = 0;

  		//LCD_puts_image_RGB565((const uint16_t *)FrameBuffer);				// 그냥 전송
  		LCD_puts_image_RGB565_DMA((const uint16_t *)FrameBuffer);			// DMA 전송
  		Remainder = 2;	// 첫번째 부분을 LCD로 보냈음을 표시. 나머지는 앞서 등록한 콜백함수에서 처리할 것임.
  	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_QSPI
                              |RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 30;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 10;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLL1QCLK, RCC_MCODIV_2);
  /** Enable USB Voltage detector 
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/* USER CODE BEGIN 4 */

// 카메라에서 한 프레임이 모두 전송되었을 때 불려짐
// 여기서 바로 LCD로의 DMA를 시작해도 되지만, 나중에 영상처리 등의 작업등을 생각해서 플래그만 세우고
// 메인루프에서 처리하도록 하자.
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
  UpdateCameraFrame = 1;
  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

// 카메라에서 VSYNC 이벤트가 발생했슴
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
}

// 카메라에서 한 라인이 들어왔슴
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
}

// 프레임 버퍼에서 DMA로 LCD로 전송이 완료되었슴.
// 데이터 수가 480x320=153,600, 즉 64K를 넘으므로 3번에 나누어서 전송해야 함.
// Remainder = 2 일 때가 첫번째 전송 완료이고, Remainder = 1 일 때가 두번째 전송 완료, Remainder = 0 이면 전송 끝.
static void TransferComplete(DMA_HandleTypeDef *hdma_memtomem_dma1_stream0)
{
 	if(Remainder == 2) 			LCD_puts_image_RGB565_DMA_REMAINDER(RemaindImage1, 480 * 136);		// 136 + 136 + 48 = 320
 	else if(Remainder == 1) LCD_puts_image_RGB565_DMA_REMAINDER(RemaindImage2, 480 * 48);
 	Remainder--;
}

// 버튼 스위치가 눌렸슴
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case SW_CENTER_Pin:
			break;

		case SW_UP_Pin:
			break;

		case SW_DOWN_Pin:
			break;

		case SW_LEFT_Pin:
			break;

		case SW_RIGHT_Pin:
			break;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
