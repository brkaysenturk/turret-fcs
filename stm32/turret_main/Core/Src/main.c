/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Turret FCS Main Controller Firmware
  *                   - TF-Luna LiDAR Reader (USART1)
  *                   - Full-Duplex Binary FCS Protocol (USART2)
  *                   - Pan/Tilt MG996R Servos (TIM3 / TIM1)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "luna_read.h"
#include "servo_driver.h"
#include "fcs_comm.h"
#include "flywheel_solenoid.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* MCU Configuration --------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Modulleri Baslat ---------------------------------------------------------*/
  Luna_Init(&huart1);
  FCS_Comm_Init(&huart2);
  Servo_Init();
  Launcher_Init(); // Pinleri guvenli PULLDOWN ve LOW (0V) durumuna kilitler

  uint32_t last_pc_cmd_time = HAL_GetTick();

  /* Infinite loop ------------------------------------------------------------*/
  uint32_t last_led_time = 0;

  while (1)
  {
    // 1. Sabit 100 Hz Periyodik Telemetri: LiDAR'dan bagimsiz periyodik gonderim
    // (LiDAR kopuk veya gecikmeli olsa bile servo geri bildirimi ve odometri asla kesilmez)
    uint16_t dist = Luna_GetDistance();
    uint16_t strength = Luna_GetStrength();
    FCS_Comm_SendTelemetry(dist, strength, Servo_GetPan(), Servo_GetTilt());

    // 2. PC'den yeni aci duzeltme komutu geldiginde motorlari sur (Atomik okuma ile veri yarisi onlendi)
    float d_pan = 0.0f;
    float d_tilt = 0.0f;
    uint8_t state = 0;
    if (FCS_Comm_GetCommand(&d_pan, &d_tilt, &state))
    {
      last_pc_cmd_time = HAL_GetTick();
      if (state == 3)
      {
        // Acil Durdurma: Motorlari dogrudan mekanik merkez pozisyonuna (90°, 90°) dondur
        Servo_SetPan(90.0f);
        Servo_SetTilt(90.0f);
        Launcher_EmergencyStop();
      }
      else
      {
        Servo_ApplyCorrection(d_pan, d_tilt);
      }

      // Firlatici baglandiginda acilabilir (Yukselen kenar ile tek darbe korumasi - cift atisi onler):
      // static uint8_t prev_fcs_state = 0;
      // if (state == 2 && prev_fcs_state != 2)
      // {
      //   Launcher_TriggerFire();
      // }
      // prev_fcs_state = state;
    }

    // 3. 300 ms Heartbeat Zaman Asimi: if/else disinda KOSULSUZ kontrol edilir
    // (PC 100 Hz hizinda komut gonderse bile zaman asimi guvenlik bekcisi her turda calisir)
    if (HAL_GetTick() - last_pc_cmd_time > 300)
    {
      Launcher_EmergencyStop();
    }

    // 4. Sistem Kalp Atisi LED Gostergesi (500 ms periyot - ISR yukunu onler)
    if (HAL_GetTick() - last_led_time >= 500)
    {
      last_led_time = HAL_GetTick();
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    // 5. Firlatici durum makinesi (Firlatici donanimi eklendiginde acilabilir)
    // Launcher_Update();

    HAL_Delay(10); // 100 Hz kontrol dongusu
  }
}

/**
  * @brief UART Kesme Callback (Tum alimlar modullere yonlendirilir)
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Luna_RxCallback(huart);
  }
  else if (huart->Instance == USART2)
  {
    FCS_Comm_RxCallback(huart);
  }
}

/**
  * @brief UART Hata Callback (ORE ve Gurultu kilidi cozumleyici)
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Luna_HandleError();
  }
  else if (huart->Instance == USART2)
  {
    FCS_Comm_HandleError();
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function (TF-Luna: 115200 8N1)
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function (PC VCP: 115200 8N1)
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  // Donanim Guvenligi: Solenoid ve Volan pinlerini aninda sifirla (bobin ve MOSFET korumasi)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
  while (1)
  {
  }
}

