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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "luna_read.h"
#include "servo_driver.h"
#include "fcs_comm.h"
#include "flywheel_solenoid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Fiziksel ates yolu guvenlik nedeniyle varsayilan olarak KAPALI.
 * Laboratuvar/donanim testinde acmak icin 1 yapin.
 */
#ifndef ENABLE_PHYSICAL_FIRE
#define ENABLE_PHYSICAL_FIRE 0
#endif

/*
 * Zaman Asimlari (ms)
 * - 300 ms: PC komut akisi kesildi -> firlaticiyi fail-safe durdur.
 * - 2000 ms: PC kalici olarak yok -> servolari yumusakca merkeze al.
 *            Aksi halde iki MG996R son konumda surekli stall akimi ceker.
 */
#define FCS_CMD_TIMEOUT_MS    300u
#define SERVO_PARK_TIMEOUT_MS 2000u

/*
 * Bagimsiz Watchdog (IWDG)
 * LSI ~32 kHz, prescaler /64 -> 500 Hz (2 ms/sayac), reload 400 -> 800 ms.
 * LSI toleransi (~17-47 kHz) nedeniyle gercek pencere ~540-1900 ms arasinda
 * degisir; bu, ~11 ms'lik dongu periyoduna karsi bolca marj birakir.
 */
#define IWDG_PRESCALER        IWDG_PRESCALER_64
#define IWDG_RELOAD_COUNTS    400u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */
/*
 * B1 mavi buton (PC13/EXTI13) mandalli yazilim E-stop'u:
 * Her basista mandal terslenir. Mandal aktifken PC komutlari yok sayilir,
 * launcher kesilir ve servolar hiz sinirli sekilde merkeze cekilir.
 */
static volatile bool b1_estop_latched = false;
static volatile bool b1_press_event = false;
static uint32_t b1_last_edge_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* MCU Configuration --------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* Watchdog'u mumkun olan en erken anda baslat: sonraki herhangi bir
     kilitlenme (saat yapilandirmasi dahil) artik reset ile kurtarilir. */
  MX_IWDG_Init();
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /* Modulleri Baslat ---------------------------------------------------------*/
  Luna_Init(&huart1);
  FCS_Comm_Init(&huart2);
  Servo_Init();
  Launcher_Init(); // Pinleri guvenli PULLDOWN ve LOW (0V) durumuna kilitler

  uint32_t last_pc_cmd_time = HAL_GetTick();
  bool servos_park = false;  // Yumusak merkez donusu (E-stop veya link kaybi)
#if ENABLE_PHYSICAL_FIRE
  static uint8_t prev_fcs_state = PROTO_STATE_TRACKING;
#endif
  uint32_t last_led_time = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 0. Watchdog beslemesi: dongu saglikliysa IWDG sayaci sifirlanir.
    //    Herhangi bir kilitlenmede IWDG reset atar ve pinler tekrar LOW'a cekilir.
    HAL_IWDG_Refresh(&hiwdg);

    // 1. Sabit ~100 Hz Periyodik Telemetri: LiDAR'dan bagimsiz periyodik gonderim
    // (LiDAR kopuk veya gecikmeli olsa bile servo geri bildirimi ve odometri asla kesilmez)
    uint16_t dist = Luna_GetDistance();
    uint16_t strength = Luna_GetStrength();
    FCS_Comm_SendTelemetry(dist, strength, Servo_GetPan(), Servo_GetTilt());

    // 2. B1 mavi buton: her basista E-stop mandalini degistir.
    //    Mandal aktifken PC komutlari yok sayilir; launcher kesilir ve servolar parka alinir.
    if (b1_press_event)
    {
      b1_press_event = false;
      b1_estop_latched = !b1_estop_latched;
      if (b1_estop_latched)
      {
        Launcher_EmergencyStop();
        servos_park = true;
#if ENABLE_PHYSICAL_FIRE
        prev_fcs_state = PROTO_STATE_TRACKING;
#endif
      }
      else
      {
        servos_park = false;
      }
    }
    if (b1_estop_latched)
    {
      Launcher_EmergencyStop();
      servos_park = true;
    }

    // 3. PC'den yeni aci duzeltme komutu geldiginde motorlari sur (Atomik okuma ile veri yarisi onlendi)
    float d_pan = 0.0f;
    float d_tilt = 0.0f;
    uint8_t state = 0;
    if (!b1_estop_latched && FCS_Comm_GetCommand(&d_pan, &d_tilt, &state))
    {
      last_pc_cmd_time = HAL_GetTick();
      if (state == PROTO_STATE_ESTOP)
      {
        /*
         * Acil Durdurma: firlaticiyi aninda kes, servolari ise BASAMAKSIZ
         * merkeze indir. Dogrudan Servo_SetPan(90) yazmak, MAX_STEP korumasini
         * atlayip servo uzerinde 45 dereceye varan ani tork sokuna yol aciyordu.
         * Artik merkeze inis Servo_HomeStep() ile hiz sinirli yapilir.
         */
        Launcher_EmergencyStop();
        servos_park = true;
#if ENABLE_PHYSICAL_FIRE
        prev_fcs_state = PROTO_STATE_TRACKING;  // E-stop sonrasi yanlis ates kenari olusmasin
#endif
      }
      else
      {
        servos_park = false;  // PC kontrolu geri aldi
        Servo_ApplyCorrection(d_pan, d_tilt);

#if ENABLE_PHYSICAL_FIRE
        /*
         * Ates tetigi, volan enable blogundan ONCE calisir. Aksi halde ayni
         * karede enerjilendirilen volan "hazir" sanilir ve dart devirsiz
         * firlatilirdi. Bu sira ile Launcher_TriggerFire() yalnizca volan
         * onceki karelerde (PC SPINUP fazinda) gercekten hazirsa ates eder.
         */
        if (state == PROTO_STATE_FIRING && prev_fcs_state != PROTO_STATE_FIRING)
        {
          (void)Launcher_TriggerFire();
        }
#endif

        /*
         * Flywheel pre-spin (bosta donme):
         * LOCKED (1) veya FIRING (2) durumunda volanlari enerjilendir.
         * PC, ates oncesi slew/settle boyunca state=1 gonderir; FIRING kenarinda
         * volanlar zaten hazir olur.
         * TRACKING (0) veya ESTOP (3) durumunda volanlari kapat.
         */
        if (state == PROTO_STATE_LOCKED || state == PROTO_STATE_FIRING)
        {
          if (!Launcher_IsFlywheelActive())
          {
            Launcher_Flywheel_Enable(true);
          }
        }
        else
        {
          if (!Launcher_IsFiring() && Launcher_IsFlywheelActive())
          {
            Launcher_Flywheel_Enable(false);
          }
        }
      }
#if ENABLE_PHYSICAL_FIRE
      prev_fcs_state = (state == PROTO_STATE_ESTOP) ? PROTO_STATE_TRACKING : state;
#endif
    }

    // 4. Heartbeat: PC komut akisi kesilirse firlaticiyi fail-safe durdur.
    if (HAL_GetTick() - last_pc_cmd_time > FCS_CMD_TIMEOUT_MS)
    {
      Launcher_EmergencyStop();
#if ENABLE_PHYSICAL_FIRE
      prev_fcs_state = PROTO_STATE_TRACKING;  // Yeniden baglanmada yanlis ates kenari olusmasin
#endif
    }

    // 5. Uzun sureli PC kaybi: servolari yumusakca merkeze al.
    //    Aksi halde iki MG996R son konumda surekli stall akimi cekip isinir.
    if (HAL_GetTick() - last_pc_cmd_time > SERVO_PARK_TIMEOUT_MS)
    {
      servos_park = true;
    }

    // 6. Yumusak merkez donusu (E-stop veya link kaybi): hiz sinirli, basamaksiz.
    if (servos_park)
    {
      (void)Servo_HomeStep();
    }

    // 7. Sistem Kalp Atisi LED Gostergesi (500 ms periyot - ISR yukunu onler)
    if (HAL_GetTick() - last_led_time >= 500)
    {
      last_led_time = HAL_GetTick();
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    // 8. Firlatici durum makinesi (non-blocking)
    Launcher_Update();

    HAL_Delay(10); // ~90-100 Hz kontrol dongusu (bloklayan telemetri TX dahil)
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
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
  * @brief EXTI Callback (B1 mavi buton: mandalli yazilim E-stop)
  *
  * Buton aktif-low ve dahili pull-up ile okunur. Mekanik sicramayi elemek icin
  * 250 ms debounce uygulanir; gercek islem ana dongude yapilir (ISR kisa kalir).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
    uint32_t now = HAL_GetTick();
    if (b1_last_edge_ms == 0u || (now - b1_last_edge_ms) >= 250u)
    {
      b1_last_edge_ms = now;
      b1_press_event = true;
    }
  }
}
/* USER CODE END 4 */

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
  * @brief IWDG Initialization Function (Bagimsiz Watchdog, ~800 ms)
  *
  * HAL_IWDG_Init sayaci baslatir; bu andan itibaren durdurulamaz, bu yuzden
  * ana dongude dongu basina HAL_IWDG_Refresh() ile beslenmelidir.
  */
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER;
  hiwdg.Init.Reload = IWDG_RELOAD_COUNTS;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

#if defined(DEBUG)
  /* Hata ayiklama kolayligi: cekirdek breakpoint'te durduruldugunda IWDG
     saymasin; aksi halde her adimda reset yenir. */
  __HAL_DBGMCU_FREEZE_IWDG();
#endif
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
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt Init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  // Donanim Guvenligi: Solenoid ve Volan pinlerini aninda sifirla (bobin ve MOSFET korumasi)
  HAL_GPIO_WritePin(FLYWHEEL_GPIO_PORT, FLYWHEEL_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_RESET);

  // Watchdog bilincli olarak BESLENMEZ: IWDG ~800 ms icinde reset atar,
  // boylece cihaz kalici olarak kilitli kalmaz; acilista pinler tekrar LOW'a cekilir.
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

