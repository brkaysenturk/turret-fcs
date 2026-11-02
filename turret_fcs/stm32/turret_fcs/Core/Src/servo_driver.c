/*
 * ==============================================================================
 * MODUL: PAN/TILT SERVO SURUCUSU (MG996R, TIM3 + TIM1)
 * ==============================================================================
 * PB4/PB5 (TIM3_CH1/CH2) uzerinden 50 Hz PWM uretir; PA8 (TIM1_CH1) pan
 * darbesinin D7 aynasidir (Servo_SetPan her zaman iki kanali ayni darbeyle surer).
 * 0..180 derece, 570..2470 us darbe araligina eslenir.
 *
 * GUVENLIK:
 * - Pan 15..165 derece, tilt 65..115 derece araligina kilitlidir.
 * - Her pakette adim siniri (MAX_STEP_DEG) ve zaman tabanli 45 derece/s
 *   hiz siniri birlikte uygulanir; PC FPS'i artarsa taret hizlanmaz.
 * - E-stop ve link kaybinda Servo_HomeStep() ile basamaksiz merkeze donulur.
 * ==============================================================================
 */
#include "servo_driver.h"

static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim1;

static float current_pan = 90.0f;
static float current_tilt = 90.0f;

// Zaman tabanli hiz siniri icin son komut ani
static uint32_t last_cmd_ms = 0;
static bool have_last_cmd = false;

static uint32_t AngleToPulse(float angle)
{
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    return (uint32_t)(SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180.0f));
}

void Servo_Init(void)
{
    // 1. TIM3: PB4 (D5) - PAN, PB5 (D4) - TILT
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83; // 84 MHz / (83 + 1) = 1 MHz (1 us tick)
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 19999; // 20 ms -> 50 Hz PWM
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = SERVO_MID_PULSE; // 90 derece merkez
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1); // D5
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2); // D4

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // 2. TIM1: PA8 (D7) - PAN darbesinin aynasi (Servo_SetPan ikisini birlikte surer)
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 83;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 19999;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);

    TIM_BreakDeadTimeConfigTypeDef bdt = {0};
    bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt);

    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    Servo_SetPan(90.0f);
    Servo_SetTilt(90.0f);
}

void Servo_SetPan(float angle_deg)
{
    if (angle_deg < PAN_MIN_DEG) angle_deg = PAN_MIN_DEG;
    if (angle_deg > PAN_MAX_DEG) angle_deg = PAN_MAX_DEG;
    current_pan = angle_deg;

    uint32_t pulse = AngleToPulse(current_pan);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)pulse); // D5
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)pulse); // D7
}

void Servo_SetTilt(float angle_deg)
{
    if (angle_deg < TILT_MIN_DEG) angle_deg = TILT_MIN_DEG;
    if (angle_deg > TILT_MAX_DEG) angle_deg = TILT_MAX_DEG;
    current_tilt = angle_deg;

    uint32_t pulse = AngleToPulse(current_tilt);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)pulse); // D4
}

void Servo_ApplyCorrection(float d_pan, float d_tilt)
{
    // 1. Zaman tabanli hiz siniri: PC kare basina delta gonderir. FPS artarsa
    // ayni delta daha sik gelir; burada gercek dt olculerek hiz sabitlenir.
    uint32_t now = HAL_GetTick();
    float dt_s = 0.010f; // ilk komutta dongu periyodu varsayimi
    if (have_last_cmd)
    {
        uint32_t dt_ms = now - last_cmd_ms;
        dt_s = (float)dt_ms / 1000.0f;
        if (dt_s < 0.005f) dt_s = 0.005f;  // 200 Hz ust sinir
        if (dt_s > 0.100f) dt_s = 0.100f;  // 10 Hz alt sinir
    }
    last_cmd_ms = now;
    have_last_cmd = true;

    // 2. Tek pakette mutlak adim siniri + zaman tabanli hiz siniri (kucuk olani)
    float limit_pan = MAX_RATE_PAN_DPS * dt_s;
    if (limit_pan > MAX_STEP_PAN_DEG) limit_pan = MAX_STEP_PAN_DEG;
    float limit_tilt = MAX_RATE_TILT_DPS * dt_s;
    if (limit_tilt > MAX_STEP_TILT_DEG) limit_tilt = MAX_STEP_TILT_DEG;

    if (d_pan > limit_pan) d_pan = limit_pan;
    if (d_pan < -limit_pan) d_pan = -limit_pan;
    if (d_tilt > limit_tilt) d_tilt = limit_tilt;
    if (d_tilt < -limit_tilt) d_tilt = -limit_tilt;

    // Kamera taretle birlikte dondugunden hata acisini mevcut aciya ekler.
    // Yon tersleme PC katmaninda yapilir; burada tekrar terslenmez.
    Servo_SetPan(current_pan + d_pan);
    Servo_SetTilt(current_tilt + d_tilt);
}

bool Servo_HomeStep(void)
{
    float d_pan = SERVO_CENTER_DEG - current_pan;
    float d_tilt = SERVO_CENTER_DEG - current_tilt;

    if (d_pan < SERVO_HOME_TOL_DEG && d_pan > -SERVO_HOME_TOL_DEG &&
        d_tilt < SERVO_HOME_TOL_DEG && d_tilt > -SERVO_HOME_TOL_DEG)
    {
        return true;
    }

    // Ayni hiz sinirlayicisindan gecer: E-stop artik servo uzerinde basamak
    // olusturmaz, merkeze ~45 derece/s ile yumusak iner.
    Servo_ApplyCorrection(d_pan, d_tilt);
    return false;
}

float Servo_GetPan(void)
{
    return current_pan;
}

float Servo_GetTilt(void)
{
    return current_tilt;
}
