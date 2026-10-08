#include "servo_driver.h"

static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim1;

static float current_pan = 90.0f;
static float current_tilt = 90.0f;

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

    // 2. TIM1: PA8 (D7) - PAN opsiyonel yedek
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
#if PAN_DIR_INVERT
    d_pan = -d_pan;
#endif
#if TILT_DIR_INVERT
    d_tilt = -d_tilt;
#endif
    // Kamera taretle birlikte dondugunden hata acisini mevcut aciya ekler
    Servo_SetPan(current_pan + d_pan);
    Servo_SetTilt(current_tilt + d_tilt);
}

float Servo_GetPan(void)
{
    return current_pan;
}

float Servo_GetTilt(void)
{
    return current_tilt;
}
