/*
 * ==============================================================================
 * MODUL: VOLAN VE SOLENOID FIRLATICI SURUCUSU (PB0 / PB1)
 * ==============================================================================
 * PB0 volan MOSFET'ini, PB1 solenoid iticiyi surer. Pinler acilista PULLDOWN
 * ve LOW'a cekilir; hata durumunda Error_Handler ayni pinleri sifirlar.
 *
 * Zamanlama (non-blocking):
 * - PUSHING   : solenoid 80 ms enerjili (dart itilir).
 * - RELOADING : 150 ms bekleme (yay mermiyi yuvaya ceker).
 * - IDLE      : tetik beklenir.
 *
 * Launcher_TriggerFire() volan hazir degilse atisi reddeder; E-stop her seyi
 * aninda keser. Boylece bobbin asiri isinmasi ve cift dart atisi onlenir.
 * ==============================================================================
 */
#include "flywheel_solenoid.h"

static LauncherState_t current_state = LAUNCHER_STATE_IDLE;
static uint32_t state_timer = 0;
static bool flywheel_active = false;

void Launcher_Init(void)
{
    // GPIO Saatlerini aktif et
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = FLYWHEEL_PIN | SOLENOID_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN; // Donanimsal olarak 0V'a asilsin (bosta yuksek voltaj engelleme)
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    // Baslangicta her iki aktuator de kesinlikle kapali (LOW / 0V)
    HAL_GPIO_WritePin(FLYWHEEL_GPIO_PORT, FLYWHEEL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_RESET);

    current_state = LAUNCHER_STATE_IDLE;
    flywheel_active = false;
}

void Launcher_EmergencyStop(void)
{
    // Acil durum: Volan ve solenoid enerjisini aninda kes
    HAL_GPIO_WritePin(FLYWHEEL_GPIO_PORT, FLYWHEEL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_RESET);
    current_state = LAUNCHER_STATE_IDLE;
    flywheel_active = false;
}

void Launcher_Flywheel_Enable(bool enable)
{
    flywheel_active = enable;
    if (enable)
    {
        HAL_GPIO_WritePin(FLYWHEEL_GPIO_PORT, FLYWHEEL_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(FLYWHEEL_GPIO_PORT, FLYWHEEL_PIN, GPIO_PIN_RESET);
    }
}

bool Launcher_IsFlywheelActive(void)
{
    return flywheel_active;
}

bool Launcher_TriggerFire(void)
{
    // Atis yalnizca bostayken baslatilir (cift tetikleme korumasi)
    if (current_state != LAUNCHER_STATE_IDLE)
    {
        return false;
    }

    // Volan hazir degilse atisi reddet. Sessizce beklemek yerine haber ver:
    // PC 80 ms FIRING penceresini kacirirsa dart yanlis anda firlatilir.
    if (!flywheel_active)
    {
        return false;
    }

    HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_SET);
    state_timer = HAL_GetTick();
    current_state = LAUNCHER_STATE_PUSHING;
    return true;
}

void Launcher_Update(void)
{
    uint32_t now = HAL_GetTick();

    switch (current_state)
    {
        case LAUNCHER_STATE_IDLE:
            // Bekleme modu
            break;

        case LAUNCHER_STATE_PUSHING:
            // Solenoid darbesi tamamlandi mi?
            if (now - state_timer >= SOLENOID_PULSE_MS)
            {
                // Solenoid enerjisini kes, yay geri ceksin
                HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_RESET);
                state_timer = now;
                current_state = LAUNCHER_STATE_RELOADING;
            }
            break;

        case LAUNCHER_STATE_RELOADING:
            // Yayin mermiyi hazneye tam oturtabilmesi icin bekleme
            if (now - state_timer >= SOLENOID_RELOAD_MS)
            {
                current_state = LAUNCHER_STATE_IDLE;
            }
            break;

        default:
            // Bozulmus/gecersiz durum: her iki aktüatoru de guvenli duruma al.
            Launcher_EmergencyStop();
            break;
    }
}

bool Launcher_IsFiring(void)
{
    return (current_state != LAUNCHER_STATE_IDLE);
}

