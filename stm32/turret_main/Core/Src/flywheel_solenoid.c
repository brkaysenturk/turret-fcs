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

void Launcher_TriggerFire(void)
{
    // Eger sistem bostaysa atis sekansini baslat
    if (current_state == LAUNCHER_STATE_IDLE)
    {
        if (!flywheel_active)
        {
            // Volanlar calismiyorsa once onlari calistirip devir almasini bekle
            Launcher_Flywheel_Enable(true);
            state_timer = HAL_GetTick();
            current_state = LAUNCHER_STATE_SPINUP;
        }
        else
        {
            // Volanlar zaten calisiyorsa dogrudan iticiyi tetikle
            HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_SET);
            state_timer = HAL_GetTick();
            current_state = LAUNCHER_STATE_PUSHING;
        }
    }
}

void Launcher_Update(void)
{
    uint32_t now = HAL_GetTick();

    switch (current_state)
    {
        case LAUNCHER_STATE_IDLE:
            // Bekleme modu
            break;

        case LAUNCHER_STATE_SPINUP:
            // Volanlarin hedeflenen RPM'e ulasmasi icin bekleme (Non-blocking)
            if (now - state_timer >= FLYWHEEL_SPINUP_MS)
            {
                // Volanlar hazir, solenoidi mermiyi itmesi icin tetikle
                HAL_GPIO_WritePin(SOLENOID_GPIO_PORT, SOLENOID_PIN, GPIO_PIN_SET);
                state_timer = now;
                current_state = LAUNCHER_STATE_PUSHING;
            }
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
    }
}

bool Launcher_IsFiring(void)
{
    return (current_state != LAUNCHER_STATE_IDLE);
}

