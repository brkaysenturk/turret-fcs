#ifndef __FLYWHEEL_SOLENOID_H
#define __FLYWHEEL_SOLENOID_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * ==============================================================================
 * ATIS SISTEMI SURUCUSU: Cift Flywheel (Volan) Motorlari & Solenoid Itici
 * ==============================================================================
 * Bu modul, cift fircali/fircasiz volan motorlarini ve dart itici solenoid
 * valfini non-blocking (islemciyi kilitlemeyen) zamanlama ile surer.
 */

// Varsayilan Pinler (Ileride donanima gore kolayca degistirilebilir)
#define FLYWHEEL_GPIO_PORT   GPIOB
#define FLYWHEEL_PIN         GPIO_PIN_0   // Volan Motoru MOSFET Tetik Pini

#define SOLENOID_GPIO_PORT   GPIOB
#define SOLENOID_PIN         GPIO_PIN_1   // Solenoid Valf MOSFET/Role Tetik Pini

// Zamanlama Ayarlari (Milisaniye)
#define SOLENOID_PULSE_MS    80   // Solenoidin ileri itme enerjilenme suresi
#define SOLENOID_RELOAD_MS   150  // Yayin mermiyi yuvaya cekme bekleme suresi
#define FLYWHEEL_SPINUP_MS   500  // Atis oncesi volanlarin nominal hiza ulasma suresi

typedef enum {
    LAUNCHER_STATE_IDLE = 0,
    LAUNCHER_STATE_SPINUP,
    LAUNCHER_STATE_PUSHING,
    LAUNCHER_STATE_RELOADING
} LauncherState_t;

/* Fonksiyonlar */
void Launcher_Init(void);
void Launcher_Update(void);              // Non-blocking durum makinesi (while icinde cagirilir)

void Launcher_Flywheel_Enable(bool enable); // Motorlari manuel ac/kapat
void Launcher_TriggerFire(void);         // Atis tahrigini baslat
bool Launcher_IsFiring(void);            // Su an atis icra ediliyor mu?

#endif /* __FLYWHEEL_SOLENOID_H */

