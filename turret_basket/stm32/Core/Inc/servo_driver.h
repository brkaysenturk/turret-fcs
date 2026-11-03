/*
 * ==============================================================================
 * MODUL: BASKET PAN/TILT SERVO BASLIGI (MG996R)
 * ==============================================================================
 * Darbe genislikleri, guvenli aci sinirlari ve hiz siniri sabitleri buradadir.
 * ==============================================================================
 */
#ifndef __SERVO_DRIVER_H
#define __SERVO_DRIVER_H

#include "main.h"
#include <stdbool.h>

/* MG996R Darbe Genislikleri (Mikrosaniye - us) */
#define SERVO_MIN_PULSE   570   /* 0 derece */
#define SERVO_MID_PULSE   1520  /* 90 derece (Merkez) */
#define SERVO_MAX_PULSE   2470  /* 180 derece */

/* Guvenli Aci Sinirlari (Pan +-75 derece, Tilt -25/+45 derece super-elevasyon) */
#define PAN_MIN_DEG       15.0f   /* 90.0 - 75.0 derece */
#define PAN_MAX_DEG       165.0f  /* 90.0 + 75.0 derece */
#define TILT_MIN_DEG      65.0f   /* 90.0 - 25.0 derece */
#define TILT_MAX_DEG      135.0f  /* 90.0 + 45.0 derece (Kupaya asirtma super-elevasyon tavani) */

/* Donanim Koruma: Tek komutta Maksimum Aci Adimi (Delta Clamping) */
#define MAX_STEP_PAN_DEG  1.5f    /* Tek pakette ani tork sokunu engeller */
#define MAX_STEP_TILT_DEG 1.5f

/* Zaman Tabanli Hiz Siniri (derece/saniye) */
#define MAX_RATE_PAN_DPS   45.0f   /* 1.50 derece/kare @ 30 FPS */
#define MAX_RATE_TILT_DPS  45.0f

/* Yumusak merkez (E-stop / PC baglantisi koptugunda) */
#define SERVO_CENTER_DEG   90.0f
#define SERVO_HOME_TOL_DEG 0.15f

void Servo_Init(void);
void Servo_SetPan(float angle_deg);
void Servo_SetTilt(float angle_deg);
void Servo_ApplyCorrection(float d_pan, float d_tilt);

/* E-stop ve link kaybi icin: merkeze dogru tek adim atar (hiz sinirli).
   Merkeze ulastiysa true doner. */
bool Servo_HomeStep(void);

float Servo_GetPan(void);
float Servo_GetTilt(void);

#endif /* __SERVO_DRIVER_H */

