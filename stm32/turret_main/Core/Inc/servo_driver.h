#ifndef __SERVO_DRIVER_H
#define __SERVO_DRIVER_H

#include "main.h"

// MG996R Darbe Genislikleri (Mikrosaniye - us)
#define SERVO_MIN_PULSE   570   // 0 derece
#define SERVO_MID_PULSE   1520  // 90 derece (Merkez)
#define SERVO_MAX_PULSE   2470  // 180 derece

// Guvenli Aci Sinirlari (Config Mount Limitleri: Pan +-75 deg, Tilt -25/+25 deg)
#define PAN_MIN_DEG       15.0f   // 90.0 - 75.0 deg
#define PAN_MAX_DEG       165.0f  // 90.0 + 75.0 deg
#define TILT_MIN_DEG      65.0f   // 90.0 - 25.0 deg (Mekanik kasa carpma ve disli siyirma korumasi)
#define TILT_MAX_DEG      115.0f  // 90.0 + 25.0 deg

// Donanim Koruma: Tek Dongude Maksimum Aci Adimi (Delta Clamping)
#define MAX_STEP_PAN_DEG  1.5f    // 100 Hz dongude ani tork sokunu engeller
#define MAX_STEP_TILT_DEG 1.5f

// Motor Donus Yoni Tersleme (0: Normal, 1: Ters Yon)
// NOT: Donus yonu tersleme PC katmaninda (PAN_INVERT=True) SSOT tarafindan yonetilir.
// Firmware katmaninda varsayilan 0 (Normal) olarak tutulmalidir.
#define PAN_DIR_INVERT    0
#define TILT_DIR_INVERT   0

void Servo_Init(void);
void Servo_SetPan(float angle_deg);
void Servo_SetTilt(float angle_deg);
void Servo_ApplyCorrection(float d_pan, float d_tilt);

float Servo_GetPan(void);
float Servo_GetTilt(void);

#endif /* __SERVO_DRIVER_H */
