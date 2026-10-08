#ifndef __SERVO_DRIVER_H
#define __SERVO_DRIVER_H

#include "main.h"

// MG996R Darbe Genislikleri (Mikrosaniye - us)
#define SERVO_MIN_PULSE   570   // 0 derece
#define SERVO_MID_PULSE   1520  // 90 derece (Merkez)
#define SERVO_MAX_PULSE   2470  // 180 derece

// Guvenli Aci Sinirlari
#define PAN_MIN_DEG       0.0f
#define PAN_MAX_DEG       180.0f
#define TILT_MIN_DEG      20.0f  // Mekanik carpma siniri
#define TILT_MAX_DEG      160.0f

// Motor Donus Yoni Tersleme (0: Normal, 1: Ters Yon)
// Montaj sekline gore taret hedeften uzaklasiyorsa 1 yapilir
#define PAN_DIR_INVERT    0
#define TILT_DIR_INVERT   0

void Servo_Init(void);
void Servo_SetPan(float angle_deg);
void Servo_SetTilt(float angle_deg);
void Servo_ApplyCorrection(float d_pan, float d_tilt);

float Servo_GetPan(void);
float Servo_GetTilt(void);

#endif /* __SERVO_DRIVER_H */
