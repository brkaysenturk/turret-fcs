#ifndef __FCS_COMM_H
#define __FCS_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define FCS_HEADER_RX   0xAA  // PC -> STM32
#define FCS_HEADER_TX   0x55  // STM32 -> PC

void FCS_Comm_Init(UART_HandleTypeDef *huart);
void FCS_Comm_RxCallback(UART_HandleTypeDef *huart);
void FCS_Comm_HandleError(void);

void FCS_Comm_SendTelemetry(uint16_t distance_cm, uint16_t signal_strength);

bool FCS_Comm_IsNewCommand(void);
float FCS_Comm_GetPan(void);
float FCS_Comm_GetTilt(void);
uint8_t FCS_Comm_GetState(void);

#endif /* __FCS_COMM_H */
