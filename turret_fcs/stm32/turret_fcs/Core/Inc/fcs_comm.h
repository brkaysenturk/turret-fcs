/*
 * ==============================================================================
 * MODUL: PC <-> STM32 IKILI PROTOKOL BASLIGI (USART2)
 * ==============================================================================
 * Cerceve boylari, header baytlari ve state kodlari fcs_comm.c ile birebir
 * aynidir. Protokol degisirse iki taraf (PC fcs_comm.py + bu baslik) birlikte
 * guncellenmelidir; aksi halde checksum dogru olsa bile alanlar kayar.
 * ==============================================================================
 */
#ifndef __FCS_COMM_H
#define __FCS_COMM_H

#include "main.h"
#include "protocol.h"
#include <stdint.h>
#include <stdbool.h>

void FCS_Comm_Init(UART_HandleTypeDef *huart);
void FCS_Comm_RxCallback(UART_HandleTypeDef *huart);
void FCS_Comm_HandleError(void);

void FCS_Comm_SendTelemetry(uint16_t distance_cm, uint16_t signal_strength, float pan_deg, float tilt_deg);

bool FCS_Comm_GetCommand(float *pan, float *tilt, uint8_t *state);

#endif /* __FCS_COMM_H */
