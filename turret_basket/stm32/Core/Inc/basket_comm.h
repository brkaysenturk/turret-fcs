/*
 * ==============================================================================
 * MODUL: BASKET PC <-> STM32 IKILI PROTOKOL BASLIGI (USART2)
 * ==============================================================================
 * 8 baytlik komut cercevesi:
 *   [0xAA][pan i16][tilt i16][speed u8][state u8][checksum u8]
 * ==============================================================================
 */
#ifndef __BASKET_COMM_H
#define __BASKET_COMM_H

#include "main.h"
#include "protocol.h"
#include <stdint.h>
#include <stdbool.h>

void Basket_Comm_Init(UART_HandleTypeDef *huart);
void Basket_Comm_RxCallback(UART_HandleTypeDef *huart);
void Basket_Comm_HandleError(void);

void Basket_Comm_SendTelemetry(uint16_t distance_cm, uint16_t signal_strength, float pan_deg, float tilt_deg);

bool Basket_Comm_GetCommand(float *pan, float *tilt, uint8_t *speed, uint8_t *state);

#endif /* __BASKET_COMM_H */
