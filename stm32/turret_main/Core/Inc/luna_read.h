/*
 * ==============================================================================
 * MODUL: TF-LUNA LiDAR BASLIGI (USART1)
 * ==============================================================================
 * 9 baytlik cerceveyi ayristirir; mesafe ve sinyal gucu son gecerli deger
 * olarak tutulur. Sicaklik baytlari henuz kullanilmaz. Ayrinti: luna_read.c.
 * ==============================================================================
 */
#ifndef __LUNA_READ_H
#define __LUNA_READ_H

#include "main.h"
#include "protocol.h"
#include <stdint.h>
#include <stdbool.h>

void Luna_Init(UART_HandleTypeDef *huart);
void Luna_RxCallback(UART_HandleTypeDef *huart);
void Luna_HandleError(void);

uint16_t Luna_GetDistance(void);
uint16_t Luna_GetStrength(void);

#endif /* __LUNA_READ_H */

