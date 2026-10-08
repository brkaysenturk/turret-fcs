#ifndef __LUNA_READ_H
#define __LUNA_READ_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

void Luna_Init(UART_HandleTypeDef *huart);
void Luna_RxCallback(UART_HandleTypeDef *huart);
void Luna_HandleError(void);

bool Luna_IsDataReady(void);
uint16_t Luna_GetDistance(void);
uint16_t Luna_GetStrength(void);

#endif /* __LUNA_READ_H */

