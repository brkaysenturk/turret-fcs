#include "luna_read.h"
#include <stddef.h>

static UART_HandleTypeDef *luna_huart = NULL;
static uint8_t luna_rx_byte = 0;
static uint8_t luna_buffer[9];
static uint8_t luna_index = 0;

static volatile uint16_t distance_cm = 0;
static volatile uint16_t signal_strength = 0;
static volatile bool data_ready_flag = false;

void Luna_Init(UART_HandleTypeDef *huart)
{
    luna_huart = huart;
    luna_index = 0;
    data_ready_flag = false;
    if (luna_huart != NULL)
    {
        HAL_UART_Receive_IT(luna_huart, &luna_rx_byte, 1);
    }
}

void Luna_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart != luna_huart) return;

    uint8_t byte = luna_rx_byte;
    if (luna_index == 0)
    {
        if (byte == 0x59)
            luna_buffer[luna_index++] = byte;
    }
    else if (luna_index == 1)
    {
        if (byte == 0x59)
            luna_buffer[luna_index++] = byte;
        else
            luna_index = 0;
    }
    else
    {
        luna_buffer[luna_index++] = byte;

        if (luna_index >= 9)
        {
            luna_index = 0;

            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++)
            {
                checksum += luna_buffer[i];
            }

            if (checksum == luna_buffer[8])
            {
                distance_cm = luna_buffer[2] | (luna_buffer[3] << 8);
                signal_strength = luna_buffer[4] | (luna_buffer[5] << 8);
                data_ready_flag = true;
            }
        }
    }

    if (luna_huart != NULL)
    {
        HAL_UART_Receive_IT(luna_huart, &luna_rx_byte, 1);
    }
}

void Luna_HandleError(void)
{
    luna_index = 0;
    if (luna_huart != NULL)
    {
        __HAL_UART_CLEAR_OREFLAG(luna_huart);
        __HAL_UART_CLEAR_NEFLAG(luna_huart);
        __HAL_UART_CLEAR_FEFLAG(luna_huart);
        __HAL_UART_CLEAR_PEFLAG(luna_huart);
        HAL_UART_Receive_IT(luna_huart, &luna_rx_byte, 1);
    }
}

bool Luna_IsDataReady(void)
{
    if (data_ready_flag)
    {
        data_ready_flag = false;
        return true;
    }
    return false;
}

uint16_t Luna_GetDistance(void)
{
    return distance_cm;
}

uint16_t Luna_GetStrength(void)
{
    return signal_strength;
}

