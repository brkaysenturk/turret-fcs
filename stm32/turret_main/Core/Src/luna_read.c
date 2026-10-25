/*
 * ==============================================================================
 * MODUL: TF-LUNA ToF LiDAR OKUYUCU (USART1, 115200 8N1)
 * ==============================================================================
 * 9 baytlik cerceve:
 *   [0x59][0x59][distL][distH][ampL][ampH][tempL][tempH][checksum]
 *
 * Mesafe ve sinyal gucu ayristirilir; sicaklik baytlari su an kullanilmaz
 * (PC telemetri semasinda karsiligi yok). checksum 8 bit toplamdir; hatali
 * cercevede son gecerli okuma korunur. UART hata kesmesinde parser sifirlanir.
 * ==============================================================================
 */
#include "luna_read.h"
#include <stddef.h>

static UART_HandleTypeDef *luna_huart = NULL;
static uint8_t luna_rx_byte = 0;
static uint8_t luna_buffer[PROTO_LUNA_FRAME_LEN];
static uint8_t luna_index = 0;

static volatile uint16_t distance_cm = 0;
static volatile uint16_t signal_strength = 0;

void Luna_Init(UART_HandleTypeDef *huart)
{
    luna_huart = huart;
    luna_index = 0;
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
        if (byte == PROTO_LUNA_HEADER)
            luna_buffer[luna_index++] = byte;
    }
    else if (luna_index == 1)
    {
        if (byte == PROTO_LUNA_HEADER)
            luna_buffer[luna_index++] = byte;
        else
            luna_index = 0;
    }
    else
    {
        luna_buffer[luna_index++] = byte;

        if (luna_index >= PROTO_LUNA_FRAME_LEN)
        {
            luna_index = 0;

            if (Proto_VerifyChecksum(luna_buffer, PROTO_LUNA_FRAME_LEN))
            {
                distance_cm = luna_buffer[2] | (luna_buffer[3] << 8);
                signal_strength = luna_buffer[4] | (luna_buffer[5] << 8);
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

uint16_t Luna_GetDistance(void)
{
    return distance_cm;
}

uint16_t Luna_GetStrength(void)
{
    return signal_strength;
}

