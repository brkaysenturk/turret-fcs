#include "fcs_comm.h"
#include <stddef.h>

static UART_HandleTypeDef *fcs_huart = NULL;
static uint8_t fcs_rx_byte = 0;
static uint8_t rx_buffer[7];
static uint8_t rx_index = 0;

static volatile float cmd_pan = 0.0f;
static volatile float cmd_tilt = 0.0f;
static volatile uint8_t cmd_state = 0;
static volatile bool new_cmd_flag = false;

void FCS_Comm_Init(UART_HandleTypeDef *huart)
{
    fcs_huart = huart;
    rx_index = 0;
    new_cmd_flag = false;
    if (fcs_huart != NULL)
    {
        HAL_UART_Receive_IT(fcs_huart, &fcs_rx_byte, 1);
    }
}

void FCS_Comm_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart != fcs_huart) return;

    uint8_t byte = fcs_rx_byte;
    if (rx_index == 0)
    {
        if (byte == FCS_HEADER_RX)
            rx_buffer[rx_index++] = byte;
    }
    else
    {
        rx_buffer[rx_index++] = byte;

        if (rx_index >= 7)
        {
            rx_index = 0;

            uint8_t checksum = 0;
            for (int i = 0; i < 6; i++)
            {
                checksum += rx_buffer[i];
            }

            if (checksum == rx_buffer[6])
            {
                int16_t p = (int16_t)(rx_buffer[1] | (rx_buffer[2] << 8));
                int16_t t = (int16_t)(rx_buffer[3] | (rx_buffer[4] << 8));

                cmd_pan = (float)p / 10.0f;
                cmd_tilt = (float)t / 10.0f;
                cmd_state = rx_buffer[5];
                new_cmd_flag = true;
            }
        }
    }

    if (fcs_huart != NULL)
    {
        HAL_UART_Receive_IT(fcs_huart, &fcs_rx_byte, 1);
    }
}

void FCS_Comm_HandleError(void)
{
    rx_index = 0;
    if (fcs_huart != NULL)
    {
        __HAL_UART_CLEAR_OREFLAG(fcs_huart);
        __HAL_UART_CLEAR_NEFLAG(fcs_huart);
        __HAL_UART_CLEAR_FEFLAG(fcs_huart);
        __HAL_UART_CLEAR_PEFLAG(fcs_huart);
        HAL_UART_Receive_IT(fcs_huart, &fcs_rx_byte, 1);
    }
}

void FCS_Comm_SendTelemetry(uint16_t distance_cm, uint16_t signal_strength, float pan_deg, float tilt_deg)
{
    if (fcs_huart == NULL) return;

    uint8_t tx_pkt[10];
    tx_pkt[0] = FCS_HEADER_TX; // 0x55
    tx_pkt[1] = (uint8_t)(distance_cm & 0xFF);
    tx_pkt[2] = (uint8_t)(distance_cm >> 8);
    tx_pkt[3] = (uint8_t)(signal_strength & 0xFF);
    tx_pkt[4] = (uint8_t)(signal_strength >> 8);

    int16_t p = (int16_t)(pan_deg * 10.0f);
    int16_t t = (int16_t)(tilt_deg * 10.0f);
    tx_pkt[5] = (uint8_t)(p & 0xFF);
    tx_pkt[6] = (uint8_t)(p >> 8);
    tx_pkt[7] = (uint8_t)(t & 0xFF);
    tx_pkt[8] = (uint8_t)(t >> 8);

    uint8_t csum = 0;
    for (int i = 0; i < 9; i++)
    {
        csum += tx_pkt[i];
    }
    tx_pkt[9] = csum;

    HAL_UART_Transmit(fcs_huart, tx_pkt, 10, 10);
}

bool FCS_Comm_IsNewCommand(void)
{
    if (new_cmd_flag)
    {
        new_cmd_flag = false;
        return true;
    }
    return false;
}

bool FCS_Comm_GetCommand(float *pan, float *tilt, uint8_t *state)
{
    if (!new_cmd_flag)
    {
        return false;
    }

    __disable_irq();
    *pan = cmd_pan;
    *tilt = cmd_tilt;
    *state = cmd_state;
    new_cmd_flag = false;
    __enable_irq();

    return true;
}

float FCS_Comm_GetPan(void)
{
    return cmd_pan;
}

float FCS_Comm_GetTilt(void)
{
    return cmd_tilt;
}

uint8_t FCS_Comm_GetState(void)
{
    return cmd_state;
}

