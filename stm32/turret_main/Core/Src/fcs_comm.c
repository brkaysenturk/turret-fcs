/*
 * ==============================================================================
 * MODUL: PC <-> STM32 IKILI HABERLESME (USART2, 115200 8N1)
 * ==============================================================================
 * TX (PC'ye) 10 bayt:
 *   [0x55][mesafe u16][sinyal u16][pan i16/10][tilt i16/10][checksum u8]
 * RX (PC'den) 7 bayt:
 *   [0xAA][pan i16/10][tilt i16/10][state u8][checksum u8]
 *
 * checksum her iki yonde de 8 bit toplamdir. state: 0=TRACKING, 1=LOCKED,
 * 2=FIRING, 3=ESTOP. E-stop yalnizca ust uste iki tutarli pakette kabul edilir;
 * tek bozuk paket tareti merkeze firlatamaz. Gecerli paketlerde aci degerleri
 * ISR icinde atomik olarak saklanir, ana dongu IRQ kapaliyken okur.
 * ==============================================================================
 */
#include "fcs_comm.h"
#include <stddef.h>

static UART_HandleTypeDef *fcs_huart = NULL;
static uint8_t fcs_rx_byte = 0;
static uint8_t rx_buffer[PROTO_CMD_FRAME_LEN];
static uint8_t rx_index = 0;

static volatile float cmd_pan = 0.0f;
static volatile float cmd_tilt = 0.0f;
static volatile uint8_t cmd_state = PROTO_STATE_TRACKING;
static volatile bool new_cmd_flag = false;

// E-stop tek bozuk paketle tetiklenmesin: ust uste iki tutarli paket sart.
static uint8_t estop_streak = 0;

static void accept_command(int16_t p, int16_t t, uint8_t state)
{
    cmd_pan = (float)p / PROTO_ANGLE_SCALE;
    cmd_tilt = (float)t / PROTO_ANGLE_SCALE;
    cmd_state = state;
    new_cmd_flag = true;
}

void FCS_Comm_Init(UART_HandleTypeDef *huart)
{
    fcs_huart = huart;
    rx_index = 0;
    new_cmd_flag = false;
    estop_streak = 0;
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
        if (byte == PROTO_PC_TO_STM_HEADER)
            rx_buffer[rx_index++] = byte;
    }
    else
    {
        rx_buffer[rx_index++] = byte;

        if (rx_index >= PROTO_CMD_FRAME_LEN)
        {
            if (Proto_VerifyChecksum(rx_buffer, PROTO_CMD_FRAME_LEN))
            {
                rx_index = 0;

                int16_t p = (int16_t)(rx_buffer[1] | (rx_buffer[2] << 8));
                int16_t t = (int16_t)(rx_buffer[3] | (rx_buffer[4] << 8));
                uint8_t st = rx_buffer[5];

                // Gecersiz durum kodu: paketi reddet, alim devam eder.
                if (st > PROTO_STATE_ESTOP)
                {
                    estop_streak = 0;
                }
                else if (st == PROTO_STATE_ESTOP)
                {
                    // E-stop yalnizca ust uste dogrulanirsa kabul edilir; boylece
                    // tek bir bozulmus paket tareti merkeze firlatmaz.
                    if (estop_streak < PROTO_ESTOP_CONFIRM_FRAMES)
                    {
                        estop_streak++;
                    }
                    else
                    {
                        accept_command(p, t, st);
                    }
                }
                else
                {
                    estop_streak = 0;
                    accept_command(p, t, st);
                }
            }
            else
            {
                // Checksum hatali: kayan pencere (sliding window) ile tampon icindeki
                // bir sonraki olasi 0xAA basligini bul ve kaydir (paket kaybini onler).
                uint8_t shift = PROTO_CMD_FRAME_LEN;
                for (uint8_t i = 1u; i < PROTO_CMD_FRAME_LEN; i++)
                {
                    if (rx_buffer[i] == PROTO_PC_TO_STM_HEADER)
                    {
                        shift = i;
                        break;
                    }
                }
                uint8_t remaining = (uint8_t)(PROTO_CMD_FRAME_LEN - shift);
                for (uint8_t j = 0u; j < remaining; j++)
                {
                    rx_buffer[j] = rx_buffer[shift + j];
                }
                rx_index = remaining;
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
    estop_streak = 0;
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

    uint8_t tx_pkt[PROTO_TLM_FRAME_LEN];
    tx_pkt[0] = PROTO_STM_TO_PC_HEADER;
    tx_pkt[1] = (uint8_t)(distance_cm & 0xFF);
    tx_pkt[2] = (uint8_t)(distance_cm >> 8);
    tx_pkt[3] = (uint8_t)(signal_strength & 0xFF);
    tx_pkt[4] = (uint8_t)(signal_strength >> 8);

    int16_t p = (int16_t)(pan_deg * PROTO_ANGLE_SCALE);
    int16_t t = (int16_t)(tilt_deg * PROTO_ANGLE_SCALE);
    tx_pkt[5] = (uint8_t)(p & 0xFF);
    tx_pkt[6] = (uint8_t)(p >> 8);
    tx_pkt[7] = (uint8_t)(t & 0xFF);
    tx_pkt[8] = (uint8_t)(t >> 8);

    uint8_t csum = 0;
    for (int i = 0; i < (int)(PROTO_TLM_FRAME_LEN - 1u); i++)
    {
        csum += tx_pkt[i];
    }
    tx_pkt[PROTO_TLM_FRAME_LEN - 1u] = csum;

    HAL_UART_Transmit(fcs_huart, tx_pkt, PROTO_TLM_FRAME_LEN, 10);
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
