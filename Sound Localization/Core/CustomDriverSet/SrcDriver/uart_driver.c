#include "uart_driver.h"
#include "audio_common.h"
#include <stdio.h>

/* Blocking send of one byte over UART4. */
static inline void UART_SendByte(uint8_t byte)
{
    while (!LL_USART_IsActiveFlag_TXE(UART4));
    LL_USART_TransmitData8(UART4, byte);
}

/* Send a 16-bit value big-endian. */
static inline void UART_SendU16BE(uint16_t v)
{
    UART_SendByte((uint8_t)(v >> 8));
    UART_SendByte((uint8_t)(v & 0xFF));
}

/* Init UART4 on PC10/PC11 (AF5), 115200 8N1. */
void Custom_UART4_Init(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef  GPIO_InitStruct  = {0};

    LL_RCC_SetUARTClockSource(LL_RCC_UART4_CLKSOURCE_PCLK1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
    /* GPIOC clock already enabled in Custom_GPIO_Init */

    /* PC10 → UART4_TX, PC11 → UART4_RX */
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_5;

    GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    USART_InitStruct.PrescalerValue      = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate            = 115200;
    USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART4, &USART_InitStruct);

    LL_USART_DisableFIFO(UART4);
    LL_USART_SetTXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_ConfigAsyncMode(UART4);
    LL_USART_Enable(UART4);

    while (!LL_USART_IsActiveFlag_TEACK(UART4) || !LL_USART_IsActiveFlag_REACK(UART4));
}

/* Send 2D angle packet (type 0x02): azimuth (0.1 deg) + strength. */
void UART_SendAnglePacket(int16_t phi_tenth_deg, uint8_t strength)
{
    UART_SendByte(0xAA);
    UART_SendByte(0xBB);
    UART_SendByte(0x02);
    UART_SendByte((uint8_t)(phi_tenth_deg >> 8));
    UART_SendByte((uint8_t)(phi_tenth_deg & 0xFF));
    UART_SendByte(strength);
    UART_SendByte(0xCC);
    UART_SendByte(0xDD);
    while (!LL_USART_IsActiveFlag_TC(UART4));
}

/* Send 3D angle packet (type 0x03): azimuth + elevation (0.1 deg) + strength. */
void UART_SendAngle3DPacket(int16_t az_tenth, int16_t el_tenth, uint8_t strength)
{
    UART_SendByte(0xAA);
    UART_SendByte(0xBB);
    UART_SendByte(0x03);
    UART_SendByte((uint8_t)(az_tenth >> 8));
    UART_SendByte((uint8_t)(az_tenth & 0xFF));
    UART_SendByte((uint8_t)(el_tenth >> 8));
    UART_SendByte((uint8_t)(el_tenth & 0xFF));
    UART_SendByte(strength);
    UART_SendByte(0xCC);
    UART_SendByte(0xDD);
    while (!LL_USART_IsActiveFlag_TC(UART4));
}

/* Send orientation packet (type 0x05): roll/pitch/yaw (0.1 deg) + flags. */
void UART_SendOrientationPacket(int16_t roll_tenth, int16_t pitch_tenth,
                                int16_t yaw_tenth, uint8_t flags)
{
    UART_SendByte(0xAA);
    UART_SendByte(0xBB);
    UART_SendByte(0x05);
    UART_SendByte((uint8_t)(roll_tenth  >> 8));
    UART_SendByte((uint8_t)(roll_tenth  & 0xFF));
    UART_SendByte((uint8_t)(pitch_tenth >> 8));
    UART_SendByte((uint8_t)(pitch_tenth & 0xFF));
    UART_SendByte((uint8_t)(yaw_tenth   >> 8));
    UART_SendByte((uint8_t)(yaw_tenth   & 0xFF));
    UART_SendByte(flags);
    UART_SendByte(0xCC);
    UART_SendByte(0xDD);
    while (!LL_USART_IsActiveFlag_TC(UART4));
}

/* Send raw capture packet (type 0x04): header + channel-major big-endian samples. */
void UART_SendRawCapture(const uint16_t *ch0, const uint16_t *ch1,
                         const uint16_t *ch2, const uint16_t *ch3)
{
    const uint16_t  n   = (uint16_t)SAMPLES_PER_CHANNEL;
    const uint16_t *chs[NUM_CH] = { ch0, ch1, ch2, ch3 };

    /* Header: [0xAA][0xBB][0x04][NCH][N_H][N_L] */
    UART_SendByte(0xAA);
    UART_SendByte(0xBB);
    UART_SendByte(0x04);
    UART_SendByte((uint8_t)NUM_CH);
    UART_SendU16BE(n);

    /* Body: channel-major, each sample big-endian uint16 */
    for (uint32_t ch = 0u; ch < (uint32_t)NUM_CH; ch++) {
        const uint16_t *p = chs[ch];
        for (uint16_t s = 0u; s < n; s++) {
            UART_SendU16BE(p[s]);
        }
    }

    /* Footer */
    UART_SendByte(0xCC);
    UART_SendByte(0xDD);
    while (!LL_USART_IsActiveFlag_TC(UART4));
}

/* Send a null-terminated string over UART4. */
void UART_SendString(const char *str)
{
    if (!str) return;
    while (*str) {
        UART_SendByte((uint8_t)*str);
        str++;
    }
}

/* Send raw capture as CSV text (RAW START/END markers for MATLAB/Excel copy). */
void UART_SendRawCaptureCSV(const uint16_t *ch0, const uint16_t *ch1,
                            const uint16_t *ch2, const uint16_t *ch3)
{
    char line[48];

    /* RAW START / RAW END markers make the block easy to cut from the log */
    UART_SendString("\r\nRAW START N=");
    snprintf(line, sizeof(line), "%u\r\nidx,M1,M2,M3,M4\r\n",
             (unsigned)SAMPLES_PER_CHANNEL);
    UART_SendString(line);

    for (uint16_t s = 0u; s < (uint16_t)SAMPLES_PER_CHANNEL; s++) {
        snprintf(line, sizeof(line), "%u,%u,%u,%u,%u\r\n",
                 (unsigned)s,
                 (unsigned)ch0[s], (unsigned)ch1[s],
                 (unsigned)ch2[s], (unsigned)ch3[s]);
        UART_SendString(line);
    }

    UART_SendString("RAW END\r\n");
    while (!LL_USART_IsActiveFlag_TC(UART4));
}
