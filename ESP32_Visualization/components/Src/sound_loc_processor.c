#include "sound_loc_processor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "uart_driver.h"
#include "web_server.h"

static const char *TAG = "SOUND_LOC_PROC";

/* UART packet parser states. Types: 0x02=2D, 0x03=3D, 0x04=raw capture,
 * 0x05=board orientation (IMU, only roll/pitch used). */
typedef enum {
    S_SOF1 = 0,
    S_SOF2,
    S_TYPE,
    S_AZ_H,
    S_AZ_L,
    S_EL_H,
    S_EL_L,
    S_STR,
    S_RAW_NCH,
    S_RAW_NH,
    S_RAW_NL,
    S_RAW_BODY,
    S_ORI_RL_H,
    S_ORI_RL_L,
    S_ORI_PT_H,
    S_ORI_PT_L,
    S_ORI_YW_H,
    S_ORI_YW_L,
    S_ORI_FLAGS,
    S_EOF1,
    S_EOF2,
} pkt_state_t;

/* Print received raw capture as CSV to console (for MATLAB copy).
 * Layout in raw[]: channel-major big-endian uint16 -> raw[(ch*n+s)*2]. */
static void raw_capture_print(const uint8_t *raw, uint8_t nch, uint16_t n) {
    printf("RAW START NCH=%u N=%u\n", (unsigned)nch, (unsigned)n);
    for (uint16_t s = 0; s < n; s++) {
        for (uint8_t ch = 0; ch < nch; ch++) {
            size_t   idx = ((size_t)ch * n + s) * 2u;
            uint16_t v   = ((uint16_t)raw[idx] << 8) | raw[idx + 1];
            printf("%u%c", (unsigned)v, (ch + 1 < nch) ? ',' : '\n');
        }
    }
    printf("RAW END\n");
}

#define LINE_BUF_SIZE 160

/* UART RX task: parse incoming packets, forward angle/orientation/raw data. */
static void rx_task(void *arg) {
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Neuspjela alokacija buffera!");
        vTaskDelete(NULL);
        return;
    }

    pkt_state_t state    = S_SOF1;
    uint8_t     pkt_type = 0;
    uint8_t     az_h     = 0;
    uint8_t     az_l     = 0;
    uint8_t     el_h     = 0;
    uint8_t     el_l     = 0;
    uint8_t     str_val  = 0;

    /* Orientation (0x05): only roll and pitch used */
    uint8_t     ori_rl_h = 0, ori_rl_l = 0;
    uint8_t     ori_pt_h = 0, ori_pt_l = 0;

    static uint8_t raw_buf[RAW_MAX_BYTES];
    uint8_t        raw_nch   = 0;
    uint16_t       raw_n     = 0;
    size_t         raw_total = 0;   /* nch * n * 2 */
    size_t         raw_idx   = 0;

    static char line_buf[LINE_BUF_SIZE];
    size_t      line_idx = 0;

    int pkt_count = 0;
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0 && pkt_count == 0) {
            ESP_LOGI(TAG, "UART prima podatke! Broj bajtova: %d", len);
            pkt_count = 1;
        }

        for (int i = 0; i < len; i++) {
            uint8_t b = data[i];

            switch (state) {
                case S_SOF1:
                    if (b == ANGLE_PKT_SOF1) {
                        line_idx = 0;
                        state = S_SOF2;
                    } else if (b == '\n') {
                        line_buf[line_idx] = '\0';
                        if (line_idx > 0) {
                            ESP_LOGI(TAG, "%s", line_buf);
                        }
                        line_idx = 0;
                    } else if (b != '\r' && b >= 0x20 && b <= 0x7E) {
                        if (line_idx < LINE_BUF_SIZE - 1) {
                            line_buf[line_idx++] = (char)b;
                        }
                    }
                    break;
                case S_SOF2:
                    state = (b == ANGLE_PKT_SOF2) ? S_TYPE : S_SOF1;
                    break;
                case S_TYPE:
                    pkt_type = b;
                    if (b == 0x02 || b == 0x03) {
                        state = S_AZ_H;
                    } else if (b == ANGLE_PKT_TYPE_RAW) {
                        state = S_RAW_NCH;
                    } else if (b == 0x05) {
                        state = S_ORI_RL_H;
                    } else {
                        state = S_SOF1;
                    }
                    break;
                case S_AZ_H:
                    az_h = b;
                    state = S_AZ_L;
                    break;
                case S_AZ_L:
                    az_l = b;
                    state = (pkt_type == 0x03) ? S_EL_H : S_STR;
                    break;
                case S_EL_H:
                    el_h = b;
                    state = S_EL_L;
                    break;
                case S_EL_L:
                    el_l = b;
                    state = S_STR;
                    break;
                case S_STR:
                    str_val = b;
                    state = S_EOF1;
                    break;
                case S_RAW_NCH:
                    raw_nch = b;
                    state = S_RAW_NH;
                    break;
                case S_RAW_NH:
                    raw_n = (uint16_t)((uint16_t)b << 8);
                    state = S_RAW_NL;
                    break;
                case S_RAW_NL:
                    raw_n |= (uint16_t)b;
                    raw_total = (size_t)raw_nch * raw_n * 2u;
                    raw_idx   = 0;
                    /* Guard: drop packet if header exceeds buffer */
                    if (raw_nch == 0 || raw_n == 0 || raw_total > RAW_MAX_BYTES) {
                        ESP_LOGW(TAG, "RAW paket prevelik/nevaljan (NCH=%u N=%u), odbacujem",
                                 (unsigned)raw_nch, (unsigned)raw_n);
                        state = S_SOF1;
                    } else {
                        state = S_RAW_BODY;
                    }
                    break;
                case S_RAW_BODY:
                    raw_buf[raw_idx++] = b;
                    if (raw_idx >= raw_total) {
                        state = S_EOF1;
                    }
                    break;
                case S_ORI_RL_H: ori_rl_h = b; state = S_ORI_RL_L; break;
                case S_ORI_RL_L: ori_rl_l = b; state = S_ORI_PT_H; break;
                case S_ORI_PT_H: ori_pt_h = b; state = S_ORI_PT_L; break;
                case S_ORI_PT_L: ori_pt_l = b; state = S_ORI_YW_H; break;
                case S_ORI_YW_H: state = S_ORI_YW_L;  break;  /* yaw ignored */
                case S_ORI_YW_L: state = S_ORI_FLAGS; break;
                case S_ORI_FLAGS: state = S_EOF1; break;      /* flags unused */
                case S_EOF1:
                    state = (b == ANGLE_PKT_EOF1) ? S_EOF2 : S_SOF1;
                    break;
                case S_EOF2:
                    if (b == ANGLE_PKT_EOF2) {
                        if (pkt_type == ANGLE_PKT_TYPE_RAW) {
                            raw_capture_print(raw_buf, raw_nch, raw_n);
                        } else if (pkt_type == 0x05) {
                            /* Board orientation (IMU), not sound direction; roll/pitch only */
                            int16_t roll_t  = (int16_t)((ori_rl_h << 8) | ori_rl_l);
                            int16_t pitch_t = (int16_t)((ori_pt_h << 8) | ori_pt_l);
                            float roll  = roll_t  / 10.0f;
                            float pitch = pitch_t / 10.0f;
                            ESP_LOGI(TAG, "ORI | Roll: %.1f° | Pitch: %.1f°", roll, pitch);
                            web_server_send_orientation(roll, pitch);
                        } else {
                            int16_t az_tenth = (int16_t)((az_h << 8) | az_l);
                            int16_t el_tenth = (pkt_type == 0x03) ? (int16_t)((el_h << 8) | el_l) : 0;
                            float azimuth = az_tenth / 10.0f;
                            float elevation = el_tenth / 10.0f;
                            ESP_LOGI(TAG, "Type %d | Az: %.1f° | El: %.1f° | Strength: %d", pkt_type, azimuth, elevation, str_val);
                            web_server_send_data(azimuth, elevation, str_val);
                        }
                    }
                    state = S_SOF1;
                    break;
            }
        }
    }
    free(data);
}

/* Start the UART RX parsing task. */
void sound_loc_processor_init(void) {
    ESP_LOGI(TAG, "Inicijalizacija procesora kuta...");
    xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, 10, NULL);
}