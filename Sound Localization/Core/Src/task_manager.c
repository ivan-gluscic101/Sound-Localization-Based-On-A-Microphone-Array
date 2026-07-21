#include "task_manager.h"
#include "adc_driver.h"
#include "uart_driver.h"
#include "audio_common.h"
#include "raw_capture.h"
#include "mpu9250.h"
#include "cmsis_os.h"
#include <string.h>

/* Localization mode selected in audio_common.h; the aliases below keep the rest
 * of the code version-independent. Priority: USE_4MIC_TIME_LOC > 3-mic default. */
#if USE_4MIC_TIME_LOC
  #include "sound_loc3d_4mic_time.h"
  typedef loc3d_4mic_time_result_t loc_result_t;
  #define LOC_INIT()            LOC3D_4MIC_TIME_Init()
  #define LOC_PROCESS(b, r)     LOC3D_4MIC_TIME_Process((b), (r))
#else
  #if defined(USE_TIME_DOMAIN_LOC) && !USE_TIME_DOMAIN_LOC
    #error "Frequency-domain (FFT) localization removed. Set USE_TIME_DOMAIN_LOC=1 or USE_4MIC_TIME_LOC=1."
  #endif
  #include "loc3d_3mic_time.h"
  typedef loc3d_3mic_time_result_t loc_result_t;
  #define LOC_INIT()            LOC3D_3MIC_TIME_Init()
  #define LOC_PROCESS(b, r)     LOC3D_3MIC_TIME_Process((b), (r))
#endif

/* USE_MOCK_ADC is defined in audio_common.h (shared with mock_adc.c). */
#define ACQ_NUM_BUFFERS  3

osMessageQId  queueDmaEventHandle;
QueueHandle_t queueSnapshotHandle;
QueueHandle_t queueResultHandle;

/* Guards UART4: UART_Task (0x03) and IMU_Task (0x05) send concurrently;
 * without the mutex the two packets' bytes would interleave on the wire. */
static osMutexId uartMutexHandle;

/* Triple snapshot buffer + index FIFO (depth ACQ_NUM_BUFFERS-1): ACQ_Task rotates
 * 0->1->2->0, LOC_Task consumes in order, so ACQ never overwrites a busy buffer. */
static uint16_t acq_snapshot[ACQ_NUM_BUFFERS][HALF_BUFFER];

/* Sliding window: [previous half | current half]; owned solely by LOC_Task. */
static uint16_t sliding_buf[2 * HALF_BUFFER];

/* Freezes processing between detection and end of UART send so RawCapture_Snapshot
 * doesn't overwrite dbg_raw_chX mid-print. Producer: LOC_Task, consumer: UART_Task. */
static volatile uint8_t capture_ready = 0;

/* Task implementations */

/* ACQ_Task: on each DMA half/full event, copy that half into a snapshot buffer. */
static void StartACQTask(void const *argument)
{
    uint8_t write_idx = 0;

    for (;;) {
        osEvent evt = osMessageGet(queueDmaEventHandle, osWaitForever);
        if (evt.status != osEventMessage) { continue; }

        /* If LOC_Task lags and the queue is full, skip this event to avoid a torn read. */
        if (uxQueueSpacesAvailable(queueSnapshotHandle) == 0u) { continue; }

#if USE_MOCK_ADC
        (void)evt;
        Mock_FillHalf(acq_snapshot[write_idx]);
#else
        /* flag=0: HT -> first half; flag=1: TC -> second half */
        const uint16_t *src = (evt.value.v == 0u)
                              ? &adc_buffer[0]
                              : &adc_buffer[HALF_BUFFER];
        memcpy(acq_snapshot[write_idx], src, HALF_BUFFER * sizeof(uint16_t));
#endif

        xQueueSend(queueSnapshotHandle, &write_idx, 0u);
        write_idx = (uint8_t)((write_idx + 1u) % ACQ_NUM_BUFFERS);
    }
}

/* LOC_Task: maintain sliding window, run localization, push result. */
static void StartLOCTask(void const *argument)
{
    loc_result_t result;
    uint8_t read_idx;

    for (;;) {
        if (xQueueReceive(queueSnapshotHandle, &read_idx, portMAX_DELAY) != pdTRUE) { continue; }

        memcpy(&sliding_buf[0],
               &sliding_buf[HALF_BUFFER],
               HALF_BUFFER * sizeof(uint16_t));
        memcpy(&sliding_buf[HALF_BUFFER],
               acq_snapshot[read_idx],
               HALF_BUFFER * sizeof(uint16_t));

        /* Skip processing while a raw CSV is still being sent (avoids overwriting dbg_raw_chX). */
        if (!capture_ready) {
            /* LOC_PROCESS finds the onset, centers the window, fills dbg_raw_chX on detection. */
            if (LOC_PROCESS(sliding_buf, &result)) {
                capture_ready = 1;   /* dbg_raw_chX now holds this window */
                xQueueSend(queueResultHandle, &result, 0);
            }
        }
    }
}

/* UART_Task: send the localization result packet (0x03). */
static void StartUARTTask(void const *argument)
{
    loc_result_t r;

    for (;;) {
        if (xQueueReceive(queueResultHandle, &r, portMAX_DELAY) == pdTRUE) {
            osMutexWait(uartMutexHandle, osWaitForever);
            UART_SendAngle3DPacket(r.az_tenth, r.el_tenth, r.strength);

            /* Raw CSV dump of the detection window is disabled; content stays in
             * dbg_raw_chX. Uncomment to re-enable printing to the ESP32 monitor: */
            /* if (capture_ready) {
                UART_SendRawCaptureCSV(dbg_raw_ch0, dbg_raw_ch1,
                                       dbg_raw_ch2, dbg_raw_ch3);
            } */
            osMutexRelease(uartMutexHandle);

            /* Release freeze; LOC_Task may look for detections again. */
            capture_ready = 0;
        }
    }
}

/* IMU_Task: read MPU-9250, fuse orientation (Mahony), send packet 0x05.
 * Priority BelowNormal; runs ~100 Hz, sends at ~20 Hz. */
static void StartIMUTask(void const *argument)
{
    mpu_orientation_t ori;
    const float dt = 0.01f;       /* 100 Hz fusion */
    uint8_t send_div = 0;
    uint8_t flags;

    /* If MPU absent, MPU_Update fails and the task idles (board runs without IMU). */
    flags = MPU_HasMag() ? 0x01u : 0x00u;

    for (;;) {
        if (MPU_Update(dt, &ori) == 0) {
            if (++send_div >= 5u) {     /* ~20 Hz send */
                send_div = 0;

                int16_t roll_t  = (int16_t)(ori.roll_deg  * 10.0f);
                int16_t pitch_t = (int16_t)(ori.pitch_deg * 10.0f);

                /* No magnetometer: yaw has no absolute reference, so send yaw=0/flag=0
                 * and let the ESP32 rotate by pitch/roll only. */
                int16_t yaw_t = flags ? (int16_t)(ori.yaw_deg * 10.0f) : 0;

                osMutexWait(uartMutexHandle, osWaitForever);
                UART_SendOrientationPacket(roll_t, pitch_t, yaw_t, flags);
                osMutexRelease(uartMutexHandle);
            }
        }
        osDelay(10);
    }
}

/* Initialization */

/* Create queues, mutex and the RTOS tasks (ACQ/LOC/UART/IMU). */
void app_tasks_init(void)
{
    LOC_INIT();   /* compute M_geom before starting tasks */

    osMessageQDef(queueDmaEvent, 8, uint32_t);
    queueDmaEventHandle = osMessageCreate(osMessageQ(queueDmaEvent), NULL);

    queueSnapshotHandle = xQueueCreate(ACQ_NUM_BUFFERS - 1, sizeof(uint8_t));
    queueResultHandle   = xQueueCreate(4, sizeof(loc_result_t));

    osMutexDef(uartMutex);
    uartMutexHandle = osMutexCreate(osMutex(uartMutex));

    osThreadDef(ACQ_Task,  StartACQTask,  osPriorityRealtime,    0, 256);
    (void)osThreadCreate(osThread(ACQ_Task),  NULL);

    osThreadDef(LOC_Task,  StartLOCTask,  osPriorityHigh,        0, 1024);
    (void)osThreadCreate(osThread(LOC_Task),  NULL);

    osThreadDef(UART_Task, StartUARTTask, osPriorityLow,         0, 256);
    (void)osThreadCreate(osThread(UART_Task), NULL);

    /* IMU_Task: float fusion needs FPU context -> 512-word stack */
    osThreadDef(IMU_Task,  StartIMUTask,  osPriorityBelowNormal, 0, 512);
    (void)osThreadCreate(osThread(IMU_Task),  NULL);
}
