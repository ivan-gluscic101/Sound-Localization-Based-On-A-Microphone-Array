#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "main.h"
#include "audio_common.h"
#include <stdint.h>

extern uint16_t adc_buffer[FULL_BUFFER];

void Custom_ADC_Init(void);
void Custom_ACQ_Start(void);

#endif /* ADC_DRIVER_H */
