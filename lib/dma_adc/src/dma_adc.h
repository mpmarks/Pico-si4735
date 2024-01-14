/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_DMA_ADC_H_
#define _PICO_DMA_ADC_H_

typedef void (*analog_samples_ready_handler_t)(void);


struct dma_adc_config {
    uint gpio;
    uint sample_rate;
    uint sample_buffer_size;
};


int dma_adc_init(const struct dma_adc_config* config);
void dma_adc_deinit();

int dma_adc_start();
void dma_adc_stop();

void dma_adc_set_samples_ready_handler(analog_samples_ready_handler_t handler);

int dma_adc_read(int16_t* buffer, size_t samples);

#endif
