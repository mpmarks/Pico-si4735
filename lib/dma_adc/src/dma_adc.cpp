/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdlib.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "dma_adc.h"

#define ANALOG_RAW_BUFFER_COUNT 2

static struct {
    struct dma_adc_config config;
    int dma_channel;
    uint16_t* raw_buffer[ANALOG_RAW_BUFFER_COUNT];
    volatile int raw_buffer_write_index;
    volatile int raw_buffer_read_index;
    uint buffer_size;
    uint dma_irq;
    analog_samples_ready_handler_t samples_ready_handler;
} dma_adc;

static void analog_dma_handler();

int dma_adc_init(const struct dma_adc_config* config) {
    memset(&dma_adc, 0x00, sizeof(dma_adc));
    memcpy(&dma_adc.config, config, sizeof(dma_adc.config));

    if (config->gpio < 26 || config->gpio > 29) {
        return -1;
    }

    size_t raw_buffer_size = config->sample_buffer_size * sizeof(dma_adc.raw_buffer[0][0]);

    dma_adc.buffer_size = config->sample_buffer_size;


    for (int i = 0; i < ANALOG_RAW_BUFFER_COUNT; i++) {
        dma_adc.raw_buffer[i] = (uint16_t*) malloc(raw_buffer_size);
        if (dma_adc.raw_buffer[i] == NULL) {
            dma_adc_deinit();

            return -1;   
        }
    }

  
    dma_adc.dma_channel = dma_claim_unused_channel(true);
    if (dma_adc.dma_channel < 0) {
        dma_adc_deinit();

        return -1;
    }

 
    float clk_div = (clock_get_hz(clk_adc) / (1.0 * config->sample_rate)) - 1;

    dma_channel_config dma_channel_cfg = dma_channel_get_default_config(dma_adc.dma_channel);

    channel_config_set_transfer_data_size(&dma_channel_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_channel_cfg, false);
    channel_config_set_write_increment(&dma_channel_cfg, true);
    channel_config_set_dreq(&dma_channel_cfg, DREQ_ADC);

    dma_adc.dma_irq = DMA_IRQ_0;

    

    dma_channel_configure(
        dma_adc.dma_channel,
        &dma_channel_cfg,
        dma_adc.raw_buffer[0],
        &adc_hw->fifo,
        dma_adc.buffer_size,
        false
    );

    adc_gpio_init(config->gpio);

 

    adc_init();
    adc_select_input(config->gpio - 26);
    adc_set_clkdiv(clk_div);

    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false    // Don't shift each sample to 8 bits when pushing to FIFO
    );

    return true;

}

void dma_adc_deinit() {
    for (int i = 0; i < ANALOG_RAW_BUFFER_COUNT; i++) {
        if (dma_adc.raw_buffer[i]) {
            free(dma_adc.raw_buffer[i]);
            dma_adc.raw_buffer[i] = NULL;
        }
    }

    if (dma_adc.dma_channel > -1) {
        dma_channel_unclaim(dma_adc.dma_channel);
        dma_adc.dma_channel = -1;
    }
}

int dma_adc_start() {
    irq_set_enabled(dma_adc.dma_irq, true);
    irq_set_exclusive_handler(dma_adc.dma_irq, analog_dma_handler);

    if (dma_adc.dma_irq == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(dma_adc.dma_channel, true);
    } else if (dma_adc.dma_irq == DMA_IRQ_1) {
        dma_channel_set_irq1_enabled(dma_adc.dma_channel, true);
    } else {
        return -1;
    }

    dma_adc.raw_buffer_write_index = 0;
    dma_adc.raw_buffer_read_index = 0;

    dma_channel_transfer_to_buffer_now(
        dma_adc.dma_channel,
        dma_adc.raw_buffer[0],
        dma_adc.buffer_size
    );

    adc_run(true); // start running the adc
    return true;
}

void dma_adc_stop() {
    adc_run(false); // stop running the adc

    dma_channel_abort(dma_adc.dma_channel);

    if (dma_adc.dma_irq == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(dma_adc.dma_channel, false);
    } else if (dma_adc.dma_irq == DMA_IRQ_1) {
        dma_channel_set_irq1_enabled(dma_adc.dma_channel, false);
    }

    irq_set_enabled(dma_adc.dma_irq, false);
}

static void analog_dma_handler() {
    // clear IRQ
    if (dma_adc.dma_irq == DMA_IRQ_0) {
        dma_hw->ints0 = (1u << dma_adc.dma_channel);
    } else if (dma_adc.dma_irq == DMA_IRQ_1) {
        dma_hw->ints1 = (1u << dma_adc.dma_channel);
    }

    // get the current buffer index
    dma_adc.raw_buffer_read_index = dma_adc.raw_buffer_write_index;

    // get the next capture index to send the dma to start
    dma_adc.raw_buffer_write_index = (dma_adc.raw_buffer_write_index + 1) % ANALOG_RAW_BUFFER_COUNT;

    // give the channel a new buffer to write to and re-trigger it
    dma_channel_transfer_to_buffer_now(
        dma_adc.dma_channel,
        dma_adc.raw_buffer[dma_adc.raw_buffer_write_index],
        dma_adc.buffer_size
    );

    if (dma_adc.samples_ready_handler) {
        dma_adc.samples_ready_handler();
    }
}

void dma_adc_set_samples_ready_handler(analog_samples_ready_handler_t handler) {
    dma_adc.samples_ready_handler = handler;
}

int dma_adc_read(int16_t* buffer, size_t samples) {
    if (samples > dma_adc.config.sample_buffer_size) {
        samples = dma_adc.config.sample_buffer_size;
    }

    if (dma_adc.raw_buffer_write_index == dma_adc.raw_buffer_read_index) {
        return 0;
    }

    uint16_t* in = dma_adc.raw_buffer[dma_adc.raw_buffer_read_index];
    int16_t* out = buffer;

    dma_adc.raw_buffer_read_index++;

    for (int i = 0; i < samples; i++) {
        *out++ = *in++;
    }

    return samples;
}
