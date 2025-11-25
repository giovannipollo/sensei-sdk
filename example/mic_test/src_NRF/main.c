/*
 * ----------------------------------------------------------------------
 *
 * File: main.c
 *
 * Last edited: 30.10.2025
 *
 * Copyright (c) 2024 ETH Zurich and University of Bologna
 *
 * Authors:
 * - Philip Wiese (wiesep@iis.ee.ethz.ch), ETH Zurich
 *
 * ----------------------------------------------------------------------
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <limits.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include "bsp/pwr_bsp.h"
#include "pwr/pwr.h"
#include <zephyr/audio/dmic.h>
#include <nrfx_pdm.h>
#include <hal/nrf_pdm.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 3000

/* Structure describing a color by its component values and name */
struct color_data {
  uint8_t r, g, b;
  const char *name;
};

/* The sequence of colors the RGB LED will display */
static const struct color_data color_sequence[] = {
    {0xFF, 0x00, 0x00, "Red"},   {0x00, 0xFF, 0x00, "Green"},  {0x00, 0x00, 0xFF, "Blue"},
    {0xFF, 0xFF, 0xFF, "White"}, {0xFF, 0xFF, 0x00, "Yellow"}, {0xFF, 0x00, 0xFF, "Purple"},
    {0x00, 0xFF, 0xFF, "Cyan"},  {0xF4, 0x79, 0x20, "Orange"},
};

#define MAX_SAMPLE_RATE 16000
#define DOWNSAMPLING_RATE 8
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)

/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT 1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) (BYTES_PER_SAMPLE * (_sample_rate / 10) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE BLOCK_SIZE(MAX_SAMPLE_RATE, 2)
#define BLOCK_COUNT 8
K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

static volatile bool keep_running = true;

#define GPIO_NODE_gap9_i2c_ctrl DT_NODELABEL(gpio_gap9_i2c_ctrl)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct device *const uart_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
static const struct device *const led = DEVICE_DT_GET_ANY(issi_is31fl3194);
static const struct gpio_dt_spec gpio_p0_6_gap9_i2c_ctrl = GPIO_DT_SPEC_GET(GPIO_NODE_gap9_i2c_ctrl, gpios);

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static int do_continuous_pdm_transfer(const struct device *dmic_dev, struct dmic_cfg *cfg) {
  int ret;

  LOG_INF("PCM output rate: %u, channels: %u", cfg->streams[0].pcm_rate, cfg->channel.req_num_chan);

  ret = dmic_configure(dmic_dev, cfg);
  if (ret < 0) {
    LOG_ERR("Failed to configure the driver: %d", ret);
    return ret;
  }

  ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
  if (ret < 0) {
    LOG_ERR("START trigger failed: %d", ret);
    return ret;
  }

  LOG_INF("Starting continuous audio capture");

  int block_num = 0;
  while (keep_running) {
    void *buffer;
    uint32_t size;

    ret = dmic_read(dmic_dev, 0, &buffer, &size, READ_TIMEOUT);
    if (ret < 0) {
      LOG_ERR("%d - read failed: %d", block_num, ret);
      break;
    }

    // Compute audio statistics from the samples in the block
    int16_t *samples = (int16_t *)buffer;
    int num_samples = size / BYTES_PER_SAMPLE;
    
    int32_t sum = 0;
    int16_t min_val = INT16_MAX;
    int16_t max_val = INT16_MIN;
    
    for (int i = 0; i < num_samples; i++) {
      sum += samples[i];
      if (samples[i] < min_val) min_val = samples[i];
      if (samples[i] > max_val) max_val = samples[i];
    }
    
    int16_t avg = sum / num_samples;
    int16_t p2p = max_val - min_val;
    
    // Print first 8 samples on first block, then just stats
    if (block_num == 0) {
      printk("Raw[0-7]: %d %d %d %d %d %d %d %d\n",
             samples[0], samples[1], samples[2], samples[3],
             samples[4], samples[5], samples[6], samples[7]);
    }
    
    // Use printk for faster output (bypasses log subsystem)
    printk("[%d] avg=%d min=%d max=%d p2p=%d\n", 
           block_num, avg, min_val, max_val, p2p);

    k_mem_slab_free(&mem_slab, buffer);
    block_num++;
  }

  ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
  if (ret < 0) {
    LOG_ERR("STOP trigger failed: %d", ret);
    return ret;
  }

  LOG_INF("Stopped after %d blocks", block_num);
  return ret;
}

int main(void) {
  int ret = 0;
  const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

  LOG_INIT();

  LOG_INF("LED Test on %s", CONFIG_BOARD);

  // Initialize power management
  // WIESEP: We do not start the power management task as it requires access to the I2C_A bus,
  // which will be used by the GAP9.
  if (pwr_init()) {
    LOG_ERR("PWR Init failed!");
  }
  pwr_start();

  if (!device_is_ready(led)) {
    LOG_ERR("LED device not ready");
    return 0;
  }

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("CDC ACM device not ready");
    return 0;
  }

  if (usb_enable(NULL)) {
    return 0;
  }
  LOG_INF("USB enabled");

  for (size_t i = 0; i < ARRAY_SIZE(color_sequence); i++) {
    ret = led_set_color(led, 0, 3, &(color_sequence[i].r));
    if (ret) {
      LOG_ERR("Failed to set color");
      return 0;
    }
    k_msleep(333);
  }

  k_msleep(100);
  if (gpio_pin_set_dt(&gpio_p0_6_gap9_i2c_ctrl, 1) < 0) {
    LOG_ERR("GAP9 I2C GPIO configuration error");
    return 0;
  }

  gap9_pwr(true);
  LOG_INF("GAP9 powered up");

  LOG_INF("DMIC continuous sample");

  if (!device_is_ready(dmic_dev)) {
    LOG_ERR("%s is not ready", dmic_dev->name);
    return 0;
  }

  struct pcm_stream_cfg stream = {
      .pcm_width = SAMPLE_BIT_WIDTH,
      .mem_slab = &mem_slab,
  };
  struct dmic_cfg cfg = {
      .io =
          {
              /* These fields can be used to limit the PDM clock
               * configurations that the driver is allowed to use
               * to those supported by the microphone.
               */
              .min_pdm_clk_freq = 1000000,
              .max_pdm_clk_freq = 3500000,
              .min_pdm_clk_dc = 40,
              .max_pdm_clk_dc = 60,
          },
      .streams = &stream,
      .channel =
          {
              .req_num_streams = 1,
          },
  };

  /* Configure for mono capture */
  cfg.channel.req_num_chan = 1;
  cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);

  cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
  cfg.streams[0].block_size = BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);

  ret = do_continuous_pdm_transfer(dmic_dev, &cfg);
  if (ret < 0) {
    return 0;
  }

  LOG_INF("Exiting");
  return 0;
}
