/*
 * ----------------------------------------------------------------------
 *
 * File: test.c
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

#include "ISPKernels.h"
#include "bsp/bsp.h"
#include "bsp/camera/hm0360.h"
#include "gaplib/ImgIO.h"
#include "pmsis.h"

//// Decomment this line if you want to skip the debayer + white balance and save a raw bayer image
// #define SKIP_ISP

#ifdef CONFIG_APP_LOG
#define APP_LOG_TAG "App"
#define APP_LOG_TRC(fmt, ...) PI_LOG_TRC(APP_LOG_TAG, fmt, ##__VA_ARGS__)
#define APP_LOG_DBG(fmt, ...) PI_LOG_DBG(APP_LOG_TAG, fmt, ##__VA_ARGS__)
#define APP_LOG_ERR(fmt, ...) PI_LOG_ERR(APP_LOG_TAG, fmt, ##__VA_ARGS__)
#else
#define APP_LOG_TRC(fmt, ...)
#define APP_LOG_DBG(fmt, ...)
#define APP_LOG_ERR(fmt, ...)
#endif

#define WIDTH CONFIG_DRIVER_HM0360_FORMAT_WIDTH
#define HEIGHT CONFIG_DRIVER_HM0360_FORMAT_HEIGHT

#define ISP_ADJUST_DELAY_US 500000

uint8_t *ImageIn;
uint8_t *ImageOut;

pi_device_t *camera_device;
pi_evt_t end_of_capture_evt;

// For now only 10 bits config works
#define BUFFER_SIZE (WIDTH * HEIGHT)

typedef struct ArgCluster {
  uint32_t Win;
  uint32_t Hin;
  uint32_t Wout;
  uint32_t Hout;
  uint8_t *ImageIn;
  uint8_t *ImageOut;
} ArgCluster_T;

static void cluster_main(ArgCluster_T *ArgC) {
  printf("cluster master start\n");
  int32_t perf_count;
  pi_perf_conf(1 << PI_PERF_CYCLES);
  pi_perf_start();

  demosaic_image_HWC(ArgC->ImageIn, ArgC->ImageOut);
  white_balance_HWC(ArgC->ImageOut, ArgC->ImageOut, 98);

  pi_perf_stop();
  perf_count = pi_perf_read(PI_PERF_CYCLES);

  printf("\nCycles on cluster: %d Cycles\n", (perf_count));
}

int main(void) {
  pi_err_t err = PI_OK;

  printf("\n*** Entering CSI2 HM0360 test ***\n\n");

  // must be a multiple of 11 Mhz
  pi_freq_set(PI_FREQ_DOMAIN_PERIPH, 225 * 1000 * 1000);
  pi_freq_set(PI_FREQ_DOMAIN_FC, 370 * 1000 * 1000);

  APP_LOG_DBG("Format : %d\n", CONFIG_DRIVER_HM0360_FORMAT);
  APP_LOG_DBG("Width  : %d\n", WIDTH);
  APP_LOG_DBG("Height : %d\n", HEIGHT);

  err = pi_open(PI_CAMERA_HM0360, &camera_device);
  if (err) {
    APP_LOG_ERR("Failed to open camera device\n");
    goto error_handler;
  }
  APP_LOG_TRC("camera device opened\n");

  ImageIn = (uint8_t *)pi_l2_malloc(WIDTH * HEIGHT);
  if (ImageIn == NULL) {
    printf("Failed to allocate memory for input image(%ld bytes).\n", WIDTH * HEIGHT);
    pmsis_exit(-1);
  }

  ImageOut = (uint8_t *)pi_l2_malloc(WIDTH * HEIGHT * 3);
  if (ImageOut == NULL) {
    printf("Failed to allocate memory for output image(%ld bytes).\n", WIDTH * HEIGHT * 3);
    pmsis_exit(-1);
  }

  APP_LOG_TRC("Buffers allocated\n");

  struct pi_device cluster_dev;
  struct pi_cluster_conf conf;
  /* Init cluster configuration structure. */
  pi_cluster_conf_init(&conf);
  // conf.cc_stack_size = 2048;
  /* Configure And open cluster. */
  pi_open_from_conf(&cluster_dev, (void *)&conf);
  if (pi_cluster_open(&cluster_dev)) {
    printf("Cluster open failed !\n");
    pmsis_exit(-4);
  }

  /* Allocating L1 memory for cluster */
  ISP_L1_Memory = (char *)pi_l1_malloc(&cluster_dev, _ISP_L1_Memory_SIZE);
  if (ISP_L1_Memory == 0) {
    printf("Failed to allocate %d bytes for L1_memory\n", _ISP_L1_Memory_SIZE);
    pmsis_exit(-5);
  }
  pi_freq_set(PI_FREQ_DOMAIN_CL, 370 * 1000 * 1000);
  pi_camera_control(camera_device, PI_CAMERA_CMD_ON, 0);
  APP_LOG_TRC("Camera powered on\n");

  pi_time_wait_us(ISP_ADJUST_DELAY_US);

  ArgCluster_T cluster_call;

  // Assinging all input variables to Cluster structure
  cluster_call.ImageIn = ImageIn;
  cluster_call.Win = WIDTH;
  cluster_call.Hin = HEIGHT;
  cluster_call.Wout = WIDTH;
  cluster_call.Hout = HEIGHT;
  cluster_call.ImageOut = ImageOut;

  /* Prepare task to be offload to Cluster. */
  struct pi_cluster_task task;
  pi_cluster_task(&task, (void *)cluster_main, &cluster_call);

  APP_LOG_TRC("Triggering capture...\n");
  pi_camera_capture_async(camera_device, ImageIn, BUFFER_SIZE, pi_evt_sig_init(&end_of_capture_evt));
  pi_camera_control(camera_device, PI_CAMERA_CMD_START, 0);
  pi_evt_wait(&end_of_capture_evt);
  pi_camera_control(camera_device, PI_CAMERA_CMD_STOP, 0);

  APP_LOG_TRC("Closing HM0360 device\n");
  err = pi_close(PI_CAMERA_HM0360);
  if (err) {
    APP_LOG_ERR("Failed to close HM0360 device\n");
    goto error_handler;
  }

#ifdef SAVE_PIC
  printf("\nSaving Grayscale Image...\n");
  WriteImageToFile("../Out.pgm", WIDTH, HEIGHT, 1, ImageIn, GRAY_SCALE_IO);
#endif // SAVE_PIC

#ifndef SKIP_ISP
  pi_cluster_send_task(&cluster_dev, &task);
#endif
  pi_l1_free(&cluster_dev, ISP_L1_Memory, _ISP_L1_Memory_SIZE);

  printf("Close cluster after end of computation.\n");
  pi_cluster_close(&cluster_dev);

#if (defined(SAVE_PIC) & !defined(SKIP_ISP))
  printf("\nSaving RGB Image...\n");
  WriteImageToFile("../Out.ppm", WIDTH, HEIGHT, 3, ImageOut, RGB888_IO);
#endif

  if (!err)
    printf("Test succeed\n");
  else
    printf("Test failed\n");
  return err;

error_handler:
  printf("Test failed\n");
  return err;
}
