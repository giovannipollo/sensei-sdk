// Copyright (c) 2022 GreenWaves Technologies
// SPDX-License-Identifier: Apache-2.0

#ifndef __ISPKERNEL_H__
#define __ISPKERNEL_H__

#include "AutoTilerLibTypes.h"
#include "ISP_BasicKernels.h"
#define _ISP_L1_Memory_SIZE 104960
#define _ISP_L2_Memory_SIZE 0
extern char *ISP_L1_Memory; /* Size given for generation: 110000 bytes, used: 104960 bytes */
extern char *ISP_L2_Memory; /* Size used for generation: 0 bytes */
extern void demosaic_image_HWC(unsigned char *In, unsigned char *Out);
extern void white_balance_HWCHistogram(unsigned char *__restrict__ In, unsigned char Percentile);
extern void white_balance_HWCNorm(unsigned char *__restrict__ In, unsigned char *__restrict__ Out);
extern void white_balance_HWC(unsigned char *__restrict__ In, unsigned char *__restrict__ Out,
                              unsigned char Percentile);
#endif
