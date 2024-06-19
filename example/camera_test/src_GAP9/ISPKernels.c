// Copyright (c) 2022 GreenWaves Technologies
// SPDX-License-Identifier: Apache-2.0

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include "ISPKernels.h"
#ifdef __EMUL__
unsigned int __L3_Read, __L3_Write, __L2_Read, __L2_Write;
#endif
L1_CL_MEM AT_L1_POINTER ISP_L1_Memory;
L2_MEM AT_L2_POINTER ISP_L2_Memory;
void demosaic_image_HWC(unsigned char *In, unsigned char *Out)

{
  /* Shared L1: 104960 bytes, L2 buffer: 0 bytes */
  /* Local variables used by this kernel */
  AT_L2_EVENT _DmaR_Evt1, *DmaR_Evt1 = &_DmaR_Evt1;
  AT_L2_EVENT _DmaW_Evt1, *DmaW_Evt1 = &_DmaW_Evt1;
  KerDeMosaic_HWC_ArgT S_KerArg0, *KerArg0 = &S_KerArg0;

  /* Iteration space related variables */
  int T0Ind, T0Ind_Total = 0, T0Ind_Last, T0Ind_NextLast;
  /* User kernel arguments related variables */
  unsigned int _N_In;
  unsigned int _SN_In;
  unsigned int _C_Out;
  unsigned int _SP_Out, _SC_Out;
  /*============================= Ker Arg Iter Spaces =========================================
  User Kernel Iteration Space:
          [Tile0 Dim: 24]
  Ker Arg: In, Tiled Space: Tile0
          Min Pipe Depth: 0, Max Pipe Depth: 1
          KerArgItSpace: 24 logical tiles, 24 physical tiles
                  @ 0 (Total Size: 307200 )[Tile0, 24:[640x21, 22:640x22, 640x21], 1]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 24:[640x21, 22:640x22, 640x21], 1]
          Tile0: [0, 13440, 13440], Tile1: [12160, 14080, 14080], Tile2; [24960, 14080, 14080]
  Ker Arg: Out, Tiled Space: Tile0
          Min Pipe Depth: -1, Max Pipe Depth: 1
          KerArgItSpace: 24 logical tiles, 24 physical tiles
                  @ 28160 (Total Size: 921600 )[Tile0, 24:[640x20, 22:640x20, 640x20], 3]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 24:[640x20, 22:640x20, 640x20], 3]
          Tile0: [0, 38400, 38400], Tile1: [38400, 38400, 38400], Tile2; [76800, 38400, 38400]
  ======================== End Ker Arg Iter Spaces =========================================*/
  /*=========================== Call Kernel, Invariant assignment =====================*/
  KerArg0->W = (unsigned int)(640);
  KerArg0->H = (unsigned int)(20);
  /*================================= Read Tiles Prolog ===============================*/
  _C_Out = 0;
  _SC_Out = 38400;
  _SP_Out = 0;
  AT_L2_COPY(0, ((AT_L2_EXT_ADDR_TYPE)In + 0), ((AT_L2_INT_ADDR_TYPE)ISP_L1_Memory + 0 + 0), 13440, 0, DmaR_Evt1);
  _N_In = 0;
  /*============================= End Read Tiles Prolog ===============================*/
  for (T0Ind = 0; T0Ind < 24; T0Ind++, T0Ind_Total++) { /* Iteration on Tile0 */
    int T0Ind_Last = (T0Ind == 23), T0Ind_NextLast = ((T0Ind + 1) == 23);
    /*================================= Prepare Tiles ===================================*/
    _SN_In = 0;
    if (!(T0Ind_Last)) {
      _N_In = _N_In + (12800 - (640 * (T0Ind == 0)));
      _SN_In = ((T0Ind_NextLast) ? 13440 : 14080);
    }
    /*============================= End Prepare Tiles ===================================*/
    /*================================= Read Tiles ======================================*/
    AT_L2_WAIT(0, DmaR_Evt1); /* Wait previous DMA read In */
    if (_SN_In) {
      AT_L2_COPY(0, ((AT_L2_EXT_ADDR_TYPE)In + _N_In),
                 ((AT_L2_INT_ADDR_TYPE)ISP_L1_Memory + 0 + 14080 * ((T0Ind_Total + 1) % 2)), 1 * (_SN_In), 0,
                 DmaR_Evt1);
    }
    /*============================= End Read Tiles ======================================*/
    /*====================== Call Kernel LOC_LOOP =========================*/
    KerArg0->In = (void *__restrict__)(ISP_L1_Memory + 0 + 14080 * ((T0Ind_Total) % 2));
    KerArg0->Out = (unsigned char *__restrict__)(ISP_L1_Memory + 28160 + 38400 * ((T0Ind_Total) % 2));
    KerArg0->isTileFirst = (unsigned int)((T0Ind == 0));
    KerArg0->isTileLast = (unsigned int)((T0Ind_Last));
    AT_FORK(gap_ncore(), (void *)KerDeMosaic_HWC_BGGR, (void *)KerArg0);
    __CALL(KerDeMosaic_HWC_BGGR, KerArg0);
    /*================================= Write Tiles =====================================*/
    if (_SP_Out)
      AT_L2_WAIT(0, DmaW_Evt1); /* Wait previous DMA write Out */
    AT_L2_COPY(0, ((AT_L2_EXT_ADDR_TYPE)Out + _C_Out),
               ((AT_L2_INT_ADDR_TYPE)ISP_L1_Memory + 28160 + 38400 * ((T0Ind_Total) % 2)), _SC_Out, 1, DmaW_Evt1);
    /*============================= End Write Tiles =====================================*/
    /*================================= Update Arg Pipeline =============================*/
    _SP_Out = _SC_Out;
    /*============================= End Update Arg Pipeline =============================*/
    /*================================= Prepare Tiles ===================================*/
    _SC_Out = 0;
    if (!(T0Ind_Last)) {
      _C_Out = _C_Out + (38400);
      _SC_Out = (38400);
    }
    /*============================= End Prepare Tiles ===================================*/
  } /* End iteration on Tile0 */
  /*================================ Write Tiles Epilog ===============================*/
  AT_L2_WAIT(0, DmaW_Evt1); /* Wait previous DMA write Out */
                            /*============================ End Write Tiles Epilog ===============================*/
}
void white_balance_HWCHistogram(unsigned char *__restrict__ In, unsigned char Percentile)

{
  /* Shared L1: 86028 bytes, L2 buffer: 0 bytes */
  /* Local variables used by this kernel */
  AT_L2_EVENT _DmaR_Evt1, *DmaR_Evt1 = &_DmaR_Evt1;
  KerWB_hist_prolog_HWC_ArgT S_KerArg0, *KerArg0 = &S_KerArg0;
  KerWB_hist_HWC_ArgT S_KerArg1, *KerArg1 = &S_KerArg1;
  KerWB_hist_epilog_HWC_ArgT S_KerArg2, *KerArg2 = &S_KerArg2;

  /* Iteration space related variables */
  int T0Ind, T0Ind_Total = 0, T0Ind_Last, T0Ind_NextLast;
  int D0Ind, D0Ind_Last;
  /* User kernel arguments related variables */
  unsigned int _N_In;
  unsigned int _SN_In;
  /*============================= Ker Arg Iter Spaces =========================================
  User Kernel Iteration Space:
          [Tile0 Dim: 30][D0 Dim: Init: 1, Tiled: 1]
  Ker Arg: In, Tiled Space: Tile0
          Min Pipe Depth: 0, Max Pipe Depth: 1
          KerArgItSpace: 30 logical tiles, 30 physical tiles
                  @ 12 (Total Size: 921600 )[Tile0, 30:[640x16, 28:640x16, 640x16], 3]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 30:[640x16, 28:640x16, 640x16], 3]
          Tile0: [0, 30720, 30720], Tile1: [30720, 30720, 30720], Tile2; [61440, 30720, 30720]
  Ker Arg: G_norm, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 30 logical tiles, 1 physical tiles
                  @ 4 (Total Size: 1 )[Tile0, 30:[1x1, 28:1x1, 1x1], 1]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 30:[1x1, 28:1x1, 1x1], 1]
          Tile0: [0, 1, 1], Tile1: [0, 1, 1], Tile2; [0, 1, 1]
  Ker Arg: B_norm, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 30 logical tiles, 1 physical tiles
                  @ 8 (Total Size: 1 )[Tile0, 30:[1x1, 28:1x1, 1x1], 1]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 30:[1x1, 28:1x1, 1x1], 1]
          Tile0: [0, 1, 1], Tile1: [0, 1, 1], Tile2; [0, 1, 1]
  Ker Arg: R_norm, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 30 logical tiles, 1 physical tiles
                  @ 0 (Total Size: 1 )[Tile0, 30:[1x1, 28:1x1, 1x1], 1]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 30:[1x1, 28:1x1, 1x1], 1]
          Tile0: [0, 1, 1], Tile1: [0, 1, 1], Tile2; [0, 1, 1]
  Ker Arg: R_hist, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 30 logical tiles, 1 physical tiles
                  @ 61452 (Total Size: 8192 )[Tile0, 30:[8x256, 28:8x256, 8x256], 4]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 30:[8x256, 28:8x256, 8x256], 4]
          Tile0: [0, 8192, 8192], Tile1: [0, 8192, 8192], Tile2; [0, 8192, 8192]
  Ker Arg: G_hist, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 30 logical tiles, 1 physical tiles
                  @ 69644 (Total Size: 8192 )[Tile0, 30:[8x256, 28:8x256, 8x256], 4]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 30:[8x256, 28:8x256, 8x256], 4]
          Tile0: [0, 8192, 8192], Tile1: [0, 8192, 8192], Tile2; [0, 8192, 8192]
  Ker Arg: B_hist, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 30 logical tiles, 1 physical tiles
                  @ 77836 (Total Size: 8192 )[Tile0, 30:[8x256, 28:8x256, 8x256], 4]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 30:[8x256, 28:8x256, 8x256], 4]
          Tile0: [0, 8192, 8192], Tile1: [0, 8192, 8192], Tile2; [0, 8192, 8192]
  ======================== End Ker Arg Iter Spaces =========================================*/
  /*=========================== Call Kernel, Invariant assignment =====================*/
  KerArg1->W = (unsigned int)(640);
  KerArg1->H = (unsigned int)(16);
  KerArg2->Percentile = (unsigned char)(Percentile);
  KerArg2->IMG_W = (unsigned int)(640);
  KerArg2->IMG_H = (unsigned int)(480);
  /*================================= Read Tiles Prolog ===============================*/
  AT_L2_COPY(0, ((AT_L2_EXT_ADDR_TYPE)In + 0), ((AT_L2_INT_ADDR_TYPE)ISP_L1_Memory + 12 + 0), 30720, 0, DmaR_Evt1);
  _N_In = 0;
  /*============================= End Read Tiles Prolog ===============================*/
  /*====================== Call Kernel LOC_LOOP_PROLOG =========================*/
  KerArg0->R_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 61452);
  KerArg0->G_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 69644);
  KerArg0->B_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 77836);
  AT_FORK(gap_ncore(), (void *)KerWB_hist_prolog_HWC, (void *)KerArg0);
  __CALL(KerWB_hist_prolog_HWC, KerArg0);
  for (T0Ind = 0; T0Ind < 30; T0Ind++, T0Ind_Total++) { /* Iteration on Tile0 */
    int T0Ind_Last = (T0Ind == 29), T0Ind_NextLast = ((T0Ind + 1) == 29);
    /*================================= Prepare Tiles ===================================*/
    _SN_In = 0;
    if (!(T0Ind_Last)) {
      _N_In = _N_In + (30720);
      _SN_In = (30720);
    }
    /*============================= End Prepare Tiles ===================================*/
    /*================================= Read Tiles ======================================*/
    AT_L2_WAIT(0, DmaR_Evt1); /* Wait previous DMA read In */
    if (_SN_In) {
      AT_L2_COPY(0, ((AT_L2_EXT_ADDR_TYPE)In + _N_In),
                 ((AT_L2_INT_ADDR_TYPE)ISP_L1_Memory + 12 + 30720 * ((T0Ind_Total + 1) % 2)), 1 * (_SN_In), 0,
                 DmaR_Evt1);
    }
    /*============================= End Read Tiles ======================================*/
    { /* Single iteration on D0 */
      int D0Ind_Last = 1;
    } /* End iteration on D0 */
    /*====================== Call Kernel LOC_LOOP =========================*/
    KerArg1->In = (unsigned char *__restrict__)(ISP_L1_Memory + 12 + 30720 * ((T0Ind_Total) % 2));
    KerArg1->R_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 61452);
    KerArg1->G_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 69644);
    KerArg1->B_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 77836);
    AT_FORK(gap_ncore(), (void *)KerWB_hist_HWC, (void *)KerArg1);
    __CALL(KerWB_hist_HWC, KerArg1);
    /*================================= Update Arg Pipeline =============================*/
    /*============================= End Update Arg Pipeline =============================*/
  } /* End iteration on Tile0 */
  /*====================== Call Kernel LOC_LOOP_EPILOG =========================*/
  KerArg2->R_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 61452);
  KerArg2->G_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 69644);
  KerArg2->B_hist = (unsigned int *__restrict__)(ISP_L1_Memory + 77836);
  KerArg2->R_norm = (unsigned char *)(ISP_L1_Memory + 0);
  KerArg2->G_norm = (unsigned char *)(ISP_L1_Memory + 4);
  KerArg2->B_norm = (unsigned char *)(ISP_L1_Memory + 8);
  KerWB_hist_epilog_HWC(KerArg2);
}
void white_balance_HWCNorm(unsigned char *__restrict__ In, unsigned char *__restrict__ Out)

{
  /* Shared L1: 92172 bytes, L2 buffer: 0 bytes */
  /* Local variables used by this kernel */
  AT_L2_EVENT _DmaR_Evt1, *DmaR_Evt1 = &_DmaR_Evt1;
  AT_L2_EVENT _DmaW_Evt1, *DmaW_Evt1 = &_DmaW_Evt1;
  KerWB_norm_HWC_ArgT S_KerArg0, *KerArg0 = &S_KerArg0;

  /* Iteration space related variables */
  int T0Ind, T0Ind_Total = 0, T0Ind_Last, T0Ind_NextLast;
  /* User kernel arguments related variables */
  unsigned int _N_In;
  unsigned int _SN_In;
  unsigned int _C_Out;
  unsigned int _SP_Out, _SC_Out;
  /*============================= Ker Arg Iter Spaces =========================================
  User Kernel Iteration Space:
          [Tile0 Dim: 20]
  Ker Arg: In, Tiled Space: Tile0
          Min Pipe Depth: 0, Max Pipe Depth: 1
          KerArgItSpace: 20 logical tiles, 20 physical tiles
                  @ 12 (Total Size: 921600 )[Tile0, 20:[640x24, 18:640x24, 640x24], 3]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 20:[640x24, 18:640x24, 640x24], 3]
          Tile0: [0, 46080, 46080], Tile1: [46080, 46080, 46080], Tile2; [92160, 46080, 46080]
  Ker Arg: G_norm, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 20 logical tiles, 1 physical tiles
                  @ 4 (Total Size: 1 )[Tile0, 20:[1x1, 18:1x1, 1x1], 1]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 20:[1x1, 18:1x1, 1x1], 1]
          Tile0: [0, 1, 1], Tile1: [0, 1, 1], Tile2; [0, 1, 1]
  Ker Arg: B_norm, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 20 logical tiles, 1 physical tiles
                  @ 8 (Total Size: 1 )[Tile0, 20:[1x1, 18:1x1, 1x1], 1]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 20:[1x1, 18:1x1, 1x1], 1]
          Tile0: [0, 1, 1], Tile1: [0, 1, 1], Tile2; [0, 1, 1]
  Ker Arg: R_norm, Tiled Space: Buffer
          Min Pipe Depth: 0, Max Pipe Depth: 0
          KerArgItSpace: 20 logical tiles, 1 physical tiles
                  @ 0 (Total Size: 1 )[Tile0, 20:[1x1, 18:1x1, 1x1], 1]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 20:[1x1, 18:1x1, 1x1], 1]
          Tile0: [0, 1, 1], Tile1: [0, 1, 1], Tile2; [0, 1, 1]
  Ker Arg: Out, Tiled Space: Tile0
          Min Pipe Depth: -1, Max Pipe Depth: 1
          KerArgItSpace: 20 logical tiles, 20 physical tiles
                  @ 12 (Total Size: 921600 )[Tile0, 20:[640x24, 18:640x24, 640x24], 3]
          KerArgItSpace (User Kernel Iter Order):
                  [Tile0, 20:[640x24, 18:640x24, 640x24], 3]
          Tile0: [0, 46080, 46080], Tile1: [46080, 46080, 46080], Tile2; [92160, 46080, 46080]
  ======================== End Ker Arg Iter Spaces =========================================*/
  /*=========================== Call Kernel, Invariant assignment =====================*/
  KerArg0->W = (unsigned int)(640);
  KerArg0->H = (unsigned int)(24);
  /*================================= Read Tiles Prolog ===============================*/
  AT_L2_COPY(0, ((AT_L2_EXT_ADDR_TYPE)In + 0), ((AT_L2_INT_ADDR_TYPE)ISP_L1_Memory + 12 + 0), 46080, 0, DmaR_Evt1);
  _N_In = 0;
  _C_Out = 0;
  _SC_Out = 46080;
  _SP_Out = 0;
  /*============================= End Read Tiles Prolog ===============================*/
  for (T0Ind = 0; T0Ind < 20; T0Ind++, T0Ind_Total++) { /* Iteration on Tile0 */
    int T0Ind_Last = (T0Ind == 19), T0Ind_NextLast = ((T0Ind + 1) == 19);
    /*================================= Prepare Tiles ===================================*/
    _SN_In = 0;
    if (!(T0Ind_Last)) {
      _N_In = _N_In + (46080);
      _SN_In = (46080);
    }
    /*============================= End Prepare Tiles ===================================*/
    /*================================= Read Tiles ======================================*/
    AT_L2_WAIT(0, DmaR_Evt1); /* Wait previous DMA read In */
    if (_SN_In) {
      AT_L2_COPY(0, ((AT_L2_EXT_ADDR_TYPE)In + _N_In),
                 ((AT_L2_INT_ADDR_TYPE)ISP_L1_Memory + 12 + 46080 * ((T0Ind_Total + 1) % 2)), 1 * (_SN_In), 0,
                 DmaR_Evt1);
    }
    /*============================= End Read Tiles ======================================*/
    /*====================== Call Kernel LOC_LOOP =========================*/
    KerArg0->In = (unsigned char *__restrict__)(ISP_L1_Memory + 12 + 46080 * ((T0Ind_Total) % 2));
    KerArg0->R_norm = (unsigned char *)(ISP_L1_Memory + 0);
    KerArg0->G_norm = (unsigned char *)(ISP_L1_Memory + 4);
    KerArg0->B_norm = (unsigned char *)(ISP_L1_Memory + 8);
    AT_FORK(gap_ncore(), (void *)KerWB_norm_HWC, (void *)KerArg0);
    __CALL(KerWB_norm_HWC, KerArg0);
    /*================================= Write Tiles =====================================*/
    if (_SP_Out)
      AT_L2_WAIT(0, DmaW_Evt1); /* Wait previous DMA write Out */
    AT_L2_COPY(0, ((AT_L2_EXT_ADDR_TYPE)Out + _C_Out),
               ((AT_L2_INT_ADDR_TYPE)ISP_L1_Memory + 12 + 46080 * ((T0Ind_Total) % 2)), _SC_Out, 1, DmaW_Evt1);
    /*============================= End Write Tiles =====================================*/
    /*================================= Update Arg Pipeline =============================*/
    _SP_Out = _SC_Out;
    /*============================= End Update Arg Pipeline =============================*/
    /*================================= Prepare Tiles ===================================*/
    _SC_Out = 0;
    if (!(T0Ind_Last)) {
      _C_Out = _C_Out + (46080);
      _SC_Out = (46080);
    }
    /*============================= End Prepare Tiles ===================================*/
  } /* End iteration on Tile0 */
  /*================================ Write Tiles Epilog ===============================*/
  AT_L2_WAIT(0, DmaW_Evt1); /* Wait previous DMA write Out */
                            /*============================ End Write Tiles Epilog ===============================*/
}
void white_balance_HWC(unsigned char *__restrict__ In, unsigned char *__restrict__ Out, unsigned char Percentile)

{
  /*
          KerArg:                             In, Total Size:   307200, Dimension:   1, Items:   307200, ItemSize:  3
          KerArg:                            Out, Total Size:   307200, Dimension:   1, Items:   307200, ItemSize:  3
          KerArg:                     Percentile, Total Size:        1, Dimension:   1, Items:        1, ItemSize:  1
  */
  white_balance_HWCHistogram((unsigned char *__restrict__)(In), (unsigned char)(Percentile));
  white_balance_HWCNorm((unsigned char *__restrict__)(In), (unsigned char *__restrict__)(Out));
}

#pragma GCC diagnostic pop
