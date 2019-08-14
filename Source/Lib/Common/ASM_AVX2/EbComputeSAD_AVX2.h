/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/
#ifndef EbComputeSAD_AVX2_h
#define EbComputeSAD_AVX2_h

#include "EbDefinitions.h"
#ifdef __cplusplus
extern "C" {
#endif

    uint32_t compute4x_m_sad_avx2_intrin(
        const uint8_t  *src,                      // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        const uint8_t  *ref,                      // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width);                         // input parameter, block width (N)

    uint32_t Compute4xMSadSub_AVX2_INTRIN(
        const uint8_t  *src,                       // input parameter, source samples Ptr
        uint32_t  src_stride,                      // input parameter, source stride
        const uint8_t  *ref,                      // input parameter, reference samples Ptr
        uint32_t  refStride,                      // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width);                         // input parameter, block width (N)

    uint32_t compute8x_m_sad_avx2_intrin(
        const uint8_t  *src,                      // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        const uint8_t  *ref,                      // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width);                         // input parameter, block width (N)

    uint32_t compute16x_m_sad_avx2_intrin(
        const uint8_t  *src,                      // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        const uint8_t  *ref,                      // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width);                         // input parameter, block width (N)

    uint32_t compute24x_m_sad_avx2_intrin(
        const uint8_t  *src,                      // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        const uint8_t  *ref,                      // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width);                         // input parameter, block width (N)

    uint32_t compute32x_m_sad_avx2_intrin(
        const uint8_t  *src,                      // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        const uint8_t  *ref,                      // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width);                         // input parameter, block width (N)

    uint32_t compute48x_m_sad_avx2_intrin(
        const uint8_t  *src,                      // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        const uint8_t  *ref,                      // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width);                         // input parameter, block width (N)

    uint32_t compute64x_m_sad_avx2_intrin(
        const uint8_t  *src,                      // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        const uint8_t  *ref,                      // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width);                         // input parameter, block width (N)

    void sad_loop_kernel_avx2_intrin(
        uint8_t  *src,                            // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        uint8_t  *ref,                            // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width,                          // input parameter, block width (N)
        uint64_t *best_sad,
        int16_t  *x_search_center,
        int16_t  *y_search_center,
        uint32_t  src_stride_raw,                 // input parameter, source stride (no line skipping)
        int16_t   search_area_width,
        int16_t   search_area_height);

    void sad_loop_kernel_avx2_hme_l0_intrin(
        uint8_t  *src,                            // input parameter, source samples Ptr
        uint32_t  src_stride,                     // input parameter, source stride
        uint8_t  *ref,                            // input parameter, reference samples Ptr
        uint32_t  ref_stride,                     // input parameter, reference stride
        uint32_t  height,                         // input parameter, block height (M)
        uint32_t  width,                          // input parameter, block width (N)
        uint64_t *best_sad,
        int16_t  *x_search_center,
        int16_t  *y_search_center,
        uint32_t  src_stride_raw,                 // input parameter, source stride (no line skipping)
        int16_t   search_area_width,
        int16_t   search_area_height);

#ifdef __cplusplus
}
#endif
#endif // EbComputeSAD_AVX2_h
