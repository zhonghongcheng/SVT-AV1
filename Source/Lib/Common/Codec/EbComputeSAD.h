/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbComputeSAD_h
#define EbComputeSAD_h

#include "EbDefinitions.h"

#include "EbCombinedAveragingSAD_Intrinsic_AVX2.h"
#include "EbComputeSAD_C.h"
#include "EbComputeSAD_SSE2.h"
#include "EbComputeSAD_SSE4_1.h"
#include "EbComputeSAD_AVX2.h"
#include "EbUtility.h"
#ifdef __cplusplus
extern "C" {
#endif

    /***************************************
    * Function Ptr Types
    ***************************************/
    typedef uint32_t(*EbSadKernelNxMType)(
        const uint8_t  *src,
        uint32_t  src_stride,
        const uint8_t  *ref,
        uint32_t  ref_stride,
        uint32_t  height,
        uint32_t  width);

    static void nxm_sad_kernel_void_func() {}

    typedef uint32_t(*EbCompute8x4SadType)(
        uint8_t  *src,                            // input parameter, source samples Ptr
        uint32_t  src_stride,                      // input parameter, source stride
        uint8_t  *ref,                            // input parameter, reference samples Ptr
        uint32_t  ref_stride);                     // input parameter, reference stride

    typedef uint32_t(*EB_COMPUTE8X8SAD_TYPE)(
        uint8_t  *src,                            // input parameter, source samples Ptr
        uint32_t  src_stride,                      // input parameter, source stride
        uint8_t  *ref,                            // input parameter, reference samples Ptr
        uint32_t  ref_stride);                     // input parameter, reference stride
    typedef void(*EbGetEightSad8x8)(
        uint8_t   *src,
        uint32_t   src_stride,
        uint8_t   *ref,
        uint32_t   ref_stride,
        uint32_t  *p_best_sad8x8,
        uint32_t  *p_best_mv8x8,
        uint32_t  *p_best_sad16x16,
        uint32_t  *p_best_mv16x16,
        uint32_t   mv,
        uint16_t  *p_sad16x16,
        EbBool     sub_sad);

    typedef void(*EbGetEightSad32x32)(
        uint16_t  *p_sad16x16,
        uint32_t  *p_best_sad32x32,
        uint32_t  *p_best_sad64x64,
        uint32_t  *p_best_mv32x32,
        uint32_t  *p_best_mv64x64,
        uint32_t   mv);

    /***************************************
    * Function Tables
    ***************************************/
    static EbSadKernelNxMType FUNC_TABLE nxm_sad_kernel_sub_sampled_func_ptr_array[ASM_TYPE_TOTAL][17] =   // [asm_type][SAD - block height]
    {
        // NON_AVX2
        {
            /*0 4xM  */ fast_loop_nx_m_sad_kernel,
            /*1 8xM  */ fast_loop_nx_m_sad_kernel,
            /*2 16xM */ fast_loop_nx_m_sad_kernel,
            /*3 24xM */ fast_loop_nx_m_sad_kernel,
            /*4 32xM */ fast_loop_nx_m_sad_kernel,
            /*5      */ 0,
            /*6 48xM */ fast_loop_nx_m_sad_kernel,
            /*7      */ 0,
            /*8 64xM */ fast_loop_nx_m_sad_kernel,
            0,0,0,0,0,0,0,fast_loop_nx_m_sad_kernel
        },
        // AVX2
        {
            /*0 4xM  */ Compute4xMSadSub_AVX2_INTRIN,
            /*1 8xM  */ compute8x_m_sad_avx2_intrin,
            /*2 16xM */ compute16x_m_sad_avx2_intrin,
            /*3 24xM */ fast_loop_nx_m_sad_kernel,
            /*4 32xM */ compute32x_m_sad_avx2_intrin,
            /*5      */ 0,
            /*6 48xM */ fast_loop_nx_m_sad_kernel,
            /*7      */ 0,
            /*8 64xM */ compute64x_m_sad_avx2_intrin,
            0,0,0,0,0,0,0,fast_loop_nx_m_sad_kernel
        },
    };
    static EbSadKernelNxMType FUNC_TABLE nxm_sad_kernel_func_ptr_array[ASM_TYPE_TOTAL][9] =   // [asm_type][SAD - block height]
    {
        // NON_AVX2
        {
            /*0 4xM  */ fast_loop_nx_m_sad_kernel,
            /*1 8xM  */ fast_loop_nx_m_sad_kernel,
            /*2 16xM */ fast_loop_nx_m_sad_kernel,
            /*3 24xM */ fast_loop_nx_m_sad_kernel,
            /*4 32xM */ fast_loop_nx_m_sad_kernel,
            /*5      */ fast_loop_nx_m_sad_kernel,  // size not supported in asm
            /*6 48xM */ fast_loop_nx_m_sad_kernel,
            /*7      */ fast_loop_nx_m_sad_kernel,  // size not supported in asm
            /*8 64xM */ fast_loop_nx_m_sad_kernel
        },
        // AVX2
        {
            /*0 4xM  */ compute4x_m_sad_avx2_intrin,
            /*1 8xM  */ compute8x_m_sad_avx2_intrin,
            /*2 16xM */ compute16x_m_sad_avx2_intrin,//compute16x_m_sad_avx2_intrin is slower than the SSE2 version
            /*3 24xM */ compute24x_m_sad_avx2_intrin,
            /*4 32xM */ compute32x_m_sad_avx2_intrin,
            /*5      */ (EbSadKernelNxMType)nxm_sad_kernel_void_func,
            /*6 48xM */ compute48x_m_sad_avx2_intrin,
            /*7      */ (EbSadKernelNxMType)nxm_sad_kernel_void_func,
            /*8 64xM */ compute64x_m_sad_avx2_intrin,
        },
    };

    static EbGetEightSad8x8 FUNC_TABLE get_eight_horizontal_search_point_results_8x8_16x16_func_ptr_array[ASM_TYPE_TOTAL] =
    {
        // NON_AVX2
        get_eight_horizontal_search_point_results_8x8_16x16_pu_sse41_intrin,
        // AVX2
        get_eight_horizontal_search_point_results_8x8_16x16_pu_avx2_intrin,
    };

    static EbGetEightSad32x32 FUNC_TABLE get_eight_horizontal_search_point_results_32x32_64x64_func_ptr_array[ASM_TYPE_TOTAL] =
    {
        // NON_AVX2
        get_eight_horizontal_search_point_results_32x32_64x64_pu_sse41_intrin,
        // AVX2
        get_eight_horizontal_search_point_results_32x32_64x64_pu_avx2_intrin,
    };

#ifdef __cplusplus
}
#endif
#endif // EbComputeSAD_h
