/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbPictureAnalysis_h
#define EbPictureAnalysis_h

#include "EbDefinitions.h"
#include "EbSystemResourceManager.h"
#include "EbNoiseExtractAVX2.h"

/**************************************
 * Context
 **************************************/
typedef struct PictureAnalysisContext
{
    EB_ALIGN(64) uint8_t            local_cache[64];
    EbFifo                     *resource_coordination_results_input_fifo_ptr;
    EbFifo                     *picture_analysis_results_output_fifo_ptr;
    EbPictureBufferDesc        *denoised_picture_ptr;
    EbPictureBufferDesc        *noise_picture_ptr;
    double                          pic_noise_variance_float;
} PictureAnalysisContext;

/***************************************
 * Extern Function Declaration
 ***************************************/
extern EbErrorType picture_analysis_context_ctor(
    EbPictureBufferDescInitData *input_picture_buffer_desc_init_data,
    EbBool                         denoise_flag,
    PictureAnalysisContext     **context_dbl_ptr,
    EbFifo                      *resource_coordination_results_input_fifo_ptr,
    EbFifo                      *picture_analysis_results_output_fifo_ptr);

extern void* picture_analysis_kernel(void *input_ptr);

#endif // EbPictureAnalysis_h
