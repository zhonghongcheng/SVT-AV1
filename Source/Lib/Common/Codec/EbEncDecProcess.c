/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

/*
* Copyright (c) 2016, Alliance for Open Media. All rights reserved
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at www.aomedia.org/license/software. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at www.aomedia.org/license/patent.
*/

#include <stdlib.h>

#include "EbEncDecTasks.h"
#include "EbEncDecResults.h"
#include "EbDefinitions.h"
#include "EbCodingLoop.h"
#include "EbSvtAv1ErrorCodes.h"
#include "EbUtility.h"
#include "grainSynthesis.h"
#if MPMD_SB
#include "EbModeDecisionConfiguration.h"
#endif

void av1_cdef_search(
    EncDecContext                *context_ptr,
    SequenceControlSet           *sequence_control_set_ptr,
    PictureControlSet            *picture_control_set_ptr
);

void av1_cdef_frame(
    EncDecContext                *context_ptr,
    SequenceControlSet           *sequence_control_set_ptr,
    PictureControlSet            *pCs
);

void av1_cdef_search16bit(
    EncDecContext                *context_ptr,
    SequenceControlSet           *sequence_control_set_ptr,
    PictureControlSet            *picture_control_set_ptr
);
void av1_cdef_frame16bit(
    uint8_t is16bit,
    SequenceControlSet           *sequence_control_set_ptr,
    PictureControlSet            *pCs
);

void av1_add_film_grain(EbPictureBufferDesc *src,
    EbPictureBufferDesc *dst,
    aom_film_grain_t *film_grain_ptr);

void av1_loop_restoration_save_boundary_lines(const Yv12BufferConfig *frame, Av1Common *cm, int32_t after_cdef);
void av1_pick_filter_restoration(const Yv12BufferConfig *src, Yv12BufferConfig * trial_frame_rst /*Av1Comp *cpi*/, Macroblock *x, Av1Common *const cm);
void av1_loop_restoration_filter_frame(Yv12BufferConfig *frame, Av1Common *cm, int32_t optimized_lr);

const int16_t encMinDeltaQpWeightTab[MAX_TEMPORAL_LAYERS] = { 100, 100, 100, 100, 100, 100 };
const int16_t encMaxDeltaQpWeightTab[MAX_TEMPORAL_LAYERS] = { 100, 100, 100, 100, 100, 100 };

const int8_t  encMinDeltaQpISliceTab[4] = { -5, -5, -3, -2 };

const int8_t  encMinDeltaQpTab[4][MAX_TEMPORAL_LAYERS] = {
    { -4, -2, -2, -1, -1, -1 },
    { -4, -2, -2, -1, -1, -1 },
    { -3, -1, -1, -1, -1, -1 },
    { -1, -0, -0, -0, -0, -0 },
};

const int8_t  encMaxDeltaQpTab[4][MAX_TEMPORAL_LAYERS] = {
    { 4, 5, 5, 5, 5, 5 },
    { 4, 5, 5, 5, 5, 5 },
    { 4, 5, 5, 5, 5, 5 },
    { 4, 5, 5, 5, 5, 5 }
};
#if MOVE_TX_LEVELS_SIGNAL_UNDER_CTX
#define FC_SKIP_TX_SR_TH025                     125 // Fast cost skip tx search threshold.
#define FC_SKIP_TX_SR_TH010                     110 // Fast cost skip tx search threshold.
#endif
/******************************************************
 * Enc Dec Context Constructor
 ******************************************************/
EbErrorType enc_dec_context_ctor(
    EncDecContext        **context_dbl_ptr,
    EbFifo                *mode_decision_configuration_input_fifo_ptr,
    EbFifo                *packetization_output_fifo_ptr,
    EbFifo                *feedback_fifo_ptr,
    EbFifo                *picture_demux_fifo_ptr,
    EbBool                  is16bit,
    EbColorFormat           color_format,
    uint32_t                max_input_luma_width,
    uint32_t                max_input_luma_height){
    (void)max_input_luma_width;
    (void)max_input_luma_height;
    EbErrorType return_error = EB_ErrorNone;
    EncDecContext *context_ptr;
    EB_MALLOC(EncDecContext*, context_ptr, sizeof(EncDecContext), EB_N_PTR);
    *context_dbl_ptr = context_ptr;

    context_ptr->is16bit = is16bit;
    context_ptr->color_format = color_format;

    // Input/Output System Resource Manager FIFOs
    context_ptr->mode_decision_input_fifo_ptr = mode_decision_configuration_input_fifo_ptr;
    context_ptr->enc_dec_output_fifo_ptr = packetization_output_fifo_ptr;
    context_ptr->enc_dec_feedback_fifo_ptr = feedback_fifo_ptr;
    context_ptr->picture_demux_output_fifo_ptr = picture_demux_fifo_ptr;

    // Trasform Scratch Memory
    EB_MALLOC(int16_t*, context_ptr->transform_inner_array_ptr, 3152, EB_N_PTR); //refer to EbInvTransform_SSE2.as. case 32x32
    // MD rate Estimation tables
    EB_MALLOC(MdRateEstimationContext*, context_ptr->md_rate_estimation_ptr, sizeof(MdRateEstimationContext), EB_N_PTR);

    // Prediction Buffer
    {
        EbPictureBufferDescInitData initData;

        initData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
        initData.max_width = SB_STRIDE_Y;
        initData.max_height = SB_STRIDE_Y;
        initData.bit_depth = EB_8BIT;
        initData.left_padding = 0;
        initData.right_padding = 0;
        initData.top_padding = 0;
        initData.bot_padding = 0;
        initData.split_mode = EB_FALSE;
        initData.color_format = color_format;

        context_ptr->input_sample16bit_buffer = (EbPictureBufferDesc *)EB_NULL;
        if (is16bit) {
            initData.bit_depth = EB_16BIT;

            return_error = eb_picture_buffer_desc_ctor(
                (EbPtr*)&context_ptr->input_sample16bit_buffer,
                (EbPtr)&initData);
            if (return_error == EB_ErrorInsufficientResources)
                return EB_ErrorInsufficientResources;
        }
    }

    // Scratch Coeff Buffer
    {
        EbPictureBufferDescInitData initData;

        initData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
        initData.max_width = SB_STRIDE_Y;
        initData.max_height = SB_STRIDE_Y;
        initData.bit_depth = EB_16BIT;
        initData.color_format = color_format;
        initData.left_padding = 0;
        initData.right_padding = 0;
        initData.top_padding = 0;
        initData.bot_padding = 0;
        initData.split_mode = EB_FALSE;

        EbPictureBufferDescInitData init32BitData;

        init32BitData.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
        init32BitData.max_width = SB_STRIDE_Y;
        init32BitData.max_height = SB_STRIDE_Y;
        init32BitData.bit_depth = EB_32BIT;
        init32BitData.color_format = color_format;
        init32BitData.left_padding = 0;
        init32BitData.right_padding = 0;
        init32BitData.top_padding = 0;
        init32BitData.bot_padding = 0;
        init32BitData.split_mode = EB_FALSE;
        return_error = eb_picture_buffer_desc_ctor(
            (EbPtr*)&context_ptr->inverse_quant_buffer,
            (EbPtr)&init32BitData);

        if (return_error == EB_ErrorInsufficientResources)
            return EB_ErrorInsufficientResources;
        return_error = eb_picture_buffer_desc_ctor(
            (EbPtr*)&context_ptr->transform_buffer,
            (EbPtr)&init32BitData);
        if (return_error == EB_ErrorInsufficientResources)
            return EB_ErrorInsufficientResources;
        return_error = eb_picture_buffer_desc_ctor(
            (EbPtr*)&context_ptr->residual_buffer,
            (EbPtr)&initData);
        if (return_error == EB_ErrorInsufficientResources)
            return EB_ErrorInsufficientResources;
    }

    // Intra Reference Samples
    return_error = intra_reference_samples_ctor(&context_ptr->intra_ref_ptr);
    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    context_ptr->intra_ref_ptr16 = (IntraReference16bitSamples *)EB_NULL;
    if (is16bit) {
        return_error = intra_reference16bit_samples_ctor(&context_ptr->intra_ref_ptr16);
        if (return_error == EB_ErrorInsufficientResources)
            return EB_ErrorInsufficientResources;
    }
    // Mode Decision Context
    return_error = mode_decision_context_ctor(&context_ptr->md_context, color_format, 0, 0);

    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    // Second Stage ME Context
    if (return_error == EB_ErrorInsufficientResources)
        return EB_ErrorInsufficientResources;
    context_ptr->md_context->enc_dec_context_ptr = context_ptr;

    return EB_ErrorNone;
}

/**************************************************
 * Reset Mode Decision Neighbor Arrays
 *************************************************/
static void ResetEncodePassNeighborArrays(PictureControlSet *picture_control_set_ptr)
{
    neighbor_array_unit_reset(picture_control_set_ptr->ep_intra_luma_mode_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_intra_chroma_mode_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_mv_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_skip_flag_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_mode_type_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_leaf_depth_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_luma_recon_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_cb_recon_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_cr_recon_neighbor_array);
#if DC_SIGN_CONTEXT_EP
    neighbor_array_unit_reset(picture_control_set_ptr->ep_luma_dc_sign_level_coeff_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_cb_dc_sign_level_coeff_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_cr_dc_sign_level_coeff_neighbor_array);
#endif
#if !OPT_LOSSLESS_0
    neighbor_array_unit_reset(picture_control_set_ptr->amvp_mv_merge_mv_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->amvp_mv_merge_mode_type_neighbor_array);
#endif
    return;
}

/**************************************************
 * Reset Coding Loop
 **************************************************/
static void ResetEncDec(
    EncDecContext         *context_ptr,
    PictureControlSet     *picture_control_set_ptr,
    SequenceControlSet    *sequence_control_set_ptr,
    uint32_t                   segment_index)
{
#if !ENABLE_CDF_UPDATE
    EB_SLICE                     slice_type;
    MdRateEstimationContext   *md_rate_estimation_array;
#endif
    context_ptr->is16bit = (EbBool)(sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);

    // QP
    //context_ptr->qp          = picture_control_set_ptr->parent_pcs_ptr->tilePtrArray[tileIndex]->tileQp;
#if ADD_DELTA_QP_SUPPORT
#if QPM
    uint16_t picture_qp = picture_control_set_ptr->picture_qp;
    context_ptr->qp = picture_qp;
    context_ptr->qp_index = picture_control_set_ptr->parent_pcs_ptr->delta_q_present_flag ? (uint8_t)quantizer_to_qindex[context_ptr->qp] : (uint8_t)picture_control_set_ptr->parent_pcs_ptr->base_qindex; ; // AMIR to check
#else
    uint16_t picture_qp = picture_control_set_ptr->parent_pcs_ptr->base_qindex;
    context_ptr->qp = picture_qp;
    context_ptr->qp_index = context_ptr->qp;
#endif
#else
    context_ptr->qp = picture_control_set_ptr->picture_qp;
#endif
    // Asuming cb and cr offset to be the same for chroma QP in both slice and pps for lambda computation

    context_ptr->chroma_qp = context_ptr->qp;

    // Lambda Assignement
    context_ptr->qp_index = (uint8_t)picture_control_set_ptr->parent_pcs_ptr->base_qindex;
    (*av1_lambda_assignment_function_table[picture_control_set_ptr->parent_pcs_ptr->pred_structure])(
#if LAMBDA_TUNING
        picture_control_set_ptr->temporal_layer_index,
#endif
        &context_ptr->fast_lambda,
        &context_ptr->full_lambda,
        &context_ptr->fast_chroma_lambda,
        &context_ptr->full_chroma_lambda,
        (uint8_t)picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr->bit_depth,
        context_ptr->qp_index);

#if ENABLE_CDF_UPDATE
    context_ptr->md_rate_estimation_ptr = picture_control_set_ptr->md_rate_estimation_array;
#else
    // Slice Type
    slice_type =
        (picture_control_set_ptr->parent_pcs_ptr->idr_flag == EB_TRUE) ? I_SLICE :
        picture_control_set_ptr->slice_type;
    // Increment the MD Rate Estimation array pointer to point to the right address based on the QP and slice type
    md_rate_estimation_array = (MdRateEstimationContext*)sequence_control_set_ptr->encode_context_ptr->md_rate_estimation_array;
#if ADD_DELTA_QP_SUPPORT
    md_rate_estimation_array += slice_type * TOTAL_NUMBER_OF_QP_VALUES + picture_control_set_ptr->parent_pcs_ptr->picture_qp;
#else
    md_rate_estimation_array += slice_type * TOTAL_NUMBER_OF_QP_VALUES + context_ptr->qp;
#endif
    // Reset MD rate Estimation table to initial values by copying from md_rate_estimation_array

    context_ptr->md_rate_estimation_ptr = md_rate_estimation_array;
#endif
#if !OPT_LOSSLESS_0
    // TMVP Map Writer pointer
    if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
        context_ptr->reference_object_write_ptr = (EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr;
    else
        context_ptr->reference_object_write_ptr = (EbReferenceObject*)EB_NULL;
#endif
    if (segment_index == 0)
        ResetEncodePassNeighborArrays(picture_control_set_ptr);
    return;
}

/******************************************************
 * EncDec Configure LCU
 ******************************************************/
static void EncDecConfigureLcu(
    EncDecContext         *context_ptr,
    LargestCodingUnit     *sb_ptr,
    PictureControlSet     *picture_control_set_ptr,
#if !QPM
    SequenceControlSet    *sequence_control_set_ptr,
#endif
    uint8_t                    picture_qp,
    uint8_t                    sb_qp)
{
#if QPM
    (void)picture_qp;
    context_ptr->qp = sb_qp;
#else
    //RC is off
    if (sequence_control_set_ptr->static_config.rate_control_mode == 0 && sequence_control_set_ptr->static_config.improve_sharpness == 0)
        context_ptr->qp = picture_qp;
    //RC is on
    else
        context_ptr->qp = sb_qp;
#endif
    // Asuming cb and cr offset to be the same for chroma QP in both slice and pps for lambda computation
    context_ptr->chroma_qp = context_ptr->qp;
    /* Note(CHKN) : when Qp modulation varies QP on a sub-LCU(CU) basis,  Lamda has to change based on Cu->QP , and then this code has to move inside the CU loop in MD */
    (void)sb_ptr;

    context_ptr->qp_index = (uint8_t)picture_control_set_ptr->parent_pcs_ptr->base_qindex;
    (*av1_lambda_assignment_function_table[picture_control_set_ptr->parent_pcs_ptr->pred_structure])(
#if LAMBDA_TUNING
        picture_control_set_ptr->temporal_layer_index,
#endif
        &context_ptr->fast_lambda,
        &context_ptr->full_lambda,
        &context_ptr->fast_chroma_lambda,
        &context_ptr->full_chroma_lambda,
        (uint8_t)picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr->bit_depth,
        context_ptr->qp_index);

    return;
}

/******************************************************
 * Update MD Segments
 *
 * This function is responsible for synchronizing the
 *   processing of MD Segment-rows.
 *   In short, the function starts processing
 *   of MD segment-rows as soon as their inputs are available
 *   and the previous segment-row has completed.  At
 *   any given time, only one segment row per picture
 *   is being processed.
 *
 * The function has two functions:
 *
 * (1) Update the Segment Completion Mask which tracks
 *   which MD Segment inputs are available.
 *
 * (2) Increment the segment-row counter (current_row_idx)
 *   as the segment-rows are completed.
 *
 * Since there is the potentential for thread collusion,
 *   a MUTEX a used to protect the sensitive data and
 *   the execution flow is separated into two paths
 *
 * (A) Initial update.
 *  -Update the Completion Mask [see (1) above]
 *  -If the picture is not currently being processed,
 *     check to see if the next segment-row is available
 *     and start processing.
 * (B) Continued processing
 *  -Upon the completion of a segment-row, check
 *     to see if the next segment-row's inputs have
 *     become available and begin processing if so.
 *
 * On last important point is that the thread-safe
 *   code section is kept minimally short. The MUTEX
 *   should NOT be locked for the entire processing
 *   of the segment-row (B) as this would block other
 *   threads from performing an update (A).
 ******************************************************/
EbBool AssignEncDecSegments(
    EncDecSegments   *segmentPtr,
    uint16_t             *segmentInOutIndex,
    EncDecTasks      *taskPtr,
    EbFifo           *srmFifoPtr)
{
    EbBool continueProcessingFlag = EB_FALSE;
    EbObjectWrapper *wrapper_ptr;
    EncDecTasks *feedbackTaskPtr;

    uint32_t rowSegmentIndex = 0;
    uint32_t segment_index;
    uint32_t rightSegmentIndex;
    uint32_t bottomLeftSegmentIndex;

    int16_t feedbackRowIndex = -1;

    uint32_t selfAssigned = EB_FALSE;

    //static FILE *trace = 0;
    //
    //if(trace == 0) {
    //    trace = fopen("seg-trace.txt","w");
    //}

    switch (taskPtr->input_type) {
    case ENCDEC_TASKS_MDC_INPUT:

        // The entire picture is provided by the MDC process, so
        //   no logic is necessary to clear input dependencies.

        // Start on Segment 0 immediately
        *segmentInOutIndex = segmentPtr->row_array[0].current_seg_index;
        taskPtr->input_type = ENCDEC_TASKS_CONTINUE;
        ++segmentPtr->row_array[0].current_seg_index;
        continueProcessingFlag = EB_TRUE;

        //fprintf(trace, "Start  Pic: %u Seg: %u\n",
        //    (unsigned) ((PictureControlSet*) taskPtr->picture_control_set_wrapper_ptr->object_ptr)->picture_number,
        //    *segmentInOutIndex);

        break;

    case ENCDEC_TASKS_ENCDEC_INPUT:

        // Setup rowSegmentIndex to release the in_progress token
        //rowSegmentIndex = taskPtr->encDecSegmentRowArray[0];

        // Start on the assigned row immediately
        *segmentInOutIndex = segmentPtr->row_array[taskPtr->enc_dec_segment_row].current_seg_index;
        taskPtr->input_type = ENCDEC_TASKS_CONTINUE;
        ++segmentPtr->row_array[taskPtr->enc_dec_segment_row].current_seg_index;
        continueProcessingFlag = EB_TRUE;

        //fprintf(trace, "Start  Pic: %u Seg: %u\n",
        //    (unsigned) ((PictureControlSet*) taskPtr->picture_control_set_wrapper_ptr->object_ptr)->picture_number,
        //    *segmentInOutIndex);

        break;

    case ENCDEC_TASKS_CONTINUE:

        // Update the Dependency List for Right and Bottom Neighbors
        segment_index = *segmentInOutIndex;
        rowSegmentIndex = segment_index / segmentPtr->segment_band_count;

        rightSegmentIndex = segment_index + 1;
        bottomLeftSegmentIndex = segment_index + segmentPtr->segment_band_count;

        // Right Neighbor
        if (segment_index < segmentPtr->row_array[rowSegmentIndex].ending_seg_index)
        {
            eb_block_on_mutex(segmentPtr->row_array[rowSegmentIndex].assignment_mutex);

            --segmentPtr->dep_map.dependency_map[rightSegmentIndex];

            if (segmentPtr->dep_map.dependency_map[rightSegmentIndex] == 0) {
                *segmentInOutIndex = segmentPtr->row_array[rowSegmentIndex].current_seg_index;
                ++segmentPtr->row_array[rowSegmentIndex].current_seg_index;
                selfAssigned = EB_TRUE;
                continueProcessingFlag = EB_TRUE;

                //fprintf(trace, "Start  Pic: %u Seg: %u\n",
                //    (unsigned) ((PictureControlSet*) taskPtr->picture_control_set_wrapper_ptr->object_ptr)->picture_number,
                //    *segmentInOutIndex);
            }

            eb_release_mutex(segmentPtr->row_array[rowSegmentIndex].assignment_mutex);
        }

        // Bottom-left Neighbor
        if (rowSegmentIndex < segmentPtr->segment_row_count - 1 && bottomLeftSegmentIndex >= segmentPtr->row_array[rowSegmentIndex + 1].starting_seg_index)
        {
            eb_block_on_mutex(segmentPtr->row_array[rowSegmentIndex + 1].assignment_mutex);

            --segmentPtr->dep_map.dependency_map[bottomLeftSegmentIndex];

            if (segmentPtr->dep_map.dependency_map[bottomLeftSegmentIndex] == 0) {
                if (selfAssigned == EB_TRUE)
                    feedbackRowIndex = (int16_t)rowSegmentIndex + 1;
                else {
                    *segmentInOutIndex = segmentPtr->row_array[rowSegmentIndex + 1].current_seg_index;
                    ++segmentPtr->row_array[rowSegmentIndex + 1].current_seg_index;
                    selfAssigned = EB_TRUE;
                    continueProcessingFlag = EB_TRUE;

                    //fprintf(trace, "Start  Pic: %u Seg: %u\n",
                    //    (unsigned) ((PictureControlSet*) taskPtr->picture_control_set_wrapper_ptr->object_ptr)->picture_number,
                    //    *segmentInOutIndex);
                }
            }
            eb_release_mutex(segmentPtr->row_array[rowSegmentIndex + 1].assignment_mutex);
        }

        if (feedbackRowIndex > 0) {
            eb_get_empty_object(
                srmFifoPtr,
                &wrapper_ptr);
            feedbackTaskPtr = (EncDecTasks*)wrapper_ptr->object_ptr;
            feedbackTaskPtr->input_type = ENCDEC_TASKS_ENCDEC_INPUT;
            feedbackTaskPtr->enc_dec_segment_row = feedbackRowIndex;
            feedbackTaskPtr->picture_control_set_wrapper_ptr = taskPtr->picture_control_set_wrapper_ptr;
            eb_post_full_object(wrapper_ptr);
        }

        break;

    default:
        break;
    }

    return continueProcessingFlag;
}
void ReconOutput(
    PictureControlSet    *picture_control_set_ptr,
    SequenceControlSet   *sequence_control_set_ptr) {
    EbObjectWrapper             *outputReconWrapperPtr;
    EbBufferHeaderType           *outputReconPtr;
    EncodeContext               *encode_context_ptr = sequence_control_set_ptr->encode_context_ptr;
    EbBool is16bit = (sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);
    // The totalNumberOfReconFrames counter has to be write/read protected as
    //   it is used to determine the end of the stream.  If it is not protected
    //   the encoder might not properly terminate.
    eb_block_on_mutex(encode_context_ptr->total_number_of_recon_frame_mutex);

#if ALT_REF_OVERLAY
    if (!picture_control_set_ptr->parent_pcs_ptr->is_alt_ref) {
#endif
        // Get Recon Buffer
        eb_get_empty_object(
            sequence_control_set_ptr->encode_context_ptr->recon_output_fifo_ptr,
            &outputReconWrapperPtr);
        outputReconPtr = (EbBufferHeaderType*)outputReconWrapperPtr->object_ptr;
        outputReconPtr->flags = 0;

        // START READ/WRITE PROTECTED SECTION
        if (encode_context_ptr->total_number_of_recon_frames == encode_context_ptr->terminating_picture_number)
            outputReconPtr->flags = EB_BUFFERFLAG_EOS;

        encode_context_ptr->total_number_of_recon_frames++;

        //eb_release_mutex(encode_context_ptr->terminating_conditions_mutex);

        // STOP READ/WRITE PROTECTED SECTION
        outputReconPtr->n_filled_len = 0;

        // Copy the Reconstructed Picture to the Output Recon Buffer
        {
            uint32_t sampleTotalCount;
            uint8_t *reconReadPtr;
            uint8_t *reconWritePtr;

            EbPictureBufferDesc *recon_ptr;
            {
                if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
                    recon_ptr = is16bit ?
                    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture16bit :
                    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture;
                else {
                    if (is16bit)
                        recon_ptr = picture_control_set_ptr->recon_picture16bit_ptr;
                    else
                        recon_ptr = picture_control_set_ptr->recon_picture_ptr;
                }
            }

            // FGN: Create a buffer if needed, copy the reconstructed picture and run the film grain synthesis algorithm

            if (sequence_control_set_ptr->seq_header.film_grain_params_present) {
                EbPictureBufferDesc  *intermediateBufferPtr;
                {
                    if (is16bit)
                        intermediateBufferPtr = picture_control_set_ptr->film_grain_picture16bit_ptr;
                    else
                        intermediateBufferPtr = picture_control_set_ptr->film_grain_picture_ptr;
                }

                aom_film_grain_t *film_grain_ptr;

                if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
                    film_grain_ptr = &((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->film_grain_params;
                else
                    film_grain_ptr = &picture_control_set_ptr->parent_pcs_ptr->film_grain_params;

                av1_add_film_grain(recon_ptr, intermediateBufferPtr, film_grain_ptr);
                recon_ptr = intermediateBufferPtr;
            }

            // End running the film grain
            // Y Recon Samples
            sampleTotalCount = ((recon_ptr->max_width - sequence_control_set_ptr->max_input_pad_right) * (recon_ptr->max_height - sequence_control_set_ptr->max_input_pad_bottom)) << is16bit;
            reconReadPtr = recon_ptr->buffer_y + (recon_ptr->origin_y << is16bit) * recon_ptr->stride_y + (recon_ptr->origin_x << is16bit);
            reconWritePtr = &(outputReconPtr->p_buffer[outputReconPtr->n_filled_len]);

            CHECK_REPORT_ERROR(
                (outputReconPtr->n_filled_len + sampleTotalCount <= outputReconPtr->n_alloc_len),
                encode_context_ptr->app_callback_ptr,
                EB_ENC_ROB_OF_ERROR);

            // Initialize Y recon buffer
            picture_copy_kernel(
                reconReadPtr,
                recon_ptr->stride_y,
                reconWritePtr,
                recon_ptr->max_width - sequence_control_set_ptr->max_input_pad_right,
                recon_ptr->width - sequence_control_set_ptr->pad_right,
                recon_ptr->height - sequence_control_set_ptr->pad_bottom,
                1 << is16bit);

            outputReconPtr->n_filled_len += sampleTotalCount;

            // U Recon Samples
            sampleTotalCount = ((recon_ptr->max_width - sequence_control_set_ptr->max_input_pad_right) * (recon_ptr->max_height - sequence_control_set_ptr->max_input_pad_bottom) >> 2) << is16bit;
            reconReadPtr = recon_ptr->buffer_cb + ((recon_ptr->origin_y << is16bit) >> 1) * recon_ptr->stride_cb + ((recon_ptr->origin_x << is16bit) >> 1);
            reconWritePtr = &(outputReconPtr->p_buffer[outputReconPtr->n_filled_len]);

            CHECK_REPORT_ERROR(
                (outputReconPtr->n_filled_len + sampleTotalCount <= outputReconPtr->n_alloc_len),
                encode_context_ptr->app_callback_ptr,
                EB_ENC_ROB_OF_ERROR);

            // Initialize U recon buffer
            picture_copy_kernel(
                reconReadPtr,
                recon_ptr->stride_cb,
                reconWritePtr,
                (recon_ptr->max_width - sequence_control_set_ptr->max_input_pad_right) >> 1,
                (recon_ptr->width - sequence_control_set_ptr->pad_right) >> 1,
                (recon_ptr->height - sequence_control_set_ptr->pad_bottom) >> 1,
                1 << is16bit);
            outputReconPtr->n_filled_len += sampleTotalCount;

            // V Recon Samples
            sampleTotalCount = ((recon_ptr->max_width - sequence_control_set_ptr->max_input_pad_right) * (recon_ptr->max_height - sequence_control_set_ptr->max_input_pad_bottom) >> 2) << is16bit;
            reconReadPtr = recon_ptr->buffer_cr + ((recon_ptr->origin_y << is16bit) >> 1) * recon_ptr->stride_cr + ((recon_ptr->origin_x << is16bit) >> 1);
            reconWritePtr = &(outputReconPtr->p_buffer[outputReconPtr->n_filled_len]);

            CHECK_REPORT_ERROR(
                (outputReconPtr->n_filled_len + sampleTotalCount <= outputReconPtr->n_alloc_len),
                encode_context_ptr->app_callback_ptr,
                EB_ENC_ROB_OF_ERROR);

            // Initialize V recon buffer

            picture_copy_kernel(
                reconReadPtr,
                recon_ptr->stride_cr,
                reconWritePtr,
                (recon_ptr->max_width - sequence_control_set_ptr->max_input_pad_right) >> 1,
                (recon_ptr->width - sequence_control_set_ptr->pad_right) >> 1,
                (recon_ptr->height - sequence_control_set_ptr->pad_bottom) >> 1,
                1 << is16bit);
            outputReconPtr->n_filled_len += sampleTotalCount;
            outputReconPtr->pts = picture_control_set_ptr->picture_number;
        }

        // Post the Recon object
        eb_post_full_object(outputReconWrapperPtr);
#if ALT_REF_OVERLAY
    }
    else {
        // Overlay and altref have 1 recon only, which is from overlay pictures. So the recon of the alt_ref is not sent to the application.
        // However, to hanlde the end of sequence properly, total_number_of_recon_frames is increamented
        encode_context_ptr->total_number_of_recon_frames++;
    }
#endif
    eb_release_mutex(encode_context_ptr->total_number_of_recon_frame_mutex);
}

void PsnrCalculations(
    PictureControlSet    *picture_control_set_ptr,
    SequenceControlSet   *sequence_control_set_ptr){
    EbBool is16bit = (sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);

    if (!is16bit) {
        EbPictureBufferDesc *recon_ptr;

        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
            recon_ptr = ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture;
        else
            recon_ptr = picture_control_set_ptr->recon_picture_ptr;

        EbPictureBufferDesc *input_picture_ptr = (EbPictureBufferDesc*)picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr;

        uint64_t sseTotal[3] = { 0 };
        uint32_t   columnIndex;
        uint32_t   row_index = 0;
        uint64_t   residualDistortion = 0;
        EbByte  inputBuffer;
        EbByte  reconCoeffBuffer;

        reconCoeffBuffer = &((recon_ptr->buffer_y)[recon_ptr->origin_x + recon_ptr->origin_y * recon_ptr->stride_y]);
        inputBuffer = &((input_picture_ptr->buffer_y)[input_picture_ptr->origin_x + input_picture_ptr->origin_y * input_picture_ptr->stride_y]);

        residualDistortion = 0;

        while (row_index < sequence_control_set_ptr->seq_header.max_frame_height) {
            columnIndex = 0;
            while (columnIndex < sequence_control_set_ptr->seq_header.max_frame_width) {
                residualDistortion += (int64_t)SQR((int64_t)(inputBuffer[columnIndex]) - (reconCoeffBuffer[columnIndex]));
                ++columnIndex;
            }

            inputBuffer += input_picture_ptr->stride_y;
            reconCoeffBuffer += recon_ptr->stride_y;
            ++row_index;
        }

        sseTotal[0] = residualDistortion;

        reconCoeffBuffer = &((recon_ptr->buffer_cb)[recon_ptr->origin_x / 2 + recon_ptr->origin_y / 2 * recon_ptr->stride_cb]);
        inputBuffer = &((input_picture_ptr->buffer_cb)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cb]);

        residualDistortion = 0;
        row_index = 0;
        while (row_index < sequence_control_set_ptr->chroma_height) {
            columnIndex = 0;
            while (columnIndex < sequence_control_set_ptr->chroma_width) {
                residualDistortion += (int64_t)SQR((int64_t)(inputBuffer[columnIndex]) - (reconCoeffBuffer[columnIndex]));
                ++columnIndex;
            }

            inputBuffer += input_picture_ptr->stride_cb;
            reconCoeffBuffer += recon_ptr->stride_cb;
            ++row_index;
        }

        sseTotal[1] = residualDistortion;

        reconCoeffBuffer = &((recon_ptr->buffer_cr)[recon_ptr->origin_x / 2 + recon_ptr->origin_y / 2 * recon_ptr->stride_cr]);
        inputBuffer = &((input_picture_ptr->buffer_cr)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cr]);
        residualDistortion = 0;
        row_index = 0;

        while (row_index < sequence_control_set_ptr->chroma_height) {
            columnIndex = 0;
            while (columnIndex < sequence_control_set_ptr->chroma_width) {
                residualDistortion += (int64_t)SQR((int64_t)(inputBuffer[columnIndex]) - (reconCoeffBuffer[columnIndex]));
                ++columnIndex;
            }

            inputBuffer += input_picture_ptr->stride_cr;
            reconCoeffBuffer += recon_ptr->stride_cr;
            ++row_index;
        }

        sseTotal[2] = residualDistortion;
        picture_control_set_ptr->parent_pcs_ptr->luma_sse = (uint32_t)sseTotal[0];
        picture_control_set_ptr->parent_pcs_ptr->cr_sse = (uint32_t)sseTotal[1];
        picture_control_set_ptr->parent_pcs_ptr->cb_sse = (uint32_t)sseTotal[2];
    }
    else {
        EbPictureBufferDesc *recon_ptr;

        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
            recon_ptr = ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture16bit;
        else
            recon_ptr = picture_control_set_ptr->recon_picture16bit_ptr;
        EbPictureBufferDesc *input_picture_ptr = (EbPictureBufferDesc*)picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr;

        uint64_t sseTotal[3] = { 0 };
        uint32_t   columnIndex;
        uint32_t   row_index = 0;
        uint64_t   residualDistortion = 0;
        EbByte  inputBuffer;
        EbByte  inputBufferBitInc;
        uint16_t*  reconCoeffBuffer;

        if (sequence_control_set_ptr->static_config.ten_bit_format == 1) {
            const uint32_t luma_width = sequence_control_set_ptr->seq_header.max_frame_width;
            const uint32_t luma_height = sequence_control_set_ptr->seq_header.max_frame_height;
            const uint32_t chroma_width = sequence_control_set_ptr->chroma_width;
            const uint32_t picture_width_in_sb = (luma_width + 64 - 1) / 64;
            const uint32_t pictureHeighInLcu = (luma_height + 64 - 1) / 64;
            const uint32_t luma2BitWidth = luma_width / 4;
            const uint32_t chroma_height = luma_height / 2;
            const uint32_t chroma2BitWidth = luma_width / 8;
            uint32_t lcuNumberInHeight, lcuNumberInWidth;

            EbByte  inputBufferOrg = &((input_picture_ptr->buffer_y)[input_picture_ptr->origin_x + input_picture_ptr->origin_y * input_picture_ptr->stride_y]);
            uint16_t*  reconBufferOrg = (uint16_t*)(&((recon_ptr->buffer_y)[(recon_ptr->origin_x << is16bit) + (recon_ptr->origin_y << is16bit) * recon_ptr->stride_y]));;

            EbByte  inputBufferOrgU = &((input_picture_ptr->buffer_cb)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cb]);;
            uint16_t*  reconBufferOrgU = reconCoeffBuffer = (uint16_t*)(&((recon_ptr->buffer_cb)[(recon_ptr->origin_x << is16bit) / 2 + (recon_ptr->origin_y << is16bit) / 2 * recon_ptr->stride_cb]));;

            EbByte  inputBufferOrgV = &((input_picture_ptr->buffer_cr)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cr]);;
            uint16_t*  reconBufferOrgV = reconCoeffBuffer = (uint16_t*)(&((recon_ptr->buffer_cr)[(recon_ptr->origin_x << is16bit) / 2 + (recon_ptr->origin_y << is16bit) / 2 * recon_ptr->stride_cr]));;

            residualDistortion = 0;
            uint64_t residualDistortionU = 0;
            uint64_t residualDistortionV = 0;

            for (lcuNumberInHeight = 0; lcuNumberInHeight < pictureHeighInLcu; ++lcuNumberInHeight)
            {
                for (lcuNumberInWidth = 0; lcuNumberInWidth < picture_width_in_sb; ++lcuNumberInWidth)
                {
                    uint32_t tbOriginX = lcuNumberInWidth * 64;
                    uint32_t tbOriginY = lcuNumberInHeight * 64;
                    uint32_t sb_width = (luma_width - tbOriginX) < 64 ? (luma_width - tbOriginX) : 64;
                    uint32_t sb_height = (luma_height - tbOriginY) < 64 ? (luma_height - tbOriginY) : 64;

                    inputBuffer = inputBufferOrg + tbOriginY * input_picture_ptr->stride_y + tbOriginX;
                    inputBufferBitInc = input_picture_ptr->buffer_bit_inc_y + tbOriginY * luma2BitWidth + (tbOriginX / 4)*sb_height;
                    reconCoeffBuffer = reconBufferOrg + tbOriginY * recon_ptr->stride_y + tbOriginX;

                    uint64_t   j, k;
                    uint16_t   outPixel;
                    uint8_t    nBitPixel;
                    uint8_t   four2bitPels;
                    uint32_t     inn_stride = sb_width / 4;

                    for (j = 0; j < sb_height; j++)
                    {
                        for (k = 0; k < sb_width / 4; k++)
                        {
                            four2bitPels = inputBufferBitInc[k + j * inn_stride];

                            nBitPixel = (four2bitPels >> 6) & 3;
                            outPixel = inputBuffer[k * 4 + 0 + j * input_picture_ptr->stride_y] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortion += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 0 + j * recon_ptr->stride_y]);

                            nBitPixel = (four2bitPels >> 4) & 3;
                            outPixel = inputBuffer[k * 4 + 1 + j * input_picture_ptr->stride_y] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortion += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 1 + j * recon_ptr->stride_y]);

                            nBitPixel = (four2bitPels >> 2) & 3;
                            outPixel = inputBuffer[k * 4 + 2 + j * input_picture_ptr->stride_y] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortion += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 2 + j * recon_ptr->stride_y]);

                            nBitPixel = (four2bitPels >> 0) & 3;
                            outPixel = inputBuffer[k * 4 + 3 + j * input_picture_ptr->stride_y] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortion += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 3 + j * recon_ptr->stride_y]);
                        }
                    }

                    //U+V

                    tbOriginX = lcuNumberInWidth * 32;
                    tbOriginY = lcuNumberInHeight * 32;
                    sb_width = (chroma_width - tbOriginX) < 32 ? (chroma_width - tbOriginX) : 32;
                    sb_height = (chroma_height - tbOriginY) < 32 ? (chroma_height - tbOriginY) : 32;

                    inn_stride = sb_width / 4;

                    inputBuffer = inputBufferOrgU + tbOriginY * input_picture_ptr->stride_cb + tbOriginX;

                    inputBufferBitInc = input_picture_ptr->buffer_bit_inc_cb + tbOriginY * chroma2BitWidth + (tbOriginX / 4)*sb_height;

                    reconCoeffBuffer = reconBufferOrgU + tbOriginY * recon_ptr->stride_cb + tbOriginX;

                    for (j = 0; j < sb_height; j++)
                    {
                        for (k = 0; k < sb_width / 4; k++)
                        {
                            four2bitPels = inputBufferBitInc[k + j * inn_stride];

                            nBitPixel = (four2bitPels >> 6) & 3;
                            outPixel = inputBuffer[k * 4 + 0 + j * input_picture_ptr->stride_cb] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortionU += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 0 + j * recon_ptr->stride_cb]);

                            nBitPixel = (four2bitPels >> 4) & 3;
                            outPixel = inputBuffer[k * 4 + 1 + j * input_picture_ptr->stride_cb] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortionU += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 1 + j * recon_ptr->stride_cb]);

                            nBitPixel = (four2bitPels >> 2) & 3;
                            outPixel = inputBuffer[k * 4 + 2 + j * input_picture_ptr->stride_cb] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortionU += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 2 + j * recon_ptr->stride_cb]);

                            nBitPixel = (four2bitPels >> 0) & 3;
                            outPixel = inputBuffer[k * 4 + 3 + j * input_picture_ptr->stride_cb] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortionU += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 3 + j * recon_ptr->stride_cb]);
                        }
                    }

                    inputBuffer = inputBufferOrgV + tbOriginY * input_picture_ptr->stride_cr + tbOriginX;
                    inputBufferBitInc = input_picture_ptr->buffer_bit_inc_cr + tbOriginY * chroma2BitWidth + (tbOriginX / 4)*sb_height;
                    reconCoeffBuffer = reconBufferOrgV + tbOriginY * recon_ptr->stride_cr + tbOriginX;

                    for (j = 0; j < sb_height; j++)
                    {
                        for (k = 0; k < sb_width / 4; k++)
                        {
                            four2bitPels = inputBufferBitInc[k + j * inn_stride];

                            nBitPixel = (four2bitPels >> 6) & 3;
                            outPixel = inputBuffer[k * 4 + 0 + j * input_picture_ptr->stride_cr] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortionV += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 0 + j * recon_ptr->stride_cr]);

                            nBitPixel = (four2bitPels >> 4) & 3;
                            outPixel = inputBuffer[k * 4 + 1 + j * input_picture_ptr->stride_cr] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortionV += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 1 + j * recon_ptr->stride_cr]);

                            nBitPixel = (four2bitPels >> 2) & 3;
                            outPixel = inputBuffer[k * 4 + 2 + j * input_picture_ptr->stride_cr] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortionV += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 2 + j * recon_ptr->stride_cr]);

                            nBitPixel = (four2bitPels >> 0) & 3;
                            outPixel = inputBuffer[k * 4 + 3 + j * input_picture_ptr->stride_cr] << 2;
                            outPixel = outPixel | nBitPixel;
                            residualDistortionV += (int64_t)SQR((int64_t)outPixel - (int64_t)reconCoeffBuffer[k * 4 + 3 + j * recon_ptr->stride_cr]);
                        }
                    }
                }
            }

            sseTotal[0] = residualDistortion;
            sseTotal[1] = residualDistortionU;
            sseTotal[2] = residualDistortionV;
        }
        else {
            reconCoeffBuffer = (uint16_t*)(&((recon_ptr->buffer_y)[(recon_ptr->origin_x << is16bit) + (recon_ptr->origin_y << is16bit) * recon_ptr->stride_y]));
            inputBuffer = &((input_picture_ptr->buffer_y)[input_picture_ptr->origin_x + input_picture_ptr->origin_y * input_picture_ptr->stride_y]);
            inputBufferBitInc = &((input_picture_ptr->buffer_bit_inc_y)[input_picture_ptr->origin_x + input_picture_ptr->origin_y * input_picture_ptr->stride_bit_inc_y]);

            residualDistortion = 0;

            while (row_index < sequence_control_set_ptr->seq_header.max_frame_height) {
                columnIndex = 0;
                while (columnIndex < sequence_control_set_ptr->seq_header.max_frame_width) {
                    residualDistortion += (int64_t)SQR((int64_t)((((inputBuffer[columnIndex]) << 2) | ((inputBufferBitInc[columnIndex] >> 6) & 3))) - (reconCoeffBuffer[columnIndex]));

                    ++columnIndex;
                }

                inputBuffer += input_picture_ptr->stride_y;
                inputBufferBitInc += input_picture_ptr->stride_bit_inc_y;
                reconCoeffBuffer += recon_ptr->stride_y;
                ++row_index;
            }

            sseTotal[0] = residualDistortion;

            reconCoeffBuffer = (uint16_t*)(&((recon_ptr->buffer_cb)[(recon_ptr->origin_x << is16bit) / 2 + (recon_ptr->origin_y << is16bit) / 2 * recon_ptr->stride_cb]));
            inputBuffer = &((input_picture_ptr->buffer_cb)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cb]);
            inputBufferBitInc = &((input_picture_ptr->buffer_bit_inc_cb)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_bit_inc_cb]);

            residualDistortion = 0;
            row_index = 0;
            while (row_index < sequence_control_set_ptr->chroma_height) {
                columnIndex = 0;
                while (columnIndex < sequence_control_set_ptr->chroma_width) {
                    residualDistortion += (int64_t)SQR((int64_t)((((inputBuffer[columnIndex]) << 2) | ((inputBufferBitInc[columnIndex] >> 6) & 3))) - (reconCoeffBuffer[columnIndex]));
                    ++columnIndex;
                }

                inputBuffer += input_picture_ptr->stride_cb;
                inputBufferBitInc += input_picture_ptr->stride_bit_inc_cb;
                reconCoeffBuffer += recon_ptr->stride_cb;
                ++row_index;
            }

            sseTotal[1] = residualDistortion;

            reconCoeffBuffer = (uint16_t*)(&((recon_ptr->buffer_cr)[(recon_ptr->origin_x << is16bit) / 2 + (recon_ptr->origin_y << is16bit) / 2 * recon_ptr->stride_cr]));
            inputBuffer = &((input_picture_ptr->buffer_cr)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cr]);
            inputBufferBitInc = &((input_picture_ptr->buffer_bit_inc_cr)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_bit_inc_cr]);

            residualDistortion = 0;
            row_index = 0;

            while (row_index < sequence_control_set_ptr->chroma_height) {
                columnIndex = 0;
                while (columnIndex < sequence_control_set_ptr->chroma_width) {
                    residualDistortion += (int64_t)SQR((int64_t)((((inputBuffer[columnIndex]) << 2) | ((inputBufferBitInc[columnIndex] >> 6) & 3))) - (reconCoeffBuffer[columnIndex]));
                    ++columnIndex;
                }

                inputBuffer += input_picture_ptr->stride_cr;
                inputBufferBitInc += input_picture_ptr->stride_bit_inc_cr;
                reconCoeffBuffer += recon_ptr->stride_cr;
                ++row_index;
            }

            sseTotal[2] = residualDistortion;
        }

        picture_control_set_ptr->parent_pcs_ptr->luma_sse = (uint32_t)sseTotal[0];
        picture_control_set_ptr->parent_pcs_ptr->cr_sse = (uint32_t)sseTotal[1];
        picture_control_set_ptr->parent_pcs_ptr->cb_sse = (uint32_t)sseTotal[2];
    }
}

void PadRefAndSetFlags(
    PictureControlSet    *picture_control_set_ptr,
    SequenceControlSet   *sequence_control_set_ptr
)
{
    EbReferenceObject   *referenceObject = (EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr;
    EbPictureBufferDesc *refPicPtr = (EbPictureBufferDesc*)referenceObject->reference_picture;
    EbPictureBufferDesc *refPic16BitPtr = (EbPictureBufferDesc*)referenceObject->reference_picture16bit;
    EbBool                is16bit = (sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);

    if (!is16bit) {
        // Y samples
        generate_padding(
            refPicPtr->buffer_y,
            refPicPtr->stride_y,
            refPicPtr->width,
            refPicPtr->height,
            refPicPtr->origin_x,
            refPicPtr->origin_y);

        // Cb samples
        generate_padding(
            refPicPtr->buffer_cb,
            refPicPtr->stride_cb,
            refPicPtr->width >> 1,
            refPicPtr->height >> 1,
            refPicPtr->origin_x >> 1,
            refPicPtr->origin_y >> 1);

        // Cr samples
        generate_padding(
            refPicPtr->buffer_cr,
            refPicPtr->stride_cr,
            refPicPtr->width >> 1,
            refPicPtr->height >> 1,
            refPicPtr->origin_x >> 1,
            refPicPtr->origin_y >> 1);
    }

    //We need this for MCP
    if (is16bit) {
        // Y samples
        generate_padding16_bit(
            refPic16BitPtr->buffer_y,
            refPic16BitPtr->stride_y << 1,
            refPic16BitPtr->width << 1,
            refPic16BitPtr->height,
            refPic16BitPtr->origin_x << 1,
            refPic16BitPtr->origin_y);

        // Cb samples
        generate_padding16_bit(
            refPic16BitPtr->buffer_cb,
            refPic16BitPtr->stride_cb << 1,
            refPic16BitPtr->width,
            refPic16BitPtr->height >> 1,
            refPic16BitPtr->origin_x,
            refPic16BitPtr->origin_y >> 1);

        // Cr samples
        generate_padding16_bit(
            refPic16BitPtr->buffer_cr,
            refPic16BitPtr->stride_cr << 1,
            refPic16BitPtr->width,
            refPic16BitPtr->height >> 1,
            refPic16BitPtr->origin_x,
            refPic16BitPtr->origin_y >> 1);

#if UNPACK_REF_POST_EP
        // Hsan: unpack ref samples (to be used @ MD)
        un_pack2d(
            (uint16_t*) refPic16BitPtr->buffer_y,
            refPic16BitPtr->stride_y,
            refPicPtr->buffer_y,
            refPicPtr->stride_y,
            refPicPtr->buffer_bit_inc_y,
            refPicPtr->stride_bit_inc_y,
            refPic16BitPtr->width  + (refPicPtr->origin_x << 1),
            refPic16BitPtr->height + (refPicPtr->origin_y << 1));

        un_pack2d(
            (uint16_t*)refPic16BitPtr->buffer_cb,
            refPic16BitPtr->stride_cb,
            refPicPtr->buffer_cb,
            refPicPtr->stride_cb,
            refPicPtr->buffer_bit_inc_cb,
            refPicPtr->stride_bit_inc_cb,
            (refPic16BitPtr->width + (refPicPtr->origin_x << 1)) >> 1,
            (refPic16BitPtr->height + (refPicPtr->origin_y << 1)) >> 1);

        un_pack2d(
            (uint16_t*)refPic16BitPtr->buffer_cr,
            refPic16BitPtr->stride_cr,
            refPicPtr->buffer_cr,
            refPicPtr->stride_cr,
            refPicPtr->buffer_bit_inc_cr,
            refPicPtr->stride_bit_inc_cr,
            (refPic16BitPtr->width + (refPicPtr->origin_x << 1)) >> 1,
            (refPic16BitPtr->height + (refPicPtr->origin_y << 1)) >> 1);
#endif
    }
#if !OPT_LOSSLESS_1
    // set up TMVP flag for the reference picture

    referenceObject->tmvp_enable_flag = (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? EB_TRUE : EB_FALSE;
#endif
    // set up the ref POC
    referenceObject->ref_poc = picture_control_set_ptr->parent_pcs_ptr->picture_number;

    // set up the QP
#if ADD_DELTA_QP_SUPPORT && !QPM
    uint16_t picture_qp = picture_control_set_ptr->parent_pcs_ptr->base_qindex;
    referenceObject->qp = (uint16_t)picture_qp;
#else
    referenceObject->qp = (uint8_t)picture_control_set_ptr->parent_pcs_ptr->picture_qp;
#endif

    // set up the Slice Type
    referenceObject->slice_type = picture_control_set_ptr->parent_pcs_ptr->slice_type;

#if TWO_PASS
   referenceObject->referenced_area_avg = picture_control_set_ptr->parent_pcs_ptr->referenced_area_avg;

#endif
}

void CopyStatisticsToRefObject(
    PictureControlSet    *picture_control_set_ptr,
    SequenceControlSet   *sequence_control_set_ptr
)
{
    picture_control_set_ptr->intra_coded_area = (100 * picture_control_set_ptr->intra_coded_area) / (sequence_control_set_ptr->seq_header.max_frame_width * sequence_control_set_ptr->seq_header.max_frame_height);
    if (picture_control_set_ptr->slice_type == I_SLICE)
        picture_control_set_ptr->intra_coded_area = 0;

    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->intra_coded_area = (uint8_t)(picture_control_set_ptr->intra_coded_area);

    uint32_t sb_index;
    for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; ++sb_index)
        ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->non_moving_index_array[sb_index] = picture_control_set_ptr->parent_pcs_ptr->non_moving_index_array[sb_index];
#if !DISABLE_OIS_USE
    EbReferenceObject  * refObjL0, *refObjL1;
    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->penalize_skipflag = EB_FALSE;
    if (picture_control_set_ptr->slice_type == B_SLICE) {
        //MRP_MD
        refObjL0 = (EbReferenceObject*)picture_control_set_ptr->ref_pic_ptr_array[REF_LIST_0]->object_ptr;
        refObjL1 = (EbReferenceObject*)picture_control_set_ptr->ref_pic_ptr_array[REF_LIST_1]->object_ptr;

        if (picture_control_set_ptr->temporal_layer_index == 0) {
            if (picture_control_set_ptr->parent_pcs_ptr->intra_coded_block_probability > 30)
                ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->penalize_skipflag = EB_TRUE;
        }
        else
            ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->penalize_skipflag = (refObjL0->penalize_skipflag || refObjL1->penalize_skipflag) ? EB_TRUE : EB_FALSE;
    }
#endif
    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->tmp_layer_idx = (uint8_t)picture_control_set_ptr->temporal_layer_index;
    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->is_scene_change = picture_control_set_ptr->parent_pcs_ptr->scene_change_flag;

    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->cdef_frame_strength = picture_control_set_ptr->parent_pcs_ptr->cdef_frame_strength;

    Av1Common* cm = picture_control_set_ptr->parent_pcs_ptr->av1_cm;
    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->sg_frame_ep = cm->sg_frame_ep;
#if TEMPORAL_MVP
    if (sequence_control_set_ptr->temporal_mvp_enabled) {
        ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->av1_frame_type = picture_control_set_ptr->parent_pcs_ptr->av1_frame_type;
        ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->order_hint = picture_control_set_ptr->parent_pcs_ptr->cur_order_hint;
        memcpy(((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->ref_order_hint, picture_control_set_ptr->parent_pcs_ptr->ref_order_hint, 7 * sizeof(uint32_t));
    }
#endif
}

#if !MEMORY_FOOTPRINT_OPT
EbErrorType QpmDeriveWeightsMinAndMax(
    PictureControlSet                    *picture_control_set_ptr,
    EncDecContext                        *context_ptr)
{
    EbErrorType                    return_error = EB_ErrorNone;
    uint32_t cu_depth;
    context_ptr->min_delta_qp_weight = encMinDeltaQpWeightTab[picture_control_set_ptr->temporal_layer_index];
    context_ptr->max_delta_qp_weight = encMaxDeltaQpWeightTab[picture_control_set_ptr->temporal_layer_index];
    //qpm_derive_delta_qp_map_weights

    EbBool adjust_min_qp_flag = EB_FALSE;

    adjust_min_qp_flag = picture_control_set_ptr->adjust_min_qp_flag;
    context_ptr->min_delta_qp_weight = 100;
    context_ptr->max_delta_qp_weight = 100;

    {
        if (picture_control_set_ptr->slice_type == I_SLICE) {
            if (picture_control_set_ptr->parent_pcs_ptr->percentage_of_edgein_light_background > 0 && picture_control_set_ptr->parent_pcs_ptr->percentage_of_edgein_light_background <= 3
                && !adjust_min_qp_flag && picture_control_set_ptr->parent_pcs_ptr->dark_back_groundlight_fore_ground) {
                context_ptr->min_delta_qp_weight = 100;
            }
            else {
                if (adjust_min_qp_flag)
                    context_ptr->min_delta_qp_weight = 250;
                else if (picture_control_set_ptr->parent_pcs_ptr->pic_homogenous_over_time_sb_percentage > 30) {
                    context_ptr->min_delta_qp_weight = 150;
                    context_ptr->max_delta_qp_weight = 50;
                }
            }
        }
        else {
            if (adjust_min_qp_flag)
                context_ptr->min_delta_qp_weight = 170;
        }
        if (picture_control_set_ptr->parent_pcs_ptr->high_dark_area_density_flag) {
            context_ptr->min_delta_qp_weight = 25;
            context_ptr->max_delta_qp_weight = 25;
        }
    }

    // Refine max_delta_qp_weight; apply conservative max_degrade_weight when most of the picture is homogenous over time.
    if (picture_control_set_ptr->parent_pcs_ptr->pic_homogenous_over_time_sb_percentage > 90)
        context_ptr->max_delta_qp_weight = context_ptr->max_delta_qp_weight >> 1;
    for (cu_depth = 0; cu_depth < 4; cu_depth++) {
        context_ptr->min_delta_qp[cu_depth] = picture_control_set_ptr->slice_type == I_SLICE ? encMinDeltaQpISliceTab[cu_depth] : encMinDeltaQpTab[cu_depth][picture_control_set_ptr->temporal_layer_index];
        context_ptr->max_delta_qp[cu_depth] = encMaxDeltaQpTab[cu_depth][picture_control_set_ptr->temporal_layer_index];
    }

    return return_error;
}
#endif
/******************************************************
* Derive EncDec Settings for OQ
Input   : encoder mode and tune
Output  : EncDec Kernel signal(s)
******************************************************/
EbErrorType signal_derivation_enc_dec_kernel_oq(
    SequenceControlSet    *sequence_control_set_ptr,
    PictureControlSet     *picture_control_set_ptr,
#if MOVE_TX_LEVELS_SIGNAL_UNDER_CTX
    EncDecContext         *ep_context_ptr,
#endif
    ModeDecisionContext   *context_ptr) {
    EbErrorType return_error = EB_ErrorNone;
#if MOVE_TX_LEVELS_SIGNAL_UNDER_CTX
    // Tx_search Level                                Settings
    // 0                                              OFF
    // 1                                              Tx search at encdec
    // 2                                              Tx search at inter-depth
    // 3                                              Tx search at full loop
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M6)
            context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
        else
            if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
            else
                context_ptr->tx_search_level = TX_SEARCH_ENC_DEC;
    else if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
    else if (picture_control_set_ptr->enc_mode <= ENC_M7) {
        if (picture_control_set_ptr->temporal_layer_index == 0)
            context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
        else
            context_ptr->tx_search_level = TX_SEARCH_ENC_DEC;
    }
    else
        context_ptr->tx_search_level = TX_SEARCH_ENC_DEC;
    
    // Set tx search skip weights (MAX_MODE_COST: no skipping; 0: always skipping)
#if FIX_TX_SEARCH_FOR_MR_MODE
    if (MR_MODE) // tx weight
        context_ptr->tx_weight = MAX_MODE_COST;
#endif
    else{
        if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
            context_ptr->tx_weight = MAX_MODE_COST;
        else if (!MR_MODE && picture_control_set_ptr->enc_mode <= ENC_M5)
            context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
        else if (!MR_MODE){
            if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
            else
                context_ptr->tx_weight = FC_SKIP_TX_SR_TH010;
        }
    }

    // Set tx search reduced set falg (0: full tx set; 1: reduced tx set; 1: two tx))
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if SC_M8_TX_REDUCED_SET_
            picture_control_set_ptr->tx_search_reduced_set = 2;
#else
        if (picture_control_set_ptr->enc_mode <= ENC_M5)
            context_ptr->tx_search_reduced_set = 0;
        else if (picture_control_set_ptr->enc_mode <= ENC_M6)
            if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
                context_ptr->tx_search_reduced_set = 0;
            else
                context_ptr->tx_search_reduced_set = 1;
        else if (picture_control_set_ptr->enc_mode <= ENC_M7)
            context_ptr->tx_search_reduced_set = 1;
        else
            context_ptr->tx_search_reduced_set = 2;
#endif
    else

    if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
        context_ptr->tx_search_reduced_set = 0;
#if M2_BAD_SLOPE_COMB && ! m2_ibc_graph
    else if (picture_control_set_ptr->enc_mode <= ENC_M2)
#else
    else if (picture_control_set_ptr->enc_mode <= ENC_M1)
#endif
        context_ptr->tx_search_reduced_set = 0;
    else if (picture_control_set_ptr->enc_mode <= ENC_M5)
        context_ptr->tx_search_reduced_set = 1;
    else
        context_ptr->tx_search_reduced_set = 1;

    // Set skip tx search based on NFL falg (0: Skip OFF ; 1: skip ON)
    context_ptr->skip_tx_search = 0;
    ep_context_ptr->tx_search_level = context_ptr->tx_search_level;
    ep_context_ptr->skip_tx_search = context_ptr->skip_tx_search;
    ep_context_ptr->tx_search_reduced_set = context_ptr->tx_search_reduced_set;
    ep_context_ptr->tx_weight = context_ptr->tx_weight;
#endif
#if MOVE_IF_LEVELS_SIGNAL_UNDER_CTX
    // Interpolation search Level                     Settings
    // 0                                              OFF
    // 1                                              Interpolation search at inter-depth
    // 2                                              Interpolation search at full loop
    // 3                                              Chroma blind interpolation search at fast loop
    // 4                                              Interpolation search at fast loop

        if (MR_MODE) // Interpolation
            context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP;
        else if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if NEW_M0_SC
            context_ptr->interpolation_search_level = IT_SEARCH_OFF;
#else
            if (enc_mode <= ENC_M1)
                context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
            else
                context_ptr->interpolation_search_level = IT_SEARCH_OFF;
#endif

        else if (picture_control_set_ptr->enc_mode <= ENC_M3)
            context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
        else if (picture_control_set_ptr->enc_mode <= ENC_M7)
            if (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0)
                context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
            else
                context_ptr->interpolation_search_level = IT_SEARCH_OFF;
        else
            context_ptr->interpolation_search_level = IT_SEARCH_OFF;
#endif
    // NFL Level MD       Settings
    // 0                  MAX_NFL 40
    // 1                  30
    // 2                  12
    // 3                  10
    // 4                  8
    // 5                  6
    // 6                  4
    // 7                  3
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M5)
            if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                context_ptr->nfl_level = (sequence_control_set_ptr->input_resolution <= INPUT_SIZE_576p_RANGE_OR_LOWER) ? 0 : 1;
            else
                context_ptr->nfl_level = 2;
        else
            if (picture_control_set_ptr->parent_pcs_ptr->slice_type == I_SLICE)
                context_ptr->nfl_level = 5;
            else if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                context_ptr->nfl_level = 6;
            else
                context_ptr->nfl_level = 7;
    else
    if (picture_control_set_ptr->enc_mode <= ENC_M5)
        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
            context_ptr->nfl_level = (sequence_control_set_ptr->input_resolution <= INPUT_SIZE_576p_RANGE_OR_LOWER) ? 0 : 1;
        else
            context_ptr->nfl_level = 2;
    else if(picture_control_set_ptr->enc_mode <= ENC_M5)
        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
            context_ptr->nfl_level = 2;
        else
            context_ptr->nfl_level = 4;
    else if (picture_control_set_ptr->enc_mode <= ENC_M6)
        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
            context_ptr->nfl_level = 4;
        else
            context_ptr->nfl_level = 5;
    else
        if (picture_control_set_ptr->parent_pcs_ptr->slice_type == I_SLICE)
            context_ptr->nfl_level = 5;
        else if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
            context_ptr->nfl_level = 6;
        else
            context_ptr->nfl_level = 7;

    // Set Chroma Mode
    // Level                Settings
    // CHROMA_MODE_0  0     Full chroma search @ MD
    // CHROMA_MODE_1  1     Fast chroma search @ MD
    // CHROMA_MODE_2  2     Chroma blind @ MD + CFL @ EP
    // CHROMA_MODE_3  3     Chroma blind @ MD + no CFL @ EP
#if SEARCH_UV_MODE
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M6)
            context_ptr->chroma_level = CHROMA_MODE_1;
        else
            if (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0)
                context_ptr->chroma_level = CHROMA_MODE_1;
            else
                context_ptr->chroma_level = (sequence_control_set_ptr->encoder_bit_depth == EB_8BIT) ?
                CHROMA_MODE_2 :
                CHROMA_MODE_3;
    else
#if CHROMA_SEARCH_MR
    if (MR_MODE || USE_MR_CHROMA) // chroma
        context_ptr->chroma_level = CHROMA_MODE_0;
    else
#endif
#if SEARCH_UV_BASE
    if (picture_control_set_ptr->enc_mode <= ENC_M5 && picture_control_set_ptr->temporal_layer_index == 0)
#else
    if (picture_control_set_ptr->enc_mode == ENC_M0)
#endif
        context_ptr->chroma_level = CHROMA_MODE_0;
    else
#endif
    if (picture_control_set_ptr->enc_mode <= ENC_M5)
        context_ptr->chroma_level = CHROMA_MODE_1;
    else
        context_ptr->chroma_level = (sequence_control_set_ptr->encoder_bit_depth == EB_8BIT) ?
            CHROMA_MODE_2 :
            CHROMA_MODE_3 ;

    // Set fast loop method
    // 1 fast loop: SSD_SEARCH not supported
    // Level                Settings
    //  0                   Collapsed fast loop
    //  1                   Decoupled fast loops ( intra/inter)
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M5)
            context_ptr->decouple_intra_inter_fast_loop = 0;
        else
            context_ptr->decouple_intra_inter_fast_loop = 1;
    else
    context_ptr->decouple_intra_inter_fast_loop = 0;

    // Set the search method when decoupled fast loop is used
    // Hsan: FULL_SAD_SEARCH not supported
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M5)
            context_ptr->decoupled_fast_loop_search_method = SSD_SEARCH;
        else
            context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;
    else if (picture_control_set_ptr->enc_mode <= ENC_M4)
            context_ptr->decoupled_fast_loop_search_method = SSD_SEARCH;
        else
            context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;

    //MD_CLASS
    //context_ptr->decouple_intra_inter_fast_loop = 0;
    //context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;// picture_control_set_ptr->enc_mode == ENC_M0 ? SSD_SEARCH : FULL_SAD_SEARCH;


    // Set the full loop escape level
    // Level                Settings
    // 0                    Off
    // 1                    On but only INTRA
    // 2                    On both INTRA and INTER
#if M9_FULL_LOOP_ESCAPE
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M5)
            context_ptr->full_loop_escape = 0;
        else
            context_ptr->full_loop_escape = 2;
    else if (picture_control_set_ptr->enc_mode <= ENC_M6)
        context_ptr->full_loop_escape = 0;
    else
        context_ptr->full_loop_escape = 2;
#else
    if (picture_control_set_ptr->enc_mode <= ENC_M7)
        context_ptr->full_loop_escape = 0;
    else
        context_ptr->full_loop_escape = 1;
#endif


    //MD_CLASS
    //context_ptr->full_loop_escape = 2;  //to use always the sorted array.


    // Set global MV injection
    // Level                Settings
    // 0                    Injection off (Hsan: but not derivation as used by MV ref derivation)
    // 1                    On
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M5)
            context_ptr->global_mv_injection = 1;
        else
            context_ptr->global_mv_injection = 0;
    else if (picture_control_set_ptr->enc_mode <= ENC_M7)
        context_ptr->global_mv_injection = 1;
    else
        context_ptr->global_mv_injection = 0;

#if NEW_NEAREST_NEW_INJECTION
#if NEW_NEAREST_NEW_M1_NREF
    if (picture_control_set_ptr->enc_mode <= ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
#else
    if (picture_control_set_ptr->enc_mode == ENC_M0)
#endif
        context_ptr->new_nearest_near_comb_injection = 1;
    else
        context_ptr->new_nearest_near_comb_injection = 0;
#endif
#if ENHANCED_Nx4_4xN_NEW_MV


#if M2_SC_CANDIDATE
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M2)
            context_ptr->nx4_4xn_parent_mv_injection = 1;
        else
            context_ptr->nx4_4xn_parent_mv_injection = 0;
    else

#elif M3_SC_CANDIDATE
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M3)
            context_ptr->nx4_4xn_parent_mv_injection = 1;
        else
            context_ptr->nx4_4xn_parent_mv_injection = 0;
    else

#endif
#if M1_0_CANDIDATE
       if (picture_control_set_ptr->enc_mode <= ENC_M1)
#else
       if (picture_control_set_ptr->enc_mode == ENC_M0)
#endif
        context_ptr->nx4_4xn_parent_mv_injection = 1;
       else
        context_ptr->nx4_4xn_parent_mv_injection = 0;
#endif
#if M9_NEAR_INJECTION
    // Set NEAR injection
    // Level                Settings
    // 0                    Off
    // 1                    On
    if (picture_control_set_ptr->enc_mode <= ENC_M8)
        context_ptr->near_mv_injection = 1;
    else
        //context_ptr->near_mv_injection = 0;
        context_ptr->near_mv_injection =
        (picture_control_set_ptr->temporal_layer_index == 0) ?
            1 :
            0;
#endif

    // Set warped motion injection
    // Level                Settings
    // 0                    OFF
    // 1                    On
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if NEW_M0_SC
            context_ptr->warped_motion_injection = 0;
#else
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
            context_ptr->warped_motion_injection = 1;
        else
            context_ptr->warped_motion_injection = 0;
#endif
    else
    context_ptr->warped_motion_injection = 1;

    // Set unipred3x3 injection
    // Level                Settings
    // 0                    OFF
    // 1                    ON FULL
    // 2                    Reduced set
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if M2_SC_CANDIDATE
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
#else
        if (picture_control_set_ptr->enc_mode <= ENC_M2)
#endif
            context_ptr->unipred3x3_injection = 1;
#if M4_SC_CANDIDATE_1
        else if (picture_control_set_ptr->enc_mode <= ENC_M3)
#else
        else if (picture_control_set_ptr->enc_mode <= ENC_M4)
#endif
            context_ptr->unipred3x3_injection = 2;
        else
            context_ptr->unipred3x3_injection = 0;
#if M3_0_CANDIDATE && ! m3_ibc_graph
    else if (picture_control_set_ptr->enc_mode <= ENC_M3)
#else
    else if (picture_control_set_ptr->enc_mode <= ENC_M2)
#endif
        context_ptr->unipred3x3_injection = 1;
    else if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->unipred3x3_injection = 2;
    else
        context_ptr->unipred3x3_injection = 0;

    // Set bipred3x3 injection
    // Level                Settings
    // 0                    OFF
    // 1                    ON FULL
    // 2                    Reduced set
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M4)
            context_ptr->bipred3x3_injection = 1;
        else
            context_ptr->bipred3x3_injection = 0;
#if m3_ibc_graph
    else if (picture_control_set_ptr->enc_mode <= ENC_M2)
#else
    else if (picture_control_set_ptr->enc_mode <= ENC_M3)
#endif
        context_ptr->bipred3x3_injection = 1;
    else if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->bipred3x3_injection = 2;
    else
        context_ptr->bipred3x3_injection = 0;

#if PREDICTIVE_ME
    // Level                Settings
    // 0                    Level 0: OFF
    // 1                    Level 1: 7x5 full-pel search + sub-pel refinement off
    // 2                    Level 2: 7x5 full-pel search +  (H + V) sub-pel refinement only = 4 half-pel + 4 quarter-pel = 8 positions + pred_me_distortion to pa_me_distortion deviation on
    // 3                    Level 3: 7x5 full-pel search +  (H + V + D only ~ the best) sub-pel refinement = up to 6 half-pel + up to 6  quarter-pel = up to 12 positions + pred_me_distortion to pa_me_distortion deviation on
    // 4                    Level 4: 7x5 full-pel search +  (H + V + D) sub-pel refinement = 8 half-pel + 8 quarter-pel = 16 positions + pred_me_distortion to pa_me_distortion deviation on
    // 5                    Level 5: 7x5 full-pel search +  (H + V + D) sub-pel refinement = 8 half-pel + 8 quarter-pel = 16 positions + pred_me_distortion to pa_me_distortion deviation off

    if (picture_control_set_ptr->slice_type != I_SLICE)
        // Hsan: kept ON for sc_content_detected as ~5% gain for minecraft clip
#if M2_SC_CANDIDATE
        if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
            if (picture_control_set_ptr->enc_mode <= ENC_M1)
                context_ptr->predictive_me_level = 4;
            else if (picture_control_set_ptr->enc_mode <= ENC_M4)
                context_ptr->predictive_me_level = 2;
            else
                context_ptr->predictive_me_level = 0;
        else
#endif
#if M2_BAD_SLOPE_COMB && !m2_ibc_graph 

        if (picture_control_set_ptr->enc_mode <= ENC_M2)
#else
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
#endif
            context_ptr->predictive_me_level = 4;
        else if (picture_control_set_ptr->enc_mode <= ENC_M4)
            context_ptr->predictive_me_level = 2;
        else
            context_ptr->predictive_me_level = 0;
    else
        context_ptr->predictive_me_level = 0;
#endif

#if AUTO_C1C2
    // Combine MD Class1&2
    // 0                    OFF
    // 1                    ON
#if M1_0_CANDIDATE
    context_ptr->combine_class12 = (picture_control_set_ptr->enc_mode <= ENC_M1) ? 0 : 1;
#else
    context_ptr->combine_class12 = (picture_control_set_ptr->enc_mode == ENC_M0) ? 0 : 1;
#endif
#endif

    // Set interpolation filter search blk size
    // Level                Settings
    // 0                    ON for 8x8 and above
    // 1                    ON for 16x16 and above
    // 2                    ON for 32x32 and above
    if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->interpolation_filter_search_blk_size = 0;
    else
        context_ptr->interpolation_filter_search_blk_size = 1;

#if PF_N2_SUPPORT
    // Set PF MD
    context_ptr->pf_md_mode = PF_OFF;
#endif

#if SPATIAL_SSE
    // Derive Spatial SSE Flag
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M4)
            context_ptr->spatial_sse_full_loop = EB_TRUE;
        else
            context_ptr->spatial_sse_full_loop = EB_FALSE;
    else if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->spatial_sse_full_loop = EB_TRUE;
    else
        context_ptr->spatial_sse_full_loop = EB_FALSE;

#endif

#if M9_INTER_SRC_SRC_FAST_LOOP
    // Derive Spatial SSE Flag
    if (picture_control_set_ptr->enc_mode <= ENC_M8)
        context_ptr->inter_fast_loop_src_src = 0;
    else
        context_ptr->inter_fast_loop_src_src = 1;
#endif

#if BLK_SKIP_DECISION
    if (context_ptr->chroma_level <= CHROMA_MODE_1)
        context_ptr->blk_skip_decision = EB_TRUE;
    else
        context_ptr->blk_skip_decision = EB_FALSE;
#endif
    // Derive Trellis Quant Coeff Optimization Flag
#if M3_SC_CANDIDATE
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M2)
            context_ptr->trellis_quant_coeff_optimization = EB_TRUE;
        else
            context_ptr->trellis_quant_coeff_optimization = EB_FALSE;
    else
#endif
#if M3_0_CANDIDATE
        if (picture_control_set_ptr->enc_mode <= ENC_M3)
#else
        if (picture_control_set_ptr->enc_mode <= ENC_M2)
#endif
            context_ptr->trellis_quant_coeff_optimization = EB_TRUE;
        else
            context_ptr->trellis_quant_coeff_optimization = EB_FALSE;
#if DISABLE_TRELLIS
    context_ptr->trellis_quant_coeff_optimization = EB_FALSE;
#endif

    // Derive redundant block
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if M0_SC_CANDIDATE
        context_ptr->redundant_blk = EB_TRUE;
#else
        if (picture_control_set_ptr->enc_mode >= ENC_M1)
            context_ptr->redundant_blk = EB_TRUE;
        else
            context_ptr->redundant_blk = EB_FALSE;
#endif
#if M0_3_CANDIDATE
    else if (picture_control_set_ptr->enc_mode >= ENC_M0 && picture_control_set_ptr->enc_mode <= ENC_M5)
#else
    else if (picture_control_set_ptr->enc_mode >= ENC_M1 && picture_control_set_ptr->enc_mode <= ENC_M5)
#endif
        context_ptr->redundant_blk = EB_TRUE;
    else
        context_ptr->redundant_blk = EB_FALSE;

#if FULL_LOOP_SPLIT
    // Derive md_staging_mode
#if M1_SC_CANDIDATE
    if ((picture_control_set_ptr->enc_mode == ENC_M0) ||(picture_control_set_ptr->enc_mode <= ENC_M1 &&  picture_control_set_ptr->parent_pcs_ptr->sc_content_detected))
#else
    if (picture_control_set_ptr->enc_mode == ENC_M0)
#endif
        context_ptr->md_staging_mode = 1;
    else if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->md_staging_mode = 3;
    else
        context_ptr->md_staging_mode = 0; //use fast-loop0->full-loop

    // Derive nic level
    context_ptr->nic_level = (picture_control_set_ptr->enc_mode == ENC_M0) ? 0 : 1;

#if USE_MDS3_C1C2_REDUCED_NIC
    context_ptr->md_staging_mode = 3;
    context_ptr->nic_level = 1;
#endif
#endif
#if INTER_INTER_WEDGE_OPT
    // Derive INTER/INTER WEDGE variance TH
    if (MR_MODE)
        context_ptr->inter_inter_wedge_variance_th = 0;
    else //if (picture_control_set_ptr->enc_mode == ENC_M0)
#if ORANGE_SET || BLUE_SET
        context_ptr->inter_inter_wedge_variance_th = 200;
#else
        context_ptr->inter_inter_wedge_variance_th = 100;
#endif
#endif

#if INTER_DEPTH_SKIP_OPT
    // Derive MD Exit TH
    if (MR_MODE)
        context_ptr->md_exit_th = 0;
    else //if (picture_control_set_ptr->enc_mode == ENC_M0)
#if ORANGE_SET
        context_ptr->md_exit_th = 20;
#elif BLUE_SET
        context_ptr->md_exit_th = 15;
#else
        context_ptr->md_exit_th = 10;
#endif
#endif

#if DIST_BASED_COUNT_1_PRONE
    // Derive distortion-based md_stage_0_count proning
    if (MR_MODE)
        context_ptr->dist_base_md_stage_0_count_th = (uint64_t) ~0;
    else //if (picture_control_set_ptr->enc_mode == ENC_M0)
#if ORANGE_SET
        context_ptr->dist_base_md_stage_0_count_th = 25;
#elif BLUE_SET
        context_ptr->dist_base_md_stage_0_count_th = 50;
#else
        context_ptr->dist_base_md_stage_0_count_th = 75;
#endif
#endif
    return return_error;
}

void move_cu_data(
    CodingUnit *src_cu,
    CodingUnit *dst_cu);
#if TWO_PASS_PART_128SUPPORT
uint32_t map_128_to_64_sb_addr(
    SequenceControlSet *sequence_control_set_ptr,
    uint32_t            sb_index,
    uint32_t            cu_origin_x,
    uint32_t            cu_origin_y,
    uint8_t             *quadrant) {
    uint32_t me_sb_size = sequence_control_set_ptr->sb_sz;
    uint32_t me_pic_width_in_sb = (sequence_control_set_ptr->seq_header.max_frame_width + sequence_control_set_ptr->sb_sz - 1) / me_sb_size;
    uint32_t me_pic_height_in_sb = (sequence_control_set_ptr->seq_header.max_frame_height + me_sb_size - 1) / me_sb_size;
    uint32_t sb64_addr;
    if (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128) {
        uint32_t me_sb_size = sequence_control_set_ptr->sb_sz;
        uint32_t me_pic_width_in_sb = (sequence_control_set_ptr->seq_header.max_frame_width + sequence_control_set_ptr->sb_sz - 1) / me_sb_size;
        uint32_t me_pic_height_in_sb = (sequence_control_set_ptr->seq_header.max_frame_height + sequence_control_set_ptr->sb_sz - 1) / me_sb_size;
        uint32_t me_sb_x = (cu_origin_x / me_sb_size);
        uint32_t me_sb_y = (cu_origin_y / me_sb_size);
        sb64_addr = me_sb_x + me_sb_y * me_pic_width_in_sb;
        *quadrant = (me_sb_x % 2) + ((me_sb_y % 2) * 2);
        if (*quadrant > 3)
            printf("Error");
    }
    else {
        sb64_addr = sb_index;
        *quadrant = 0;
    }
    return sb64_addr;
}
#endif
#if MPMD_SB
void reset_local_cu_cost(ModeDecisionContext * context_ptr) {
    for (uint64_t cu_idx = 0; cu_idx < BLOCK_MAX_COUNT_SB_128; cu_idx++) {
        context_ptr->md_local_cu_unit[cu_idx].cost = MAX_MODE_COST;
        //context_ptr->md_cu_arr_nsq[cu_idx]. =
    }
}
void reset_ep_pipe_sb(ModeDecisionContext * context_ptr) {
    for (uint64_t cu_idx = 0; cu_idx < BLOCK_MAX_COUNT_SB_128; cu_idx++) {
        context_ptr->md_ep_pipe_sb[cu_idx].skip_cost = MAX_MODE_COST;
        context_ptr->md_ep_pipe_sb[cu_idx].merge_cost = MAX_MODE_COST;
        context_ptr->md_ep_pipe_sb[cu_idx].chroma_distortion = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].y_full_distortion[0] = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].y_full_distortion[1] = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].y_coeff_bits = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].y_has_coeff = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].fast_luma_rate = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].y_count_non_zero_coeffs[0] = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].y_count_non_zero_coeffs[1] = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].y_count_non_zero_coeffs[2] = 0;
        context_ptr->md_ep_pipe_sb[cu_idx].y_count_non_zero_coeffs[3] = 0;
    }
}

static PART from_part_to_shape[] = {
    PART_N,  //PARTITION_NONE,
    PART_H,  //PARTITION_HORZ,
    PART_V,  //PARTITION_VERT,
    PART_S,  //PARTITION_SPLIT
    PART_HA, //PARTITION_HORZ_A,
    PART_HB, //PARTITION_HORZ_B,
    PART_VA, //PARTITION_VERT_A,
    PART_VB, //PARTITION_VERT_B,
    PART_H4, //PARTITION_HORZ_4,
    PART_V4  //PARTITION_VERT_4
};
void mpmd_set_child_to_be_considered(
    MdcLcuData *cu_ptr,
    uint32_t    blk_index,
    int32_t     sb_size,
    int8_t      depth_step) {
    uint32_t child_block_idx_1, child_block_idx_2, child_block_idx_3, child_block_idx_4;
    uint32_t tot_d1_blocks, block_1d_idx;
    const BlockGeom * blk_geom = get_blk_geom_mds(blk_index);
    tot_d1_blocks =
        blk_geom->sq_size == 128 ? 17 :
        blk_geom->sq_size > 8 ? 25 :
        blk_geom->sq_size == 8 ? 5 : 1;
    if (blk_geom->sq_size > 4) {
        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(blk_index + block_1d_idx);
            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                resultsPtr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
            }*/
            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_TRUE;
        }
        //Set first child to be considered
        child_block_idx_1 = blk_index + d1_depth_offset[sb_size == BLOCK_128X128][blk_geom->depth];
        const BlockGeom * child1_blk_geom = get_blk_geom_mds(child_block_idx_1);
        uint32_t child1_tot_d1_blocks =
            child1_blk_geom->sq_size == 128 ? 17 :
            child1_blk_geom->sq_size > 8 ? 25 :
            child1_blk_geom->sq_size == 8 ? 5 : 1;

        for (block_1d_idx = 0; block_1d_idx < child1_tot_d1_blocks; block_1d_idx++) {
            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(child_block_idx_1 + block_1d_idx);
            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                resultsPtr->leaf_data_array[child_block_idx_1 + block_1d_idx].consider_block = 1;
            }*/
            cu_ptr->leaf_data_array[child_block_idx_1 + block_1d_idx].consider_block = 1;
            cu_ptr->leaf_data_array[child_block_idx_1 + block_1d_idx].refined_split_flag = EB_FALSE;
        }
        if (depth_step > 1)
            mpmd_set_child_to_be_considered(
                cu_ptr,
                child_block_idx_1,
                sb_size,
                depth_step -1);
        //Set second child to be considered
        child_block_idx_2 = child_block_idx_1 + ns_depth_offset[sb_size == BLOCK_128X128][blk_geom->depth + 1];
        const BlockGeom * child2_blk_geom = get_blk_geom_mds(child_block_idx_2);
        uint32_t child2_tot_d1_blocks =
            child2_blk_geom->sq_size == 128 ? 17 :
            child2_blk_geom->sq_size > 8 ? 25 :
            child2_blk_geom->sq_size == 8 ? 5 : 1;
        for (block_1d_idx = 0; block_1d_idx < child2_tot_d1_blocks; block_1d_idx++) {
            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(child_block_idx_2 + block_1d_idx);
            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                resultsPtr->leaf_data_array[child_block_idx_2 + block_1d_idx].consider_block = 1;
            }*/
            cu_ptr->leaf_data_array[child_block_idx_2 + block_1d_idx].consider_block = 1;
            cu_ptr->leaf_data_array[child_block_idx_2 + block_1d_idx].refined_split_flag = EB_FALSE;
        }
        if (depth_step > 1)
            mpmd_set_child_to_be_considered(
                cu_ptr,
                child_block_idx_2,
                sb_size,
                depth_step -1);
        //Set third child to be considered
        child_block_idx_3 = child_block_idx_2 + ns_depth_offset[sb_size == BLOCK_128X128][blk_geom->depth + 1];
        const BlockGeom * child3_blk_geom = get_blk_geom_mds(child_block_idx_3);
        uint32_t child3_tot_d1_blocks =
            child3_blk_geom->sq_size == 128 ? 17 :
            child3_blk_geom->sq_size > 8 ? 25 :
            child3_blk_geom->sq_size == 8 ? 5 : 1;

        for (block_1d_idx = 0; block_1d_idx < child3_tot_d1_blocks; block_1d_idx++) {
            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(child_block_idx_3 + block_1d_idx);
            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                resultsPtr->leaf_data_array[child_block_idx_3 + block_1d_idx].consider_block = 1;
            }*/
            cu_ptr->leaf_data_array[child_block_idx_3 + block_1d_idx].consider_block = 1;
            cu_ptr->leaf_data_array[child_block_idx_3 + block_1d_idx].refined_split_flag = EB_FALSE;
        }
        if (depth_step > 1)
            mpmd_set_child_to_be_considered(
                cu_ptr,
                child_block_idx_3,
                sb_size,
                depth_step -1);
        //Set forth child to be considered
        child_block_idx_4 = child_block_idx_3 + ns_depth_offset[sb_size == BLOCK_128X128][blk_geom->depth + 1];
        const BlockGeom * child4_blk_geom = get_blk_geom_mds(child_block_idx_4);
        uint32_t child4_tot_d1_blocks =
            child4_blk_geom->sq_size == 128 ? 17 :
            child4_blk_geom->sq_size > 8 ? 25 :
            child4_blk_geom->sq_size == 8 ? 5 : 1;
        for (block_1d_idx = 0; block_1d_idx < child4_tot_d1_blocks; block_1d_idx++) {
            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(child_block_idx_4 + block_1d_idx);
            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                resultsPtr->leaf_data_array[child_block_idx_4 + block_1d_idx].consider_block = 1;
            }*/
            cu_ptr->leaf_data_array[child_block_idx_4 + block_1d_idx].consider_block = 1;
            cu_ptr->leaf_data_array[child_block_idx_4 + block_1d_idx].refined_split_flag = EB_FALSE;
        }
        if (depth_step > 1)
            mpmd_set_child_to_be_considered(
                cu_ptr,
                child_block_idx_4,
                sb_size,
                depth_step -1);
    }
}
void mpmd_init_considered_block(
    SequenceControlSet  *sequence_control_set_ptr,
    PictureControlSet   *picture_control_set_ptr,
    ModeDecisionContext *context_ptr,
    uint32_t            sb_index,
    uint8_t             mpmd_depth_level,
    uint8_t             mpmd_1d_level) {
    MdcLcuData *cu_ptr = &picture_control_set_ptr->mpmd_sb_array[sb_index];
    cu_ptr->leaf_count = 0;
    uint32_t  blk_index = 0;
    uint32_t  parent_depth_idx_mds,sparent_depth_idx_mds,child_block_idx_1,child_block_idx_2,child_block_idx_3,child_block_idx_4;
    uint8_t  is_complete_sb = sequence_control_set_ptr->sb_geom[sb_index].is_complete_sb;

    uint32_t tot_d1_blocks,block_1d_idx;
    EbBool split_flag;
    uint32_t depth_refinement_mode = AllD;
    switch (mpmd_depth_level) {
    case 0:
        depth_refinement_mode = is_complete_sb ? Pred : AllD;
        break;
    case 1:
        depth_refinement_mode = is_complete_sb ? Predp1 : AllD;
        break;
    case 2:
        depth_refinement_mode = is_complete_sb ? Predp2 : AllD;
        break;
    case 3:
        depth_refinement_mode = is_complete_sb ? Predp3 : AllD;
        break;
    case 4:
        depth_refinement_mode = is_complete_sb ? Predm1p1 : AllD;
        break;
    case 5:
        depth_refinement_mode = is_complete_sb ? Predm1p2 : AllD;
        break;
    case 6:
        depth_refinement_mode = is_complete_sb ? Predm1p3 : AllD;
        break;
    case 7:
        depth_refinement_mode = is_complete_sb ? Predm2p3 : AllD;
        break;
    case MAX_MDC_LEVEL:
        depth_refinement_mode = AllD;
        break;
    default:
        printf("not supported mdc_depth_level");
        break;
    }

    while (blk_index < sequence_control_set_ptr->max_block_cnt){
        const BlockGeom * blk_geom = get_blk_geom_mds(blk_index);
        tot_d1_blocks =
                blk_geom->sq_size == 128 ? 17 :
                blk_geom->sq_size > 8 ? 25 :
                blk_geom->sq_size == 8 ? 5 : 1;
        //if the parentSq is inside inject this block
        uint8_t is_blk_allowed = picture_control_set_ptr->slice_type != I_SLICE ? 1 : (blk_geom->sq_size < 128) ? 1 : 0;

        if(depth_refinement_mode == AllD)
            split_flag = blk_geom->sq_size > 4 ? EB_TRUE : EB_FALSE;
        else
            split_flag = context_ptr->md_cu_arr_nsq[blk_index].split_flag;
        if (sequence_control_set_ptr->sb_geom[sb_index].block_is_inside_md_scan[blk_index] && is_blk_allowed){
            if (blk_geom->shape == PART_N) {
                switch (depth_refinement_mode) {
                case Pred:
                    // Set predicted block to be considered
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE) {
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            const BlockGeom * blk_geom_1d = get_blk_geom_mds(blk_index + block_1d_idx);
#if MPMD_SB_1PART_IN_FP
                            if (mpmd_1d_level == 0) {
                                if (blk_geom_1d->shape == from_part_to_shape[context_ptr->md_cu_arr_nsq[blk_geom_1d->sqi_mds].part]) {
                                    cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                                }
                            }else
                                cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
#else
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
#endif
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                    }
                    break;
                case Predp1:
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE){
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(blk_index + block_1d_idx);
                            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                resultsPtr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            }*/
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        mpmd_set_child_to_be_considered(
                            cu_ptr,
                            blk_index,
                            sequence_control_set_ptr->seq_header.sb_size,
                            1);
                    }
                    break;
                case Predp2:
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE){
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(blk_index + block_1d_idx);
                            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                resultsPtr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            }*/
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        mpmd_set_child_to_be_considered(
                            cu_ptr,
                            blk_index,
                            sequence_control_set_ptr->seq_header.sb_size,
                            2);
                    }
                    break;
                case Predp3:
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE){
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(blk_index + block_1d_idx);
                            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                resultsPtr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            }*/
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        mpmd_set_child_to_be_considered(
                            cu_ptr,
                            blk_index,
                            sequence_control_set_ptr->seq_header.sb_size,
                            3);
                    }
                    break;
                case Predm1p2:
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE) {
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        if (blk_geom->sq_size < (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128 ? 128 : 64) && blk_geom->sq_size > 4) {
                            //Set parent to be considered
                            parent_depth_idx_mds = (blk_geom->sqi_mds - (blk_geom->quadi - 3) * ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth]) - parent_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
                            const BlockGeom * parent_blk_geom = get_blk_geom_mds(parent_depth_idx_mds);
                            uint32_t parent_tot_d1_blocks =
                                parent_blk_geom->sq_size == 128 ? 17 :
                                parent_blk_geom->sq_size > 8 ? 25 :
                                parent_blk_geom->sq_size == 8 ? 5 : 1;
                            for (block_1d_idx = 0; block_1d_idx < parent_tot_d1_blocks; block_1d_idx++) {
                                cu_ptr->leaf_data_array[parent_depth_idx_mds + block_1d_idx].consider_block = 1;
                            }
                        }
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(blk_index + block_1d_idx);
                            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                resultsPtr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            }*/
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        mpmd_set_child_to_be_considered(
                            cu_ptr,
                            blk_index,
                            sequence_control_set_ptr->seq_header.sb_size,
                            2);
                    }
                    break;
                case Predm1p3:
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE) {
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        if (blk_geom->sq_size < (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128 ? 128 : 64) && blk_geom->sq_size > 4) {
                            //Set parent to be considered
                            parent_depth_idx_mds = (blk_geom->sqi_mds - (blk_geom->quadi - 3) * ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth]) - parent_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
                            const BlockGeom * parent_blk_geom = get_blk_geom_mds(parent_depth_idx_mds);
                            uint32_t parent_tot_d1_blocks =
                                parent_blk_geom->sq_size == 128 ? 17 :
                                parent_blk_geom->sq_size > 8 ? 25 :
                                parent_blk_geom->sq_size == 8 ? 5 : 1;
                            for (block_1d_idx = 0; block_1d_idx < parent_tot_d1_blocks; block_1d_idx++) {
                                cu_ptr->leaf_data_array[parent_depth_idx_mds + block_1d_idx].consider_block = 1;
                            }
                        }
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(blk_index + block_1d_idx);
                            if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                resultsPtr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            }*/
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        mpmd_set_child_to_be_considered(
                            cu_ptr,
                            blk_index,
                            sequence_control_set_ptr->seq_header.sb_size,
                            3);
                    }
                    break;
                case Predm2p3:
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE) {
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        if (blk_geom->sq_size < (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128 ? 128 : 64) && blk_geom->sq_size > 4) {
                            //Set parent to be considered
                            parent_depth_idx_mds = (blk_geom->sqi_mds - (blk_geom->quadi - 3) * ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth]) - parent_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
                            const BlockGeom * parent_blk_geom = get_blk_geom_mds(parent_depth_idx_mds);
                            uint32_t parent_tot_d1_blocks =
                                parent_blk_geom->sq_size == 128 ? 17 :
                                parent_blk_geom->sq_size > 8 ? 25 :
                                parent_blk_geom->sq_size == 8 ? 5 : 1;
                            for (block_1d_idx = 0; block_1d_idx < parent_tot_d1_blocks; block_1d_idx++) {
                                cu_ptr->leaf_data_array[parent_depth_idx_mds + block_1d_idx].consider_block = 1;
                            }

                            if (parent_blk_geom->sq_size < (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128 ? 128 : 64) && parent_blk_geom->sq_size > 4) {
                                //Set parent to be considered
                                sparent_depth_idx_mds = (parent_blk_geom->sqi_mds - (parent_blk_geom->quadi - 3) * ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][parent_blk_geom->depth]) - parent_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][parent_blk_geom->depth];
                                const BlockGeom * sparent_blk_geom = get_blk_geom_mds(sparent_depth_idx_mds);
                                uint32_t sparent_tot_d1_blocks =
                                    parent_blk_geom->sq_size == 128 ? 17 :
                                    parent_blk_geom->sq_size > 8 ? 25 :
                                    parent_blk_geom->sq_size == 8 ? 5 : 1;
                                for (block_1d_idx = 0; block_1d_idx < sparent_tot_d1_blocks; block_1d_idx++) {
                                    cu_ptr->leaf_data_array[sparent_depth_idx_mds + block_1d_idx].consider_block = 1;
                                }
                            }
                        }
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        mpmd_set_child_to_be_considered(
                            cu_ptr,
                            blk_index,
                            sequence_control_set_ptr->seq_header.sb_size,
                            3);
                    }
                    break;
                case Predm1:
                    // Set predicted block to be considered
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE) {
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        if (blk_geom->sq_size < (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128 ? 128 : 64) && blk_geom->sq_size > 4) {
                            //Set parent to be considered
                            parent_depth_idx_mds = (blk_geom->sqi_mds - (blk_geom->quadi - 3) * ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth]) - parent_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
                            const BlockGeom * parent_blk_geom = get_blk_geom_mds(parent_depth_idx_mds);
                            uint32_t parent_tot_d1_blocks =
                                parent_blk_geom->sq_size == 128 ? 17 :
                                parent_blk_geom->sq_size > 8 ? 25 :
                                parent_blk_geom->sq_size == 8 ? 5 : 1;
                            for (block_1d_idx = 0; block_1d_idx < parent_tot_d1_blocks; block_1d_idx++) {
                                cu_ptr->leaf_data_array[parent_depth_idx_mds + block_1d_idx].consider_block = 1;
                            }
                        }
                    }
                    break;
                case Predm1p1:
                    // Set predicted block to be considered
                    if (context_ptr->md_cu_arr_nsq[blk_index].split_flag == EB_FALSE) {
                        for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_FALSE;
                        }
                        if (blk_geom->sq_size < (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128 ? 128 : 64) && blk_geom->sq_size > 4) {
                            //Set parent to be considered
                            parent_depth_idx_mds = (blk_geom->sqi_mds - (blk_geom->quadi - 3) * ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth]) - parent_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
                            const BlockGeom * parent_blk_geom = get_blk_geom_mds(parent_depth_idx_mds);
                            uint32_t parent_tot_d1_blocks =
                                parent_blk_geom->sq_size == 128 ? 17 :
                                parent_blk_geom->sq_size > 8 ? 25 :
                                parent_blk_geom->sq_size == 8 ? 5 : 1;
                            for (block_1d_idx = 0; block_1d_idx < parent_tot_d1_blocks; block_1d_idx++) {
                                cu_ptr->leaf_data_array[parent_depth_idx_mds + block_1d_idx].consider_block = 1;
                            }
                        }

                        if (blk_geom->sq_size > 4) {
                            // Set predicted block to be considered
                            for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                                /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(blk_index + block_1d_idx);
                                if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                    resultsPtr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                                }*/
                                cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                                cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = EB_TRUE;
                            }
                            //Set first child to be considered
                            child_block_idx_1 = blk_index + d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
                            const BlockGeom * child1_blk_geom = get_blk_geom_mds(child_block_idx_1);
                            uint32_t child1_tot_d1_blocks =
                                child1_blk_geom->sq_size == 128 ? 17 :
                                child1_blk_geom->sq_size > 8 ? 25 :
                                child1_blk_geom->sq_size == 8 ? 5 : 1;

                            for (block_1d_idx = 0; block_1d_idx < child1_tot_d1_blocks; block_1d_idx++) {
                                /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(child_block_idx_1 + block_1d_idx);
                                if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                    resultsPtr->leaf_data_array[child_block_idx_1 + block_1d_idx].consider_block = 1;
                                }*/
                                cu_ptr->leaf_data_array[child_block_idx_1 + block_1d_idx].consider_block = 1;
                                cu_ptr->leaf_data_array[child_block_idx_1 + block_1d_idx].refined_split_flag = EB_FALSE;
                            }
                            //Set second child to be considered
                            child_block_idx_2 = child_block_idx_1 + ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth +1];
                            const BlockGeom * child2_blk_geom = get_blk_geom_mds(child_block_idx_2);
                            uint32_t child2_tot_d1_blocks =
                                child2_blk_geom->sq_size == 128 ? 17 :
                                child2_blk_geom->sq_size > 8 ? 25 :
                                child2_blk_geom->sq_size == 8 ? 5 : 1;
                            for (block_1d_idx = 0; block_1d_idx < child2_tot_d1_blocks; block_1d_idx++) {
                                /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(child_block_idx_2 + block_1d_idx);
                                if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                    resultsPtr->leaf_data_array[child_block_idx_2 + block_1d_idx].consider_block = 1;
                                }*/
                                cu_ptr->leaf_data_array[child_block_idx_2 + block_1d_idx].consider_block = 1;
                                cu_ptr->leaf_data_array[child_block_idx_2 + block_1d_idx].refined_split_flag = EB_FALSE;
                            }
                            //Set third child to be considered
                            child_block_idx_3 = child_block_idx_2 + ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth +1];
                            const BlockGeom * child3_blk_geom = get_blk_geom_mds(child_block_idx_3);
                            uint32_t child3_tot_d1_blocks =
                                child3_blk_geom->sq_size == 128 ? 17 :
                                child3_blk_geom->sq_size > 8 ? 25 :
                                child3_blk_geom->sq_size == 8 ? 5 : 1;

                            for (block_1d_idx = 0; block_1d_idx < child3_tot_d1_blocks; block_1d_idx++) {
                                /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(child_block_idx_3 + block_1d_idx);
                                if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                    resultsPtr->leaf_data_array[child_block_idx_3 + block_1d_idx].consider_block = 1;
                                }*/
                                cu_ptr->leaf_data_array[child_block_idx_3 + block_1d_idx].consider_block = 1;
                                cu_ptr->leaf_data_array[child_block_idx_3 + block_1d_idx].refined_split_flag = EB_FALSE;
                            }
                            //Set forth child to be considered
                            child_block_idx_4 = child_block_idx_3 + ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth + 1];
                            const BlockGeom * child4_blk_geom = get_blk_geom_mds(child_block_idx_4);
                            uint32_t child4_tot_d1_blocks =
                                child4_blk_geom->sq_size == 128 ? 17 :
                                child4_blk_geom->sq_size > 8 ? 25 :
                                child4_blk_geom->sq_size == 8 ? 5 : 1;
                            for (block_1d_idx = 0; block_1d_idx < child4_tot_d1_blocks; block_1d_idx++) {
                                /*const BlockGeom * blk_geom_1d = get_blk_geom_mds(child_block_idx_4 + block_1d_idx);
                                if (blk_geom_1d->shape == PART_H || blk_geom_1d->sq_size == 4) {
                                    resultsPtr->leaf_data_array[child_block_idx_4 + block_1d_idx].consider_block = 1;
                                }*/
                                cu_ptr->leaf_data_array[child_block_idx_4 + block_1d_idx].consider_block = 1;
                                cu_ptr->leaf_data_array[child_block_idx_4 + block_1d_idx].refined_split_flag = EB_FALSE;
                            }
                        }
                    }
                    break;
                case AllD:
                    // Set all block to be considered
                    for (block_1d_idx = 0; block_1d_idx < tot_d1_blocks; block_1d_idx++) {
                        cu_ptr->leaf_data_array[blk_index + block_1d_idx].consider_block = 1;
                        cu_ptr->leaf_data_array[blk_index + block_1d_idx].refined_split_flag = blk_geom->sq_size > 4 ? EB_TRUE : EB_FALSE;
                    }
                    break;
                default:
                    printf("Error! invalid mdc_refinement_mode\n");
                    break;
                }
            }
        }
        blk_index += split_flag ? d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] : ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
    }
}
void check_for_errors(
    uint64_t max_block_cnt,
    MdcLcuData* src_mdc,
    MdcLcuData* dst_mdc,
    ModeDecisionContext *context_ptr) {
    
    printf("stage N-1: %d,stage N: %d\n", dst_mdc->leaf_count, src_mdc->leaf_count);

    if (dst_mdc->leaf_count < src_mdc->leaf_count) printf("error leaf_count\n");
 
}
void init_cu_arr_nsq(
    uint64_t max_block_cnt,
    ModeDecisionContext *context_ptr,
    uint32_t sb_index) {
    
    uint32_t blk_index = 0;
    while (blk_index < max_block_cnt){
        const BlockGeom * blk_geom = get_blk_geom_mds(blk_index);
        context_ptr->md_cu_arr_nsq[blk_index].interintra_mode = blk_geom->sq_size > 4 ? EB_TRUE : EB_FALSE;
        context_ptr->md_cu_arr_nsq[blk_index].interintra_mode = 0;
        context_ptr->md_cu_arr_nsq[blk_index].is_interintra_used = 0;
        context_ptr->md_cu_arr_nsq[blk_index].use_wedge_interintra =0;
        context_ptr->md_cu_arr_nsq[blk_index].interintra_wedge_index =0;//inter_intra wedge index
        context_ptr->md_cu_arr_nsq[blk_index].ii_wedge_sign = 0;
        
//        TransformUnit             transform_unit_array[TRANSFORM_UNIT_MAX_COUNT]; // 2-bytes * 21 = 42-bytes
//        PredictionUnit            prediction_unit_array[MAX_NUM_OF_PU_PER_CU];    // 35-bytes * 4 = 140 bytes
//#if COMP_MODE
//        INTERINTER_COMPOUND_DATA               interinter_comp;
//        uint8_t                                compound_idx;
//        uint8_t                                comp_group_idx;
//#endif
        context_ptr->md_cu_arr_nsq[blk_index].skip_flag_context = 0;
        context_ptr->md_cu_arr_nsq[blk_index].prediction_mode_flag=0;
        context_ptr->md_cu_arr_nsq[blk_index].block_has_coeff=0;
        context_ptr->md_cu_arr_nsq[blk_index].split_flag_context=0;

        context_ptr->md_cu_arr_nsq[blk_index].qp = 0;
        context_ptr->md_cu_arr_nsq[blk_index].ref_qp = 0;
        context_ptr->md_cu_arr_nsq[blk_index].delta_qp = 0; 
        context_ptr->md_cu_arr_nsq[blk_index].org_delta_qp = 0;

        // Coded Tree
        context_ptr->md_cu_arr_nsq[blk_index].leaf_index=0;
        context_ptr->md_cu_arr_nsq[blk_index].split_flag = 0;
        context_ptr->md_cu_arr_nsq[blk_index].skip_flag = 0;
        context_ptr->md_cu_arr_nsq[blk_index].mdc_split_flag = 0;

        //MacroBlockD                *av1xd;
        // uint8_t ref_mv_count[MODE_CTX_REF_FRAMES];
        for (int i = 0; i < MODE_CTX_REF_FRAMES; i++) {
            context_ptr->md_cu_arr_nsq[blk_index].inter_mode_ctx[i];
            context_ptr->md_cu_arr_nsq[blk_index].ref_mvs[i][0];
            context_ptr->md_cu_arr_nsq[blk_index].ref_mvs[i][1];
        }
        context_ptr->md_cu_arr_nsq[blk_index].drl_index = 0;
        context_ptr->md_cu_arr_nsq[blk_index].pred_mode = 0;
        context_ptr->md_cu_arr_nsq[blk_index].predmv[0].as_int = 0;
        context_ptr->md_cu_arr_nsq[blk_index].predmv[1].as_int = 0;
        context_ptr->md_cu_arr_nsq[blk_index].skip_coeff_context = 0;
        context_ptr->md_cu_arr_nsq[blk_index].reference_mode_context = 0;
        context_ptr->md_cu_arr_nsq[blk_index].compoud_reference_type_context = 0;
        for (int i = 0; i < 3; i++) {
            memset(context_ptr->md_cu_arr_nsq[blk_index].quantized_dc[i], 0, sizeof(int32_t) * MAX_TXB_COUNT);
        }

        context_ptr->md_cu_arr_nsq[blk_index].is_inter_ctx = 0;
        context_ptr->md_cu_arr_nsq[blk_index].interp_filters = 0;
        context_ptr->md_cu_arr_nsq[blk_index].part = 0;
        context_ptr->md_cu_arr_nsq[blk_index].shape = 0;
        context_ptr->md_cu_arr_nsq[blk_index].mds_idx = 0;     //equivalent of leaf_index in the nscu context. we will keep both for now and use the right one on a case by case basis.
        //uint8_t                    *neigh_left_recon[3];  //only for MD
        //uint8_t                    *neigh_top_recon[3];
        context_ptr->md_cu_arr_nsq[blk_index].best_d1_blk = 0;
        context_ptr->md_cu_arr_nsq[blk_index].tx_depth = 0;

        blk_index++;
    }
}
void copy_split_flag(
    uint64_t max_block_cnt,
    MdcLcuData* src_mdc,
    MdcLcuData* dst_mdc,
    ModeDecisionContext *context_ptr) {
    dst_mdc->leaf_count = src_mdc->leaf_count;
    EbMdcLeafData *src = src_mdc->leaf_data_array;
    EbMdcLeafData *dst = dst_mdc->leaf_data_array;
    uint64_t blk_index = 0;
    while (blk_index < max_block_cnt){
        uint32_t mds_idx = context_ptr->md_cu_arr_nsq[blk_index].mds_idx;
        src[src[blk_index].mds_idx].split_flag = context_ptr->md_cu_arr_nsq[mds_idx].split_flag;
        blk_index++;
    }
}
void copy_mdc_array_data(
    uint64_t max_block_cnt,
    MdcLcuData* src_mdc,
    MdcLcuData* dst_mdc) {
    dst_mdc->leaf_count = src_mdc->leaf_count;
    EbMdcLeafData *src = src_mdc->leaf_data_array;
    EbMdcLeafData *dst = dst_mdc->leaf_data_array;
    uint64_t blk_index = 0;
    while (blk_index < max_block_cnt){
        dst[blk_index].mds_idx = src[blk_index].mds_idx;
        dst[blk_index].split_flag = src[blk_index].split_flag;
        dst[blk_index].early_split_flag = src[blk_index].early_split_flag;
        dst[blk_index].tot_d1_blocks = src[blk_index].tot_d1_blocks;
        dst[blk_index].leaf_index = src[blk_index].leaf_index;
        dst[blk_index].consider_block = src[blk_index].consider_block;
        dst[blk_index].ol_best_nsq_shape1 = src[blk_index].ol_best_nsq_shape1;
        dst[blk_index].ol_best_nsq_shape2 = src[blk_index].ol_best_nsq_shape2;
        dst[blk_index].ol_best_nsq_shape3 = src[blk_index].ol_best_nsq_shape3;
        dst[blk_index].ol_best_nsq_shape4 = src[blk_index].ol_best_nsq_shape4;
        dst[blk_index].ol_best_nsq_shape5 = src[blk_index].ol_best_nsq_shape5;
        dst[blk_index].ol_best_nsq_shape6 = src[blk_index].ol_best_nsq_shape6;
        dst[blk_index].ol_best_nsq_shape7 = src[blk_index].ol_best_nsq_shape7;
        dst[blk_index].ol_best_nsq_shape8 = src[blk_index].ol_best_nsq_shape8;
        dst[blk_index].open_loop_ranking = src[blk_index].open_loop_ranking;
        dst[blk_index].refined_split_flag = src[blk_index].refined_split_flag;
        blk_index++;
    }
}
void mpmd_init_split_flags(
    SequenceControlSet  *sequence_control_set_ptr,
    PictureControlSet   *picture_control_set_ptr,
    uint32_t            sb_index) {
    MdcLcuData *cu_ptr = &picture_control_set_ptr->mpmd_sb_array[sb_index];
    MdcLcuData *mdc_cu_ptr = &picture_control_set_ptr->mdc_sb_array[sb_index];
    uint32_t  blk_index = 0;
    copy_mdc_array_data(sequence_control_set_ptr->max_block_cnt,mdc_cu_ptr,cu_ptr);
    while (blk_index < sequence_control_set_ptr->max_block_cnt) {
        const BlockGeom * blk_geom = get_blk_geom_mds(blk_index);
        cu_ptr->leaf_data_array[blk_index].consider_block = 0;
        cu_ptr->leaf_data_array[blk_index].split_flag = blk_geom->sq_size > 4 ? EB_TRUE : EB_FALSE;
        cu_ptr->leaf_data_array[blk_index].refined_split_flag = blk_geom->sq_size > 4 ? EB_TRUE : EB_FALSE;
        blk_index++;
    }
}
void mpmd_forward_considered_blocks(
    SequenceControlSet *sequence_control_set_ptr,
    PictureControlSet *picture_control_set_ptr,
    ModeDecisionContext *context_ptr,
    uint32_t sb_index) {
    MdcLcuData *cu_ptr = &picture_control_set_ptr->mpmd_sb_array[sb_index];
    cu_ptr->leaf_count = 0;
    uint32_t  blk_index = 0;
    uint32_t d1_blocks_accumlated,tot_d1_blocks,d1_block_idx;
    EbBool split_flag;
    while (blk_index < sequence_control_set_ptr->max_block_cnt)
    {
        const BlockGeom * blk_geom = get_blk_geom_mds(blk_index);
        split_flag = blk_geom->sq_size > 4 ? EB_TRUE : EB_FALSE;
        //if the parent sq is inside inject this block
        uint8_t is_blk_allowed = picture_control_set_ptr->slice_type != I_SLICE ? 1 : (blk_geom->sq_size < 128) ? 1 : 0;
        //init consider block flag
        if (sequence_control_set_ptr->sb_geom[sb_index].block_is_inside_md_scan[blk_index] && is_blk_allowed)
        {
            tot_d1_blocks =  blk_geom->sq_size == 128 ? 17 :
                blk_geom->sq_size > 8 ? 25 :
                blk_geom->sq_size == 8 ? 5 : 1;
            d1_blocks_accumlated = 0;
            for (d1_block_idx = 0; d1_block_idx < tot_d1_blocks; d1_block_idx++) {
                d1_blocks_accumlated += cu_ptr->leaf_data_array[blk_index + d1_block_idx].consider_block ? 1 : 0;
            }

            for (uint32_t idx = 0; idx < tot_d1_blocks; ++idx) {
                if (cu_ptr->leaf_data_array[blk_index].consider_block) {
                    cu_ptr->leaf_data_array[cu_ptr->leaf_count].tot_d1_blocks = d1_blocks_accumlated;
                    cu_ptr->leaf_data_array[cu_ptr->leaf_count].leaf_index = 0;//valid only for square 85 world. will be removed.
                    cu_ptr->leaf_data_array[cu_ptr->leaf_count].mds_idx = blk_index;
                    split_flag = cu_ptr->leaf_data_array[blk_index].refined_split_flag;
                    cu_ptr->leaf_data_array[cu_ptr->leaf_count++].split_flag = split_flag;
                }
                blk_index++;
            }
        }
        blk_index += split_flag ? d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] - tot_d1_blocks : ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] - tot_d1_blocks;
        //blk_index += (d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] - tot_d1_blocks);
    }
}
/******************************************************
* Set mode decision settings
******************************************************/
EbErrorType mpmd_settings(
    SequenceControlSet    *sequence_control_set_ptr,
    PictureControlSet     *picture_control_set_ptr,
    uint8_t                enc_mode,
    uint8_t                is_last_md_pass,
    ModeDecisionContext   *context_ptr) {
    EbErrorType return_error = EB_ErrorNone;
#if MOVE_TX_LEVELS_SIGNAL_UNDER_CTX
    // Tx_search Level                                Settings
    // 0                                              OFF
    // 1                                              Tx search at encdec
    // 2                                              Tx search at inter-depth
    // 3                                              Tx search at full loop
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M6)
            context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
        else
            if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
            else
                context_ptr->tx_search_level = TX_SEARCH_ENC_DEC;
    else if (enc_mode <= ENC_M4)
        context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
    else if (enc_mode <= ENC_M7) {
        if (picture_control_set_ptr->temporal_layer_index == 0)
            context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
        else
            context_ptr->tx_search_level = TX_SEARCH_ENC_DEC;
    }
    else
        context_ptr->tx_search_level = TX_SEARCH_ENC_DEC;

    // Set tx search skip weights (MAX_MODE_COST: no skipping; 0: always skipping)
#if FIX_TX_SEARCH_FOR_MR_MODE
    if (MR_MODE) // tx weight
        context_ptr->tx_weight = MAX_MODE_COST;
#endif
    else{
        if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
            context_ptr->tx_weight = MAX_MODE_COST;
        else if (!MR_MODE && enc_mode <= ENC_M5)
            context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
        else if (!MR_MODE){
            if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
            else
                context_ptr->tx_weight = FC_SKIP_TX_SR_TH010;
        }
    }

    // Set tx search reduced set falg (0: full tx set; 1: reduced tx set; 1: two tx))
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if SC_M8_TX_REDUCED_SET_
            picture_control_set_ptr->tx_search_reduced_set = 2;
#else
        if (enc_mode <= ENC_M5)
            context_ptr->tx_search_reduced_set = 0;
        else if (enc_mode <= ENC_M6)
            if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
                context_ptr->tx_search_reduced_set = 0;
            else
                context_ptr->tx_search_reduced_set = 1;
        else if (enc_mode <= ENC_M7)
            context_ptr->tx_search_reduced_set = 1;
        else
            context_ptr->tx_search_reduced_set = 2;
#endif
    else

    if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
        context_ptr->tx_search_reduced_set = 0;
#if M2_BAD_SLOPE_COMB && ! m2_ibc_graph
    else if (enc_mode <= ENC_M2)
#else
    else if (enc_mode <= ENC_M1)
#endif
        context_ptr->tx_search_reduced_set = 0;
    else if (enc_mode <= ENC_M5)
        context_ptr->tx_search_reduced_set = 1;
    else
        context_ptr->tx_search_reduced_set = 1;

    // Set skip tx search based on NFL falg (0: Skip OFF ; 1: skip ON)
    context_ptr->skip_tx_search = 0;
#endif
#if MOVE_IF_LEVELS_SIGNAL_UNDER_CTX
    // Interpolation search Level                     Settings
    // 0                                              OFF
    // 1                                              Interpolation search at inter-depth
    // 2                                              Interpolation search at full loop
    // 3                                              Chroma blind interpolation search at fast loop
    // 4                                              Interpolation search at fast loop

        if (MR_MODE) // Interpolation
            context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP;
        else if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if NEW_M0_SC
            context_ptr->interpolation_search_level = IT_SEARCH_OFF;
#else
            if (enc_mode <= ENC_M1)
                context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
            else
                context_ptr->interpolation_search_level = IT_SEARCH_OFF;
#endif

        else if (enc_mode <= ENC_M3)
            context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
        else if (enc_mode <= ENC_M7)
            if (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0)
                context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
            else
                context_ptr->interpolation_search_level = IT_SEARCH_OFF;
        else
            context_ptr->interpolation_search_level = IT_SEARCH_OFF;
#endif
#if ADD_FLAG_FOR_SKIP_NEW_MV_FEATURE
    context_ptr->skip_newmv_basedon_parent_sq_has_coeff = 1;
#endif
#if MPMD_LOWER_NSQ_LEVEL_IN_FP
    // [0-6] with 0 is the fastest and 6 is the slowest.
    context_ptr->nsq_max_shapes_md = picture_control_set_ptr->parent_pcs_ptr->nsq_max_shapes_md;
#endif
    // NFL Level MD       Settings
    // 0                  MAX_NFL 40
    // 1                  30
    // 2                  12
    // 3                  10
    // 4                  8
    // 5                  6
    // 6                  4
    // 7                  3
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M5)
            if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                context_ptr->nfl_level = (sequence_control_set_ptr->input_resolution <= INPUT_SIZE_576p_RANGE_OR_LOWER) ? 0 : 1;
            else
                context_ptr->nfl_level = 2;
        else
            if (picture_control_set_ptr->parent_pcs_ptr->slice_type == I_SLICE)
                context_ptr->nfl_level = 5;
            else if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                context_ptr->nfl_level = 6;
            else
                context_ptr->nfl_level = 7;
    else
    if (enc_mode <= ENC_M5)
        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
            context_ptr->nfl_level = (sequence_control_set_ptr->input_resolution <= INPUT_SIZE_576p_RANGE_OR_LOWER) ? 0 : 1;
        else
            context_ptr->nfl_level = 2;
    else if(enc_mode <= ENC_M5)
        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
            context_ptr->nfl_level = 2;
        else
            context_ptr->nfl_level = 4;
    else if (enc_mode <= ENC_M6)
        if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
            context_ptr->nfl_level = 4;
        else
            context_ptr->nfl_level = 5;
    else
        if (picture_control_set_ptr->parent_pcs_ptr->slice_type == I_SLICE)
            context_ptr->nfl_level = 5;
        else if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
            context_ptr->nfl_level = 6;
        else
            context_ptr->nfl_level = 7;

    // Set Chroma Mode
    // Level                Settings
    // CHROMA_MODE_0  0     Full chroma search @ MD
    // CHROMA_MODE_1  1     Fast chroma search @ MD
    // CHROMA_MODE_2  2     Chroma blind @ MD + CFL @ EP
    // CHROMA_MODE_3  3     Chroma blind @ MD + no CFL @ EP
#if SEARCH_UV_MODE
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M6)
            context_ptr->chroma_level = CHROMA_MODE_1;
        else
            if (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0)
                context_ptr->chroma_level = CHROMA_MODE_1;
            else
                context_ptr->chroma_level = (sequence_control_set_ptr->encoder_bit_depth == EB_8BIT) ?
                CHROMA_MODE_2 :
                CHROMA_MODE_3;
    else
#if CHROMA_SEARCH_MR
    if (MR_MODE || USE_MR_CHROMA) // chroma
        context_ptr->chroma_level = CHROMA_MODE_0;
    else
#endif
#if SEARCH_UV_BASE
    if (enc_mode <= ENC_M5 && picture_control_set_ptr->temporal_layer_index == 0)
#else
    if (enc_mode == ENC_M0)
#endif
        context_ptr->chroma_level = CHROMA_MODE_0;
    else
#endif
    if (enc_mode <= ENC_M5)
        context_ptr->chroma_level = CHROMA_MODE_1;
    else
        context_ptr->chroma_level = (sequence_control_set_ptr->encoder_bit_depth == EB_8BIT) ?
            CHROMA_MODE_2 :
            CHROMA_MODE_3 ;

    // Set fast loop method
    // 1 fast loop: SSD_SEARCH not supported
    // Level                Settings
    //  0                   Collapsed fast loop
    //  1                   Decoupled fast loops ( intra/inter)
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M5)
            context_ptr->decouple_intra_inter_fast_loop = 0;
        else
            context_ptr->decouple_intra_inter_fast_loop = 1;
    else
    context_ptr->decouple_intra_inter_fast_loop = 0;

    // Set the search method when decoupled fast loop is used
    // Hsan: FULL_SAD_SEARCH not supported
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M5)
            context_ptr->decoupled_fast_loop_search_method = SSD_SEARCH;
        else
            context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;
    else if (enc_mode <= ENC_M4)
            context_ptr->decoupled_fast_loop_search_method = SSD_SEARCH;
        else
            context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;

    // Set the full loop escape level
    // Level                Settings
    // 0                    Off
    // 1                    On but only INTRA
    // 2                    On both INTRA and INTER
#if M9_FULL_LOOP_ESCAPE
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M5)
            context_ptr->full_loop_escape = 0;
        else
            context_ptr->full_loop_escape = 2;
    else if (enc_mode <= ENC_M6)
        context_ptr->full_loop_escape = 0;
    else
        context_ptr->full_loop_escape = 2;
#else
    if (enc_mode <= ENC_M7)
        context_ptr->full_loop_escape = 0;
    else
        context_ptr->full_loop_escape = 1;
#endif

    // Set global MV injection
    // Level                Settings
    // 0                    Injection off (Hsan: but not derivation as used by MV ref derivation)
    // 1                    On
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M5)
            context_ptr->global_mv_injection = 1;
        else
            context_ptr->global_mv_injection = 0;
    else if (enc_mode <= ENC_M7)
        context_ptr->global_mv_injection = 1;
    else
        context_ptr->global_mv_injection = 0;

#if NEW_NEAREST_NEW_INJECTION
#if NEW_NEAREST_NEW_M1_NREF
    if (enc_mode <= ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
#else
    if (enc_mode == ENC_M0)
#endif
        context_ptr->new_nearest_near_comb_injection = 1;
    else
        context_ptr->new_nearest_near_comb_injection = 0;
#endif
#if ENHANCED_Nx4_4xN_NEW_MV
#if M1_0_CANDIDATE
    if (enc_mode <= ENC_M1)
#else
    if (enc_mode == ENC_M0)
#endif
        context_ptr->nx4_4xn_parent_mv_injection = 1;
    else
        context_ptr->nx4_4xn_parent_mv_injection = 0;
#endif
#if M9_NEAR_INJECTION
    // Set NEAR injection
    // Level                Settings
    // 0                    Off
    // 1                    On
    if (enc_mode <= ENC_M8)
        context_ptr->near_mv_injection = 1;
    else
        //context_ptr->near_mv_injection = 0;
        context_ptr->near_mv_injection =
        (picture_control_set_ptr->temporal_layer_index == 0) ?
            1 :
            0;
#endif

    // Set warped motion injection
    // Level                Settings
    // 0                    OFF
    // 1                    On
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if NEW_M0_SC
            context_ptr->warped_motion_injection = 0;
#else
        if (enc_mode <= ENC_M1)
            context_ptr->warped_motion_injection = 1;
        else
            context_ptr->warped_motion_injection = 0;
#endif
    else
    context_ptr->warped_motion_injection = 1;

    // Set unipred3x3 injection
    // Level                Settings
    // 0                    OFF
    // 1                    ON FULL
    // 2                    Reduced set
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M2)
            context_ptr->unipred3x3_injection = 1;
        else if (enc_mode <= ENC_M4)
            context_ptr->unipred3x3_injection = 2;
        else
            context_ptr->unipred3x3_injection = 0;
#if M3_0_CANDIDATE && ! m3_ibc_graph
    else if (enc_mode <= ENC_M3)
#else
    else if (enc_mode <= ENC_M2)
#endif
        context_ptr->unipred3x3_injection = 1;
    else if (enc_mode <= ENC_M4)
        context_ptr->unipred3x3_injection = 2;
    else
        context_ptr->unipred3x3_injection = 0;

    // Set bipred3x3 injection
    // Level                Settings
    // 0                    OFF
    // 1                    ON FULL
    // 2                    Reduced set
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M4)
            context_ptr->bipred3x3_injection = 1;
        else
            context_ptr->bipred3x3_injection = 0;
#if m3_ibc_graph
    else if (enc_mode <= ENC_M2)
#else
    else if (enc_mode <= ENC_M3)
#endif
        context_ptr->bipred3x3_injection = 1;
    else if (enc_mode <= ENC_M4)
        context_ptr->bipred3x3_injection = 2;
    else
        context_ptr->bipred3x3_injection = 0;

#if PREDICTIVE_ME
    // Level                Settings
    // 0                    Level 0: OFF
    // 1                    Level 1: 7x5 full-pel search + sub-pel refinement off
    // 2                    Level 2: 7x5 full-pel search +  (H + V) sub-pel refinement only = 4 half-pel + 4 quarter-pel = 8 positions + pred_me_distortion to pa_me_distortion deviation on
    // 3                    Level 3: 7x5 full-pel search +  (H + V + D only ~ the best) sub-pel refinement = up to 6 half-pel + up to 6  quarter-pel = up to 12 positions + pred_me_distortion to pa_me_distortion deviation on
    // 4                    Level 4: 7x5 full-pel search +  (H + V + D) sub-pel refinement = 8 half-pel + 8 quarter-pel = 16 positions + pred_me_distortion to pa_me_distortion deviation on
    // 5                    Level 5: 7x5 full-pel search +  (H + V + D) sub-pel refinement = 8 half-pel + 8 quarter-pel = 16 positions + pred_me_distortion to pa_me_distortion deviation off

    if (picture_control_set_ptr->slice_type != I_SLICE)
        // Hsan: kept ON for sc_content_detected as ~5% gain for minecraft clip
#if M2_BAD_SLOPE_COMB && !m2_ibc_graph
        if (enc_mode <= ENC_M2)
#else
        if (enc_mode <= ENC_M1)
#endif
            context_ptr->predictive_me_level = 4;
        else if (enc_mode <= ENC_M4)
            context_ptr->predictive_me_level = 2;
        else
            context_ptr->predictive_me_level = 0;
    else
        context_ptr->predictive_me_level = 0;
#endif

#if AUTO_C1C2
    // Combine MD Class1&2
    // 0                    OFF
    // 1                    ON
#if M1_0_CANDIDATE
    context_ptr->combine_class12 = (enc_mode <= ENC_M1) ? 0 : 1;
#else
    context_ptr->combine_class12 = (enc_mode == ENC_M0) ? 0 : 1;
#endif
#endif

    // Set interpolation filter search blk size
    // Level                Settings
    // 0                    ON for 8x8 and above
    // 1                    ON for 16x16 and above
    // 2                    ON for 32x32 and above
    if (enc_mode <= ENC_M4)
        context_ptr->interpolation_filter_search_blk_size = 0;
    else
        context_ptr->interpolation_filter_search_blk_size = 1;

#if PF_N2_SUPPORT
    // Set PF MD
    context_ptr->pf_md_mode = PF_OFF;
#endif

#if SPATIAL_SSE
    // Derive Spatial SSE Flag
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M4)
            context_ptr->spatial_sse_full_loop = EB_TRUE;
        else
            context_ptr->spatial_sse_full_loop = EB_FALSE;
    else if (enc_mode <= ENC_M4)
        context_ptr->spatial_sse_full_loop = EB_TRUE;
    else
        context_ptr->spatial_sse_full_loop = EB_FALSE;

#endif

#if M9_INTER_SRC_SRC_FAST_LOOP
    // Derive Spatial SSE Flag
    if (enc_mode <= ENC_M8)
        context_ptr->inter_fast_loop_src_src = 0;
    else
        context_ptr->inter_fast_loop_src_src = 1;
#endif

#if BLK_SKIP_DECISION
    if (context_ptr->chroma_level <= CHROMA_MODE_1)
        context_ptr->blk_skip_decision = EB_TRUE;
    else
        context_ptr->blk_skip_decision = EB_FALSE;
#endif
    // Derive Trellis Quant Coeff Optimization Flag
#if M3_0_CANDIDATE 
    if (enc_mode <= ENC_M3)
#else
    if (enc_mode <= ENC_M2)
#endif
        context_ptr->trellis_quant_coeff_optimization = EB_TRUE;
    else
        context_ptr->trellis_quant_coeff_optimization = EB_FALSE;
#if DISABLE_TRELLIS
    context_ptr->trellis_quant_coeff_optimization = EB_FALSE;
#endif

    // Derive redundant block
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
#if M0_SC_CANDIDATE
        context_ptr->redundant_blk = EB_TRUE;
#else
        if (enc_mode >= ENC_M1)
            context_ptr->redundant_blk = EB_TRUE;
        else
            context_ptr->redundant_blk = EB_FALSE;
#endif
#if M0_3_CANDIDATE
    else if (enc_mode >= ENC_M0 && enc_mode <= ENC_M5)
#else
    else if (enc_mode >= ENC_M1 && enc_mode <= ENC_M5)
#endif
        context_ptr->redundant_blk = EB_TRUE;
    else
        context_ptr->redundant_blk = EB_FALSE;

#if FULL_LOOP_SPLIT
    // Derive md_staging_mode
    if (enc_mode == ENC_M0)
        context_ptr->md_staging_mode = 1;
    else if (enc_mode <= ENC_M4)
        context_ptr->md_staging_mode = 3;
    else
        context_ptr->md_staging_mode = 0; //use fast-loop0->full-loop

    // Derive nic level
    context_ptr->nic_level = (enc_mode == ENC_M0) ? 0 : 1;

#if USE_MDS3_C1C2_REDUCED_NIC
    context_ptr->md_staging_mode = 3;
    context_ptr->nic_level = 1;
#endif
#endif
#if INTER_INTER_WEDGE_OPT
    // Derive INTER/INTER WEDGE variance TH
    if (MR_MODE)
        context_ptr->inter_inter_wedge_variance_th = 0;
    else //if (enc_mode == ENC_M0)
#if ORANGE_SET || BLUE_SET
        context_ptr->inter_inter_wedge_variance_th = 200;
#else
        context_ptr->inter_inter_wedge_variance_th = 100;
#endif
#endif

#if INTER_DEPTH_SKIP_OPT
    // Derive MD Exit TH
    if (MR_MODE)
        context_ptr->md_exit_th = 0;
    else //if (enc_mode == ENC_M0)
#if ORANGE_SET
        context_ptr->md_exit_th = 20;
#elif BLUE_SET
        context_ptr->md_exit_th = 15;
#else
        context_ptr->md_exit_th = 10;
#endif
#endif

#if DIST_BASED_COUNT_1_PRONE
    // Derive distortion-based md_stage_0_count proning
    if (MR_MODE)
        context_ptr->dist_base_md_stage_0_count_th = (uint64_t) ~0;
    else //if (enc_mode == ENC_M0)
#if ORANGE_SET
        context_ptr->dist_base_md_stage_0_count_th = 25;
#elif BLUE_SET
        context_ptr->dist_base_md_stage_0_count_th = 50;
#else
        context_ptr->dist_base_md_stage_0_count_th = 75;
#endif
#endif
#if MPMD_TEST
    if (!is_last_md_pass) {
        context_ptr->skip_newmv_basedon_parent_sq_has_coeff = 1;
        context_ptr->nsq_max_shapes_md = 1;
        context_ptr->tx_search_reduced_set = 2;
        context_ptr->tx_search_level = TX_SEARCH_OFF;
        context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
        context_ptr->interpolation_search_level = IT_SEARCH_OFF;
    }
    else {
        context_ptr->skip_newmv_basedon_parent_sq_has_coeff = 0;
        /*context_ptr->nsq_max_shapes_md = 8;
        context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
        context_ptr->tx_weight = MAX_MODE_COST;
        context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;*/
    }
#endif
    return return_error;
}
#if MPMD_TEST
uint8_t md_mode_settings_offset[13] = { 8,0,0,0,0,0,0,0,0,0,0,0,0 };
#else
uint8_t md_mode_settings_offset[13] = { 0,0,0,0,0,0,0,0,0,0,0,0,0 };
#endif
EbErrorType mpmd_pass_settings(
    SequenceControlSet    *sequence_control_set_ptr,
    PictureControlSet     *picture_control_set_ptr,
    uint8_t                is_last_md_pass,
    ModeDecisionContext   *context_ptr) {
    EbErrorType return_error = EB_ErrorNone;
    uint8_t enc_mode = picture_control_set_ptr->enc_mode;
    uint8_t md_mode = enc_mode;
    if (!is_last_md_pass) {
        md_mode = md_mode + md_mode_settings_offset[enc_mode];
    }
    mpmd_settings(
        sequence_control_set_ptr,
        picture_control_set_ptr,
        md_mode,
        is_last_md_pass,
        context_ptr);
    return return_error;
}
#endif
#if CABAC_UP
void av1_estimate_syntax_rate___partial(
    MdRateEstimationContext        *md_rate_estimation_array,
    FRAME_CONTEXT                  *fc);
#endif
/******************************************************
 * EncDec Kernel
 ******************************************************/
void* enc_dec_kernel(void *input_ptr)
{
    // Context & SCS & PCS
    EncDecContext                         *context_ptr = (EncDecContext*)input_ptr;
    PictureControlSet                     *picture_control_set_ptr;
    SequenceControlSet                    *sequence_control_set_ptr;

    // Input
    EbObjectWrapper                       *encDecTasksWrapperPtr;
    EncDecTasks                           *encDecTasksPtr;

    // Output
    EbObjectWrapper                       *encDecResultsWrapperPtr;
    EncDecResults                         *encDecResultsPtr;
    // SB Loop variables
    LargestCodingUnit                       *sb_ptr;
    uint16_t                                 sb_index;
    uint8_t                                  sb_sz;
    uint8_t                                  lcuSizeLog2;
    uint32_t                                 x_lcu_index;
    uint32_t                                 y_lcu_index;
    uint32_t                                 sb_origin_x;
    uint32_t                                 sb_origin_y;
    EbBool                                   lastLcuFlag;
    EbBool                                   endOfRowFlag;
    uint32_t                                 lcuRowIndexStart;
    uint32_t                                 lcuRowIndexCount;
    uint32_t                                 picture_width_in_sb;
    MdcLcuData                              *mdcPtr;
#if MPMD_SB
    MdcLcuData                              *mpmd_sb;
#endif

    // Variables
    EbBool                                   is16bit;

    // Segments
    //EbBool                                 initialProcessCall;
    uint16_t                                 segment_index;
    uint32_t                                 xLcuStartIndex;
    uint32_t                                 yLcuStartIndex;
    uint32_t                                 lcuStartIndex;
    uint32_t                                 lcuSegmentCount;
    uint32_t                                 lcuSegmentIndex;
    uint32_t                                 segmentRowIndex;
    uint32_t                                 segmentBandIndex;
    uint32_t                                 segmentBandSize;
    EncDecSegments                          *segments_ptr;
    for (;;) {
        // Get Mode Decision Results
        eb_get_full_object(
            context_ptr->mode_decision_input_fifo_ptr,
            &encDecTasksWrapperPtr);

        encDecTasksPtr = (EncDecTasks*)encDecTasksWrapperPtr->object_ptr;
        picture_control_set_ptr = (PictureControlSet*)encDecTasksPtr->picture_control_set_wrapper_ptr->object_ptr;
        sequence_control_set_ptr = (SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr;
        segments_ptr = picture_control_set_ptr->enc_dec_segment_ctrl;
        lastLcuFlag = EB_FALSE;
        is16bit = (EbBool)(sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);
        (void)is16bit;
        (void)endOfRowFlag;

        // EncDec Kernel Signal(s) derivation

        signal_derivation_enc_dec_kernel_oq(
            sequence_control_set_ptr,
            picture_control_set_ptr,
#if MOVE_TX_LEVELS_SIGNAL_UNDER_CTX
            context_ptr,
#endif
            context_ptr->md_context);

        // SB Constants
        sb_sz = (uint8_t)sequence_control_set_ptr->sb_size_pix;
        lcuSizeLog2 = (uint8_t)Log2f(sb_sz);
        context_ptr->sb_sz = sb_sz;
        picture_width_in_sb = (sequence_control_set_ptr->seq_header.max_frame_width + sb_sz - 1) >> lcuSizeLog2;
        endOfRowFlag = EB_FALSE;
        lcuRowIndexStart = lcuRowIndexCount = 0;
        context_ptr->tot_intra_coded_area = 0;

        // Segment-loop
        while (AssignEncDecSegments(segments_ptr, &segment_index, encDecTasksPtr, context_ptr->enc_dec_feedback_fifo_ptr) == EB_TRUE)
        {
            xLcuStartIndex = segments_ptr->x_start_array[segment_index];
            yLcuStartIndex = segments_ptr->y_start_array[segment_index];
            lcuStartIndex = yLcuStartIndex * picture_width_in_sb + xLcuStartIndex;
            lcuSegmentCount = segments_ptr->valid_lcu_count_array[segment_index];

            segmentRowIndex = segment_index / segments_ptr->segment_band_count;
            segmentBandIndex = segment_index - segmentRowIndex * segments_ptr->segment_band_count;
            segmentBandSize = (segments_ptr->lcu_band_count * (segmentBandIndex + 1) + segments_ptr->segment_band_count - 1) / segments_ptr->segment_band_count;

            // Reset Coding Loop State
            reset_mode_decision( // HT done
                context_ptr->md_context,
                picture_control_set_ptr,
#if EIGTH_PEL_MV
                sequence_control_set_ptr,
#endif
                segment_index);

            // Reset EncDec Coding State
            ResetEncDec(    // HT done
                context_ptr,
                picture_control_set_ptr,
                sequence_control_set_ptr,
                segment_index);

            if (picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr != NULL)
                ((EbReferenceObject  *)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->average_intensity = picture_control_set_ptr->parent_pcs_ptr->average_intensity[0];
#if !MEMORY_FOOTPRINT_OPT
            if (sequence_control_set_ptr->static_config.improve_sharpness) {
                QpmDeriveWeightsMinAndMax(
                    picture_control_set_ptr,
                    context_ptr);
            }
#endif
            for (y_lcu_index = yLcuStartIndex, lcuSegmentIndex = lcuStartIndex; lcuSegmentIndex < lcuStartIndex + lcuSegmentCount; ++y_lcu_index) {
                for (x_lcu_index = xLcuStartIndex; x_lcu_index < picture_width_in_sb && (x_lcu_index + y_lcu_index < segmentBandSize) && lcuSegmentIndex < lcuStartIndex + lcuSegmentCount; ++x_lcu_index, ++lcuSegmentIndex) {
                    sb_index = (uint16_t)(y_lcu_index * picture_width_in_sb + x_lcu_index);
                    sb_ptr = picture_control_set_ptr->sb_ptr_array[sb_index];
                    sb_origin_x = x_lcu_index << lcuSizeLog2;
                    sb_origin_y = y_lcu_index << lcuSizeLog2;
                    lastLcuFlag = (sb_index == sequence_control_set_ptr->sb_tot_cnt - 1) ? EB_TRUE : EB_FALSE;
                    endOfRowFlag = (x_lcu_index == picture_width_in_sb - 1) ? EB_TRUE : EB_FALSE;
                    lcuRowIndexStart = (x_lcu_index == picture_width_in_sb - 1 && lcuRowIndexCount == 0) ? y_lcu_index : lcuRowIndexStart;
                    lcuRowIndexCount = (x_lcu_index == picture_width_in_sb - 1) ? lcuRowIndexCount + 1 : lcuRowIndexCount;
                    mdcPtr = &picture_control_set_ptr->mdc_sb_array[sb_index];
#if MPMD_SB
                    mpmd_sb = &picture_control_set_ptr->mpmd_sb_array[sb_index];
#endif
                    context_ptr->sb_index = sb_index;
                    context_ptr->md_context->cu_use_ref_src_flag = (picture_control_set_ptr->parent_pcs_ptr->use_src_ref) && (picture_control_set_ptr->parent_pcs_ptr->edge_results_ptr[sb_index].edge_block_num == EB_FALSE || picture_control_set_ptr->parent_pcs_ptr->sb_flat_noise_array[sb_index]) ? EB_TRUE : EB_FALSE;

#if CABAC_UP
                    if (picture_control_set_ptr->update_cdf) {
#if ENABLE_CDF_UPDATE
                        picture_control_set_ptr->rate_est_array[sb_index] = *picture_control_set_ptr->md_rate_estimation_array;
#else
                        MdRateEstimationContext* md_rate_estimation_array = sequence_control_set_ptr->encode_context_ptr->md_rate_estimation_array;
                        md_rate_estimation_array += picture_control_set_ptr->slice_type * TOTAL_NUMBER_OF_QP_VALUES + context_ptr->md_context->qp;

                        //this is temp, copy all default tables
                        picture_control_set_ptr->rate_est_array[sb_index] = *md_rate_estimation_array;
#endif
#if CABAC_SERIAL
                        if (sb_index == 0)
                            picture_control_set_ptr->ec_ctx_array[sb_index] = *picture_control_set_ptr->coeff_est_entropy_coder_ptr->fc;
                        else
                            picture_control_set_ptr->ec_ctx_array[sb_index] = picture_control_set_ptr->ec_ctx_array[sb_index - 1];
#else
                        if (sb_origin_x == 0)
                            picture_control_set_ptr->ec_ctx_array[sb_index] = *picture_control_set_ptr->coeff_est_entropy_coder_ptr->fc;
                        else
                            picture_control_set_ptr->ec_ctx_array[sb_index] = picture_control_set_ptr->ec_ctx_array[sb_index - 1];
#endif

                        //construct the tables using the latest CDFs : Coeff Only here ---to check if I am using all the uptodate CDFs here
                        av1_estimate_syntax_rate___partial(
                            &picture_control_set_ptr->rate_est_array[sb_index],
                            &picture_control_set_ptr->ec_ctx_array[sb_index]);

                        av1_estimate_coefficients_rate(
                            &picture_control_set_ptr->rate_est_array[sb_index],
                            &picture_control_set_ptr->ec_ctx_array[sb_index]);

                        //let the candidate point to the new rate table.
                        uint32_t  candidateIndex;
                        for (candidateIndex = 0; candidateIndex < MODE_DECISION_CANDIDATE_MAX_COUNT; ++candidateIndex)
                            context_ptr->md_context->fast_candidate_ptr_array[candidateIndex]->md_rate_estimation_ptr = &picture_control_set_ptr->rate_est_array[sb_index];
                    }
#endif
                    // Configure the LCU
                    mode_decision_configure_lcu(
                        context_ptr->md_context,
#if !QPM
                        sb_ptr,
#endif
                        picture_control_set_ptr,
#if !QPM
                        sequence_control_set_ptr,
#endif
                        (uint8_t)context_ptr->qp,
                        (uint8_t)sb_ptr->qp);

                    uint32_t lcuRow;
                    if (picture_control_set_ptr->parent_pcs_ptr->enable_in_loop_motion_estimation_flag) {
                        EbPictureBufferDesc       *input_picture_ptr;

                        input_picture_ptr = picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr;

                        // Load the SB from the input to the intermediate SB buffer
                        uint32_t bufferIndex = (input_picture_ptr->origin_y + sb_origin_y) * input_picture_ptr->stride_y + input_picture_ptr->origin_x + sb_origin_x;

                        // Copy the source superblock to the me local buffer
                        uint32_t sb_height = (sequence_control_set_ptr->seq_header.max_frame_height - sb_origin_y) < MAX_SB_SIZE ? sequence_control_set_ptr->seq_header.max_frame_height - sb_origin_y : MAX_SB_SIZE;
                        uint32_t sb_width = (sequence_control_set_ptr->seq_header.max_frame_width - sb_origin_x) < MAX_SB_SIZE ? sequence_control_set_ptr->seq_header.max_frame_width - sb_origin_x : MAX_SB_SIZE;
                        uint32_t is_complete_sb = sequence_control_set_ptr->sb_geom[sb_index].is_complete_sb;

                        if (!is_complete_sb)
                            memset(context_ptr->ss_mecontext->sb_buffer, 0, MAX_SB_SIZE*MAX_SB_SIZE);
                        for (lcuRow = 0; lcuRow < sb_height; lcuRow++) {
                            EB_MEMCPY((&(context_ptr->ss_mecontext->sb_buffer[lcuRow * MAX_SB_SIZE])), (&(input_picture_ptr->buffer_y[bufferIndex + lcuRow * input_picture_ptr->stride_y])), sb_width * sizeof(uint8_t));
                        }

                        context_ptr->ss_mecontext->sb_src_ptr = &(context_ptr->ss_mecontext->sb_buffer[0]);
                        context_ptr->ss_mecontext->sb_src_stride = context_ptr->ss_mecontext->sb_buffer_stride;
                        // Set in-loop ME Search Area
                        int16_t mv_l0_x;
                        int16_t mv_l0_y;
                        int16_t mv_l1_x;
                        int16_t mv_l1_y;

#if MRP_ME
                        mv_l0_x = 0;
                        mv_l0_y = 0;
                        mv_l1_x = 0;
                        mv_l1_y = 0;
#else
                        uint32_t me_sb_addr;
                        if (sequence_control_set_ptr->sb_size == BLOCK_128X128) {
                            uint32_t me_sb_size = sequence_control_set_ptr->sb_sz;
                            uint32_t me_pic_width_in_sb = (sequence_control_set_ptr->seq_header.frame_width_bits + me_sb_size - 1) / me_sb_size;
                            uint32_t me_pic_height_in_sb = (sequence_control_set_ptr->seq_header.max_frame_height + me_sb_size - 1) / me_sb_size;
                            uint32_t me_sb_x = (sb_origin_x / me_sb_size);
                            uint32_t me_sb_y = (sb_origin_y / me_sb_size);
                            uint32_t me_sb_addr_0 = me_sb_x + me_sb_y * me_pic_width_in_sb;
                            uint32_t me_sb_addr_1 = (me_sb_x + 1) < me_pic_width_in_sb ? (me_sb_x + 1) + ((me_sb_y + 0) * me_pic_width_in_sb) : me_sb_addr_0;
                            uint32_t me_sb_addr_2 = (me_sb_y + 1) < me_pic_height_in_sb ? (me_sb_x + 0) + ((me_sb_y + 1) * me_pic_width_in_sb) : me_sb_addr_0;
                            uint32_t me_sb_addr_3 = ((me_sb_x + 1) < me_pic_width_in_sb) && ((me_sb_y + 1) < me_pic_height_in_sb) ? (me_sb_x + 1) + ((me_sb_y + 1) * me_pic_width_in_sb) : me_sb_addr_0;

                            MeCuResults * me_block_results_0 = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr_0][0];
                            MeCuResults * me_block_results_1 = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr_1][0];
                            MeCuResults * me_block_results_2 = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr_2][0];
                            MeCuResults * me_block_results_3 = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr_3][0];

                            // Compute average open_loop 64x64 mvs
                            mv_l0_x = ((me_block_results_0->x_mv_l0 + me_block_results_1->x_mv_l0 + me_block_results_2->x_mv_l0 + me_block_results_3->x_mv_l0) >> 2) >> 2;
                            mv_l0_y = ((me_block_results_0->y_mv_l0 + me_block_results_1->y_mv_l0 + me_block_results_2->y_mv_l0 + me_block_results_3->y_mv_l0) >> 2) >> 2;
                            mv_l1_x = ((me_block_results_0->x_mv_l1 + me_block_results_1->x_mv_l1 + me_block_results_2->x_mv_l1 + me_block_results_3->x_mv_l1) >> 2) >> 2;
                            mv_l1_y = ((me_block_results_0->y_mv_l1 + me_block_results_1->y_mv_l1 + me_block_results_2->y_mv_l1 + me_block_results_3->y_mv_l1) >> 2) >> 2;
                        }
                        else {
                            me_sb_addr = sb_index;
                            MeCuResults * mePuResult = &picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr][0];

                            mv_l0_x = mePuResult->x_mv_l0 >> 2;
                            mv_l0_y = mePuResult->y_mv_l0 >> 2;
                            mv_l1_x = mePuResult->x_mv_l1 >> 2;
                            mv_l1_y = mePuResult->y_mv_l1 >> 2;
                        }
#endif

                        context_ptr->ss_mecontext->search_area_width = 64;
                        context_ptr->ss_mecontext->search_area_height = 64;

                        // perform in-loop ME
                        in_loop_motion_estimation_sblock(
                            picture_control_set_ptr,
                            sb_origin_x,
                            sb_origin_y,
                            mv_l0_x,
                            mv_l0_y,
                            mv_l1_x,
                            mv_l1_y,
                            context_ptr->ss_mecontext);
                    }

#if MPMD_SB
                    uint8_t mpmd_pass_num = 1;
                    uint8_t mpmd_pass_idx;
                    uint8_t  is_complete_sb = sequence_control_set_ptr->sb_geom[sb_index].is_complete_sb;
                        if (picture_control_set_ptr->slice_type != I_SLICE && is_complete_sb) {
                            for (mpmd_pass_idx = 0; mpmd_pass_idx < mpmd_pass_num; ++mpmd_pass_idx) {
#if TEST_DEPTH_REFINEMENT
                                uint8_t mpmd_depth_level = 4; // 1 SQ PART only
#else
                                uint8_t mpmd_depth_level = 0; // 1 SQ PART only
#endif
                                uint8_t mpmd_1d_level = 0; // 1 NSQ PART only

                                init_cu_arr_nsq(
                                    sequence_control_set_ptr->max_block_cnt,
                                    context_ptr->md_context,
                                    sb_index);
#if MPMD_SB_REF
                                mpmd_pass_settings(
                                    sequence_control_set_ptr,
                                    picture_control_set_ptr,
                                    0, //is_last_md_pass,
                                    context_ptr->md_context);
                                mpmd_init_split_flags(
                                    sequence_control_set_ptr,
                                    picture_control_set_ptr,
                                    sb_index);
#endif
                                // Save a clean copy of the neighbor arrays 
                                copy_neighbour_arrays(
                                    picture_control_set_ptr,
                                    context_ptr->md_context,
                                    MD_NEIGHBOR_ARRAY_INDEX, 4,
                                    0,
                                    sb_origin_x,
                                    sb_origin_y);
                                // Init local cu and ep_pipe
                                reset_local_cu_cost(context_ptr->md_context);
                                reset_ep_pipe_sb(context_ptr->md_context);

                                // mode decision for sb
                                mode_decision_sb(
                                    0,//is_last_adp_stage
                                    sequence_control_set_ptr,
                                    picture_control_set_ptr,
                                    mdcPtr,
#if MPMD_SB
                                    mpmd_sb,
#endif
                                    sb_ptr,
                                    sb_origin_x,
                                    sb_origin_y,
                                    sb_index,
                                    context_ptr->ss_mecontext,
                                    context_ptr->md_context);
                                // Restore the clean copy of the neighbor arrays for next md pass
                                copy_neighbour_arrays(
                                    picture_control_set_ptr,
                                    context_ptr->md_context,
                                    4, MD_NEIGHBOR_ARRAY_INDEX,
                                    0,
                                    sb_origin_x,
                                    sb_origin_y);
#if MPMD_SB_REF
                                // Mark blocks to be concidered in next md pass
                                mpmd_init_considered_block(
                                    sequence_control_set_ptr,
                                    picture_control_set_ptr,
                                    context_ptr->md_context,
                                    sb_index,
                                    mpmd_depth_level,
                                    mpmd_1d_level);
                                // Set split flags and block indexes to be tested in next md pass
                                mpmd_forward_considered_blocks(
                                    sequence_control_set_ptr,
                                    picture_control_set_ptr,
                                    context_ptr->md_context,
                                    sb_index);
                                //check_for_errors(sequence_control_set_ptr->max_block_cnt,mpmd_sb, mdcPtr,context_ptr->md_context);
                                //Update mdc array for next stage
                                copy_mdc_array_data(sequence_control_set_ptr->max_block_cnt, mpmd_sb, mdcPtr);
#endif

                            }
                            // Reset for final pass
                            reset_local_cu_cost(context_ptr->md_context);
                            reset_ep_pipe_sb(context_ptr->md_context);
                            init_cu_arr_nsq(
                                sequence_control_set_ptr->max_block_cnt,
                                context_ptr->md_context,
                                sb_index);
#if MPMD_SB_REF
                            mpmd_pass_settings(
                                sequence_control_set_ptr,
                                picture_control_set_ptr,
                                1, //is_last_md_pass,
                                context_ptr->md_context);
#endif
                        }
#endif
                    // Final mode decision pass
                    mode_decision_sb(
#if MPMD_SB
                        1,
#endif
                        sequence_control_set_ptr,
                        picture_control_set_ptr,
                        mdcPtr,
#if MPMD_SB
                        mpmd_sb,
#endif
                        sb_ptr,
                        sb_origin_x,
                        sb_origin_y,
                        sb_index,
                        context_ptr->ss_mecontext,
                        context_ptr->md_context);

                    // Configure the LCU
                    EncDecConfigureLcu(
                        context_ptr,
                        sb_ptr,
                        picture_control_set_ptr,
#if !QPM
                        sequence_control_set_ptr,
#endif
                        (uint8_t)context_ptr->qp,
                        (uint8_t)sb_ptr->qp);

#if NO_ENCDEC
                    no_enc_dec_pass(
                        sequence_control_set_ptr,
                        picture_control_set_ptr,
                        sb_ptr,
                        sb_index,
                        sb_origin_x,
                        sb_origin_y,
                        sb_ptr->qp,
                        context_ptr);
#else
                    // Encode Pass
                    av1_encode_pass(
                        sequence_control_set_ptr,
                        picture_control_set_ptr,
                        sb_ptr,
                        sb_index,
                        sb_origin_x,
                        sb_origin_y,
#if !MEMORY_FOOTPRINT_OPT
                        sb_ptr->qp,
#endif
                        context_ptr);
#endif

                    if (picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr != NULL)
                        ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->intra_coded_area_sb[sb_index] = (uint8_t)((100 * context_ptr->intra_coded_area_sb[sb_index]) / (64 * 64));
#if TWO_PASS_PART
                    // Store the split flag of the square blocks from the first pass
                    if (sequence_control_set_ptr->static_config.use_output_stat_file) {
                        eb_block_on_mutex(picture_control_set_ptr->first_pass_split_mutex);
                        uint32_t sq_idx = 0;
                        EbBool split_flag;
                        uint32_t blk_index = 0;
                        while (blk_index < sequence_control_set_ptr->max_block_cnt){
                            const BlockGeom * blk_geom = get_blk_geom_mds(blk_index);
                            //if the parentSq is inside inject this block
                            uint8_t is_blk_allowed = picture_control_set_ptr->slice_type != I_SLICE ? 1 : (blk_geom->sq_size < 128) ? 1 : 0;

                            if (blk_geom->shape == PART_N && blk_geom->sq_size > 4) {
                                picture_control_set_ptr->parent_pcs_ptr->stat_struct_first_pass_ptr->first_pass_split_flag[sb_index][sq_idx++] = context_ptr->first_pass_split_flag[sb_index][blk_index];
                                if (sq_idx > NUMBER_OF_SPLIT_FLAG)
                                    printf("Error: EbEncDecProcess: number of sq_idx > NUMBER_OF_SPLIT_FLAG not sufficient\n");
                            }
                            split_flag = context_ptr->first_pass_split_flag[sb_index][blk_index];
                            blk_index += split_flag ? d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] : ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
                        }
                        eb_release_mutex(picture_control_set_ptr->first_pass_split_mutex);
                    }
#endif
                }
                xLcuStartIndex = (xLcuStartIndex > 0) ? xLcuStartIndex - 1 : 0;
            }
        }

        eb_block_on_mutex(picture_control_set_ptr->intra_mutex);
        picture_control_set_ptr->intra_coded_area += (uint32_t)context_ptr->tot_intra_coded_area;
        eb_release_mutex(picture_control_set_ptr->intra_mutex);

        if (lastLcuFlag) {
            // Copy film grain data from parent picture set to the reference object for further reference
            if (sequence_control_set_ptr->seq_header.film_grain_params_present)
            {
                if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE && picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr) {
                    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->film_grain_params
                        = picture_control_set_ptr->parent_pcs_ptr->film_grain_params;
                }
            }

#if ENABLE_CDF_UPDATE
            if (picture_control_set_ptr->parent_pcs_ptr->frame_end_cdf_update_mode && picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE && picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr)
                for (int frame = LAST_FRAME; frame <= ALTREF_FRAME; ++frame)
                    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->global_motion[frame]
                        = picture_control_set_ptr->parent_pcs_ptr->global_motion[frame];
#endif

            EB_MEMCPY(picture_control_set_ptr->parent_pcs_ptr->av1x->sgrproj_restore_cost, context_ptr->md_rate_estimation_ptr->sgrproj_restore_fac_bits, 2 * sizeof(int32_t));
            EB_MEMCPY(picture_control_set_ptr->parent_pcs_ptr->av1x->switchable_restore_cost, context_ptr->md_rate_estimation_ptr->switchable_restore_fac_bits, 3 * sizeof(int32_t));
            EB_MEMCPY(picture_control_set_ptr->parent_pcs_ptr->av1x->wiener_restore_cost, context_ptr->md_rate_estimation_ptr->wiener_restore_fac_bits, 2 * sizeof(int32_t));
            picture_control_set_ptr->parent_pcs_ptr->av1x->rdmult = context_ptr->full_lambda;
        }

        if (lastLcuFlag)
        {
            // Get Empty EncDec Results
            eb_get_empty_object(
                context_ptr->enc_dec_output_fifo_ptr,
                &encDecResultsWrapperPtr);
            encDecResultsPtr = (EncDecResults*)encDecResultsWrapperPtr->object_ptr;
            encDecResultsPtr->picture_control_set_wrapper_ptr = encDecTasksPtr->picture_control_set_wrapper_ptr;
            //CHKN these are not needed for DLF
            encDecResultsPtr->completed_lcu_row_index_start = 0;
            encDecResultsPtr->completed_lcu_row_count = ((sequence_control_set_ptr->seq_header.max_frame_height + sequence_control_set_ptr->sb_size_pix - 1) >> lcuSizeLog2);
            // Post EncDec Results
            eb_post_full_object(encDecResultsWrapperPtr);
        }
        // Release Mode Decision Results
        eb_release_object(encDecTasksWrapperPtr);
    }
    return EB_NULL;
}

void av1_add_film_grain(EbPictureBufferDesc *src,
    EbPictureBufferDesc *dst,
    aom_film_grain_t *film_grain_ptr) {
    uint8_t *luma, *cb, *cr;
    int32_t height, width, luma_stride, chroma_stride;
    int32_t use_high_bit_depth = 0;
    int32_t chroma_subsamp_x = 0;
    int32_t chroma_subsamp_y = 0;

    aom_film_grain_t params = *film_grain_ptr;

    switch (src->bit_depth) {
    case EB_8BIT:
        params.bit_depth = 8;
        use_high_bit_depth = 0;
        chroma_subsamp_x = 1;
        chroma_subsamp_y = 1;
        break;
    case EB_10BIT:
        params.bit_depth = 10;
        use_high_bit_depth = 1;
        chroma_subsamp_x = 1;
        chroma_subsamp_y = 1;
        break;
    default:  //todo: Throw an error if unknown format?
        params.bit_depth = 10;
        use_high_bit_depth = 1;
        chroma_subsamp_x = 1;
        chroma_subsamp_y = 1;
    }

    dst->max_width = src->max_width;
    dst->max_height = src->max_height;

    fgn_copy_rect(src->buffer_y + ((src->origin_y * src->stride_y + src->origin_x) << use_high_bit_depth), src->stride_y,
        dst->buffer_y + ((dst->origin_y * dst->stride_y + dst->origin_x) << use_high_bit_depth), dst->stride_y,
        dst->width, dst->height, use_high_bit_depth);

    fgn_copy_rect(src->buffer_cb + ((src->stride_cb * (src->origin_y >> chroma_subsamp_y)
        + (src->origin_x >> chroma_subsamp_x)) << use_high_bit_depth), src->stride_cb,
        dst->buffer_cb + ((dst->stride_cb * (dst->origin_y >> chroma_subsamp_y)
            + (dst->origin_x >> chroma_subsamp_x)) << use_high_bit_depth), dst->stride_cb,
        dst->width >> chroma_subsamp_x, dst->height >> chroma_subsamp_y,
        use_high_bit_depth);

    fgn_copy_rect(src->buffer_cr + ((src->stride_cr * (src->origin_y >> chroma_subsamp_y)
        + (src->origin_x >> chroma_subsamp_x)) << use_high_bit_depth), src->stride_cr,
        dst->buffer_cr + ((dst->stride_cr * (dst->origin_y >> chroma_subsamp_y)
            + (dst->origin_x >> chroma_subsamp_x)) << use_high_bit_depth), dst->stride_cr,
        dst->width >> chroma_subsamp_x, dst->height >> chroma_subsamp_y,
        use_high_bit_depth);

    luma = dst->buffer_y + ((dst->origin_y * dst->stride_y + dst->origin_x) << use_high_bit_depth);
    cb = dst->buffer_cb + ((dst->stride_cb * (dst->origin_y >> chroma_subsamp_y)
        + (dst->origin_x >> chroma_subsamp_x)) << use_high_bit_depth);
    cr = dst->buffer_cr + ((dst->stride_cr * (dst->origin_y >> chroma_subsamp_y)
        + (dst->origin_x >> chroma_subsamp_x)) << use_high_bit_depth);

    luma_stride = dst->stride_y;
    chroma_stride = dst->stride_cb;

    width = dst->width;
    height = dst->height;

    av1_add_film_grain_run(&params, luma, cb, cr, height, width, luma_stride,
        chroma_stride, use_high_bit_depth, chroma_subsamp_y,
        chroma_subsamp_x);
    return;
}
