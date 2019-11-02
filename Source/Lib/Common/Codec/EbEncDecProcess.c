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
void eb_av1_cdef_search(
    EncDecContext                *context_ptr,
    SequenceControlSet           *sequence_control_set_ptr,
    PictureControlSet            *picture_control_set_ptr
);

void eb_av1_cdef_frame(
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

void eb_av1_add_film_grain(EbPictureBufferDesc *src,
    EbPictureBufferDesc *dst,
    aom_film_grain_t *film_grain_ptr);

void eb_av1_loop_restoration_save_boundary_lines(const Yv12BufferConfig *frame, Av1Common *cm, int32_t after_cdef);
void eb_av1_pick_filter_restoration(const Yv12BufferConfig *src, Yv12BufferConfig * trial_frame_rst /*Av1Comp *cpi*/, Macroblock *x, Av1Common *const cm);
void eb_av1_loop_restoration_filter_frame(Yv12BufferConfig *frame, Av1Common *cm, int32_t optimized_lr);

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

static void enc_dec_context_dctor(EbPtr p)
{
    EncDecContext* obj = (EncDecContext*)p;
    EB_DELETE(obj->md_context);
    EB_DELETE(obj->residual_buffer);
    EB_DELETE(obj->transform_buffer);
    EB_DELETE(obj->inverse_quant_buffer);
    EB_DELETE(obj->input_sample16bit_buffer);
    if (obj->is_md_rate_estimation_ptr_owner)
        EB_FREE(obj->md_rate_estimation_ptr);
    EB_FREE_ARRAY(obj->transform_inner_array_ptr);
}

/******************************************************
 * Enc Dec Context Constructor
 ******************************************************/
EbErrorType enc_dec_context_ctor(
    EncDecContext         *context_ptr,
    EbFifo                *mode_decision_configuration_input_fifo_ptr,
    EbFifo                *packetization_output_fifo_ptr,
    EbFifo                *feedback_fifo_ptr,
    EbFifo                *picture_demux_fifo_ptr,
#if PAL_SUP
    uint8_t                 cfg_palette,
#endif
    EbBool                  is16bit,
    EbColorFormat           color_format,
    EbBool                  enable_hbd_mode_decision,
    uint32_t                max_input_luma_width,
    uint32_t                max_input_luma_height)
{
    (void)max_input_luma_width;
    (void)max_input_luma_height;

    context_ptr->dctor = enc_dec_context_dctor;
    context_ptr->is16bit = is16bit;
    context_ptr->color_format = color_format;

    // Input/Output System Resource Manager FIFOs
    context_ptr->mode_decision_input_fifo_ptr = mode_decision_configuration_input_fifo_ptr;
    context_ptr->enc_dec_output_fifo_ptr = packetization_output_fifo_ptr;
    context_ptr->enc_dec_feedback_fifo_ptr = feedback_fifo_ptr;
    context_ptr->picture_demux_output_fifo_ptr = picture_demux_fifo_ptr;

    // Trasform Scratch Memory
    EB_MALLOC_ARRAY(context_ptr->transform_inner_array_ptr, 3152); //refer to EbInvTransform_SSE2.as. case 32x32
    // MD rate Estimation tables
    EB_MALLOC(context_ptr->md_rate_estimation_ptr, sizeof(MdRateEstimationContext));
    context_ptr->is_md_rate_estimation_ptr_owner = EB_TRUE;

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

            EB_NEW(
                context_ptr->input_sample16bit_buffer,
                eb_picture_buffer_desc_ctor,
                (EbPtr)&initData);
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
        EB_NEW(
            context_ptr->inverse_quant_buffer,
            eb_picture_buffer_desc_ctor,
            (EbPtr)&init32BitData);
        EB_NEW(
            context_ptr->transform_buffer,
            eb_picture_buffer_desc_ctor,
            (EbPtr)&init32BitData);
        EB_NEW(
            context_ptr->residual_buffer,
            eb_picture_buffer_desc_ctor,
            (EbPtr)&initData);
    }

    // Mode Decision Context
 #if PAL_SUP
    EB_NEW(
        context_ptr->md_context,
        mode_decision_context_ctor,
        color_format, 0, 0, enable_hbd_mode_decision , cfg_palette );
#else
    EB_NEW(
        context_ptr->md_context,
        mode_decision_context_ctor,
        color_format, 0, 0, enable_hbd_mode_decision);
#endif
    if (enable_hbd_mode_decision)
        context_ptr->md_context->input_sample16bit_buffer = context_ptr->input_sample16bit_buffer;

    context_ptr->md_context->enc_dec_context_ptr = context_ptr;

    return EB_ErrorNone;
}

/**************************************************
 * Reset Segmentation Map
 *************************************************/
static void reset_segmentation_map(SegmentationNeighborMap *segmentation_map){
    if(segmentation_map->data!=NULL)
        EB_MEMSET(segmentation_map->data, ~0, segmentation_map->map_size);
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
    neighbor_array_unit_reset(picture_control_set_ptr->ep_luma_dc_sign_level_coeff_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_cb_dc_sign_level_coeff_neighbor_array);
    neighbor_array_unit_reset(picture_control_set_ptr->ep_cr_dc_sign_level_coeff_neighbor_array);
    // TODO(Joel): 8-bit ep_luma_recon_neighbor_array (Cb,Cr) when is16bit==0?
    EbBool is16bit = (EbBool)(picture_control_set_ptr->parent_pcs_ptr->sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);
    if (is16bit) {
        neighbor_array_unit_reset(picture_control_set_ptr->ep_luma_recon_neighbor_array16bit);
        neighbor_array_unit_reset(picture_control_set_ptr->ep_cb_recon_neighbor_array16bit);
        neighbor_array_unit_reset(picture_control_set_ptr->ep_cr_recon_neighbor_array16bit);
    }
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
    context_ptr->is16bit = (EbBool)(sequence_control_set_ptr->static_config.encoder_bit_depth > EB_8BIT);

#if ADD_DELTA_QP_SUPPORT
    uint16_t picture_qp = picture_control_set_ptr->picture_qp;
    context_ptr->qp = picture_qp;
    context_ptr->qp_index = picture_control_set_ptr->parent_pcs_ptr->frm_hdr.delta_q_params.delta_q_present ? (uint8_t)quantizer_to_qindex[context_ptr->qp] : (uint8_t)picture_control_set_ptr->parent_pcs_ptr->frm_hdr.quantization_params.base_q_idx;
#else
    context_ptr->qp = picture_control_set_ptr->picture_qp;
#endif
    // Asuming cb and cr offset to be the same for chroma QP in both slice and pps for lambda computation

    context_ptr->chroma_qp = (uint8_t)context_ptr->qp;

    // Lambda Assignement
    context_ptr->qp_index = (uint8_t)picture_control_set_ptr->
        parent_pcs_ptr->frm_hdr.quantization_params.base_q_idx;
    (*av1_lambda_assignment_function_table[picture_control_set_ptr->parent_pcs_ptr->pred_structure])(
        &context_ptr->fast_lambda,
        &context_ptr->full_lambda,
        &context_ptr->fast_chroma_lambda,
        &context_ptr->full_chroma_lambda,
        (uint8_t)picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr->bit_depth,
        context_ptr->qp_index,
        picture_control_set_ptr->hbd_mode_decision);
    // Reset MD rate Estimation table to initial values by copying from md_rate_estimation_array
    if (context_ptr->is_md_rate_estimation_ptr_owner) {
        EB_FREE(context_ptr->md_rate_estimation_ptr);
        context_ptr->is_md_rate_estimation_ptr_owner = EB_FALSE;
    }
    context_ptr->md_rate_estimation_ptr = picture_control_set_ptr->md_rate_estimation_array;
    if (segment_index == 0){
        ResetEncodePassNeighborArrays(picture_control_set_ptr);
        reset_segmentation_map(picture_control_set_ptr->segmentation_neighbor_map);
    }

    return;
}

/******************************************************
 * EncDec Configure LCU
 ******************************************************/
static void EncDecConfigureLcu(
    EncDecContext         *context_ptr,
    LargestCodingUnit     *sb_ptr,
    PictureControlSet     *picture_control_set_ptr,
    uint8_t                    sb_qp)
{
    context_ptr->qp = sb_qp;

    // Asuming cb and cr offset to be the same for chroma QP in both slice and pps for lambda computation
    context_ptr->chroma_qp = (uint8_t)context_ptr->qp;
    /* Note(CHKN) : when Qp modulation varies QP on a sub-LCU(CU) basis,  Lamda has to change based on Cu->QP , and then this code has to move inside the CU loop in MD */
    (void)sb_ptr;
    context_ptr->qp_index = (uint8_t)picture_control_set_ptr->
        parent_pcs_ptr->frm_hdr.quantization_params.base_q_idx;
    (*av1_lambda_assignment_function_table[picture_control_set_ptr->parent_pcs_ptr->pred_structure])(
        &context_ptr->fast_lambda,
        &context_ptr->full_lambda,
        &context_ptr->fast_chroma_lambda,
        &context_ptr->full_chroma_lambda,
        (uint8_t)picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr->bit_depth,
        context_ptr->qp_index,
        picture_control_set_ptr->hbd_mode_decision);

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

    if (!picture_control_set_ptr->parent_pcs_ptr->is_alt_ref) {
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
                    film_grain_ptr = &picture_control_set_ptr->parent_pcs_ptr->frm_hdr.film_grain_params;

                eb_av1_add_film_grain(recon_ptr, intermediateBufferPtr, film_grain_ptr);
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
    }
    else {
        // Overlay and altref have 1 recon only, which is from overlay pictures. So the recon of the alt_ref is not sent to the application.
        // However, to hanlde the end of sequence properly, total_number_of_recon_frames is increamented
        encode_context_ptr->total_number_of_recon_frames++;
    }
    eb_release_mutex(encode_context_ptr->total_number_of_recon_frame_mutex);
}

void psnr_calculations(
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

        EbByte buffer_y;
        EbByte buffer_cb;
        EbByte buffer_cr;

        // if current source picture was temporally filtered, use an alternative buffer which stores
        // the original source picture
        if(picture_control_set_ptr->parent_pcs_ptr->temporal_filtering_on == EB_TRUE){
            buffer_y = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_ptr[0];
            buffer_cb = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_ptr[1];
            buffer_cr = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_ptr[2];
        }
        else {
            buffer_y = input_picture_ptr->buffer_y;
            buffer_cb = input_picture_ptr->buffer_cb;
            buffer_cr = input_picture_ptr->buffer_cr;
        }

        reconCoeffBuffer = &((recon_ptr->buffer_y)[recon_ptr->origin_x + recon_ptr->origin_y * recon_ptr->stride_y]);
        inputBuffer = &(buffer_y[input_picture_ptr->origin_x + input_picture_ptr->origin_y * input_picture_ptr->stride_y]);

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
        inputBuffer = &(buffer_cb[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cb]);

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
        inputBuffer = &(buffer_cr[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cr]);
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
        picture_control_set_ptr->parent_pcs_ptr->cb_sse = (uint32_t)sseTotal[1];
        picture_control_set_ptr->parent_pcs_ptr->cr_sse = (uint32_t)sseTotal[2];

        if(picture_control_set_ptr->parent_pcs_ptr->temporal_filtering_on == EB_TRUE) {
            EB_FREE_ARRAY(buffer_y);
            EB_FREE_ARRAY(buffer_cb);
            EB_FREE_ARRAY(buffer_cr);
        }
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

            // if current source picture was temporally filtered, use an alternative buffer which stores
            // the original source picture
            EbByte buffer_y, buffer_bit_inc_y;
            EbByte buffer_cb, buffer_bit_inc_cb;
            EbByte buffer_cr, buffer_bit_inc_cr;

            if(picture_control_set_ptr->parent_pcs_ptr->temporal_filtering_on == EB_TRUE){
                buffer_y = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_ptr[0];
                buffer_bit_inc_y = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_bit_inc_ptr[0];
                buffer_cb = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_ptr[1];
                buffer_bit_inc_cb = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_bit_inc_ptr[1];
                buffer_cr = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_ptr[2];
                buffer_bit_inc_cr = picture_control_set_ptr->parent_pcs_ptr->save_enhanced_picture_bit_inc_ptr[2];
            }else{
                buffer_y = input_picture_ptr->buffer_y;
                buffer_bit_inc_y = input_picture_ptr->buffer_bit_inc_y;
                buffer_cb = input_picture_ptr->buffer_cb;
                buffer_bit_inc_cb = input_picture_ptr->buffer_bit_inc_cb;
                buffer_cr = input_picture_ptr->buffer_cr;
                buffer_bit_inc_cr = input_picture_ptr->buffer_bit_inc_cr;
            }

            inputBuffer = &((buffer_y)[input_picture_ptr->origin_x + input_picture_ptr->origin_y * input_picture_ptr->stride_y]);
            inputBufferBitInc = &((buffer_bit_inc_y)[input_picture_ptr->origin_x + input_picture_ptr->origin_y * input_picture_ptr->stride_bit_inc_y]);

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
            inputBuffer = &((buffer_cb)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cb]);
            inputBufferBitInc = &((buffer_bit_inc_cb)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_bit_inc_cb]);

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
            inputBuffer = &((buffer_cr)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_cr]);
            inputBufferBitInc = &((buffer_bit_inc_cr)[input_picture_ptr->origin_x / 2 + input_picture_ptr->origin_y / 2 * input_picture_ptr->stride_bit_inc_cr]);

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

            if(picture_control_set_ptr->parent_pcs_ptr->temporal_filtering_on == EB_TRUE) {
                EB_FREE_ARRAY(buffer_y);
                EB_FREE_ARRAY(buffer_cb);
                EB_FREE_ARRAY(buffer_cr);
                EB_FREE_ARRAY(buffer_bit_inc_y);
                EB_FREE_ARRAY(buffer_bit_inc_cb);
                EB_FREE_ARRAY(buffer_bit_inc_cr);
            }
        }

        picture_control_set_ptr->parent_pcs_ptr->luma_sse = (uint32_t)sseTotal[0];
        picture_control_set_ptr->parent_pcs_ptr->cb_sse = (uint32_t)sseTotal[1];
        picture_control_set_ptr->parent_pcs_ptr->cr_sse = (uint32_t)sseTotal[2];
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

        // Hsan: unpack ref samples (to be used @ MD)
        un_pack2d(
            (uint16_t*) refPic16BitPtr->buffer_y,
            refPic16BitPtr->stride_y,
            refPicPtr->buffer_y,
            refPicPtr->stride_y,
            refPicPtr->buffer_bit_inc_y,
            refPicPtr->stride_bit_inc_y,
            refPic16BitPtr->width  + (refPicPtr->origin_x << 1),
            refPic16BitPtr->height + (refPicPtr->origin_y << 1),
            sequence_control_set_ptr->static_config.asm_type);

        un_pack2d(
            (uint16_t*)refPic16BitPtr->buffer_cb,
            refPic16BitPtr->stride_cb,
            refPicPtr->buffer_cb,
            refPicPtr->stride_cb,
            refPicPtr->buffer_bit_inc_cb,
            refPicPtr->stride_bit_inc_cb,
            (refPic16BitPtr->width + (refPicPtr->origin_x << 1)) >> 1,
            (refPic16BitPtr->height + (refPicPtr->origin_y << 1)) >> 1,
            sequence_control_set_ptr->static_config.asm_type);

        un_pack2d(
            (uint16_t*)refPic16BitPtr->buffer_cr,
            refPic16BitPtr->stride_cr,
            refPicPtr->buffer_cr,
            refPicPtr->stride_cr,
            refPicPtr->buffer_bit_inc_cr,
            refPicPtr->stride_bit_inc_cr,
            (refPic16BitPtr->width + (refPicPtr->origin_x << 1)) >> 1,
            (refPic16BitPtr->height + (refPicPtr->origin_y << 1)) >> 1,
            sequence_control_set_ptr->static_config.asm_type);
    }
    // set up the ref POC
    referenceObject->ref_poc = picture_control_set_ptr->parent_pcs_ptr->picture_number;

    // set up the QP
    referenceObject->qp = (uint8_t)picture_control_set_ptr->parent_pcs_ptr->picture_qp;

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
    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->tmp_layer_idx = (uint8_t)picture_control_set_ptr->temporal_layer_index;
    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->is_scene_change = picture_control_set_ptr->parent_pcs_ptr->scene_change_flag;

    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->cdef_frame_strength = picture_control_set_ptr->parent_pcs_ptr->cdef_frame_strength;

    Av1Common* cm = picture_control_set_ptr->parent_pcs_ptr->av1_cm;
    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->sg_frame_ep = cm->sg_frame_ep;
    if (sequence_control_set_ptr->mfmv_enabled) {
        ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->frame_type = picture_control_set_ptr->parent_pcs_ptr->frm_hdr.frame_type;
        ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->order_hint = picture_control_set_ptr->parent_pcs_ptr->cur_order_hint;
        memcpy(((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->ref_order_hint, picture_control_set_ptr->parent_pcs_ptr->ref_order_hint, 7 * sizeof(uint32_t));
    }
}

/******************************************************
* Derive EncDec Settings for OQ
Input   : encoder mode and tune
Output  : EncDec Kernel signal(s)
******************************************************/
EbErrorType signal_derivation_enc_dec_kernel_oq(
    SequenceControlSet    *sequence_control_set_ptr,
    PictureControlSet     *picture_control_set_ptr,
    ModeDecisionContext   *context_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    // Set Chroma Mode
    // Level                Settings
    // CHROMA_MODE_0  0     Full chroma search @ MD
    // CHROMA_MODE_1  1     Fast chroma search @ MD
    // CHROMA_MODE_2  2     Chroma blind @ MD + CFL @ EP
    // CHROMA_MODE_3  3     Chroma blind @ MD + no CFL @ EP
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
#if rtime_presets

        if (MR_MODE) 
            context_ptr->chroma_level = CHROMA_MODE_0;
        else
            if (picture_control_set_ptr->enc_mode <= ENC_M5 && picture_control_set_ptr->temporal_layer_index == 0)


                context_ptr->chroma_level = CHROMA_MODE_0;
            else
                if (picture_control_set_ptr->enc_mode <= ENC_M5)
                    context_ptr->chroma_level = CHROMA_MODE_1;
                else
                    context_ptr->chroma_level = (sequence_control_set_ptr->encoder_bit_depth == EB_8BIT) ?
                    CHROMA_MODE_2 :
                    CHROMA_MODE_3;

#else
    if (MR_MODE)
        context_ptr->chroma_level = CHROMA_MODE_0;
    else
    if (picture_control_set_ptr->enc_mode == ENC_M0 && picture_control_set_ptr->temporal_layer_index == 0)
        context_ptr->chroma_level = CHROMA_MODE_0;
    else
    if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->chroma_level = CHROMA_MODE_1;
    else
        context_ptr->chroma_level = (sequence_control_set_ptr->encoder_bit_depth == EB_8BIT) ?
            CHROMA_MODE_2 :
            CHROMA_MODE_3 ;
#endif
    // Set fast loop method
    // 1 fast loop: SSD_SEARCH not supported
    // Level                Settings
    //  0                   Collapsed fast loop
    //  1                   Decoupled fast loops ( intra/inter)
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
            context_ptr->decouple_intra_inter_fast_loop = 0;
        else
            context_ptr->decouple_intra_inter_fast_loop = 1;
    else
    context_ptr->decouple_intra_inter_fast_loop = 0;

    // Set the search method when decoupled fast loop is used
    // Hsan: FULL_SAD_SEARCH not supported
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
            context_ptr->decoupled_fast_loop_search_method = SSD_SEARCH;
        else
            context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;
    else
        if (picture_control_set_ptr->enc_mode <= ENC_M4)
            context_ptr->decoupled_fast_loop_search_method = SSD_SEARCH;
        else
            context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;

    // Set the full loop escape level
    // Level                Settings
    // 0                    Off
    // 1                    On but only INTRA
    // 2                    On both INTRA and INTER
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
            context_ptr->full_loop_escape = 0;
        else
            context_ptr->full_loop_escape = 2;
    else
    if (picture_control_set_ptr->enc_mode <= ENC_M5)
        context_ptr->full_loop_escape = 0;
    else
        context_ptr->full_loop_escape = 2;

    // Set global MV injection
    // Level                Settings
    // 0                    Injection off (Hsan: but not derivation as used by MV ref derivation)
    // 1                    On
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
            context_ptr->global_mv_injection = 1;
        else
            context_ptr->global_mv_injection = 0;
    else
    if (picture_control_set_ptr->enc_mode <= ENC_M7)
        context_ptr->global_mv_injection = 1;
    else
        context_ptr->global_mv_injection = 0;
#if FIX_NEAREST_NEW
    if (picture_control_set_ptr->enc_mode <= ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
#else
    if (picture_control_set_ptr->enc_mode == ENC_M0)
#endif
        context_ptr->new_nearest_near_comb_injection = 1;
    else
        context_ptr->new_nearest_near_comb_injection = 0;
#if rtime_presets
    if (picture_control_set_ptr->enc_mode <= ENC_M1)
#else
    if (picture_control_set_ptr->enc_mode == ENC_M0)
#endif
        context_ptr->nx4_4xn_parent_mv_injection = 1;
    else
        context_ptr->nx4_4xn_parent_mv_injection = 0;

    // Set warped motion injection
    // Level                Settings
    // 0                    OFF
    // 1                    On
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        context_ptr->warped_motion_injection = 0;
    else
    context_ptr->warped_motion_injection = 1;

    // Set unipred3x3 injection
    // Level                Settings
    // 0                    OFF
    // 1                    ON FULL
    // 2                    Reduced set
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
            context_ptr->unipred3x3_injection = 1;
        else
            context_ptr->unipred3x3_injection = 0;
    else
#if rtime_presets
     if (picture_control_set_ptr->enc_mode <= ENC_M3)
#else
    if (picture_control_set_ptr->enc_mode <= ENC_M1)
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
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
            context_ptr->bipred3x3_injection = 1;
        else
            context_ptr->bipred3x3_injection = 0;
    else
#if rtime_presets
    if (picture_control_set_ptr->enc_mode <= ENC_M3)
#else
    if (picture_control_set_ptr->enc_mode <= ENC_M1)
#endif
        context_ptr->bipred3x3_injection = 1;
    else if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->bipred3x3_injection = 2;
    else
        context_ptr->bipred3x3_injection = 0;

    // Level                Settings
    // 0                    Level 0: OFF
    // 1                    Level 1: 7x5 full-pel search + sub-pel refinement off
    // 2                    Level 2: 7x5 full-pel search +  (H + V) sub-pel refinement only = 4 half-pel + 4 quarter-pel = 8 positions + pred_me_distortion to pa_me_distortion deviation on
    // 3                    Level 3: 7x5 full-pel search +  (H + V + D only ~ the best) sub-pel refinement = up to 6 half-pel + up to 6  quarter-pel = up to 12 positions + pred_me_distortion to pa_me_distortion deviation on
    // 4                    Level 4: 7x5 full-pel search +  (H + V + D) sub-pel refinement = 8 half-pel + 8 quarter-pel = 16 positions + pred_me_distortion to pa_me_distortion deviation on
    // 5                    Level 5: 7x5 full-pel search +  (H + V + D) sub-pel refinement = 8 half-pel + 8 quarter-pel = 16 positions + pred_me_distortion to pa_me_distortion deviation off
    if (picture_control_set_ptr->slice_type != I_SLICE)
#if rtime_presets
        if (picture_control_set_ptr->enc_mode <= ENC_M2)
#else
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
#endif
#if M0_tune
            context_ptr->predictive_me_level = (picture_control_set_ptr->enc_mode <= ENC_M0)? 5: 4;
#else
            context_ptr->predictive_me_level = 4;
#endif
        else if (picture_control_set_ptr->enc_mode <= ENC_M4)
            context_ptr->predictive_me_level = 2;
        else
            context_ptr->predictive_me_level = 0;
    else
        context_ptr->predictive_me_level = 0;

    // Derive md_staging_mode
    //
#if REMOVE_MD_STAGE_1
    // MD_STAGING_MODE_0
    // Default Parameters
    //
    // MD_STAGING_MODE_1
    //  __________________________________________________________________________________________________________________
    // |        | md_stage_0                  | md_stage_2                     | md_stage_3                              |
    // |________|_____________________________|________________________________|_________________________________________|
    // |CLASS_0 |Prediction for Luma & Chroma |T, Q, Q-1, T-1 for Luma Only    |T, Q, Q-1, T-1 or Luma & Chroma          |
    // |CLASS_6 |                             |No RDOQ                         |RDOQ                                     |
    // |        |                             |No Tx Type Search               |Tx Type Search                           |
    // |        |                             |No Tx Size Search               |Tx Size Search                           |
    // |        |                             |                                |CFL vs. Independent                      |
    // |________|_____________________________|________________________________|_________________________________________|
    // |CLASS_1 |Prediction for Luma Only     |T, Q, Q-1, T-1 for Luma Only    |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |CLASS_2 |No Interpolation Search      |No RDOQ                         |RDOQ                                     |
    // |CLASS_3 |Bilinear Interpolation       |No Tx Type Search               |Tx Type Search                           |
    // |CLASS_4 |                             |No Tx Size Search               |Tx Size Search                           |
    // |CLASS_5 |                             |Interpolation Search            |                                         |
    // |________|_____________________________|________________________________|_________________________________________|

    if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->md_staging_mode = MD_STAGING_MODE_1;
    else
        context_ptr->md_staging_mode = MD_STAGING_MODE_0;
#else
    // MD_STAGING_MODE_1
    //  _______________________________________________________________________________________________________________________________________________
    // |        | md_stage_0                  | md_stage_1                  | md_stage_2                     | md_stage_3                              |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_0 |Prediction for Luma & Chroma |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 or Luma & Chroma          |
    // |        |No Interpolation Search      |                             |RDOQ                            |RDOQ                                     |
    // |        |Regular Interpolation        |                             |No Tx Search                    |Tx Search                                |
    // |        |                             |                             |No ATB                          |ATB                                      |
    // |        |                             |                             |                                |CFL vs. Independent                      |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_1 |Prediction for Chroma        |Prediction for Luma & Chroma |Bypassed                        |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |                                |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |                                |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_2 |Prediction for Chroma        |Prediction for Luma & Chroma |Bypassed                        |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |                                |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |                                |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_3 |Prediction for Chroma        |Prediction for Luma & Chroma |Bypassed                        |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |                                |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |                                |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    //
    // MD_STAGING_MODE_2
    //  _______________________________________________________________________________________________________________________________________________
    // |        | md_stage_0                  | md_stage_1                  | md_stage_2                     | md_stage_3                              |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_0 |Prediction for Luma & Chroma |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 or Luma & Chroma          |
    // |        |No Interpolation Search      |                             |No RDOQ                         |RDOQ                                     |
    // |        |Regular Interpolation        |                             |No Tx Search                    |Tx Search                                |
    // |        |                             |                             |No ATB                          |ATB                                      |
    // |        |                             |                             |                                |CFL vs. Independent                      |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_1 |Prediction for Chroma        |Prediction for Luma & Chroma |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_2 |Prediction for Chroma        |Prediction for Luma & Chroma |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_3 |Prediction for Chroma        |Prediction for Luma & Chroma |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    //
    // MD_STAGING_MODE_3
    //  _______________________________________________________________________________________________________________________________________________
    // |        | md_stage_0                  | md_stage_1                  | md_stage_2                     | md_stage_3                              |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_0 |Prediction for Luma & Chroma |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 or Luma & Chroma          |
    // |        |No Interpolation Search      |                             |No RDOQ                         |RDOQ                                     |
    // |        |Regular Interpolation        |                             |No Tx Search                    |Tx Search                                |
    // |        |                             |                             |No ATB                          |ATB                                      |
    // |        |                             |                             |                                |CFL vs. Independent                      |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_1 |Prediction for Chroma        |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |                             |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_2 |Prediction for Chroma        |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |                             |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_3 |Prediction for Chroma        |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |                             |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|

    if (picture_control_set_ptr->enc_mode == ENC_M0)
        context_ptr->md_staging_mode = MD_STAGING_MODE_1;
    else if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->md_staging_mode = MD_STAGING_MODE_3;
    else
        context_ptr->md_staging_mode = MD_STAGING_MODE_0; // Default structure = fast loop + full loop = md_stage_0 + md_stage_3
#endif
    // Combine MD Class1&2
    // 0                    OFF
    // 1                    ON
#if rtime_presets
    context_ptr->combine_class12 = (picture_control_set_ptr->enc_mode <= ENC_M1) ? 0 : 1;
#else
    context_ptr->combine_class12 = (picture_control_set_ptr->enc_mode == ENC_M0) ? 0 : 1;
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

    // Set PF MD
    context_ptr->pf_md_mode = PF_OFF;

    // Derive Spatial SSE Flag
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M6)
            context_ptr->spatial_sse_full_loop = EB_TRUE;
        else
            context_ptr->spatial_sse_full_loop = EB_FALSE;
    else
    if (picture_control_set_ptr->enc_mode <= ENC_M4)
        context_ptr->spatial_sse_full_loop = EB_TRUE;
    else
        context_ptr->spatial_sse_full_loop = EB_FALSE;

    if (context_ptr->chroma_level <= CHROMA_MODE_1)
        context_ptr->blk_skip_decision = EB_TRUE;
    else
        context_ptr->blk_skip_decision = EB_FALSE;
    // Derive Trellis Quant Coeff Optimization Flag
#if rtime_presets
    if (picture_control_set_ptr->enc_mode <= ENC_M3)
#else
    if (picture_control_set_ptr->enc_mode == ENC_M0)
#endif
        context_ptr->trellis_quant_coeff_optimization = EB_TRUE;
    else
        context_ptr->trellis_quant_coeff_optimization = EB_FALSE;

    // Derive redundant block
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (picture_control_set_ptr->enc_mode <= ENC_M1)
            context_ptr->redundant_blk = EB_TRUE;
        else
            context_ptr->redundant_blk = EB_FALSE;
    else
    if (picture_control_set_ptr->enc_mode <= ENC_M5)
        context_ptr->redundant_blk = EB_TRUE;
    else
        context_ptr->redundant_blk = EB_FALSE;
    if (sequence_control_set_ptr->static_config.encoder_bit_depth == EB_8BIT)
#if FIX_ESTIMATE_INTRA
        if (MR_MODE)
#else
        if (MR_MODE || picture_control_set_ptr->enc_mode == ENC_M0)
#endif
            context_ptr->edge_based_skip_angle_intra = 0;
        else
#if FIX_ESTIMATE_INTRA
#if M0_tune
            context_ptr->edge_based_skip_angle_intra = (picture_control_set_ptr->enc_mode == ENC_M0) ? 0 : 1;
#else
#if rtime_presets
            context_ptr->edge_based_skip_angle_intra = ((picture_control_set_ptr->enc_mode > ENC_M3) || (picture_control_set_ptr->enc_mode == ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0)) ? 0 : 1;
#else
            context_ptr->edge_based_skip_angle_intra = (picture_control_set_ptr->enc_mode == ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0) ? 0 : 1;
#endif
#endif
#else
            context_ptr->edge_based_skip_angle_intra = 1;
#endif
    else
        context_ptr->edge_based_skip_angle_intra = 0;
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected || picture_control_set_ptr->enc_mode == ENC_M0)
        context_ptr->prune_ref_frame_for_rec_partitions = 0;
    else
        context_ptr->prune_ref_frame_for_rec_partitions = 1;

#if SPEED_OPT
    // Derive INTER/INTER WEDGE variance TH
    if (MR_MODE)
        context_ptr->inter_inter_wedge_variance_th = 0;
    else
        context_ptr->inter_inter_wedge_variance_th = 100;

    // Derive MD Exit TH
    if (MR_MODE)
        context_ptr->md_exit_th = 0;
    else
        context_ptr->md_exit_th = (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected) ? 10 : 18;

    // Derive distortion-based md_stage_0_count proning
    if (MR_MODE)
        context_ptr->dist_base_md_stage_0_count_th = (uint64_t)~0;
    else
        context_ptr->dist_base_md_stage_0_count_th = 75;
#endif


#if LESS_RECTANGULAR_CHECK_LEVEL

    // Weighting (expressed as a percentage) applied to 
    // square shape costs for determining if a and b 
    // shapes should be skipped. Namely:
    // skip HA and HB if h_cost > (weighted sq_cost)
    // skip VA and VB if v_cost > (weighted sq_cost)
    
    if (MR_MODE)
        context_ptr->sq_to_h_v_weight_to_skip_a_b = (uint32_t)~0;
    else
        context_ptr->sq_to_h_v_weight_to_skip_a_b = 100;
#endif


    return return_error;
}

void move_cu_data(
    CodingUnit *src_cu,
    CodingUnit *dst_cu);
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
#if MPMD_ADAPTIVE_REFINEMENT
    uint8_t             best_depth,
#endif
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
#if MPMD_ADAPTIVE_REFINEMENT
                // Adjust depth refinement mode
                uint8_t refined_depth = depth_refinement_mode;
                if (depth_refinement_mode != AllD) {
                   /* if (best_depth < blk_geom->depth)
                        depth_refinement_mode = Predm1;*/
                   /* if (best_depth > blk_geom->depth)
                        depth_refinement_mode = Predp1;*/
                     if (best_depth == blk_geom->depth && blk_index == 0)
                        refined_depth = Pred;
                }
                context_ptr->mdc_number_of_64x64 += sb_index == 0 ? 1 : 0;
                switch (refined_depth) {
#else
                switch (depth_refinement_mode) {
#endif
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
#if MARK_CU
                            cu_ptr->leaf_data_array[blk_index + block_1d_idx].depth_offset = 0;
#endif
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
#if MARK_CU
                                cu_ptr->leaf_data_array[parent_depth_idx_mds + block_1d_idx].depth_offset = -1;
#endif
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
#if MARK_CU
                                cu_ptr->leaf_data_array[blk_index + block_1d_idx].depth_offset = 0;
#endif
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
#if MARK_CU
                                cu_ptr->leaf_data_array[child_block_idx_1 + block_1d_idx].depth_offset = 1;
#endif
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
#if MARK_CU
                                cu_ptr->leaf_data_array[child_block_idx_2 + block_1d_idx].depth_offset = 1;
#endif
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
#if MARK_CU
                                cu_ptr->leaf_data_array[child_block_idx_3 + block_1d_idx].depth_offset = 1;
#endif
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
#if MARK_CU
                                cu_ptr->leaf_data_array[child_block_idx_4 + block_1d_idx].depth_offset = 1;
#endif
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
#if USE_1SP_MODE
        for (uint16_t pred_indx = 0; pred_indx < MAX_NFL_BUFF; pred_indx++) 
            for (uint16_t md_stage_idx = 0; md_stage_idx < 4; md_stage_idx++)
                context_ptr->md_local_cu_unit[blk_index].best_pred_modes_table[md_stage_idx][pred_indx] = UNVALID;

#if USE_1SP_SKIP
        for (uint16_t skip_idx = 0; skip_idx < MAX_NFL_BUFF; skip_idx++) 
            for (uint16_t md_stage_idx = 0; md_stage_idx < 4; md_stage_idx++)
                context_ptr->md_local_cu_unit[blk_index].best_skip_table[md_stage_idx][skip_idx] = UNVALID;
#endif
        
#endif

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
        dst[blk_index].refined_split_flag = src[blk_index].refined_split_flag;
#if MARK_CU
        dst[blk_index].fp_depth_offset = src[blk_index].fp_depth_offset;
#endif
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
#if MARK_CU
        cu_ptr->leaf_data_array[blk_index].fp_depth_offset = UNVALID;
        cu_ptr->leaf_data_array[blk_index].depth_offset = UNVALID;
#endif
        blk_index++;
    }
}
#if SKIP_2ND_PASS_BASED_ON_1ST_PASS
void gather_rd_sb(
    SequenceControlSet *sequence_control_set_ptr,
    PictureControlSet *picture_control_set_ptr,
    ModeDecisionContext *context_ptr,
    uint32_t sb_index) {
    uint32_t  blk_index = 0;
    while (blk_index < sequence_control_set_ptr->max_block_cnt) {
        const BlockGeom * blk_geom = get_blk_geom_mds(blk_index);
        uint8_t tot_d1_blocks =  blk_geom->sq_size == 128 ? 17 :
                blk_geom->sq_size > 8 ? 25 :
                blk_geom->sq_size == 8 ? 5 : 1;
        uint32_t mds_idx = blk_geom->sqi_mds;
        uint8_t is_blk_allowed = picture_control_set_ptr->slice_type != I_SLICE ? 1 : (blk_geom->sq_size < 128) ? 1 : 0;
        //init consider block flag
        uint8_t split_flag = context_ptr->md_cu_arr_nsq[mds_idx].split_flag;
        if (sequence_control_set_ptr->sb_geom[sb_index].block_is_inside_md_scan[blk_index] && is_blk_allowed) {
            if (context_ptr->md_cu_arr_nsq[mds_idx].split_flag == 0) {
                if (blk_geom->shape == PART_N) {
                    context_ptr->sb_cost += context_ptr->md_local_cu_unit[mds_idx].cost;
                    context_ptr->sb_dist += context_ptr->md_local_cu_unit[mds_idx].full_distortion;
                }
            }
        }
        blk_index += split_flag ? d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] : ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
    }
}
#endif
void mpmd_forward_considered_blocks(
    SequenceControlSet *sequence_control_set_ptr,
    PictureControlSet *picture_control_set_ptr,
#if USE_MSMD_OUTPUT
    uint8_t md_stage_idx,
#endif
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
#if MARK_CU
                    cu_ptr->leaf_data_array[cu_ptr->leaf_count].fp_depth_offset =  cu_ptr->leaf_data_array[blk_index].depth_offset;
#endif
                    cu_ptr->leaf_data_array[cu_ptr->leaf_count++].split_flag = split_flag;
                }
#if USE_1SP_MODE
                for (uint16_t fp_pred_indx = 0; fp_pred_indx < BEST_PRED_MODE_TH; fp_pred_indx++) {
#if !USE_MSMD_OUTPUT
                    uint16_t md_stage_idx = 3;
#endif
                    if (context_ptr->md_local_cu_unit[blk_index].best_pred_modes_table[md_stage_idx][fp_pred_indx] < MB_MODE_COUNT && context_ptr->md_local_cu_unit[blk_index].best_pred_modes_table[md_stage_idx][fp_pred_indx] >= 0)
                        context_ptr->fp_depth_mode_valid[context_ptr->md_local_cu_unit[blk_index].best_pred_modes_table[md_stage_idx][fp_pred_indx]] = 1;
                }
#if USE_1SP_SKIP
                for (uint16_t skip_idx = 0; skip_idx < BEST_PRED_MODE_TH; skip_idx++) {
                    uint8_t md_stage_idx = 3;
                     if (context_ptr->md_local_cu_unit[blk_index].best_skip_table[md_stage_idx][skip_idx] <= 1 && context_ptr->md_local_cu_unit[blk_index].best_skip_table[md_stage_idx][skip_idx] >= 0)
                        context_ptr->fp_depth_skip_valid[context_ptr->md_local_cu_unit[blk_index].best_skip_table[md_stage_idx][skip_idx]] = 1;
                }
#endif
#endif
                blk_index++;
            }
        }
        blk_index += split_flag ? d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] - tot_d1_blocks : ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] - tot_d1_blocks;
        //blk_index += (d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth] - tot_d1_blocks);
    }
}
#if MPMD_ADD_SB_SETTINGS_TO_CTX
#define FC_SKIP_TX_SR_TH025                     125 // Fast cost skip tx search threshold.
#define FC_SKIP_TX_SR_TH010                     110 // Fast cost skip tx search threshold.
/******************************************************
* Derive Multi-Processes Settings for OQ
Input   : encoder mode and tune
Output  : Multi-Processes signal(s)
******************************************************/
EbErrorType mpmd_update_pic_settings_sb(
    SequenceControlSet        *sequence_control_set_ptr,
    PictureParentControlSet   *picture_control_set_ptr,
    ModeDecisionContext       *context_ptr,
    uint8_t                    enc_mode,
    uint8_t                    is_last_md_pass) {
    EbErrorType return_error = EB_ErrorNone;
    FrameHeader *frm_hdr = &picture_control_set_ptr->frm_hdr;
    //  MDC Partitioning Method              Settings
    //  PIC_ALL_DEPTH_MODE                   ALL sq and nsq: SB size -> 4x4
    //  PIC_ALL_C_DEPTH_MODE                 ALL sq and nsq: SB size -> 4x4  (No 4xN ; Nx4)
    //  PIC_SQ_DEPTH_MODE                    ONLY sq: SB size -> 4x4
    //  PIC_SQ_NON4_DEPTH_MODE               ONLY sq: SB size -> 8x8  (No 4x4)

    uint8_t sc_content_detected = picture_control_set_ptr->sc_content_detected;
    picture_control_set_ptr->max_number_of_pus_per_sb = (picture_control_set_ptr->pic_depth_mode <= PIC_ALL_C_DEPTH_MODE) ? MAX_ME_PU_COUNT : SQUARE_PU_COUNT;

    // NSQ search Level                               Settings
    // NSQ_SEARCH_OFF                                 OFF
    // NSQ_SEARCH_LEVEL1                              Allow only NSQ Inter-NEAREST/NEAR/GLOBAL if parent SQ has no coeff + reordering nsq_table number and testing only 1 NSQ SHAPE
    // NSQ_SEARCH_LEVEL2                              Allow only NSQ Inter-NEAREST/NEAR/GLOBAL if parent SQ has no coeff + reordering nsq_table number and testing only 2 NSQ SHAPE
    // NSQ_SEARCH_LEVEL3                              Allow only NSQ Inter-NEAREST/NEAR/GLOBAL if parent SQ has no coeff + reordering nsq_table number and testing only 3 NSQ SHAPE
    // NSQ_SEARCH_LEVEL4                              Allow only NSQ Inter-NEAREST/NEAR/GLOBAL if parent SQ has no coeff + reordering nsq_table number and testing only 4 NSQ SHAPE
    // NSQ_SEARCH_LEVEL5                              Allow only NSQ Inter-NEAREST/NEAR/GLOBAL if parent SQ has no coeff + reordering nsq_table number and testing only 5 NSQ SHAPE
    // NSQ_SEARCH_LEVEL6                              Allow only NSQ Inter-NEAREST/NEAR/GLOBAL if parent SQ has no coeff + reordering nsq_table number and testing only 6 NSQ SHAPE
    // NSQ_SEARCH_FULL                                Allow NSQ Intra-FULL and Inter-FULL
    
    if (MR_MODE)
        context_ptr->nsq_search_level = NSQ_SEARCH_FULL;
    else if (sc_content_detected)
        if (enc_mode <= ENC_M1)
            context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL6;
        else if (enc_mode <= ENC_M2)
            if (picture_control_set_ptr->temporal_layer_index == 0)
                context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL6;
            else if (picture_control_set_ptr->is_used_as_reference_flag)
                context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL4;
            else
                context_ptr->nsq_search_level = NSQ_SEARCH_OFF;
        else if (enc_mode <= ENC_M3)
            if (picture_control_set_ptr->temporal_layer_index == 0)
                context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL6;
            else if (picture_control_set_ptr->is_used_as_reference_flag)
                context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL1;
            else
                context_ptr->nsq_search_level = NSQ_SEARCH_OFF;
        else
            context_ptr->nsq_search_level = NSQ_SEARCH_OFF;
#if PREDICT_NSQ_SHAPE
    else if (picture_control_set_ptr->mdc_depth_level == (MAX_MDC_LEVEL - 1))
        context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL7;
#endif
#if rtime_presets
    else if (enc_mode <= ENC_M0)
        context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL6;
    else if (enc_mode <= ENC_M1)
        context_ptr->nsq_search_level = (picture_control_set_ptr->is_used_as_reference_flag) ? NSQ_SEARCH_LEVEL6 : NSQ_SEARCH_LEVEL3;
    else if (enc_mode <= ENC_M2)
        if (picture_control_set_ptr->is_used_as_reference_flag)
            context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL5;
        else
            context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL2;
#if M3_NSQ_MDC_CANDIDATE_IN_M4
        else if (picture_control_set_ptr->enc_mode <= ENC_M4)
            if (picture_control_set_ptr->is_used_as_reference_flag)
                context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL2;
            else
                context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL1;
#elif M3_NSQ_MDC_CANDIDATE
    else if (enc_mode <= ENC_M3)
        if (picture_control_set_ptr->is_used_as_reference_flag)
            context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL2;
        else
            context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL1;
#endif
    else
        context_ptr->nsq_search_level = NSQ_SEARCH_OFF;
#else
    else if (enc_mode <= ENC_M1)
        context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL6;

    else if (enc_mode <= ENC_M2)
        if (picture_control_set_ptr->is_used_as_reference_flag)
            context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL5;
        else
            context_ptr->nsq_search_level = NSQ_SEARCH_LEVEL3;
    else
        context_ptr->nsq_search_level = NSQ_SEARCH_OFF;
#endif
    if (context_ptr->nsq_search_level > NSQ_SEARCH_OFF)
        assert(sequence_control_set_ptr->nsq_present == 1 && "use nsq_present 1");

    switch (context_ptr->nsq_search_level) {
    case NSQ_SEARCH_OFF:
        context_ptr->nsq_max_shapes_md = 0;
        break;
    case NSQ_SEARCH_LEVEL1:
        context_ptr->nsq_max_shapes_md = 1;
        break;
    case NSQ_SEARCH_LEVEL2:
        context_ptr->nsq_max_shapes_md = 2;
        break;
    case NSQ_SEARCH_LEVEL3:
        context_ptr->nsq_max_shapes_md = 3;
        break;
    case NSQ_SEARCH_LEVEL4:
        context_ptr->nsq_max_shapes_md = 4;
        break;
    case NSQ_SEARCH_LEVEL5:
        context_ptr->nsq_max_shapes_md = 5;
        break;
    case NSQ_SEARCH_LEVEL6:
        context_ptr->nsq_max_shapes_md = 6;
        break;
#if PREDICT_NSQ_SHAPE
    case NSQ_SEARCH_LEVEL7:
        context_ptr->nsq_max_shapes_md = 7;
        break;
#endif
    case NSQ_SEARCH_FULL:
        context_ptr->nsq_max_shapes_md = 6;
        break;
    default:
        printf("nsq_search_level is not supported\n");
        break;
    }
    if (picture_control_set_ptr->pic_depth_mode > PIC_SQ_DEPTH_MODE)
        assert(context_ptr->nsq_search_level == NSQ_SEARCH_OFF);
    // Interpolation search Level                     Settings
    // 0                                              OFF
    // 1                                              Interpolation search at inter-depth
    // 2                                              Interpolation search at full loop
    // 3                                              Chroma blind interpolation search at fast loop
    // 4                                              Interpolation search at fast loop
    if (MR_MODE)
        context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP;
    else if (sc_content_detected)
        context_ptr->interpolation_search_level = IT_SEARCH_OFF;
#if rtime_presets
    else if (enc_mode <= ENC_M3)
        context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
#else
    else if (enc_mode <= ENC_M1)
        context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
    else if (enc_mode <= ENC_M3)
        if (picture_control_set_ptr->is_used_as_reference_flag)
            context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
        else
            context_ptr->interpolation_search_level = IT_SEARCH_OFF;
#endif
    else if (enc_mode <= ENC_M7)
        if (picture_control_set_ptr->temporal_layer_index == 0)
            context_ptr->interpolation_search_level = IT_SEARCH_FAST_LOOP_UV_BLIND;
        else
            context_ptr->interpolation_search_level = IT_SEARCH_OFF;
    else
        context_ptr->interpolation_search_level = IT_SEARCH_OFF;


  

#if PAL_SUP
    // Tx_search Level                                Settings
    // 0                                              OFF
    // 1                                              Tx search at encdec
    // 2                                              Tx search at inter-depth
    // 3                                              Tx search at full loop
    if (sc_content_detected)
        if (enc_mode <= ENC_M6)
            context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
        else
            if (picture_control_set_ptr->is_used_as_reference_flag)
                context_ptr->tx_search_level = TX_SEARCH_FULL_LOOP;
            else
                context_ptr->tx_search_level = TX_SEARCH_ENC_DEC;
    else
    if (enc_mode <= ENC_M4)
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
#if rtime_presets
    if (MR_MODE) // tx weight
        context_ptr->tx_weight = MAX_MODE_COST;
    else {
        if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
            context_ptr->tx_weight = MAX_MODE_COST;
        else if (!MR_MODE && enc_mode <= ENC_M5)
            context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
        else if (!MR_MODE) {
            if (picture_control_set_ptr->is_used_as_reference_flag)
                context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
            else
                context_ptr->tx_weight = FC_SKIP_TX_SR_TH010;
        }
    }
#else
    if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
        context_ptr->tx_weight = MAX_MODE_COST;
    else if (!MR_MODE && enc_mode <= ENC_M1)
        context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
    else if (!MR_MODE){
        if (picture_control_set_ptr->is_used_as_reference_flag)
            context_ptr->tx_weight = FC_SKIP_TX_SR_TH025;
        else
            context_ptr->tx_weight = FC_SKIP_TX_SR_TH010;
    }
#endif

    // Set tx search reduced set falg (0: full tx set; 1: reduced tx set; 1: two tx))
    if (sc_content_detected)
        if (enc_mode <= ENC_M1)
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
    else

    if (context_ptr->tx_search_level == TX_SEARCH_ENC_DEC)
        context_ptr->tx_search_reduced_set = 0;
#if rtime_presets
    else if (enc_mode <= ENC_M2)
        context_ptr->tx_search_reduced_set = 0;
    else
        context_ptr->tx_search_reduced_set = 1;
#else
    else if (enc_mode <= ENC_M1)
        context_ptr->tx_search_reduced_set = 0;
    else if (enc_mode <= ENC_M3)
        if (picture_control_set_ptr->is_used_as_reference_flag)
            context_ptr->tx_search_reduced_set = 0;
        else
            context_ptr->tx_search_reduced_set = 1;
    else
        context_ptr->tx_search_reduced_set = 1;
#endif
    // Intra prediction modes                       Settings
    // 0                                            FULL
    // 1                                            LIGHT per block : disable_z2_prediction && disable_angle_refinement  for 64/32/4
    // 2                                            OFF per block : disable_angle_prediction for 64/32/4
    // 3                                            OFF : disable_angle_prediction
    // 4                                            OIS based Intra
    // 5                                            Light OIS based Intra

    if (picture_control_set_ptr->slice_type == I_SLICE)
    if (sc_content_detected)
        if (enc_mode <= ENC_M6)
            context_ptr->intra_pred_mode = 0;
        else
            context_ptr->intra_pred_mode = 4;
    else
        if (enc_mode <= ENC_M6)
            context_ptr->intra_pred_mode = 0;
        else
            context_ptr->intra_pred_mode = 4;
    else {
    if (sc_content_detected)
        if (enc_mode == ENC_M0)
            context_ptr->intra_pred_mode = 0;
        else if (enc_mode <= ENC_M2)
            if (picture_control_set_ptr->temporal_layer_index == 0)
                context_ptr->intra_pred_mode = 1;
            else
                context_ptr->intra_pred_mode = 2;
        else if (enc_mode <= ENC_M6)
            if (picture_control_set_ptr->temporal_layer_index == 0)
                context_ptr->intra_pred_mode = 2;
            else
                context_ptr->intra_pred_mode = 3;
        else
            context_ptr->intra_pred_mode = 4;
    else
#if rtime_presets
        if ((enc_mode <= ENC_M1) || (enc_mode <= ENC_M2 && picture_control_set_ptr->temporal_layer_index == 0))
            context_ptr->intra_pred_mode = 0;
#else
        if (enc_mode == ENC_M0)
            context_ptr->intra_pred_mode = 0;
        else if (enc_mode  <= ENC_M1)
            if (picture_control_set_ptr->temporal_layer_index == 0)
                context_ptr->intra_pred_mode = 1;
            else
                context_ptr->intra_pred_mode = 2;
#endif
        else if(enc_mode <= ENC_M6)
            if (picture_control_set_ptr->temporal_layer_index == 0)
                context_ptr->intra_pred_mode = 1;
            else
                context_ptr->intra_pred_mode = 3;
        else
            context_ptr->intra_pred_mode = 4;
    }

    if (MR_MODE)
        context_ptr->intra_pred_mode = 0;

    // Skip sub blk based on neighbors depth        Settings
    // 0                                            OFF
    // 1                                            ON
    context_ptr->skip_sub_blks =   0;
    
    // Set atb mode      Settings
    // 0                 OFF: no transform partitioning
    // 1                 ON for INTRA blocks
    if (enc_mode <= ENC_M1 && sequence_control_set_ptr->static_config.encoder_bit_depth == EB_8BIT)

#if SPEED_OPT
        context_ptr->atb_mode = (MR_MODE || picture_control_set_ptr->temporal_layer_index == 0) ? 1 : 0;
#else
        context_ptr->atb_mode = 1;
#endif
    else
        context_ptr->atb_mode = 0;
    
    // Set skip atb                          Settings
    // 0                                     OFF
    // 1                                     ON

#if SPEED_OPT
    if (MR_MODE || picture_control_set_ptr->sc_content_detected)
#else
    if (MR_MODE || enc_mode == ENC_M0 || picture_control_set_ptr->sc_content_detected)
#endif
        context_ptr->coeff_based_skip_atb_sb = 0;
    else
        context_ptr->coeff_based_skip_atb_sb = 1;
    
    // Set Wedge mode      Settings
    // 0                 FULL: Full search
    // 1                 Fast: If two predictors are very similar, skip wedge compound mode search
    // 2                 Fast: estimate Wedge sign
    // 3                 Fast: Mode 1 & Mode 2
    
    context_ptr->wedge_mode = 0;

#if II_COMP_FLAG
    // inter intra pred                      Settings
    // 0                                     OFF
    // 1                                     ON
    context_ptr->enable_inter_intra = picture_control_set_ptr->slice_type != I_SLICE ? sequence_control_set_ptr->seq_header.enable_interintra_compound : 0;
#endif
    
    // Set compound mode      Settings
    // 0                 OFF: No compond mode search : AVG only
    // 1                 ON: compond mode search: AVG/DIST/DIFF
    // 2                 ON: AVG/DIST/DIFF/WEDGE
    
    if (sequence_control_set_ptr->compound_mode)
#if M0_tune
        if (picture_control_set_ptr->sc_content_detected)
            context_ptr->compound_mode = (enc_mode <= ENC_M0) ? 2 : 0;
        else
            context_ptr->compound_mode = enc_mode <= ENC_M1 ? 2 : 1;
#else
        context_ptr->compound_mode = picture_control_set_ptr->sc_content_detected ? 0 :
        enc_mode <= ENC_M1 ? 2 : 1;
#endif
    else
        context_ptr->compound_mode = 0;
#if SHUT_CPMPOUND
    context_ptr->compound_mode = 0;
#endif

    // set compound_types_to_try
    if (context_ptr->compound_mode)
        context_ptr->compound_types_to_try = context_ptr->compound_mode == 1 ? MD_COMP_DIFF0 : MD_COMP_WEDGE;
    else
        context_ptr->compound_types_to_try = MD_COMP_AVG;

    return return_error;
}
#endif
#endif

/******************************************************
* Set mode decision settings
******************************************************/
EbErrorType mpmd_settings(
    SequenceControlSet    *sequence_control_set_ptr,
    PictureControlSet     *picture_control_set_ptr,
    ModeDecisionContext   *context_ptr,
    uint8_t                enc_mode,
    uint8_t                is_last_md_pass) {
    EbErrorType return_error = EB_ErrorNone;

#if MPMD_ADD_SB_SETTINGS_TO_CTX
    mpmd_update_pic_settings_sb(
        sequence_control_set_ptr,
        picture_control_set_ptr->parent_pcs_ptr,
        context_ptr,
        enc_mode,
        is_last_md_pass);
#endif
    // Set Chroma Mode
    // Level                Settings
    // CHROMA_MODE_0  0     Full chroma search @ MD
    // CHROMA_MODE_1  1     Fast chroma search @ MD
    // CHROMA_MODE_2  2     Chroma blind @ MD + CFL @ EP
    // CHROMA_MODE_3  3     Chroma blind @ MD + no CFL @ EP
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
#if rtime_presets

        if (MR_MODE) 
            context_ptr->chroma_level = CHROMA_MODE_0;
        else
            if (enc_mode <= ENC_M5 && picture_control_set_ptr->temporal_layer_index == 0)


                context_ptr->chroma_level = CHROMA_MODE_0;
            else
                if (enc_mode <= ENC_M5)
                    context_ptr->chroma_level = CHROMA_MODE_1;
                else
                    context_ptr->chroma_level = (sequence_control_set_ptr->encoder_bit_depth == EB_8BIT) ?
                    CHROMA_MODE_2 :
                    CHROMA_MODE_3;

#else
    if (MR_MODE)
        context_ptr->chroma_level = CHROMA_MODE_0;
    else
    if (enc_mode == ENC_M0 && picture_control_set_ptr->temporal_layer_index == 0)
        context_ptr->chroma_level = CHROMA_MODE_0;
    else
    if (enc_mode <= ENC_M4)
        context_ptr->chroma_level = CHROMA_MODE_1;
    else
        context_ptr->chroma_level = (sequence_control_set_ptr->encoder_bit_depth == EB_8BIT) ?
            CHROMA_MODE_2 :
            CHROMA_MODE_3 ;
#endif
    // Set fast loop method
    // 1 fast loop: SSD_SEARCH not supported
    // Level                Settings
    //  0                   Collapsed fast loop
    //  1                   Decoupled fast loops ( intra/inter)
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M1)
            context_ptr->decouple_intra_inter_fast_loop = 0;
        else
            context_ptr->decouple_intra_inter_fast_loop = 1;
    else
    context_ptr->decouple_intra_inter_fast_loop = 0;

    // Set the search method when decoupled fast loop is used
    // Hsan: FULL_SAD_SEARCH not supported
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M1)
            context_ptr->decoupled_fast_loop_search_method = SSD_SEARCH;
        else
            context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;
    else
        if (enc_mode <= ENC_M4)
            context_ptr->decoupled_fast_loop_search_method = SSD_SEARCH;
        else
            context_ptr->decoupled_fast_loop_search_method = FULL_SAD_SEARCH;

    // Set the full loop escape level
    // Level                Settings
    // 0                    Off
    // 1                    On but only INTRA
    // 2                    On both INTRA and INTER
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M1)
            context_ptr->full_loop_escape = 0;
        else
            context_ptr->full_loop_escape = 2;
    else
    if (enc_mode <= ENC_M5)
        context_ptr->full_loop_escape = 0;
    else
        context_ptr->full_loop_escape = 2;

    // Set global MV injection
    // Level                Settings
    // 0                    Injection off (Hsan: but not derivation as used by MV ref derivation)
    // 1                    On
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M1)
            context_ptr->global_mv_injection = 1;
        else
            context_ptr->global_mv_injection = 0;
    else
    if (enc_mode <= ENC_M7)
        context_ptr->global_mv_injection = 1;
    else
        context_ptr->global_mv_injection = 0;
#if FIX_NEAREST_NEW
    if (enc_mode <= ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag)
#else
    if (enc_mode == ENC_M0)
#endif
        context_ptr->new_nearest_near_comb_injection = 1;
    else
        context_ptr->new_nearest_near_comb_injection = 0;
#if rtime_presets
    if (enc_mode <= ENC_M1)
#else
    if (enc_mode == ENC_M0)
#endif
        context_ptr->nx4_4xn_parent_mv_injection = 1;
    else
        context_ptr->nx4_4xn_parent_mv_injection = 0;

    // Set warped motion injection
    // Level                Settings
    // 0                    OFF
    // 1                    On
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        context_ptr->warped_motion_injection = 0;
    else
    context_ptr->warped_motion_injection = 1;

    // Set unipred3x3 injection
    // Level                Settings
    // 0                    OFF
    // 1                    ON FULL
    // 2                    Reduced set
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M1)
            context_ptr->unipred3x3_injection = 1;
        else
            context_ptr->unipred3x3_injection = 0;
    else
#if rtime_presets
     if (enc_mode <= ENC_M3)
#else
    if (enc_mode <= ENC_M1)
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
        if (enc_mode <= ENC_M1)
            context_ptr->bipred3x3_injection = 1;
        else
            context_ptr->bipred3x3_injection = 0;
    else
#if rtime_presets
    if (enc_mode <= ENC_M3)
#else
    if (enc_mode <= ENC_M1)
#endif
        context_ptr->bipred3x3_injection = 1;
    else if (enc_mode <= ENC_M4)
        context_ptr->bipred3x3_injection = 2;
    else
        context_ptr->bipred3x3_injection = 0;

    // Level                Settings
    // 0                    Level 0: OFF
    // 1                    Level 1: 7x5 full-pel search + sub-pel refinement off
    // 2                    Level 2: 7x5 full-pel search +  (H + V) sub-pel refinement only = 4 half-pel + 4 quarter-pel = 8 positions + pred_me_distortion to pa_me_distortion deviation on
    // 3                    Level 3: 7x5 full-pel search +  (H + V + D only ~ the best) sub-pel refinement = up to 6 half-pel + up to 6  quarter-pel = up to 12 positions + pred_me_distortion to pa_me_distortion deviation on
    // 4                    Level 4: 7x5 full-pel search +  (H + V + D) sub-pel refinement = 8 half-pel + 8 quarter-pel = 16 positions + pred_me_distortion to pa_me_distortion deviation on
    // 5                    Level 5: 7x5 full-pel search +  (H + V + D) sub-pel refinement = 8 half-pel + 8 quarter-pel = 16 positions + pred_me_distortion to pa_me_distortion deviation off
    if (picture_control_set_ptr->slice_type != I_SLICE)
#if rtime_presets
        if (enc_mode <= ENC_M2)
#else
        if (enc_mode <= ENC_M1)
#endif
#if M0_tune
            context_ptr->predictive_me_level = (enc_mode <= ENC_M0)? 5: 4;
#else
            context_ptr->predictive_me_level = 4;
#endif
        else if (enc_mode <= ENC_M4)
            context_ptr->predictive_me_level = 2;
        else
            context_ptr->predictive_me_level = 0;
    else
        context_ptr->predictive_me_level = 0;

    // Derive md_staging_mode
    //
#if REMOVE_MD_STAGE_1
    // MD_STAGING_MODE_0
    // Default Parameters
    //
    // MD_STAGING_MODE_1
    //  __________________________________________________________________________________________________________________
    // |        | md_stage_0                  | md_stage_2                     | md_stage_3                              |
    // |________|_____________________________|________________________________|_________________________________________|
    // |CLASS_0 |Prediction for Luma & Chroma |T, Q, Q-1, T-1 for Luma Only    |T, Q, Q-1, T-1 or Luma & Chroma          |
    // |CLASS_6 |                             |No RDOQ                         |RDOQ                                     |
    // |        |                             |No Tx Type Search               |Tx Type Search                           |
    // |        |                             |No Tx Size Search               |Tx Size Search                           |
    // |        |                             |                                |CFL vs. Independent                      |
    // |________|_____________________________|________________________________|_________________________________________|
    // |CLASS_1 |Prediction for Luma Only     |T, Q, Q-1, T-1 for Luma Only    |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |CLASS_2 |No Interpolation Search      |No RDOQ                         |RDOQ                                     |
    // |CLASS_3 |Bilinear Interpolation       |No Tx Type Search               |Tx Type Search                           |
    // |CLASS_4 |                             |No Tx Size Search               |Tx Size Search                           |
    // |CLASS_5 |                             |Interpolation Search            |                                         |
    // |________|_____________________________|________________________________|_________________________________________|

    if (enc_mode <= ENC_M4)
        context_ptr->md_staging_mode = MD_STAGING_MODE_1;
    else
        context_ptr->md_staging_mode = MD_STAGING_MODE_0;
#else
    // MD_STAGING_MODE_1
    //  _______________________________________________________________________________________________________________________________________________
    // |        | md_stage_0                  | md_stage_1                  | md_stage_2                     | md_stage_3                              |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_0 |Prediction for Luma & Chroma |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 or Luma & Chroma          |
    // |        |No Interpolation Search      |                             |RDOQ                            |RDOQ                                     |
    // |        |Regular Interpolation        |                             |No Tx Search                    |Tx Search                                |
    // |        |                             |                             |No ATB                          |ATB                                      |
    // |        |                             |                             |                                |CFL vs. Independent                      |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_1 |Prediction for Chroma        |Prediction for Luma & Chroma |Bypassed                        |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |                                |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |                                |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_2 |Prediction for Chroma        |Prediction for Luma & Chroma |Bypassed                        |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |                                |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |                                |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_3 |Prediction for Chroma        |Prediction for Luma & Chroma |Bypassed                        |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |                                |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |                                |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    //
    // MD_STAGING_MODE_2
    //  _______________________________________________________________________________________________________________________________________________
    // |        | md_stage_0                  | md_stage_1                  | md_stage_2                     | md_stage_3                              |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_0 |Prediction for Luma & Chroma |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 or Luma & Chroma          |
    // |        |No Interpolation Search      |                             |No RDOQ                         |RDOQ                                     |
    // |        |Regular Interpolation        |                             |No Tx Search                    |Tx Search                                |
    // |        |                             |                             |No ATB                          |ATB                                      |
    // |        |                             |                             |                                |CFL vs. Independent                      |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_1 |Prediction for Chroma        |Prediction for Luma & Chroma |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_2 |Prediction for Chroma        |Prediction for Luma & Chroma |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_3 |Prediction for Chroma        |Prediction for Luma & Chroma |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |Interpolation Search         |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    //
    // MD_STAGING_MODE_3
    //  _______________________________________________________________________________________________________________________________________________
    // |        | md_stage_0                  | md_stage_1                  | md_stage_2                     | md_stage_3                              |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_0 |Prediction for Luma & Chroma |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 or Luma & Chroma          |
    // |        |No Interpolation Search      |                             |No RDOQ                         |RDOQ                                     |
    // |        |Regular Interpolation        |                             |No Tx Search                    |Tx Search                                |
    // |        |                             |                             |No ATB                          |ATB                                      |
    // |        |                             |                             |                                |CFL vs. Independent                      |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_1 |Prediction for Chroma        |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |                             |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_2 |Prediction for Chroma        |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |                             |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|
    // |CLASS_3 |Prediction for Chroma        |Bypassed                     |T, Q, Q-1, T-1 for Luma         |T, Q, Q-1, T-1 for Luma & Chroma         |
    // |        |No Interpolation Search      |                             |No RDOQ                         |Tx Search                                |
    // |        |Bilinear Interpolation       |                             |No Tx Search                    |                                         |
    // |        |                             |                             |                                |                                         |
    // |________|_____________________________|_____________________________|________________________________|_________________________________________|

    if (enc_mode == ENC_M0)
        context_ptr->md_staging_mode = MD_STAGING_MODE_1;
    else if (enc_mode <= ENC_M4)
        context_ptr->md_staging_mode = MD_STAGING_MODE_3;
    else
        context_ptr->md_staging_mode = MD_STAGING_MODE_0; // Default structure = fast loop + full loop = md_stage_0 + md_stage_3
#endif
    // Combine MD Class1&2
    // 0                    OFF
    // 1                    ON
#if rtime_presets
    context_ptr->combine_class12 = (enc_mode <= ENC_M1) ? 0 : 1;
#else
    context_ptr->combine_class12 = (enc_mode == ENC_M0) ? 0 : 1;
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

    // Set PF MD
    context_ptr->pf_md_mode = PF_OFF;

    // Derive Spatial SSE Flag
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M6)
            context_ptr->spatial_sse_full_loop = EB_TRUE;
        else
            context_ptr->spatial_sse_full_loop = EB_FALSE;
    else
    if (enc_mode <= ENC_M4)
        context_ptr->spatial_sse_full_loop = EB_TRUE;
    else
        context_ptr->spatial_sse_full_loop = EB_FALSE;

    if (context_ptr->chroma_level <= CHROMA_MODE_1)
        context_ptr->blk_skip_decision = EB_TRUE;
    else
        context_ptr->blk_skip_decision = EB_FALSE;
    // Derive Trellis Quant Coeff Optimization Flag
#if rtime_presets
    if (enc_mode <= ENC_M3)
#else
    if (enc_mode == ENC_M0)
#endif
        context_ptr->trellis_quant_coeff_optimization = EB_TRUE;
    else
        context_ptr->trellis_quant_coeff_optimization = EB_FALSE;

    // Derive redundant block
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected)
        if (enc_mode <= ENC_M1)
            context_ptr->redundant_blk = EB_TRUE;
        else
            context_ptr->redundant_blk = EB_FALSE;
    else
    if (enc_mode <= ENC_M5)
        context_ptr->redundant_blk = EB_TRUE;
    else
        context_ptr->redundant_blk = EB_FALSE;
    if (sequence_control_set_ptr->static_config.encoder_bit_depth == EB_8BIT)
#if FIX_ESTIMATE_INTRA
        if (MR_MODE)
#else
        if (MR_MODE || enc_mode == ENC_M0)
#endif
            context_ptr->edge_based_skip_angle_intra = 0;
        else
#if FIX_ESTIMATE_INTRA
#if M0_tune
            context_ptr->edge_based_skip_angle_intra = (enc_mode == ENC_M0) ? 0 : 1;
#else
#if rtime_presets
            context_ptr->edge_based_skip_angle_intra = ((enc_mode > ENC_M3) || (enc_mode == ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0)) ? 0 : 1;
#else
            context_ptr->edge_based_skip_angle_intra = (enc_mode == ENC_M0 && picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0) ? 0 : 1;
#endif
#endif
#else
            context_ptr->edge_based_skip_angle_intra = 1;
#endif
    else
        context_ptr->edge_based_skip_angle_intra = 0;
    if (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected || enc_mode == ENC_M0)
        context_ptr->prune_ref_frame_for_rec_partitions = 0;
    else
        context_ptr->prune_ref_frame_for_rec_partitions = 1;

#if SPEED_OPT
    // Derive INTER/INTER WEDGE variance TH
    if (MR_MODE)
        context_ptr->inter_inter_wedge_variance_th = 0;
    else
        context_ptr->inter_inter_wedge_variance_th = 100;

    // Derive MD Exit TH
    if (MR_MODE)
        context_ptr->md_exit_th = 0;
    else
        context_ptr->md_exit_th = (picture_control_set_ptr->parent_pcs_ptr->sc_content_detected) ? 10 : 18;

    // Derive distortion-based md_stage_0_count proning
    if (MR_MODE)
        context_ptr->dist_base_md_stage_0_count_th = (uint64_t)~0;
    else
        context_ptr->dist_base_md_stage_0_count_th = 75;
#endif


#if LESS_RECTANGULAR_CHECK_LEVEL

    // Weighting (expressed as a percentage) applied to 
    // square shape costs for determining if a and b 
    // shapes should be skipped. Namely:
    // skip HA and HB if h_cost > (weighted sq_cost)
    // skip VA and VB if v_cost > (weighted sq_cost)
    
    if (MR_MODE)
        context_ptr->sq_to_h_v_weight_to_skip_a_b = (uint32_t)~0;
    else
        context_ptr->sq_to_h_v_weight_to_skip_a_b = 100;
#endif


    return return_error;
}

#if MPMD_TEST
uint8_t md_mode_settings_offset[13] = { 0,0,0,0,0,0,0,0,0,0,0,0,0 };
#else
uint8_t md_mode_settings_offset[13] = { 4,0,0,0,0,0,0,0,0,0,0,0,0 };
#endif
EbErrorType mpmd_pass_settings(
    SequenceControlSet    *sequence_control_set_ptr,
    PictureControlSet     *picture_control_set_ptr,
    ModeDecisionContext   *context_ptr,
    uint8_t                is_last_md_pass){
    EbErrorType return_error = EB_ErrorNone;
    uint8_t enc_mode = picture_control_set_ptr->enc_mode;
    uint8_t md_mode = enc_mode;
    if (!is_last_md_pass) {
        md_mode = md_mode + md_mode_settings_offset[enc_mode];
    }
    mpmd_settings(
        sequence_control_set_ptr,
        picture_control_set_ptr,
        context_ptr,
        md_mode,
        is_last_md_pass);
    return return_error;
}
#if MPMD_ADAPTIVE_REFINEMENT
uint8_t set_refinement_level_sb(
    SequenceControlSet  *sequence_control_set_ptr,
    PictureControlSet   *picture_control_set_ptr,
    ModeDecisionContext *context_ptr,
    uint32_t            sb_index) {

    LargestCodingUnit *sb_ptr = picture_control_set_ptr->sb_ptr_array[sb_index];
    uint64_t mincost = MAX_CU_COST;
    uint8_t i,best_depth;
    for (i = 0; i < 6; i++) {
        if (mincost < sb_ptr->md_depth_cost[i]) {
            mincost = sb_ptr->md_depth_cost[i];
            best_depth = i;
        }
    }

    return best_depth;
}
#endif
#endif
void av1_estimate_syntax_rate___partial(
    MdRateEstimationContext        *md_rate_estimation_array,
    FRAME_CONTEXT                  *fc);
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

    segment_index = 0;

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
            reset_mode_decision(
#if EIGHT_PEL_PREDICTIVE_ME
                sequence_control_set_ptr,
#endif
                context_ptr->md_context,
                picture_control_set_ptr,
                segment_index);

            // Reset EncDec Coding State
            ResetEncDec(    // HT done
                context_ptr,
                picture_control_set_ptr,
                sequence_control_set_ptr,
                segment_index);

            if (picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr != NULL)
                ((EbReferenceObject  *)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->average_intensity = picture_control_set_ptr->parent_pcs_ptr->average_intensity[0];
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

                    if (picture_control_set_ptr->update_cdf) {
                        picture_control_set_ptr->rate_est_array[sb_index] = *picture_control_set_ptr->md_rate_estimation_array;
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
                    // Configure the LCU
                    mode_decision_configure_lcu(
                        context_ptr->md_context,
                        picture_control_set_ptr,
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

                        mv_l0_x = 0;
                        mv_l0_y = 0;
                        mv_l1_x = 0;
                        mv_l1_y = 0;

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
#if SKIP_2ND_PASS_BASED_ON_1ST_PASS
                    uint8_t skip_next_pass = 0;
                    context_ptr->md_context->sb_cost = 0;
                    context_ptr->md_context->sb_dist = 0;
                    uint64_t dist_th = 1000;
                    // Th0 
                    uint64_t cost_th0 = 200000000 / context_ptr->md_context->qp;
                    // Th1
                    uint64_t cost_th1 = 2000000000 / context_ptr->md_context->qp;
                    // Th2
                    uint64_t cost_th2 = 20000000000 / context_ptr->md_context->qp;
                    // Th3
                    //uint64_t cost_th3 = 200000000000 / context_ptr->md_context->qp;
                    //Th4
                    //uint64_t cost_th4 = 2000000000000 / context_ptr->md_context->qp;
#endif

                    mpmd_pass_settings(
                        sequence_control_set_ptr,
                        picture_control_set_ptr,
                        context_ptr->md_context,
                        1 );//is_last_md_pass,

                    uint8_t  is_complete_sb = sequence_control_set_ptr->sb_geom[sb_index].is_complete_sb;
                    uint8_t enable_mpmd = (picture_control_set_ptr->slice_type != I_SLICE && is_complete_sb) ? 1 : 0;
                    if (enable_mpmd) {
                        for (mpmd_pass_idx = 0; mpmd_pass_idx < mpmd_pass_num; ++mpmd_pass_idx) {

                            init_cu_arr_nsq(
                                sequence_control_set_ptr->max_block_cnt,
                                context_ptr->md_context,
                                sb_index);

                            mpmd_pass_settings(
                                sequence_control_set_ptr,
                                picture_control_set_ptr,
                                context_ptr->md_context,
                                0);//is_last_md_pass,
#if MPMD_SB_REF
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
                                mpmd_pass_idx,
                                sequence_control_set_ptr,
                                picture_control_set_ptr,
                                mdcPtr,
                                mpmd_sb,
                                sb_ptr,
                                sb_origin_x,
                                sb_origin_y,
                                sb_index,
                                context_ptr->ss_mecontext,
                                context_ptr->md_context);
#if MPMD_ADAPTIVE_REFINEMENT
                            uint8_t best_depth = set_refinement_level_sb(
                                sequence_control_set_ptr,
                                picture_control_set_ptr,
                                context_ptr->md_context,
                                sb_index);
#endif
                            // Restore the clean copy of the neighbor arrays for next md pass
#if SKIP_2ND_PASS_BASED_ON_1ST_PASS
                            gather_rd_sb(
                                sequence_control_set_ptr,
                                picture_control_set_ptr,
                                context_ptr->md_context,
                                sb_index);
                            skip_next_pass =  (context_ptr->md_context->sb_cost < cost_th0) ? 1 : 0;
                            uint8_t mpmd_depth_level = 6; // 1 SQ PART only
                            uint8_t mpmd_1d_level = 0; // 1 NSQ PART only
                            if(context_ptr->md_context->sb_cost < cost_th1)
                                mpmd_depth_level = 1;
                            else if (context_ptr->md_context->sb_cost < cost_th2)
                                mpmd_depth_level = 4;
                            if (!skip_next_pass) {
#endif
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
#if USE_MSMD_OUTPUT
                                    mpmd_pass_idx,
#endif
                                    context_ptr->md_context,
                                    sb_index);
                                //check_for_errors(sequence_control_set_ptr->max_block_cnt,mpmd_sb, mdcPtr,context_ptr->md_context);
                                //Update mdc array for next stage
                                copy_mdc_array_data(sequence_control_set_ptr->max_block_cnt, mpmd_sb, mdcPtr);
#endif
#if SKIP_2ND_PASS_BASED_ON_1ST_PASS
                            }
#endif

                        }
                        // Reset for final pass
#if SKIP_2ND_PASS_BASED_ON_1ST_PASS
                        if (!skip_next_pass) {
#endif
                            reset_local_cu_cost(context_ptr->md_context);
                            reset_ep_pipe_sb(context_ptr->md_context);
                            init_cu_arr_nsq(
                                sequence_control_set_ptr->max_block_cnt,
                                context_ptr->md_context,
                                sb_index);

                            mpmd_pass_settings(
                                sequence_control_set_ptr,
                                picture_control_set_ptr,
                                context_ptr->md_context,
                                1);

#if SKIP_2ND_PASS_BASED_ON_1ST_PASS
                        }
#endif
                    }
#endif
#if SKIP_2ND_PASS_BASED_ON_1ST_PASS
                   if (!skip_next_pass)
#endif
                    mode_decision_sb(
#if MPMD_SB
                        1,
                        mpmd_pass_num,
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
                        context_ptr);
#endif

                    if (picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr != NULL)
                        ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->intra_coded_area_sb[sb_index] = (uint8_t)((100 * context_ptr->intra_coded_area_sb[sb_index]) / (64 * 64));
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
                        = picture_control_set_ptr->parent_pcs_ptr->frm_hdr.film_grain_params;
                }
            }
            if (picture_control_set_ptr->parent_pcs_ptr->frame_end_cdf_update_mode && picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE && picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr)
                for (int frame = LAST_FRAME; frame <= ALTREF_FRAME; ++frame)
                    ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->global_motion[frame]
                    = picture_control_set_ptr->parent_pcs_ptr->global_motion[frame];
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

void eb_av1_add_film_grain(EbPictureBufferDesc *src,
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

    eb_av1_add_film_grain_run(&params, luma, cb, cr, height, width, luma_stride,
        chroma_stride, use_high_bit_depth, chroma_subsamp_y,
        chroma_subsamp_x);
    return;
}
