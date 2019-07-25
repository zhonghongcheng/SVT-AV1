/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EbFullLoop_h
#define EbFullLoop_h

#include "EbModeDecisionProcess.h"
#ifdef __cplusplus
extern "C" {
#endif

    void full_loop_r(
        LargestCodingUnit            *sb_ptr,
        ModeDecisionCandidateBuffer  *candidateBuffer,
        ModeDecisionContext          *context_ptr,
        EbPictureBufferDesc          *input_picture_ptr,
        PictureControlSet            *picture_control_set_ptr,
        uint32_t                          component_mask,
        uint32_t                          cb_qp,
        uint32_t                          cr_qp,
        uint32_t                          *cb_count_non_zero_coeffs,
        uint32_t                          *cr_count_non_zero_coeffs);

    void cu_full_distortion_fast_tu_mode_r(
        LargestCodingUnit            *sb_ptr,
        ModeDecisionCandidateBuffer  *candidateBuffer,
        ModeDecisionContext            *context_ptr,
        ModeDecisionCandidate           *candidate_ptr,
        PictureControlSet            *picture_control_set_ptr,
        uint64_t                          cbFullDistortion[DIST_CALC_TOTAL],
        uint64_t                          crFullDistortion[DIST_CALC_TOTAL],
        uint32_t                          count_non_zero_coeffs[3][MAX_NUM_OF_TU_PER_CU],
        COMPONENT_TYPE                  component_type,
        uint64_t                         *cb_coeff_bits,
        uint64_t                         *cr_coeff_bits,
#if SPATIAL_SSE
        EbBool                           is_full_loop,
#endif
        EbAsm                            asm_type);

    void product_full_loop(
        ModeDecisionCandidateBuffer  *candidateBuffer,
        ModeDecisionContext          *context_ptr,
        PictureControlSet            *picture_control_set_ptr,
        uint32_t                          qp,
        uint32_t                           *y_count_non_zero_coeffs,
        uint64_t                         *y_coeff_bits,
        uint64_t                         *y_full_distortion);

    void product_full_loop_tx_search(
        ModeDecisionCandidateBuffer  *candidateBuffer,
        ModeDecisionContext          *context_ptr,
#if ADAPTIVE_TXB_SEARCH_LEVEL
        uint64_t                     ref_best_rd,
#endif
        PictureControlSet            *picture_control_set_ptr);
    extern uint32_t d2_inter_depth_block_decision(
        ModeDecisionContext          *context_ptr,
        uint32_t                        blk_mds,
        LargestCodingUnit            *tb_ptr,
        uint32_t                          lcuAddr,
        uint32_t                          tbOriginX,
        uint32_t                          tbOriginY,
        uint64_t                          full_lambda,
        MdRateEstimationContext      *md_rate_estimation_ptr,
        PictureControlSet            *picture_control_set_ptr);
#if MD_EXIT
    /// compute the cost of curr depth, and the depth above
    extern void   compute_depth_costs_md_skip(
        ModeDecisionContext    *context_ptr,
        SequenceControlSet     *sequence_control_set_ptr,
        uint32_t                  curr_depth_mds,
        uint32_t                  above_depth_mds,
        uint32_t                  step,
        uint64_t                 *above_depth_cost,
        uint64_t                 *curr_depth_cost);
#endif


    void  d1_non_square_block_decision(
        ModeDecisionContext               *context_ptr
    );

#ifdef __cplusplus
}
#endif
#endif // EbFullLoop_h
