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

#include "EbDefinitions.h"
#include "EbMdRateEstimation.h"
#if RATE_ESTIMATION_UPDATE
#include "EbCommonUtils.h"
#include "EbSequenceControlSet.h"
#include "filter.h"
#include "EbEntropyCoding.h"
#endif
#include "EbBitstreamUnit.h"

static INLINE int32_t get_interinter_wedge_bits(BlockSize sb_type) {
    const int32_t wbits = wedge_params_lookup[sb_type].bits;
    return (wbits > 0) ? wbits + 1 : 0;
}

/**************************************************************
* AV1GetCostSymbold
* Calculate the cost of a symbol with
* probability p15 / 2^15
***************************************************************/
static INLINE int32_t av1_cost_symbol(AomCdfProb p15) {
    assert(0 < p15 && p15 < CDF_PROB_TOP);
    const int32_t shift = CDF_PROB_BITS - 1 - get_msb(p15);
    const int32_t prob = get_prob(p15 << shift, CDF_PROB_TOP);
    assert(prob >= 128);
    return av1_prob_cost[prob - 128] + av1_cost_literal(shift);
}

/*************************************************************
* av1_get_syntax_rate_from_cdf
**************************************************************/
void av1_get_syntax_rate_from_cdf(
    int32_t                      *costs,
    const AomCdfProb       *cdf,
    const int32_t                *inv_map)
{
    int32_t i;
    AomCdfProb prev_cdf = 0;
    for (i = 0;; ++i) {
        AomCdfProb p15 = AOM_ICDF(cdf[i]) - prev_cdf;
        p15 = (p15 < EC_MIN_PROB) ? EC_MIN_PROB : p15;
        prev_cdf = AOM_ICDF(cdf[i]);

        if (inv_map)
            costs[inv_map[i]] = av1_cost_symbol(p15);
        else
            costs[i] = av1_cost_symbol(p15);

        // Stop once we reach the end of the CDF
        if (cdf[i] == AOM_ICDF(CDF_PROB_TOP)) break;
    }
}

///tmp function to be removed once we have updated all syntax CDFs
void av1_estimate_syntax_rate___partial(
    MdRateEstimationContext  *md_rate_estimation_array,
    FRAME_CONTEXT              *fc)
{
    int32_t i, j;

    md_rate_estimation_array->initialized = 1;
#if CABAC_UP1
    for (i = 0; i < PARTITION_CONTEXTS; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->partitionFacBits[i], fc->partition_cdf[i], NULL);
#endif

#if CABAC_UP2
    //if (cm->skip_mode_flag) { // NM - Hardcoded to true
    for (i = 0; i < SKIP_CONTEXTS; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->skipModeFacBits[i], fc->skip_mode_cdfs[i], NULL);
    //}
#endif

    for (i = TX_4X4; i < EXT_TX_SIZES; ++i) {
        int32_t s;
        for (s = 1; s < EXT_TX_SETS_INTER; ++s) {
            if (use_inter_ext_tx_for_txsize[s][i])
                av1_get_syntax_rate_from_cdf(md_rate_estimation_array->inter_tx_type_fac_bits[s][i], fc->inter_ext_tx_cdf[s][i], av1_ext_tx_inv[av1_ext_tx_set_idx_to_type[1][s]]);
        }
        for (s = 1; s < EXT_TX_SETS_INTRA; ++s) {
            if (use_intra_ext_tx_for_txsize[s][i]) {
                for (j = 0; j < INTRA_MODES; ++j)
                    av1_get_syntax_rate_from_cdf(md_rate_estimation_array->intra_tx_type_fac_bits[s][i][j], fc->intra_ext_tx_cdf[s][i][j], av1_ext_tx_inv[av1_ext_tx_set_idx_to_type[0][s]]);
            }
        }
    }
}
#if FILTER_INTRA_FLAG
int av1_filter_intra_allowed_bsize(  uint8_t enable_filter_intra,  BlockSize bs);
#if !PAL_SUP
int av1_filter_intra_allowed(uint8_t   enable_filter_intra, BlockSize bsize, uint32_t  mode);
#endif
#endif
/*************************************************************
* av1_estimate_syntax_rate()
* Estimate the rate for each syntax elements and for
* all scenarios based on the frame CDF
**************************************************************/
void av1_estimate_syntax_rate(
    MdRateEstimationContext  *md_rate_estimation_array,
    EbBool                     is_i_slice,
    FRAME_CONTEXT              *fc)
{
    int32_t i, j;

    md_rate_estimation_array->initialized = 1;

    for (i = 0; i < PARTITION_CONTEXTS; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->partition_fac_bits[i], fc->partition_cdf[i], NULL);

    //if (cm->skip_mode_flag) { // NM - Hardcoded to true
    for (i = 0; i < SKIP_CONTEXTS; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->skip_mode_fac_bits[i], fc->skip_mode_cdfs[i], NULL);
    //}

    for (i = 0; i < SKIP_CONTEXTS; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->skip_fac_bits[i], fc->skip_cdfs[i], NULL);
    for (i = 0; i < KF_MODE_CONTEXTS; ++i)
        for (j = 0; j < KF_MODE_CONTEXTS; ++j)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->y_mode_fac_bits[i][j], fc->kf_y_cdf[i][j], NULL);

    for (i = 0; i < BlockSize_GROUPS; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->mb_mode_fac_bits[i], fc->y_mode_cdf[i], NULL);

    for (i = 0; i < CFL_ALLOWED_TYPES; ++i) {
        for (j = 0; j < INTRA_MODES; ++j)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->intra_uv_mode_fac_bits[i][j], fc->uv_mode_cdf[i][j], NULL);
    }

    av1_get_syntax_rate_from_cdf(md_rate_estimation_array->filter_intra_mode_fac_bits, fc->filter_intra_mode_cdf, NULL);
#if FILTER_INTRA_FLAG
    for (i = 0; i < BlockSizeS_ALL; ++i) {
        if (av1_filter_intra_allowed_bsize(1,i))
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->filter_intra_fac_bits[i], fc->filter_intra_cdfs[i], NULL);
    }
#else
    // NM - To be added when intra filtering is adopted
    /*for (i = 0; i < BlockSizeS_ALL; ++i) {
        if (av1_filter_intra_allowed_bsize(cm, i))
            av1_FacBits_tokens_from_cdf(md_rate_estimation_array->filter_intra_fac_bits[i],
            fc->filter_intra_cdfs[i], NULL);
    }*/

    // NM - To be added when inter filtering is adopted
#endif
    for (i = 0; i < SWITCHABLE_FILTER_CONTEXTS; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->switchable_interp_fac_bitss[i], fc->switchable_interp_cdf[i], NULL);

    for (i = 0; i < PALATTE_BSIZE_CTXS; ++i) {
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->palette_ysize_fac_bits[i], fc->palette_y_size_cdf[i], NULL);
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->palette_uv_size_fac_bits[i], fc->palette_uv_size_cdf[i], NULL);
        for (j = 0; j < PALETTE_Y_MODE_CONTEXTS; ++j)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->palette_ymode_fac_bits[i][j], fc->palette_y_mode_cdf[i][j], NULL);
    }

    for (i = 0; i < PALETTE_UV_MODE_CONTEXTS; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->palette_uv_mode_fac_bits[i], fc->palette_uv_mode_cdf[i], NULL);
    for (i = 0; i < PALETTE_SIZES; ++i) {
        for (j = 0; j < PALETTE_COLOR_INDEX_CONTEXTS; ++j) {
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->palette_ycolor_fac_bitss[i][j], fc->palette_y_color_index_cdf[i][j], NULL);
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->palette_uv_color_fac_bits[i][j], fc->palette_uv_color_index_cdf[i][j], NULL);
        }
    }

    int32_t sign_FacBits[CFL_JOINT_SIGNS];
    av1_get_syntax_rate_from_cdf(sign_FacBits, fc->cfl_sign_cdf, NULL);
    for (int32_t joint_sign = 0; joint_sign < CFL_JOINT_SIGNS; joint_sign++) {
        int32_t *FacBits_u = md_rate_estimation_array->cfl_alpha_fac_bits[joint_sign][CFL_PRED_U];
        int32_t *FacBits_v = md_rate_estimation_array->cfl_alpha_fac_bits[joint_sign][CFL_PRED_V];
        if (CFL_SIGN_U(joint_sign) == CFL_SIGN_ZERO)
            memset(FacBits_u, 0, CFL_ALPHABET_SIZE * sizeof(*FacBits_u));
        else {
            const AomCdfProb *cdf_u = fc->cfl_alpha_cdf[CFL_CONTEXT_U(joint_sign)];
            av1_get_syntax_rate_from_cdf(FacBits_u, cdf_u, NULL);
        }
        if (CFL_SIGN_V(joint_sign) == CFL_SIGN_ZERO)
            memset(FacBits_v, 0, CFL_ALPHABET_SIZE * sizeof(*FacBits_v));
        else {
            int32_t cdf_index = CFL_CONTEXT_V(joint_sign);
            if ((cdf_index < CFL_ALPHA_CONTEXTS) && (cdf_index >= 0)) {
                const AomCdfProb *cdf_v = fc->cfl_alpha_cdf[cdf_index];
                av1_get_syntax_rate_from_cdf(FacBits_v, cdf_v, NULL);
            }
        }
        for (int32_t u = 0; u < CFL_ALPHABET_SIZE; u++)
            FacBits_u[u] += sign_FacBits[joint_sign];
    }

    for (i = 0; i < MAX_TX_CATS; ++i)
        for (j = 0; j < TX_SIZE_CONTEXTS; ++j)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->tx_size_fac_bits[i][j], fc->tx_size_cdf[i][j],
                NULL);

    for (i = 0; i < TXFM_PARTITION_CONTEXTS; ++i) {
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->txfm_partition_fac_bits[i],
            fc->txfm_partition_cdf[i], NULL);
    }

    for (i = TX_4X4; i < EXT_TX_SIZES; ++i) {
        int32_t s;
        for (s = 1; s < EXT_TX_SETS_INTER; ++s) {
            if (use_inter_ext_tx_for_txsize[s][i])
                av1_get_syntax_rate_from_cdf(md_rate_estimation_array->inter_tx_type_fac_bits[s][i], fc->inter_ext_tx_cdf[s][i], av1_ext_tx_inv[av1_ext_tx_set_idx_to_type[1][s]]);
        }
        for (s = 1; s < EXT_TX_SETS_INTRA; ++s) {
            if (use_intra_ext_tx_for_txsize[s][i]) {
                for (j = 0; j < INTRA_MODES; ++j)
                    av1_get_syntax_rate_from_cdf(md_rate_estimation_array->intra_tx_type_fac_bits[s][i][j], fc->intra_ext_tx_cdf[s][i][j], av1_ext_tx_inv[av1_ext_tx_set_idx_to_type[0][s]]);
            }
        }
    }
    for (i = 0; i < DIRECTIONAL_MODES; ++i)
        av1_get_syntax_rate_from_cdf(md_rate_estimation_array->angle_delta_fac_bits[i], fc->angle_delta_cdf[i], NULL);
    av1_get_syntax_rate_from_cdf(md_rate_estimation_array->switchable_restore_fac_bits, fc->switchable_restore_cdf, NULL);
    av1_get_syntax_rate_from_cdf(md_rate_estimation_array->wiener_restore_fac_bits, fc->wiener_restore_cdf, NULL);
    av1_get_syntax_rate_from_cdf(md_rate_estimation_array->sgrproj_restore_fac_bits, fc->sgrproj_restore_cdf, NULL);
    av1_get_syntax_rate_from_cdf(md_rate_estimation_array->intrabc_fac_bits, fc->intrabc_cdf, NULL);

    if (!is_i_slice) { // NM - Hardcoded to true
    //if (1){
        for (i = 0; i < COMP_INTER_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->comp_inter_fac_bits[i], fc->comp_inter_cdf[i], NULL);
        for (i = 0; i < REF_CONTEXTS; ++i) {
            for (j = 0; j < SINGLE_REFS - 1; ++j)
                av1_get_syntax_rate_from_cdf(md_rate_estimation_array->single_ref_fac_bits[i][j], fc->single_ref_cdf[i][j], NULL);
        }

        for (i = 0; i < COMP_REF_TYPE_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->comp_ref_type_fac_bits[i], fc->comp_ref_type_cdf[i], NULL);
        for (i = 0; i < UNI_COMP_REF_CONTEXTS; ++i) {
            for (j = 0; j < UNIDIR_COMP_REFS - 1; ++j)
                av1_get_syntax_rate_from_cdf(md_rate_estimation_array->uni_comp_ref_fac_bits[i][j], fc->uni_comp_ref_cdf[i][j], NULL);
        }

        for (i = 0; i < REF_CONTEXTS; ++i) {
            for (j = 0; j < FWD_REFS - 1; ++j)
                av1_get_syntax_rate_from_cdf(md_rate_estimation_array->comp_ref_fac_bits[i][j], fc->comp_ref_cdf[i][j], NULL);
        }

        for (i = 0; i < REF_CONTEXTS; ++i) {
            for (j = 0; j < BWD_REFS - 1; ++j)
                av1_get_syntax_rate_from_cdf(md_rate_estimation_array->comp_bwd_ref_fac_bits[i][j], fc->comp_bwdref_cdf[i][j], NULL);
        }

        for (i = 0; i < INTRA_INTER_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->intra_inter_fac_bits[i], fc->intra_inter_cdf[i], NULL);
        for (i = 0; i < NEWMV_MODE_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->new_mv_mode_fac_bits[i], fc->newmv_cdf[i], NULL);
        for (i = 0; i < GLOBALMV_MODE_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->zero_mv_mode_fac_bits[i], fc->zeromv_cdf[i], NULL);
        for (i = 0; i < REFMV_MODE_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->ref_mv_mode_fac_bits[i], fc->refmv_cdf[i], NULL);
        for (i = 0; i < DRL_MODE_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->drl_mode_fac_bits[i], fc->drl_cdf[i], NULL);
        for (i = 0; i < INTER_MODE_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->inter_compound_mode_fac_bits[i], fc->inter_compound_mode_cdf[i], NULL);
        for (i = 0; i < BlockSizeS_ALL; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->compound_type_fac_bits[i], fc->compound_type_cdf[i], NULL);
        for (i = 0; i < BlockSizeS_ALL; ++i) {
            if (get_interinter_wedge_bits((BlockSize)i))
                av1_get_syntax_rate_from_cdf(md_rate_estimation_array->wedge_idx_fac_bits[i], fc->wedge_idx_cdf[i], NULL);
        }
        for (i = 0; i < BlockSize_GROUPS; ++i) {
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->inter_intra_fac_bits[i], fc->interintra_cdf[i], NULL);
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->inter_intra_mode_fac_bits[i], fc->interintra_mode_cdf[i], NULL);
        }
        for (i = 0; i < BlockSizeS_ALL; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->wedge_inter_intra_fac_bits[i], fc->wedge_interintra_cdf[i], NULL);
        for (i = BLOCK_8X8; i < BlockSizeS_ALL; i++)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->motion_mode_fac_bits[i], fc->motion_mode_cdf[i], NULL);
        for (i = BLOCK_8X8; i < BlockSizeS_ALL; i++)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->motion_mode_fac_bits1[i], fc->obmc_cdf[i], NULL);
        for (i = 0; i < COMP_INDEX_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->comp_idx_fac_bits[i], fc->compound_index_cdf[i], NULL);
        for (i = 0; i < COMP_GROUP_IDX_CONTEXTS; ++i)
            av1_get_syntax_rate_from_cdf(md_rate_estimation_array->comp_group_idx_fac_bits[i], fc->comp_group_idx_cdf[i], NULL);
    }
}

static const uint8_t log_in_base_2[] = {
    0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10
};

static INLINE int32_t mv_class_base(MvClassType c) {
    return c ? CLASS0_SIZE << (c + 2) : 0;
}

MvClassType av1_get_mv_class(int32_t z, int32_t *offset) {
    const MvClassType c = (z >= CLASS0_SIZE * 4096)
        ? MV_CLASS_10
        : (MvClassType)log_in_base_2[z >> 3];
    if (offset) *offset = z - mv_class_base(c);
    return c;
}

//void eb_av1_build_nmv_cost_table(int32_t *mvjoint, int32_t *mvcost[2],
//    const NmvContext *ctx,
//    MvSubpelPrecision precision)

void eb_av1_build_nmv_cost_table(int32_t *mvjoint, int32_t *mvcost[2],
    const NmvContext *ctx,
    MvSubpelPrecision precision);

/**************************************************************************
* av1_estimate_mv_rate()
* Estimate the rate of motion vectors
* based on the frame CDF
***************************************************************************/
void av1_estimate_mv_rate(
    PictureControlSet     *picture_control_set_ptr,
    MdRateEstimationContext  *md_rate_estimation_array,
#if RATE_ESTIMATION_UPDATE
    FRAME_CONTEXT            *fc)
#else
    NmvContext                *nmv_ctx)
#endif
{
    int32_t *nmvcost[2];
    int32_t *nmvcost_hp[2];
    FrameHeader *frm_hdr = &picture_control_set_ptr->parent_pcs_ptr->frm_hdr;

    nmvcost[0] = &md_rate_estimation_array->nmv_costs[0][MV_MAX];
    nmvcost[1] = &md_rate_estimation_array->nmv_costs[1][MV_MAX];
    nmvcost_hp[0] = &md_rate_estimation_array->nmv_costs_hp[0][MV_MAX];
    nmvcost_hp[1] = &md_rate_estimation_array->nmv_costs_hp[1][MV_MAX];

    eb_av1_build_nmv_cost_table(
        md_rate_estimation_array->nmv_vec_cost,//out
        frm_hdr->allow_high_precision_mv ? nmvcost_hp : nmvcost, //out
#if RATE_ESTIMATION_UPDATE
        &fc->nmvc,
#else
        nmv_ctx,
#endif
        frm_hdr->allow_high_precision_mv);
#if EIGHT_PEL_FIX
    md_rate_estimation_array->nmvcoststack[0] = frm_hdr->allow_high_precision_mv ?
        &md_rate_estimation_array->nmv_costs_hp[0][MV_MAX] : &md_rate_estimation_array->nmv_costs[0][MV_MAX];
    md_rate_estimation_array->nmvcoststack[1] = frm_hdr->allow_high_precision_mv ?
        &md_rate_estimation_array->nmv_costs_hp[1][MV_MAX] : &md_rate_estimation_array->nmv_costs[1][MV_MAX];
#else
    md_rate_estimation_array->nmvcoststack[0] = &md_rate_estimation_array->nmv_costs[0][MV_MAX];
    md_rate_estimation_array->nmvcoststack[1] = &md_rate_estimation_array->nmv_costs[1][MV_MAX];
#endif
    if (frm_hdr->allow_intrabc) {
        int32_t *dvcost[2] = { &md_rate_estimation_array->dv_cost[0][MV_MAX], &md_rate_estimation_array->dv_cost[1][MV_MAX] };
#if RATE_ESTIMATION_UPDATE
        eb_av1_build_nmv_cost_table(md_rate_estimation_array->dv_joint_cost, dvcost, &fc->ndvc, MV_SUBPEL_NONE);
#else
        eb_av1_build_nmv_cost_table(md_rate_estimation_array->dv_joint_cost, dvcost, &picture_control_set_ptr->coeff_est_entropy_coder_ptr->fc->ndvc, MV_SUBPEL_NONE);
#endif
    }
}
/**************************************************************************
* av1_estimate_coefficients_rate()
* Estimate the rate of the quantised coefficient
* based on the frame CDF
***************************************************************************/
void av1_estimate_coefficients_rate(
    MdRateEstimationContext  *md_rate_estimation_array,
    FRAME_CONTEXT              *fc)
{
    int32_t num_planes = 3; // NM - Hardcoded to 3
    const int32_t nplanes = AOMMIN(num_planes, PLANE_TYPES);
    int32_t eob_multi_size = 0;
    int32_t plane = 0;
    int32_t ctx = 0;
    int32_t tx_size = 0;

    for (eob_multi_size = 0; eob_multi_size < 7; ++eob_multi_size) {
        for (plane = 0; plane < nplanes; ++plane) {
            LvMapEobCost *pcost = &md_rate_estimation_array->eob_frac_bits[eob_multi_size][plane];
            for (ctx = 0; ctx < 2; ++ctx) {
                AomCdfProb *pcdf;
                switch (eob_multi_size) {
                case 0: pcdf = fc->eob_flag_cdf16[plane][ctx]; break;
                case 1: pcdf = fc->eob_flag_cdf32[plane][ctx]; break;
                case 2: pcdf = fc->eob_flag_cdf64[plane][ctx]; break;
                case 3: pcdf = fc->eob_flag_cdf128[plane][ctx]; break;
                case 4: pcdf = fc->eob_flag_cdf256[plane][ctx]; break;
                case 5: pcdf = fc->eob_flag_cdf512[plane][ctx]; break;
                case 6:
                default: pcdf = fc->eob_flag_cdf1024[plane][ctx]; break;
                }
                av1_get_syntax_rate_from_cdf(pcost->eob_cost[ctx], pcdf, NULL);
            }
        }
    }
    for (tx_size = 0; tx_size < TX_SIZES; ++tx_size) {
        for (plane = 0; plane < nplanes; ++plane) {
            LvMapCoeffCost *pcost = &md_rate_estimation_array->coeff_fac_bits[tx_size][plane];

            for (ctx = 0; ctx < TXB_SKIP_CONTEXTS; ++ctx)
                av1_get_syntax_rate_from_cdf(pcost->txb_skip_cost[ctx],
                    fc->txb_skip_cdf[tx_size][ctx], NULL);

            for (ctx = 0; ctx < SIG_COEF_CONTEXTS_EOB; ++ctx)
                av1_get_syntax_rate_from_cdf(pcost->base_eob_cost[ctx],
                    fc->coeff_base_eob_cdf[tx_size][plane][ctx],
                    NULL);
            for (ctx = 0; ctx < SIG_COEF_CONTEXTS; ++ctx)
                av1_get_syntax_rate_from_cdf(pcost->base_cost[ctx],
                    fc->coeff_base_cdf[tx_size][plane][ctx], NULL);
            for (int ctx = 0; ctx < SIG_COEF_CONTEXTS; ++ctx) {
                pcost->base_cost[ctx][4] = 0;
                pcost->base_cost[ctx][5] = pcost->base_cost[ctx][1] +
                    av1_cost_literal(1) -
                    pcost->base_cost[ctx][0];
                pcost->base_cost[ctx][6] =
                    pcost->base_cost[ctx][2] - pcost->base_cost[ctx][1];
                pcost->base_cost[ctx][7] =
                    pcost->base_cost[ctx][3] - pcost->base_cost[ctx][2];
            }
            for (ctx = 0; ctx < EOB_COEF_CONTEXTS; ++ctx)
                av1_get_syntax_rate_from_cdf(pcost->eob_extra_cost[ctx],
                    fc->eob_extra_cdf[tx_size][plane][ctx], NULL);

            for (ctx = 0; ctx < DC_SIGN_CONTEXTS; ++ctx)
                av1_get_syntax_rate_from_cdf(pcost->dc_sign_cost[ctx],
                    fc->dc_sign_cdf[plane][ctx], NULL);

            for (ctx = 0; ctx < LEVEL_CONTEXTS; ++ctx) {
                int32_t br_rate[BR_CDF_SIZE];
                int32_t prev_cost = 0;
                int32_t i, j;
                av1_get_syntax_rate_from_cdf(br_rate, fc->coeff_br_cdf[tx_size][plane][ctx], NULL);
                // printf("br_rate: ");
                // for(j = 0; j < BR_CDF_SIZE; j++)
                //  printf("%4d ", br_rate[j]);
                // printf("\n");
                for (i = 0; i < COEFF_BASE_RANGE; i += BR_CDF_SIZE - 1) {
                    for (j = 0; j < BR_CDF_SIZE - 1; j++)
                        pcost->lps_cost[ctx][i + j] = prev_cost + br_rate[j];
                    prev_cost += br_rate[j];
                }
                pcost->lps_cost[ctx][i] = prev_cost;
                // printf("lps_cost: %d %d %2d : ", tx_size, plane, ctx);
                // for (i = 0; i <= COEFF_BASE_RANGE; i++)
                //  printf("%5d ", pcost->lps_cost[ctx][i]);
                // printf("\n");
            }
            for (int ctx = 0; ctx < LEVEL_CONTEXTS; ++ctx) {
                pcost->lps_cost[ctx][0 + COEFF_BASE_RANGE + 1] =
                    pcost->lps_cost[ctx][0];
                for (int i = 1; i <= COEFF_BASE_RANGE; ++i) {
                    pcost->lps_cost[ctx][i + COEFF_BASE_RANGE + 1] =
                        pcost->lps_cost[ctx][i] - pcost->lps_cost[ctx][i - 1];
                }
            }
        }
    }
}
#if RATE_ESTIMATION_UPDATE
extern int av1_allow_intrabc(const Av1Common *const cm);
int av1_filter_intra_allowed(
    uint8_t   enable_filter_intra,
    BlockSize bsize,
#if PAL_SUP
    uint8_t palette_size,
#endif
    uint32_t  mode);

INLINE int32_t is_chroma_reference(int32_t mi_row, int32_t mi_col, BlockSize bsize,
    int32_t subsampling_x, int32_t subsampling_y);

int32_t is_inter_block(const BlockModeInfo *mbmi);

int av1_allow_palette(int allow_screen_content_tools,
    BlockSize sb_type);

int av1_get_palette_bsize_ctx(BlockSize bsize);

int av1_get_palette_mode_ctx(const MacroBlockD *xd);

extern INLINE int has_uni_comp_refs(const MbModeInfo *mbmi);

// The mode info data structure has a one element border above and to the
// left of the entries corresponding to real macroblocks.
// The prediction flags in these dummy entries are initialized to 0.
// 0 - inter/inter, inter/--, --/inter, --/--
// 1 - intra/inter, inter/intra
// 2 - intra/--, --/intra
// 3 - intra/intra
int av1_get_intra_inter_context(const MacroBlockD *xd) {
    const MbModeInfo *const above_mbmi = xd->above_mbmi;
    const MbModeInfo *const left_mbmi = xd->left_mbmi;
    const int has_above = xd->up_available;
    const int has_left = xd->left_available;

    if (has_above && has_left) {  // both edges available
        const int above_intra = !is_inter_block(&above_mbmi->block_mi);
        const int left_intra = !is_inter_block(&left_mbmi->block_mi);
        return left_intra && above_intra ? 3 : left_intra || above_intra;
    }
    else if (has_above || has_left) {  // one edge available
        return 2 * !is_inter_block(has_above ? &above_mbmi->block_mi : &left_mbmi->block_mi);
    }
    else {
        return 0;
    }
}
extern  int8_t av1_ref_frame_type(const MvReferenceFrame *const rf);
uint16_t compound_mode_ctx_map_[3][COMP_NEWMV_CTXS] = {
   { 0, 1, 1, 1, 1 },
   { 1, 2, 3, 4, 4 },
   { 4, 4, 5, 6, 7 },
};
static int16_t Av1ModeContextAnalyzer(
    const int16_t *const mode_context, const MvReferenceFrame *const rf) {
    const int8_t ref_frame = av1_ref_frame_type(rf);

    if (rf[1] <= INTRA_FRAME) return mode_context[ref_frame];

    const int16_t newmv_ctx = mode_context[ref_frame] & NEWMV_CTX_MASK;
    const int16_t refmv_ctx =
        (mode_context[ref_frame] >> REFMV_OFFSET) & REFMV_CTX_MASK;
    assert((refmv_ctx >> 1) < 3);
    const int16_t comp_ctx = compound_mode_ctx_map_[refmv_ctx >> 1][AOMMIN(
        newmv_ctx, COMP_NEWMV_CTXS - 1)];
    return comp_ctx;
}

#define INTER_FILTER_COMP_OFFSET (SWITCHABLE_FILTERS + 1)
#define INTER_FILTER_DIR_OFFSET ((SWITCHABLE_FILTERS + 1) * 2)
int av1_get_pred_context_switchable_interp(const MacroBlockD *xd, int dir) {

    const MbModeInfo *const mbmi = &xd->mi[0]->mbmi;
    const int ctx_offset =
        (mbmi->block_mi.ref_frame[1] > INTRA_FRAME) * INTER_FILTER_COMP_OFFSET;
    assert(dir == 0 || dir == 1);
    const MvReferenceFrame ref_frame = mbmi->block_mi.ref_frame[0];
    // Note:
    // The mode info data structure has a one element border above and to the
    // left of the entries corresponding to real macroblocks.
    // The prediction flags in these dummy entries are initialized to 0.
    int filter_type_ctx = ctx_offset + (dir & 0x01) * INTER_FILTER_DIR_OFFSET;
    int left_type = SWITCHABLE_FILTERS;
    int above_type = SWITCHABLE_FILTERS;

    if (xd->left_available)
        left_type = get_ref_filter_type(&xd->mi[-1]->mbmi.block_mi, xd, dir, ref_frame);

    if (xd->up_available)
        above_type =
        get_ref_filter_type(&xd->mi[-xd->mi_stride]->mbmi.block_mi, xd, dir, ref_frame);

    if (left_type == above_type) {
        filter_type_ctx += left_type;
    }
    else if (left_type == SWITCHABLE_FILTERS) {
        assert(above_type != SWITCHABLE_FILTERS);
        filter_type_ctx += above_type;
    }
    else if (above_type == SWITCHABLE_FILTERS) {
        assert(left_type != SWITCHABLE_FILTERS);
        filter_type_ctx += left_type;
    }
    else {
        filter_type_ctx += SWITCHABLE_FILTERS;
    }

    return filter_type_ctx;
}
// Return the number of elements in the partition CDF when
// partitioning the (square) block with luma block size of bsize.
static INLINE int32_t partition_cdf_length(BlockSize bsize) {
    if (bsize <= BLOCK_8X8)
        return PARTITION_TYPES;
    else if (bsize == BLOCK_128X128)
        return EXT_PARTITION_TYPES - 2;
    else
        return EXT_PARTITION_TYPES;
}

extern void av1_set_ref_frame(
    MvReferenceFrame *rf,
    int8_t ref_frame_type);

typedef uint32_t InterpFilters;
static INLINE InterpFilter av1_extract_interp_filter(InterpFilters filters,
    int32_t x_filter);

static AOM_INLINE void update_filter_type_cdf(
    MacroBlockD      *xd,
    const MbModeInfo *const mbmi) {
    int dir;
    for (dir = 0; dir < 2; ++dir) {
        const int ctx = av1_get_pred_context_switchable_interp(xd, dir);

        InterpFilter filter = av1_extract_interp_filter(mbmi->block_mi.interp_filters, dir);
        update_cdf(xd->tile_ctx->switchable_interp_cdf[ctx], filter,
            SWITCHABLE_FILTERS);
    }
}
MvClassType av1_get_mv_class(int32_t z, int32_t *offset);
static void update_mv_component_stats(
    int comp, 
    NmvComponent *mvcomp,
    MvSubpelPrecision precision) {
    assert(comp != 0);
    int offset;
    const int sign = comp < 0;
    const int mag = sign ? -comp : comp;
    const int mv_class = av1_get_mv_class(mag - 1, &offset);
    const int d = offset >> 3;         // int mv data
    const int fr = (offset >> 1) & 3;  // fractional mv data
    const int hp = offset & 1;         // high precision mv data

    // Sign
    update_cdf(mvcomp->sign_cdf, sign, 2);

    // Class
    update_cdf(mvcomp->classes_cdf, mv_class, MV_CLASSES);

    // Integer bits
    if (mv_class == MV_CLASS_0) {
        update_cdf(mvcomp->class0_cdf, d, CLASS0_SIZE);
    }
    else {
        const int n = mv_class + CLASS0_BITS - 1;  // number of bits
        for (int i = 0; i < n; ++i)
            update_cdf(mvcomp->bits_cdf[i], (d >> i) & 1, 2);
    }
    // Fractional bits
    if (precision > MV_SUBPEL_NONE) {
        AomCdfProb *fp_cdf =
            mv_class == MV_CLASS_0 ? mvcomp->class0_fp_cdf[d] : mvcomp->fp_cdf;
        update_cdf(fp_cdf, fr, MV_FP_SIZE);
    }
    // High precision bit
    if (precision > MV_SUBPEL_LOW_PRECISION) {
        AomCdfProb *hp_cdf =
            mv_class == MV_CLASS_0 ? mvcomp->class0_hp_cdf : mvcomp->hp_cdf;
        update_cdf(hp_cdf, hp, 2);
    }
}

MvJointType av1_get_mv_joint(const MV *mv);

void av1_update_mv_stats(const MV *mv, const MV *ref, NmvContext *mvctx,
    MvSubpelPrecision precision) {
    const MV diff = { mv->row - ref->row, mv->col - ref->col };
    const MvJointType j = av1_get_mv_joint(&diff);

    update_cdf(mvctx->joints_cdf, j, MV_JOINTS);

    if (mv_joint_vertical(j))
        update_mv_component_stats(diff.row, &mvctx->comps[0], precision);

    if (mv_joint_horizontal(j))
        update_mv_component_stats(diff.col, &mvctx->comps[1], precision);
}

static AOM_INLINE void update_inter_mode_stats(FRAME_CONTEXT *fc,
    PredictionMode mode,
    int16_t mode_context) {

    int16_t mode_ctx = mode_context & NEWMV_CTX_MASK;
    if (mode == NEWMV) {
#if CONFIG_ENTROPY_STATS
        ++counts->newmv_mode[mode_ctx][0];
#endif
        update_cdf(fc->newmv_cdf[mode_ctx], 0, 2);
        return;
    }

#if CONFIG_ENTROPY_STATS
    ++counts->newmv_mode[mode_ctx][1];
#endif
    update_cdf(fc->newmv_cdf[mode_ctx], 1, 2);

    mode_ctx = (mode_context >> GLOBALMV_OFFSET) & GLOBALMV_CTX_MASK;
    if (mode == GLOBALMV) {
#if CONFIG_ENTROPY_STATS
        ++counts->zeromv_mode[mode_ctx][0];
#endif
        update_cdf(fc->zeromv_cdf[mode_ctx], 0, 2);
        return;
    }

#if CONFIG_ENTROPY_STATS
    ++counts->zeromv_mode[mode_ctx][1];
#endif
    update_cdf(fc->zeromv_cdf[mode_ctx], 1, 2);

    mode_ctx = (mode_context >> REFMV_OFFSET) & REFMV_CTX_MASK;
#if CONFIG_ENTROPY_STATS
    ++counts->refmv_mode[mode_ctx][mode != NEARESTMV];
#endif
    update_cdf(fc->refmv_cdf[mode_ctx], mode != NEARESTMV, 2);
}

static AOM_INLINE void update_palette_cdf(MacroBlockD *xd,
    const MbModeInfo *const mbmi,
    CodingUnit            *cu_ptr) {
    FRAME_CONTEXT *fc = xd->tile_ctx;
    const BlockGeom          *blk_geom = get_blk_geom_mds(cu_ptr->mds_idx);
    const BlockSize bsize = blk_geom->bsize;
    const PaletteModeInfo *const pmi = &cu_ptr->palette_info.pmi;
    const int palette_bsize_ctx = av1_get_palette_bsize_ctx(bsize);
    //  (void)counts;

    if (mbmi->block_mi.mode == DC_PRED) {
        const int n = pmi->palette_size[0];
        const int palette_mode_ctx = av1_get_palette_mode_ctx(xd);

#if CONFIG_ENTROPY_STATS
        ++counts->palette_y_mode[palette_bsize_ctx][palette_mode_ctx][n > 0];
#endif
        update_cdf(fc->palette_y_mode_cdf[palette_bsize_ctx][palette_mode_ctx],
            n > 0, 2);
        if (n > 0) {
#if CONFIG_ENTROPY_STATS
            ++counts->palette_y_size[palette_bsize_ctx][n - PALETTE_MIN_SIZE];
#endif
            update_cdf(fc->palette_y_size_cdf[palette_bsize_ctx],
                n - PALETTE_MIN_SIZE, PALETTE_SIZES);
        }
    }
    uint32_t intra_chroma_mode = mbmi->block_mi.uv_mode;
    const int num_planes = 3;
    const int uv_dc_pred =
        num_planes > 1 && intra_chroma_mode == UV_DC_PRED
        //&& is_chroma_reference(mi_row, mi_col, bsize, 1, 1); AMIR to update mi_row and mi_col
        ;
    if (uv_dc_pred) {
        //if (mbmi->block_mi.uv_mode == UV_DC_PRED) {
        const int n = pmi->palette_size[1];
        const int palette_uv_mode_ctx = (pmi->palette_size[0] > 0);

#if CONFIG_ENTROPY_STATS
        ++counts->palette_uv_mode[palette_uv_mode_ctx][n > 0];
#endif
        update_cdf(fc->palette_uv_mode_cdf[palette_uv_mode_ctx], n > 0, 2);

        if (n > 0) {
#if CONFIG_ENTROPY_STATS
            ++counts->palette_uv_size[palette_bsize_ctx][n - PALETTE_MIN_SIZE];
#endif
            update_cdf(fc->palette_uv_size_cdf[palette_bsize_ctx],
                n - PALETTE_MIN_SIZE, PALETTE_SIZES);
        }
    }
}

static AOM_INLINE void sum_intra_stats(
    PictureControlSet  *picture_control_set_ptr,
    CodingUnit         *cu_ptr,
    const MbModeInfo    *above_mi,
    const MbModeInfo    *left_mi,
    const int           intraonly, 
    const int           mi_row,
    const int           mi_col) {

    const AV1_COMMON *const cm = picture_control_set_ptr->parent_pcs_ptr->av1_cm;
    MacroBlockD *xd = cu_ptr->av1xd;
    const MbModeInfo *const mbmi = &xd->mi[0]->mbmi;
    FRAME_CONTEXT *fc = xd->tile_ctx;
    const PredictionMode y_mode = mbmi->block_mi.mode;
    const BlockGeom          *blk_geom = get_blk_geom_mds(cu_ptr->mds_idx);
    const BlockSize bsize = mbmi->block_mi.sb_type;
    if (intraonly) {
#if CONFIG_ENTROPY_STATS
        const PREDICTION_MODE above = av1_above_block_mode(above_mi);
        const PREDICTION_MODE left = av1_left_block_mode(left_mi);
        const int above_ctx = intra_mode_context[above];
        const int left_ctx = intra_mode_context[left];
        ++counts->kf_y_mode[above_ctx][left_ctx][y_mode];
#endif  // CONFIG_ENTROPY_STATS
        update_cdf(get_y_mode_cdf(fc, above_mi, left_mi), y_mode, INTRA_MODES);
    }
    else {
#if CONFIG_ENTROPY_STATS
        ++counts->y_mode[size_group_lookup[bsize]][y_mode];
#endif  // CONFIG_ENTROPY_STATS
        update_cdf(fc->y_mode_cdf[size_group_lookup[bsize]], y_mode, INTRA_MODES);
    }
    if (xd->use_intrabc == 0 && av1_filter_intra_allowed(
        picture_control_set_ptr->parent_pcs_ptr->sequence_control_set_ptr->seq_header.enable_filter_intra,
        bsize, cu_ptr->palette_info.pmi.palette_size[0], y_mode)) {
        const int use_filter_intra_mode =
            cu_ptr->filter_intra_mode != FILTER_INTRA_MODES;
#if CONFIG_ENTROPY_STATS
        ++counts->filter_intra[mbmi->sb_type][use_filter_intra_mode];
        if (use_filter_intra_mode) {
            ++counts
                ->filter_intra_mode[mbmi->filter_intra_mode_info.filter_intra_mode];
        }
#endif  // CONFIG_ENTROPY_STATS
        update_cdf(fc->filter_intra_cdfs[bsize], use_filter_intra_mode, 2);
        if (use_filter_intra_mode) {
            update_cdf(fc->filter_intra_mode_cdf,
                cu_ptr->filter_intra_mode,
                FILTER_INTRA_MODES);
        }
    }
    if (av1_is_directional_mode(y_mode) && av1_use_angle_delta(bsize)) {
#if CONFIG_ENTROPY_STATS
        ++counts->angle_delta[mbmi->mode - V_PRED]
            [mbmi->angle_delta[PLANE_TYPE_Y] + MAX_ANGLE_DELTA];
#endif
        update_cdf(fc->angle_delta_cdf[y_mode - V_PRED],
            mbmi->block_mi.angle_delta[PLANE_TYPE_Y] + MAX_ANGLE_DELTA,
            2 * MAX_ANGLE_DELTA + 1);
    }
    uint8_t   subSamplingX = 1; // NM - subsampling_x is harcoded to 1 for 420 chroma sampling.
    uint8_t   subSamplingY = 1; // NM - subsampling_y is harcoded to 1 for 420 chroma sampling.
    if (!is_chroma_reference(mi_row, mi_col, bsize,
        subSamplingX, subSamplingY))
        return;

    const UvPredictionMode uv_mode = mbmi->block_mi.uv_mode;
    const int cfl_allowed = blk_geom->bwidth <= 32 && blk_geom->bheight <= 32;
#if CONFIG_ENTROPY_STATS
    ++counts->uv_mode[cfl_allowed][y_mode][uv_mode];
#endif  // CONFIG_ENTROPY_STATS
    update_cdf(fc->uv_mode_cdf[cfl_allowed][y_mode], uv_mode,
        UV_INTRA_MODES - !cfl_allowed);
    if (uv_mode == UV_CFL_PRED) {
        const int8_t joint_sign = mbmi->block_mi.cfl_alpha_signs;
        const uint8_t idx = mbmi->block_mi.cfl_alpha_idx;

#if CONFIG_ENTROPY_STATS
        ++counts->cfl_sign[joint_sign];
#endif
        update_cdf(fc->cfl_sign_cdf, joint_sign, CFL_JOINT_SIGNS);
        if (CFL_SIGN_U(joint_sign) != CFL_SIGN_ZERO) {
            AomCdfProb *cdf_u = fc->cfl_alpha_cdf[CFL_CONTEXT_U(joint_sign)];

#if CONFIG_ENTROPY_STATS
            ++counts->cfl_alpha[CFL_CONTEXT_U(joint_sign)][CFL_IDX_U(idx)];
#endif
            update_cdf(cdf_u, CFL_IDX_U(idx), CFL_ALPHABET_SIZE);
        }
        if (CFL_SIGN_V(joint_sign) != CFL_SIGN_ZERO) {
            AomCdfProb *cdf_v = fc->cfl_alpha_cdf[CFL_CONTEXT_V(joint_sign)];

#if CONFIG_ENTROPY_STATS
            ++counts->cfl_alpha[CFL_CONTEXT_V(joint_sign)][CFL_IDX_V(idx)];
#endif
            update_cdf(cdf_v, CFL_IDX_V(idx), CFL_ALPHABET_SIZE);
        }
    }
    if (av1_is_directional_mode(get_uv_mode(uv_mode)) && 
        av1_use_angle_delta(bsize)) {
#if CONFIG_ENTROPY_STATS
        ++counts->angle_delta[uv_mode - UV_V_PRED]
            [mbmi->angle_delta[PLANE_TYPE_UV] + MAX_ANGLE_DELTA];
#endif
        update_cdf(fc->angle_delta_cdf[uv_mode - UV_V_PRED],
            mbmi->block_mi.angle_delta[PLANE_TYPE_UV] + MAX_ANGLE_DELTA,
            2 * MAX_ANGLE_DELTA + 1);
    }
    if (av1_allow_palette(picture_control_set_ptr->parent_pcs_ptr->frm_hdr.allow_screen_content_tools, bsize)) {
        update_palette_cdf(xd, mbmi, cu_ptr);
    }
}
void update_stats(
    PictureControlSet  *picture_control_set_ptr,
    CodingUnit            *cu_ptr,
    int                 mi_row,
    int                 mi_col) {
    const AV1_COMMON *const cm = picture_control_set_ptr->parent_pcs_ptr->av1_cm;
    MacroBlockD *xd = cu_ptr->av1xd;
    const MbModeInfo *const mbmi = &xd->mi[0]->mbmi;

    const BlockGeom          *blk_geom = get_blk_geom_mds(cu_ptr->mds_idx);
    BlockSize bsize = blk_geom->bsize;
    FRAME_CONTEXT *fc = xd->tile_ctx;
    const int seg_ref_active = picture_control_set_ptr->parent_pcs_ptr->frm_hdr.segmentation_params.segmentation_enabled &&
        picture_control_set_ptr->parent_pcs_ptr->frm_hdr.segmentation_params.seg_id_pre_skip;
    // segfeature_active(&cm->seg, mbmi->block_mi.segment_id, SEG_LVL_REF_FRAME); // AMIR to check

    if (picture_control_set_ptr->parent_pcs_ptr->skip_mode_flag && !seg_ref_active &&
        is_comp_ref_allowed(bsize)) {
        const int skip_mode_ctx = av1_get_skip_mode_context(xd);
#if CONFIG_ENTROPY_STATS
        td->counts->skip_mode[skip_mode_ctx][mbmi->skip_mode]++;
#endif
        update_cdf(fc->skip_mode_cdfs[skip_mode_ctx], mbmi->block_mi.skip_mode, 2);
    }

    if (!mbmi->block_mi.skip_mode && !seg_ref_active) {
        const int skip_ctx = av1_get_skip_context(xd);
#if CONFIG_ENTROPY_STATS
        td->counts->skip[skip_ctx][mbmi->skip]++;
#endif
        update_cdf(fc->skip_cdfs[skip_ctx], mbmi->block_mi.skip, 2);
    }

#if CONFIG_ENTROPY_STATS
    // delta quant applies to both intra and inter
    const int super_block_upper_left =
        ((mi_row & (cm->seq_params.mib_size - 1)) == 0) &&
        ((mi_col & (cm->seq_params.mib_size - 1)) == 0);
    const DeltaQInfo *const delta_q_info = &cm->delta_q_info;
    if (delta_q_info->delta_q_present_flag &&
        (bsize != cm->seq_params.sb_size || !mbmi->skip) &&
        super_block_upper_left) {
        const int dq =
            (mbmi->current_qindex - xd->current_qindex) / delta_q_info->delta_q_res;
        const int absdq = abs(dq);
        for (int i = 0; i < AOMMIN(absdq, DELTA_Q_SMALL); ++i) {
            td->counts->delta_q[i][1]++;
        }
        if (absdq < DELTA_Q_SMALL) td->counts->delta_q[absdq][0]++;
        if (delta_q_info->delta_lf_present_flag) {
            if (delta_q_info->delta_lf_multi) {
                const int frame_lf_count =
                    av1_num_planes(cm) > 1 ? FRAME_LF_COUNT : FRAME_LF_COUNT - 2;
                for (int lf_id = 0; lf_id < frame_lf_count; ++lf_id) {
                    const int delta_lf = (mbmi->delta_lf[lf_id] - xd->delta_lf[lf_id]) /
                        delta_q_info->delta_lf_res;
                    const int abs_delta_lf = abs(delta_lf);
                    for (int i = 0; i < AOMMIN(abs_delta_lf, DELTA_LF_SMALL); ++i) {
                        td->counts->delta_lf_multi[lf_id][i][1]++;
                    }
                    if (abs_delta_lf < DELTA_LF_SMALL)
                        td->counts->delta_lf_multi[lf_id][abs_delta_lf][0]++;
                }
            }
            else {
                const int delta_lf =
                    (mbmi->delta_lf_from_base - xd->delta_lf_from_base) /
                    delta_q_info->delta_lf_res;
                const int abs_delta_lf = abs(delta_lf);
                for (int i = 0; i < AOMMIN(abs_delta_lf, DELTA_LF_SMALL); ++i) {
                    td->counts->delta_lf[i][1]++;
                }
                if (abs_delta_lf < DELTA_LF_SMALL)
                    td->counts->delta_lf[abs_delta_lf][0]++;
            }
        }
    }
#endif
    if (!is_inter_block(&mbmi->block_mi)) {
        sum_intra_stats(
            picture_control_set_ptr, 
            cu_ptr, 
            xd->above_mbmi,
            xd->left_mbmi,
            frame_is_intra_only(picture_control_set_ptr->parent_pcs_ptr),
            mi_row,
            mi_col);
    }
    if (av1_allow_intrabc(cm)) {
        update_cdf(fc->intrabc_cdf, is_intrabc_block(&mbmi->block_mi), 2);
#if CONFIG_ENTROPY_STATS
        ++td->counts->intrabc[is_intrabc_block(mbmi)];
#endif  // CONFIG_ENTROPY_STATS
    }

    if (frame_is_intra_only(picture_control_set_ptr->parent_pcs_ptr) || mbmi->block_mi.skip_mode) return;
    const int inter_block = is_inter_block(&mbmi->block_mi);
    if (!seg_ref_active) {
#if CONFIG_ENTROPY_STATS
        counts->intra_inter[av1_get_intra_inter_context(xd)][inter_block]++;
#endif
        update_cdf(fc->intra_inter_cdf[av1_get_intra_inter_context(xd)],
            inter_block, 2);
        // If the segment reference feature is enabled we have only a single
        // reference frame allowed for the segment so exclude it from
        // the reference frame counts used to work out probabilities.
        if (inter_block) {
            const MvReferenceFrame ref0 = mbmi->block_mi.ref_frame[0];
            const MvReferenceFrame ref1 = mbmi->block_mi.ref_frame[1];
            if (picture_control_set_ptr->parent_pcs_ptr->frm_hdr.reference_mode == REFERENCE_MODE_SELECT) {
                if (is_comp_ref_allowed(bsize)) {
#if CONFIG_ENTROPY_STATS
                    counts->comp_inter[av1_get_reference_mode_context(xd)]
                        [has_second_ref(mbmi)]++;
#endif  // CONFIG_ENTROPY_STATS
                    update_cdf(av1_get_reference_mode_cdf(xd), has_second_ref(mbmi), 2);
                }
            }

            if (has_second_ref(mbmi)) {
                const CompReferenceType comp_ref_type = has_uni_comp_refs(mbmi)
                    ? UNIDIR_COMP_REFERENCE
                    : BIDIR_COMP_REFERENCE;
                update_cdf(av1_get_comp_reference_type_cdf(xd), comp_ref_type,
                    COMP_REFERENCE_TYPES);
#if CONFIG_ENTROPY_STATS
                counts->comp_ref_type[av1_get_comp_reference_type_context(xd)]
                    [comp_ref_type]++;
#endif  // CONFIG_ENTROPY_STATS

                if (comp_ref_type == UNIDIR_COMP_REFERENCE) {
                    const int bit = (ref0 == BWDREF_FRAME);
                    update_cdf(av1_get_pred_cdf_uni_comp_ref_p(xd), bit, 2);
#if CONFIG_ENTROPY_STATS
                    counts
                        ->uni_comp_ref[av1_get_pred_context_uni_comp_ref_p(xd)][0][bit]++;
#endif  // CONFIG_ENTROPY_STATS
                    if (!bit) {
                        const int bit1 = (ref1 == LAST3_FRAME || ref1 == GOLDEN_FRAME);
                        update_cdf(av1_get_pred_cdf_uni_comp_ref_p1(xd), bit1, 2);
#if CONFIG_ENTROPY_STATS
                        counts->uni_comp_ref[av1_get_pred_context_uni_comp_ref_p1(xd)][1]
                            [bit1]++;
#endif  // CONFIG_ENTROPY_STATS
                        if (bit1) {
                            update_cdf(av1_get_pred_cdf_uni_comp_ref_p2(xd),
                                ref1 == GOLDEN_FRAME, 2);
#if CONFIG_ENTROPY_STATS
                            counts->uni_comp_ref[av1_get_pred_context_uni_comp_ref_p2(xd)][2]
                                [ref1 == GOLDEN_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                        }
                    }
                }
                else {
                    const int bit = (ref0 == GOLDEN_FRAME || ref0 == LAST3_FRAME);
                    update_cdf(av1_get_pred_cdf_comp_ref_p(xd), bit, 2);
#if CONFIG_ENTROPY_STATS
                    counts->comp_ref[av1_get_pred_context_comp_ref_p(xd)][0][bit]++;
#endif  // CONFIG_ENTROPY_STATS
                    if (!bit) {
                        update_cdf(av1_get_pred_cdf_comp_ref_p1(xd), ref0 == LAST2_FRAME,
                            2);
#if CONFIG_ENTROPY_STATS
                        counts->comp_ref[av1_get_pred_context_comp_ref_p1(xd)][1]
                            [ref0 == LAST2_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                    }
                    else {
                        update_cdf(av1_get_pred_cdf_comp_ref_p2(xd), ref0 == GOLDEN_FRAME,
                            2);
#if CONFIG_ENTROPY_STATS
                        counts->comp_ref[av1_get_pred_context_comp_ref_p2(xd)][2]
                            [ref0 == GOLDEN_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                    }
                    update_cdf(av1_get_pred_cdf_comp_bwdref_p(xd), ref1 == ALTREF_FRAME,
                        2);
#if CONFIG_ENTROPY_STATS
                    counts->comp_bwdref[av1_get_pred_context_comp_bwdref_p(xd)][0]
                        [ref1 == ALTREF_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                    if (ref1 != ALTREF_FRAME) {
                        update_cdf(av1_get_pred_cdf_comp_bwdref_p1(xd),
                            ref1 == ALTREF2_FRAME, 2);
#if CONFIG_ENTROPY_STATS
                        counts->comp_bwdref[av1_get_pred_context_comp_bwdref_p1(xd)][1]
                            [ref1 == ALTREF2_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                    }
                }
            }
            else {
                const int bit = (ref0 >= BWDREF_FRAME);
                update_cdf(av1_get_pred_cdf_single_ref_p1(xd), bit, 2);
#if CONFIG_ENTROPY_STATS
                counts->single_ref[av1_get_pred_context_single_ref_p1(xd)][0][bit]++;
#endif  // CONFIG_ENTROPY_STATS
                if (bit) {
                    assert(ref0 <= ALTREF_FRAME);
                    update_cdf(av1_get_pred_cdf_single_ref_p2(xd), ref0 == ALTREF_FRAME,
                        2);
#if CONFIG_ENTROPY_STATS
                    counts->single_ref[av1_get_pred_context_single_ref_p2(xd)][1]
                        [ref0 == ALTREF_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                    if (ref0 != ALTREF_FRAME) {
                        update_cdf(av1_get_pred_cdf_single_ref_p6(xd),
                            ref0 == ALTREF2_FRAME, 2);
#if CONFIG_ENTROPY_STATS
                        counts->single_ref[av1_get_pred_context_single_ref_p6(xd)][5]
                            [ref0 == ALTREF2_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                    }
                }
                else {
                    const int bit1 = !(ref0 == LAST2_FRAME || ref0 == LAST_FRAME);
                    update_cdf(av1_get_pred_cdf_single_ref_p3(xd), bit1, 2);
#if CONFIG_ENTROPY_STATS
                    counts->single_ref[av1_get_pred_context_single_ref_p3(xd)][2][bit1]++;
#endif  // CONFIG_ENTROPY_STATS
                    if (!bit1) {
                        update_cdf(av1_get_pred_cdf_single_ref_p4(xd), ref0 != LAST_FRAME,
                            2);
#if CONFIG_ENTROPY_STATS
                        counts->single_ref[av1_get_pred_context_single_ref_p4(xd)][3]
                            [ref0 != LAST_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                    }
                    else {
                        update_cdf(av1_get_pred_cdf_single_ref_p5(xd), ref0 != LAST3_FRAME,
                            2);
#if CONFIG_ENTROPY_STATS
                        counts->single_ref[av1_get_pred_context_single_ref_p5(xd)][4]
                            [ref0 != LAST3_FRAME]++;
#endif  // CONFIG_ENTROPY_STATS
                    }
                }
            }

            if (picture_control_set_ptr->parent_pcs_ptr->sequence_control_set_ptr->seq_header.enable_interintra_compound &&
                is_interintra_allowed(mbmi)) {
                const int bsize_group = size_group_lookup[bsize];
                if (mbmi->block_mi.ref_frame[1] == INTRA_FRAME) {
#if CONFIG_ENTROPY_STATS
                    counts->interintra[bsize_group][1]++;
#endif
                    update_cdf(fc->interintra_cdf[bsize_group], 1, 2);
#if CONFIG_ENTROPY_STATS
                    counts->interintra_mode[bsize_group][mbmi->interintra_mode]++;
#endif
                    update_cdf(fc->interintra_mode_cdf[bsize_group],
                        cu_ptr->interintra_mode, INTERINTRA_MODES);
                    if (is_interintra_wedge_used(bsize)) {
#if CONFIG_ENTROPY_STATS
                        counts->wedge_interintra[bsize][mbmi->use_wedge_interintra]++;
#endif
                        update_cdf(fc->wedge_interintra_cdf[bsize],
                            cu_ptr->use_wedge_interintra, 2);
                        if (cu_ptr->use_wedge_interintra) {
#if CONFIG_ENTROPY_STATS
                            counts->wedge_idx[bsize][mbmi->interintra_wedge_index]++;
#endif
                            update_cdf(fc->wedge_idx_cdf[bsize], cu_ptr->interintra_wedge_index,
                                16);
                        }
                    }
                }
                else {
#if CONFIG_ENTROPY_STATS
                    counts->interintra[bsize_group][0]++;
#endif
                    update_cdf(fc->interintra_cdf[bsize_group], 0, 2);
                }
            }

            const MotionMode motion_allowed =
                picture_control_set_ptr->parent_pcs_ptr->frm_hdr.is_motion_mode_switchable
                ? motion_mode_allowed(picture_control_set_ptr,
                    cu_ptr,
                    bsize,
                    mbmi->block_mi.ref_frame[0],
                    mbmi->block_mi.ref_frame[1],
                    mbmi->block_mi.mode)
                : SIMPLE_TRANSLATION;
            if (mbmi->block_mi.ref_frame[1] != INTRA_FRAME) {
                if (motion_allowed == WARPED_CAUSAL) {
#if CONFIG_ENTROPY_STATS
                    counts->motion_mode[bsize][mbmi->motion_mode]++;
#endif
                    update_cdf(fc->motion_mode_cdf[bsize], mbmi->block_mi.motion_mode,
                        MOTION_MODES);
                }
                else if (motion_allowed == OBMC_CAUSAL) {
#if CONFIG_ENTROPY_STATS
                    counts->obmc[bsize][mbmi->motion_mode == OBMC_CAUSAL]++;
#endif
                    update_cdf(fc->obmc_cdf[bsize], mbmi->block_mi.motion_mode == OBMC_CAUSAL, 2);
                }
            }

            if (has_second_ref(mbmi)) {
                assert(picture_control_set_ptr->parent_pcs_ptr->frm_hdr.reference_mode != SINGLE_REFERENCE &&
                    is_inter_compound_mode(mbmi->block_mi.mode) &&
                    mbmi->block_mi.motion_mode == SIMPLE_TRANSLATION);

                const int masked_compound_used = is_any_masked_compound_used(bsize) &&
                    picture_control_set_ptr->parent_pcs_ptr->sequence_control_set_ptr->seq_header.enable_masked_compound;
                if (masked_compound_used) {
                    const int comp_group_idx_ctx = get_comp_group_idx_context_enc(xd);
#if CONFIG_ENTROPY_STATS
                    ++counts->comp_group_idx[comp_group_idx_ctx][mbmi->comp_group_idx];
#endif
                    update_cdf(fc->comp_group_idx_cdf[comp_group_idx_ctx],
                        mbmi->comp_group_idx, 2);
                }

                if (mbmi->comp_group_idx == 0) {
                    const int comp_index_ctx = get_comp_index_context_enc(
                        picture_control_set_ptr->parent_pcs_ptr,
                        picture_control_set_ptr->parent_pcs_ptr->cur_order_hint,// cur_frame_index,
                        picture_control_set_ptr->parent_pcs_ptr->ref_order_hint[mbmi->block_mi.ref_frame[0] - 1],// bck_frame_index,
                        picture_control_set_ptr->parent_pcs_ptr->ref_order_hint[mbmi->block_mi.ref_frame[1] - 1],// fwd_frame_index,
                        cu_ptr->av1xd);
#if CONFIG_ENTROPY_STATS
                    ++counts->compound_index[comp_index_ctx][mbmi->compound_idx];
#endif
                    update_cdf(fc->compound_index_cdf[comp_index_ctx], mbmi->block_mi.compound_idx,
                        2);
                }
                else {
                    assert(masked_compound_used);
                    if (is_interinter_compound_used(COMPOUND_WEDGE, bsize)) {
#if CONFIG_ENTROPY_STATS
                        ++counts->compound_type[bsize][mbmi->interinter_comp.type -
                            COMPOUND_WEDGE];
#endif
                        update_cdf(fc->compound_type_cdf[bsize],
                            cu_ptr->interinter_comp.type - COMPOUND_WEDGE,
                            MASKED_COMPOUND_TYPES);
                    }
                }
            }
            if (cu_ptr->interinter_comp.type == COMPOUND_WEDGE) {
                if (is_interinter_compound_used(COMPOUND_WEDGE, bsize)) {
#if CONFIG_ENTROPY_STATS
                    counts->wedge_idx[bsize][mbmi->interinter_comp.wedge_index]++;
#endif
                    update_cdf(fc->wedge_idx_cdf[bsize],
                        cu_ptr->interinter_comp.wedge_index, 16);
                }
            }
        }
    }

    if (inter_block && picture_control_set_ptr->parent_pcs_ptr->av1_cm->interp_filter == SWITCHABLE &&
        mbmi->block_mi.motion_mode != WARPED_CAUSAL &&
        !is_nontrans_global_motion_EC(mbmi->block_mi.ref_frame[0], mbmi->block_mi.ref_frame[1], cu_ptr, bsize, picture_control_set_ptr->parent_pcs_ptr)) {
        update_filter_type_cdf(xd, mbmi);
    }
    if (inter_block &&
        !seg_ref_active/*segfeature_active(&cm->seg, mbmi->segment_id, SEG_LVL_SKIP)*/) { // AMIR to check
        const PredictionMode mode = mbmi->block_mi.mode;
        MvReferenceFrame rf[2];
        av1_set_ref_frame(rf, cu_ptr->prediction_unit_array[0].ref_frame_type);
        const int16_t mode_ctx =
            Av1ModeContextAnalyzer(cu_ptr->inter_mode_ctx, rf);
        if (has_second_ref(mbmi)) {
#if CONFIG_ENTROPY_STATS
            ++counts->inter_compound_mode[mode_ctx][INTER_COMPOUND_OFFSET(mode)];
#endif
            update_cdf(fc->inter_compound_mode_cdf[mode_ctx],
                INTER_COMPOUND_OFFSET(mode), INTER_COMPOUND_MODES);
        }
        else {
            update_inter_mode_stats(fc, mode, mode_ctx);
        }

        const int new_mv = mbmi->block_mi.mode == NEWMV || mbmi->block_mi.mode == NEW_NEWMV;
        if (new_mv) {
            const uint8_t ref_frame_type = av1_ref_frame_type(mbmi->block_mi.ref_frame);
            for (int idx = 0; idx < 2; ++idx) {
                if (xd->ref_mv_count[ref_frame_type] > idx + 1) {
                    const uint8_t drl_ctx =
                        av1_drl_ctx(xd->final_ref_mv_stack, idx);
                    update_cdf(fc->drl_cdf[drl_ctx], mbmi->block_mi.ref_mv_idx != idx, 2);
#if CONFIG_ENTROPY_STATS
                    ++counts->drl_mode[drl_ctx][mbmi->ref_mv_idx != idx];
#endif
                    if (mbmi->block_mi.ref_mv_idx == idx) break;
                }
            }
        }

        if (have_nearmv_in_inter_mode(mbmi->block_mi.mode)) {
            const uint8_t ref_frame_type = av1_ref_frame_type(mbmi->block_mi.ref_frame);
            for (int idx = 1; idx < 3; ++idx) {
                if (xd->ref_mv_count[ref_frame_type] > idx + 1) {
                    const uint8_t drl_ctx =
                        av1_drl_ctx(xd->final_ref_mv_stack, idx);
                    update_cdf(fc->drl_cdf[drl_ctx], mbmi->block_mi.ref_mv_idx != idx - 1, 2);
#if CONFIG_ENTROPY_STATS
                    ++counts->drl_mode[drl_ctx][mbmi->ref_mv_idx != idx - 1];
#endif
                    if (mbmi->block_mi.ref_mv_idx == idx - 1) break;
                }
            }
        }
        if (have_newmv_in_inter_mode(mbmi->block_mi.mode)) {
            const int allow_hp = picture_control_set_ptr->parent_pcs_ptr->frm_hdr.force_integer_mv
                ? MV_SUBPEL_NONE
                : picture_control_set_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv;
            if (new_mv) {
                IntMv ref_mv;
                for (int ref = 0; ref < 1 + has_second_ref(mbmi); ++ref) {
                    ref_mv = cu_ptr->predmv[ref];
                    MV mv;
                    mv.row = cu_ptr->prediction_unit_array[0].mv[ref].y;
                    mv.col = cu_ptr->prediction_unit_array[0].mv[ref].x;
                    if (cu_ptr->prediction_unit_array[0].inter_pred_direction_index == UNI_PRED_LIST_1)
                    {
                        mv.row = cu_ptr->prediction_unit_array[0].mv[1].y;
                        mv.col = cu_ptr->prediction_unit_array[0].mv[1].x;
                    }
                    av1_update_mv_stats(&mbmi->block_mi.mv[ref].as_mv, &ref_mv.as_mv, &fc->nmvc,
                        allow_hp);
                }
            }
            else if (mbmi->block_mi.mode == NEAREST_NEWMV || mbmi->block_mi.mode == NEAR_NEWMV) {
                IntMv ref_mv = cu_ptr->predmv[1];
                MV mv;
                mv.row = cu_ptr->prediction_unit_array[0].mv[1].y;
                mv.col = cu_ptr->prediction_unit_array[0].mv[1].x;
                av1_update_mv_stats(&mv,&ref_mv.as_mv, &fc->nmvc,
                    allow_hp);
            }
            else if (mbmi->block_mi.mode == NEW_NEARESTMV || mbmi->block_mi.mode == NEW_NEARMV) {
                IntMv ref_mv = cu_ptr->predmv[0];

                MV mv;
                mv.row = cu_ptr->prediction_unit_array[0].mv[0].y;
                mv.col = cu_ptr->prediction_unit_array[0].mv[0].x;
                av1_update_mv_stats(&mv, &ref_mv.as_mv, &fc->nmvc,
                    allow_hp);
            }
        }
    }
}
void update_part_stats(
    PictureControlSet  *picture_control_set_ptr,
    CodingUnit         *cu_ptr,
    int                 mi_row,
    int                 mi_col) {

    const AV1_COMMON *const cm = picture_control_set_ptr->parent_pcs_ptr->av1_cm;
    MacroBlockD *xd = cu_ptr->av1xd;
    const MbModeInfo *const mbmi = &xd->mi[0]->mbmi;
    const BlockGeom          *blk_geom = get_blk_geom_mds(cu_ptr->mds_idx);
    BlockSize bsize = blk_geom->bsize;
    FRAME_CONTEXT *fc = xd->tile_ctx;

    if (mi_row >= cm->mi_rows || mi_col >= cm->mi_cols) return;
    const int hbs = mi_size_wide[bsize] / 2;
    const int is_partition_root = bsize >= BLOCK_8X8;
    if (is_partition_root) {
        const PartitionType partition = cu_ptr->part;
        int ctx;

        NeighborArrayUnit    *partition_context_neighbor_array = picture_control_set_ptr->ep_partition_context_neighbor_array;
        uint32_t partition_context_left_neighbor_index = get_neighbor_array_unit_left_index(
            partition_context_neighbor_array,
            (mi_row << MI_SIZE_LOG2));
        uint32_t partition_context_top_neighbor_index = get_neighbor_array_unit_top_index(
            partition_context_neighbor_array,
            (mi_col << MI_SIZE_LOG2));

        const PartitionContextType above_ctx = (((PartitionContext*)partition_context_neighbor_array->top_array)[partition_context_top_neighbor_index].above == (int8_t)INVALID_NEIGHBOR_DATA) ?
            0 : ((PartitionContext*)partition_context_neighbor_array->top_array)[partition_context_top_neighbor_index].above;
        const PartitionContextType left_ctx = (((PartitionContext*)partition_context_neighbor_array->left_array)[partition_context_left_neighbor_index].left == (int8_t)INVALID_NEIGHBOR_DATA) ?
            0 : ((PartitionContext*)partition_context_neighbor_array->left_array)[partition_context_left_neighbor_index].left;

        const int32_t bsl = mi_size_wide_log2[bsize] - mi_size_wide_log2[BLOCK_8X8];
        int32_t above = (above_ctx >> bsl) & 1, left = (left_ctx >> bsl) & 1;

        assert(mi_size_wide_log2[bsize] == mi_size_high_log2[bsize]);
        assert(bsl >= 0);

        ctx = (left * 2 + above) + bsl * PARTITION_PLOFFSET;

        const int has_rows = (mi_row + hbs) < cm->mi_rows;
        const int has_cols = (mi_col + hbs) < cm->mi_cols;

        if (has_rows && has_cols) {
#if CONFIG_ENTROPY_STATS
            td->counts->partition[ctx][partition]++;
#endif
            if (picture_control_set_ptr->update_cdf) {
                update_cdf(fc->partition_cdf[ctx], partition,
                    partition_cdf_length(bsize));

            }
        }
    }
}
#endif