/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include "EbModeDecisionConfiguration.h"
#include "EbRateDistortionCost.h"
#include "EbUtility.h"
#include "EbModeDecisionProcess.h"
#include "EbDefinitions.h"
#if ADD_MDC_FULL_COST
#include "aom_dsp_rtcd.h"
#include "EbAdaptiveMotionVectorPrediction.h"
#endif
/********************************************
 * Constants
 ********************************************/
int pa_to_ep_block_index[85] = {
    0    ,
    25   ,
    50   ,
    75   ,    84   ,    93   ,    102  ,
    111  ,
    136  ,    145  ,    154  ,    163  ,
    172  ,
    197  ,    206  ,    215  ,    224  ,
    233  ,
    258  ,    267  ,    276  ,    285  ,
    294  ,
    319  ,
    344  ,    353  ,    362  ,    371  ,
    380  ,
    405  ,    414  ,    423  ,    432  ,
    441  ,
    466  ,    475  ,    484  ,   493  ,
    502  ,
    527  ,    536  ,    545  ,    554  ,
    563  ,
    588  ,
    613  ,    622  ,    631  ,    640  ,
    649  ,
    674  ,    683  ,    692  ,    701  ,
    710  ,
    735  ,    744  ,    753  ,    762  ,
    771  ,
    796  ,    805  ,    814  ,    823  ,
    832  ,
    857  ,
    882  ,    891  ,    900  ,    909  ,
    918  ,
    943  ,    952  ,    961  ,    970  ,
    979  ,
    1004 ,    1013 ,    1022 ,    1031 ,
    1040 ,
    1065 ,    1074 ,    1083 ,    1092
};

#define ADD_CU_STOP_SPLIT             0   // Take into account & Stop Splitting
#define ADD_CU_CONTINUE_SPLIT         1   // Take into account & Continue Splitting
#define DO_NOT_ADD_CU_CONTINUE_SPLIT  2   // Do not take into account & Continue Splitting

#define DEPTH_64                      0   // Depth corresponding to the CU size
#define DEPTH_32                      1   // Depth corresponding to the CU size
#define DEPTH_16                      2   // Depth corresponding to the CU size
#define DEPTH_8                       3   // Depth corresponding to the CU size

static const uint8_t parentCuIndex[85] =
{
    0,
    0, 0, 0, 1, 2, 3, 5, 0, 1, 2, 3, 10, 0, 1, 2, 3, 15, 0, 1, 2, 3,
    21, 0, 0, 1, 2, 3, 5, 0, 1, 2, 3, 10, 0, 1, 2, 3, 15, 0, 1, 2, 3,
    42, 0, 0, 1, 2, 3, 5, 0, 1, 2, 3, 10, 0, 1, 2, 3, 15, 0, 1, 2, 3,
    36, 0, 0, 1, 2, 3, 5, 0, 1, 2, 3, 10, 0, 1, 2, 3, 15, 0, 1, 2, 3,
};

const uint8_t incrementalCount[85] = {
    //64x64
    0,
    //32x32
    4, 4,
    4, 4,
    //16x16
    0, 0, 0, 0,
    0, 4, 0, 4,
    0, 0, 0, 0,
    0, 4, 0, 4,
    //8x8
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 4, 0, 0, 0, 4,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 4, 0, 0, 0, 4
};

extern uint32_t get_me_info_index(
    uint32_t         max_me_block,
    const BlockGeom *blk_geom,
    uint32_t         geom_offset_x,
    uint32_t         geom_offset_y);

/*******************************************
mdcSetDepth : set depth to be tested
*******************************************/
#define REFINEMENT_P        0x01
#define REFINEMENT_Pp1      0x02
#define REFINEMENT_Pp2      0x04
#define REFINEMENT_Pp3      0x08
#define REFINEMENT_Pm1      0x10
#define REFINEMENT_Pm2      0x20
#define REFINEMENT_Pm3      0x40

EbErrorType MdcRefinement(
    MdcpLocalCodingUnit                   *local_cu_array,
    uint32_t                                  cu_index,
    uint32_t                                  depth,
    uint8_t                                   refinementLevel,
    uint8_t                                   lowestLevel)
{
    EbErrorType return_error = EB_ErrorNone;

    if (refinementLevel & REFINEMENT_P) {
        if (lowestLevel == REFINEMENT_P)
            local_cu_array[cu_index].stop_split = EB_TRUE;
    }
    else
        local_cu_array[cu_index].selected_cu = EB_FALSE;
    if (refinementLevel & REFINEMENT_Pp1) {
        if (depth < 3 && cu_index < 81) {
            local_cu_array[cu_index + 1].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + depth_offset[depth + 1]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1]].selected_cu = EB_TRUE;
        }
        if (lowestLevel == REFINEMENT_Pp1) {
            if (depth < 3 && cu_index < 81) {
                local_cu_array[cu_index + 1].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + depth_offset[depth + 1]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1]].stop_split = EB_TRUE;
            }
        }
    }

    if (refinementLevel & REFINEMENT_Pp2) {
        if (depth < 2 && cu_index < 65) {
            local_cu_array[cu_index + 1 + 1].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 1 + depth_offset[depth + 2]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 1 + 2 * depth_offset[depth + 2]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 1 + 3 * depth_offset[depth + 2]].selected_cu = EB_TRUE;

            local_cu_array[cu_index + 1 + depth_offset[depth + 1] + 1].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + depth_offset[depth + 1] + 1 + depth_offset[depth + 2]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + depth_offset[depth + 1] + 1 + 2 * depth_offset[depth + 2]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + depth_offset[depth + 1] + 1 + 3 * depth_offset[depth + 2]].selected_cu = EB_TRUE;

            local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1] + 1].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1] + 1 + depth_offset[depth + 2]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1] + 1 + 2 * depth_offset[depth + 2]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1] + 1 + 3 * depth_offset[depth + 2]].selected_cu = EB_TRUE;

            local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1] + 1].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1] + 1 + depth_offset[depth + 2]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1] + 1 + 2 * depth_offset[depth + 2]].selected_cu = EB_TRUE;
            local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1] + 1 + 3 * depth_offset[depth + 2]].selected_cu = EB_TRUE;
        }
        if (lowestLevel == REFINEMENT_Pp2) {
            if (depth < 2 && cu_index < 65) {
                local_cu_array[cu_index + 1 + 1].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 1 + depth_offset[depth + 2]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 1 + 2 * depth_offset[depth + 2]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 1 + 3 * depth_offset[depth + 2]].stop_split = EB_TRUE;

                local_cu_array[cu_index + 1 + depth_offset[depth + 1] + 1].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + depth_offset[depth + 1] + 1 + depth_offset[depth + 2]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + depth_offset[depth + 1] + 1 + 2 * depth_offset[depth + 2]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + depth_offset[depth + 1] + 1 + 3 * depth_offset[depth + 2]].stop_split = EB_TRUE;

                local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1] + 1].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1] + 1 + depth_offset[depth + 2]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1] + 1 + 2 * depth_offset[depth + 2]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 2 * depth_offset[depth + 1] + 1 + 3 * depth_offset[depth + 2]].stop_split = EB_TRUE;

                local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1] + 1].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1] + 1 + depth_offset[depth + 2]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1] + 1 + 2 * depth_offset[depth + 2]].stop_split = EB_TRUE;
                local_cu_array[cu_index + 1 + 3 * depth_offset[depth + 1] + 1 + 3 * depth_offset[depth + 2]].stop_split = EB_TRUE;
            }
        }
    }

    if (refinementLevel & REFINEMENT_Pp3) {
        uint8_t inLoop;
        uint8_t outLoop;
        uint8_t cu_index = 2;
        if (depth == 0) {
            for (outLoop = 0; outLoop < 16; ++outLoop) {
                for (inLoop = 0; inLoop < 4; ++inLoop)
                    local_cu_array[++cu_index].selected_cu = EB_TRUE;
                cu_index += cu_index == 21 ? 2 : cu_index == 42 ? 2 : cu_index == 63 ? 2 : 1;
            }
            if (lowestLevel == REFINEMENT_Pp3) {
                cu_index = 2;
                for (outLoop = 0; outLoop < 16; ++outLoop) {
                    for (inLoop = 0; inLoop < 4; ++inLoop)
                        local_cu_array[++cu_index].stop_split = EB_TRUE;
                    cu_index += cu_index == 21 ? 2 : cu_index == 42 ? 2 : cu_index == 63 ? 2 : 1;
                }
            }
        }
    }

    if (refinementLevel & REFINEMENT_Pm1) {
        if (depth > 0)
            local_cu_array[cu_index - 1 - parentCuIndex[cu_index]].selected_cu = EB_TRUE;
        if (lowestLevel == REFINEMENT_Pm1) {
            if (depth > 0)
                local_cu_array[cu_index - 1 - parentCuIndex[cu_index]].stop_split = EB_TRUE;
        }
    }

    if (refinementLevel & REFINEMENT_Pm2) {
        if (depth == 2)
            local_cu_array[0].selected_cu = EB_TRUE;
        if (depth == 3) {
            local_cu_array[1].selected_cu = EB_TRUE;
            local_cu_array[22].selected_cu = EB_TRUE;
            local_cu_array[43].selected_cu = EB_TRUE;
            local_cu_array[64].selected_cu = EB_TRUE;
        }
        if (lowestLevel == REFINEMENT_Pm2) {
            if (depth == 2)
                local_cu_array[0].stop_split = EB_TRUE;
            if (depth == 3) {
                local_cu_array[1].stop_split = EB_TRUE;
                local_cu_array[22].stop_split = EB_TRUE;
                local_cu_array[43].stop_split = EB_TRUE;
                local_cu_array[64].stop_split = EB_TRUE;
            }
        }
    }

    if (refinementLevel & REFINEMENT_Pm3) {
        if (depth == 3)
            local_cu_array[0].selected_cu = EB_TRUE;
        if (lowestLevel == REFINEMENT_Pm2) {
            if (depth == 3)
                local_cu_array[0].stop_split = EB_TRUE;
        }
    }

    return return_error;
}
#if !OPT_LOSSLESS_0
/*******************************************
Derive the contouring class
If (AC energy < 32 * 32) then apply aggressive action (Class 1),
else if (AC energy < 32 * 32 * 1.6) OR (32 * 32 * 3.5 < AC energy < 32 * 32 * 4.5 AND non-8x8) then moderate action (Class 2),
else no action
*******************************************/
uint8_t derive_contouring_class(
    PictureParentControlSet   *parent_pcs_ptr,
    uint16_t                       sb_index,
    uint8_t                        leaf_index)
{
    uint8_t contouringClass = 0;

    SequenceControlSet *sequence_control_set_ptr = (SequenceControlSet*)parent_pcs_ptr->sequence_control_set_wrapper_ptr->object_ptr;

    if (parent_pcs_ptr->is_sb_homogeneous_over_time[sb_index]) {
        if (leaf_index > 0) {
            SbParams            *sb_params = &sequence_control_set_ptr->sb_params_array[sb_index];
            if (sb_params->is_edge_sb) {
                if (parent_pcs_ptr->sb_y_src_energy_cu_array[sb_index][(leaf_index - 1) / 21 + 1] < ANTI_CONTOURING_TH_1)
                    contouringClass = 2;
                else if (parent_pcs_ptr->sb_y_src_energy_cu_array[sb_index][(leaf_index - 1) / 21 + 1] < ANTI_CONTOURING_TH_2)
                    contouringClass = 3;
                else if (parent_pcs_ptr->sb_y_src_energy_cu_array[sb_index][(leaf_index - 1) / 21 + 1] < (ANTI_CONTOURING_TH_1 + ANTI_CONTOURING_TH_2))
                    contouringClass = 3;
            }
            else {
                if (parent_pcs_ptr->sb_y_src_energy_cu_array[sb_index][(leaf_index - 1) / 21 + 1] < ANTI_CONTOURING_TH_0)
                    contouringClass = 1;
                else if (parent_pcs_ptr->sb_y_src_energy_cu_array[sb_index][(leaf_index - 1) / 21 + 1] < ANTI_CONTOURING_TH_1)
                    contouringClass = 2;
                else if (parent_pcs_ptr->sb_y_src_energy_cu_array[sb_index][(leaf_index - 1) / 21 + 1] < ANTI_CONTOURING_TH_2)
                    contouringClass = 3;
            }
        }
    }
    return(contouringClass);
}
#endif

void RefinementPredictionLoop(
    SequenceControlSet                   *sequence_control_set_ptr,
    PictureControlSet                    *picture_control_set_ptr,
#if !MEMORY_FOOTPRINT_OPT
    LargestCodingUnit                    *sb_ptr,
#endif
    uint32_t                              sb_index,
    ModeDecisionConfigurationContext     *context_ptr)
{
    MdcpLocalCodingUnit    *local_cu_array         = context_ptr->local_cu_array;
    SbParams               *sb_params            = &sequence_control_set_ptr->sb_params_array[sb_index];
    uint32_t                  cu_index             = 0;
#if !MEMORY_FOOTPRINT_OPT
    sb_ptr->pred64 = EB_FALSE;
#endif
    while (cu_index < CU_MAX_COUNT)
    {
        if (sb_params->raster_scan_cu_validity[md_scan_to_raster_scan[cu_index]] && (local_cu_array[cu_index].early_split_flag == EB_FALSE))
        {
            local_cu_array[cu_index].selected_cu = EB_TRUE;
#if !MEMORY_FOOTPRINT_OPT
            sb_ptr->pred64 = (cu_index == 0) ? EB_TRUE : sb_ptr->pred64;
#endif
            uint32_t depth = get_coded_unit_stats(cu_index)->depth;
            uint8_t refinementLevel;
            {
#if ADP_BQ
                if (picture_control_set_ptr->parent_pcs_ptr->pic_depth_mode == PIC_SB_SWITCH_SQ_DEPTH_MODE && picture_control_set_ptr->parent_pcs_ptr->sb_depth_mode_array[sb_index] == SB_PRED_OPEN_LOOP_DEPTH_MODE)
#else
                if (picture_control_set_ptr->parent_pcs_ptr->pic_depth_mode == PIC_SB_SWITCH_DEPTH_MODE && picture_control_set_ptr->parent_pcs_ptr->sb_depth_mode_array[sb_index] == SB_PRED_OPEN_LOOP_DEPTH_MODE)
#endif
                    refinementLevel = Pred;
                else

#if ADP_BQ
                    if (picture_control_set_ptr->parent_pcs_ptr->pic_depth_mode == PIC_SB_SWITCH_SQ_DEPTH_MODE && picture_control_set_ptr->parent_pcs_ptr->sb_depth_mode_array[sb_index] == SB_FAST_OPEN_LOOP_DEPTH_MODE)
#else
                    if (picture_control_set_ptr->parent_pcs_ptr->pic_depth_mode == PIC_SB_SWITCH_DEPTH_MODE && picture_control_set_ptr->parent_pcs_ptr->sb_depth_mode_array[sb_index] == SB_FAST_OPEN_LOOP_DEPTH_MODE)
#endif
                        refinementLevel = ndp_level_1[depth];
                    else  { // SB_OPEN_LOOP_DEPTH_MODE
                        refinementLevel = ndp_level_0[depth];
                    }

                if (picture_control_set_ptr->parent_pcs_ptr->cu8x8_mode == CU_8x8_MODE_1) {
                    refinementLevel = ((refinementLevel & REFINEMENT_Pp1) && depth == 2) ? refinementLevel - REFINEMENT_Pp1 :
                        ((refinementLevel & REFINEMENT_Pp2) && depth == 1) ? refinementLevel - REFINEMENT_Pp2 :
                        ((refinementLevel & REFINEMENT_Pp3) && depth == 0) ? refinementLevel - REFINEMENT_Pp3 : refinementLevel;
                }

                uint8_t lowestLevel = 0x00;

                lowestLevel = (refinementLevel & REFINEMENT_Pp3) ? REFINEMENT_Pp3 : (refinementLevel & REFINEMENT_Pp2) ? REFINEMENT_Pp2 : (refinementLevel & REFINEMENT_Pp1) ? REFINEMENT_Pp1 :
                    (refinementLevel & REFINEMENT_P) ? REFINEMENT_P :
                    (refinementLevel & REFINEMENT_Pm1) ? REFINEMENT_Pm1 : (refinementLevel & REFINEMENT_Pm2) ? REFINEMENT_Pm2 : (refinementLevel & REFINEMENT_Pm3) ? REFINEMENT_Pm3 : 0x00;

                MdcRefinement(
                    &(*context_ptr->local_cu_array),
                    cu_index,
                    depth,
                    refinementLevel,
                    lowestLevel);
            }

            cu_index += depth_offset[depth];
        }
        else
            cu_index++;
    } // End while 1 CU Loop
}

#if !DISABLE_OIS_USE
void PrePredictionRefinement(
    SequenceControlSet                   *sequence_control_set_ptr,
    PictureControlSet                    *picture_control_set_ptr,
    LargestCodingUnit                    *sb_ptr,
    uint32_t                                  sb_index,
    uint32_t                                 *startDepth,
    uint32_t                                 *endDepth
)
{
    SbParams    *sb_params = &sequence_control_set_ptr->sb_params_array[sb_index];

    EB_SLICE        slice_type = picture_control_set_ptr->slice_type;

    uint8_t           edge_block_num = picture_control_set_ptr->parent_pcs_ptr->edge_results_ptr[sb_index].edge_block_num;

    SbStat      *sb_stat_ptr = &(picture_control_set_ptr->parent_pcs_ptr->sb_stat_array[sb_index]);
    uint8_t           stationary_edge_over_time_flag = sb_stat_ptr->stationary_edge_over_time_flag;

    uint8_t           aura_status_iii = sb_ptr->aura_status_iii;

    if (picture_control_set_ptr->parent_pcs_ptr->high_dark_low_light_area_density_flag && picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index > 0 && picture_control_set_ptr->parent_pcs_ptr->sharp_edge_sb_flag[sb_index] && !picture_control_set_ptr->parent_pcs_ptr->similar_colocated_sb_array_ii[sb_index])
        *startDepth = DEPTH_16;
    if ((slice_type != I_SLICE && picture_control_set_ptr->high_intra_slection == 0) && (sb_params->is_complete_sb)) {
        if (picture_control_set_ptr->scene_caracteristic_id == EB_FRAME_CARAC_0) {
            if (picture_control_set_ptr->parent_pcs_ptr->grass_percentage_in_picture > 60 && aura_status_iii)
                *startDepth = DEPTH_16;
        }
    }

    if (picture_control_set_ptr->parent_pcs_ptr->logo_pic_flag && edge_block_num)
        *startDepth = DEPTH_16;
    // S-LOGO

    if (stationary_edge_over_time_flag > 0) {
        *startDepth = DEPTH_16;
        *endDepth = DEPTH_16;
    }

    if (picture_control_set_ptr->parent_pcs_ptr->complex_sb_array[sb_ptr->index] == SB_COMPLEXITY_STATUS_2)
        *startDepth = DEPTH_16;
}
#endif

void ForwardCuToModeDecision(
    SequenceControlSet                   *sequence_control_set_ptr,
    PictureControlSet                    *picture_control_set_ptr,
    uint32_t                                  sb_index,
    ModeDecisionConfigurationContext     *context_ptr
)
{
    uint8_t                   cu_index = 0;
    uint32_t                  cuClass = DO_NOT_ADD_CU_CONTINUE_SPLIT;
    EbBool                 split_flag = EB_TRUE;
    MdcLcuData           *resultsPtr = &picture_control_set_ptr->mdc_sb_array[sb_index];
    SbParams            *sb_params = &sequence_control_set_ptr->sb_params_array[sb_index];
    MdcpLocalCodingUnit  *local_cu_array = context_ptr->local_cu_array;
    EB_SLICE                slice_type = picture_control_set_ptr->slice_type;

    // CU Loop
    const CodedUnitStats *cuStatsPtr = get_coded_unit_stats(0);
#if !MEMORY_FOOTPRINT_OPT
    SbStat *sb_stat_ptr = &(picture_control_set_ptr->parent_pcs_ptr->sb_stat_array[sb_index]);
    EbBool    testAllDepthIntraSliceFlag = EB_FALSE;
    testAllDepthIntraSliceFlag = slice_type == I_SLICE &&
        (sb_stat_ptr->stationary_edge_over_time_flag || picture_control_set_ptr->parent_pcs_ptr->logo_pic_flag ||
        (picture_control_set_ptr->parent_pcs_ptr->very_low_var_pic_flag && picture_control_set_ptr->parent_pcs_ptr->low_motion_content_flag)) ?
        EB_TRUE : testAllDepthIntraSliceFlag;
#endif

    resultsPtr->leaf_count = 0;
    uint8_t   enable_blk_4x4 = 0;
    cu_index = 0;

    while (cu_index < CU_MAX_COUNT)
    {
        split_flag = EB_TRUE;
        if (sb_params->raster_scan_cu_validity[md_scan_to_raster_scan[cu_index]])
        {
            cuStatsPtr = get_coded_unit_stats(cu_index);

            switch (cuStatsPtr->depth) {
            case 0:
            case 1:
            case 2:

                cuClass = DO_NOT_ADD_CU_CONTINUE_SPLIT;

                if (slice_type == I_SLICE) {
#if MEMORY_FOOTPRINT_OPT
                    cuClass = local_cu_array[cu_index].selected_cu == EB_TRUE ? ADD_CU_CONTINUE_SPLIT : cuClass;
                    cuClass = local_cu_array[cu_index].stop_split == EB_TRUE ? ADD_CU_STOP_SPLIT : cuClass;
#else
                    if (testAllDepthIntraSliceFlag)
                        cuClass = ADD_CU_CONTINUE_SPLIT;
                    else {
                        cuClass = local_cu_array[cu_index].selected_cu == EB_TRUE ? ADD_CU_CONTINUE_SPLIT : cuClass;
                        cuClass = local_cu_array[cu_index].stop_split == EB_TRUE ? ADD_CU_STOP_SPLIT : cuClass;
                    }
#endif
                }
                else {
                    cuClass = local_cu_array[cu_index].selected_cu == EB_TRUE ? ADD_CU_CONTINUE_SPLIT : cuClass;
                    cuClass = local_cu_array[cu_index].stop_split == EB_TRUE ? ADD_CU_STOP_SPLIT : cuClass;
                }

                // Take into account MAX CU size & MAX intra size (from the API)
                cuClass = (cuStatsPtr->size > sequence_control_set_ptr->max_cu_size || (slice_type == I_SLICE && cuStatsPtr->size > sequence_control_set_ptr->max_intra_size)) ?
                    DO_NOT_ADD_CU_CONTINUE_SPLIT :
                    cuClass;

                // Take into account MIN CU size & Min intra size(from the API)
                cuClass = (cuStatsPtr->size == sequence_control_set_ptr->min_cu_size || (slice_type == I_SLICE && cuStatsPtr->size == sequence_control_set_ptr->min_intra_size)) ?
                    ADD_CU_STOP_SPLIT :
                    cuClass;

                switch (cuClass) {
                case ADD_CU_STOP_SPLIT:
                    // Stop
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].leaf_index = cu_index;
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].mds_idx = pa_to_ep_block_index[cu_index];
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].tot_d1_blocks = 1;
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count++].split_flag = split_flag = EB_FALSE;

                    break;

                case ADD_CU_CONTINUE_SPLIT:
                    // Go Down + consider the current CU as candidate
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].leaf_index = cu_index;
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].mds_idx = pa_to_ep_block_index[cu_index];
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].tot_d1_blocks = 1;
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count++].split_flag = split_flag = EB_TRUE;

                    break;

                case DO_NOT_ADD_CU_CONTINUE_SPLIT:
                    // Go Down + do not consider the current CU as candidate
                    split_flag = EB_TRUE;

                    break;

                default:
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].leaf_index = cu_index;
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].mds_idx = pa_to_ep_block_index[cu_index];
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count].tot_d1_blocks = 1;
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count++].split_flag = split_flag = EB_TRUE;

                    break;
                }

                break;
            case 3:

                resultsPtr->leaf_data_array[resultsPtr->leaf_count].leaf_index = cu_index;
                resultsPtr->leaf_data_array[resultsPtr->leaf_count].mds_idx = pa_to_ep_block_index[cu_index];
                resultsPtr->leaf_data_array[resultsPtr->leaf_count].tot_d1_blocks = 1;

                if (enable_blk_4x4) {
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count++].split_flag = split_flag = EB_TRUE;

                    int first_4_index = pa_to_ep_block_index[cu_index] + d1_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][cuStatsPtr->depth];
                    for (int i = 0; i < 4; ++i) {
                        resultsPtr->leaf_data_array[resultsPtr->leaf_count].leaf_index = cu_index;

                        resultsPtr->leaf_data_array[resultsPtr->leaf_count].mds_idx = first_4_index + i;
                        resultsPtr->leaf_data_array[resultsPtr->leaf_count].tot_d1_blocks = 1;

                        resultsPtr->leaf_data_array[resultsPtr->leaf_count++].split_flag = split_flag = EB_FALSE;
                    }
                }else
                    resultsPtr->leaf_data_array[resultsPtr->leaf_count++].split_flag = split_flag = EB_FALSE;

                break;

            default:
                resultsPtr->leaf_data_array[resultsPtr->leaf_count].leaf_index = cu_index;
                resultsPtr->leaf_data_array[resultsPtr->leaf_count].mds_idx = pa_to_ep_block_index[cu_index];
                resultsPtr->leaf_data_array[resultsPtr->leaf_count].tot_d1_blocks = 1;
                resultsPtr->leaf_data_array[resultsPtr->leaf_count++].split_flag = split_flag = EB_TRUE;
                break;
            }
        }

        cu_index += (split_flag == EB_TRUE) ? 1 : depth_offset[cuStatsPtr->depth];
    } // End CU Loop
}

void MdcInterDepthDecision(
    ModeDecisionConfigurationContext     *context_ptr,
    uint32_t                                 origin_x,
    uint32_t                                 origin_y,
    uint32_t                                 endDepth,
    uint32_t                                 cu_index)
{
    uint32_t               leftCuIndex;
    uint32_t               topCuIndex;
    uint32_t               topLeftCuIndex;
    uint32_t               depthZeroCandidateCuIndex;
    uint32_t               depthOneCandidateCuIndex = cu_index;
    uint32_t               depthTwoCandidateCuIndex = cu_index;
    uint64_t               depthNRate = 0;
    uint64_t               depthNPlusOneRate = 0;
    uint64_t               depthNCost = 0;
    uint64_t               depthNPlusOneCost = 0;
    MdcpLocalCodingUnit *local_cu_array = context_ptr->local_cu_array;
    /*** Stage 0: Inter depth decision: depth 2 vs depth 3 ***/
    // Walks to the last coded 8x8 block for merging
    uint8_t  group_of8x8_blocks_count = context_ptr->group_of8x8_blocks_count;
    uint8_t  group_of16x16_blocks_count = context_ptr->group_of16x16_blocks_count;
    if ((GROUP_OF_4_8x8_BLOCKS(origin_x, origin_y))) {
        group_of8x8_blocks_count++;

        // From the last coded cu index, get the indices of the left, top, and top left cus
        leftCuIndex = cu_index - DEPTH_THREE_STEP;
        topCuIndex = leftCuIndex - DEPTH_THREE_STEP;
        topLeftCuIndex = topCuIndex - DEPTH_THREE_STEP;

        // From the top left index, get the index of the candidate pu for merging
        depthTwoCandidateCuIndex = topLeftCuIndex - 1;

        // Compute depth N cost
        local_cu_array[depthTwoCandidateCuIndex].split_context = 0;
        depthNCost = (local_cu_array[depthTwoCandidateCuIndex]).early_cost + depthNRate;

        if (endDepth < 3) {
            (local_cu_array[depthTwoCandidateCuIndex]).early_split_flag = EB_FALSE;
            (local_cu_array[depthTwoCandidateCuIndex]).early_cost = depthNCost;
        }
        else {
            depthNPlusOneCost = (local_cu_array[cu_index]).early_cost + (local_cu_array[leftCuIndex]).early_cost + (local_cu_array[topCuIndex]).early_cost + (local_cu_array[topLeftCuIndex]).early_cost + depthNPlusOneRate;

            if (depthNCost <= depthNPlusOneCost) {
                // If the cost is low enough to warrant not spliting further:
                // 1. set the split flag of the candidate pu for merging to false
                // 2. update the last pu index
                (local_cu_array[depthTwoCandidateCuIndex]).early_split_flag = EB_FALSE;
                (local_cu_array[depthTwoCandidateCuIndex]).early_cost = depthNCost;
            }
            else {
                // If the cost is not low enough:
                // update the cost of the candidate pu for merging
                // this update is required for the next inter depth decision
                (&local_cu_array[depthTwoCandidateCuIndex])->early_cost = depthNPlusOneCost;
            }
        }
    }

    // Walks to the last coded 16x16 block for merging
    if (GROUP_OF_4_16x16_BLOCKS(get_coded_unit_stats(depthTwoCandidateCuIndex)->origin_x, get_coded_unit_stats(depthTwoCandidateCuIndex)->origin_y) &&
        (group_of8x8_blocks_count == 4)) {
        group_of8x8_blocks_count = 0;
        group_of16x16_blocks_count++;

        // From the last coded pu index, get the indices of the left, top, and top left pus
        leftCuIndex = depthTwoCandidateCuIndex - DEPTH_TWO_STEP;
        topCuIndex = leftCuIndex - DEPTH_TWO_STEP;
        topLeftCuIndex = topCuIndex - DEPTH_TWO_STEP;

        // From the top left index, get the index of the candidate pu for merging
        depthOneCandidateCuIndex = topLeftCuIndex - 1;

        if (get_coded_unit_stats(depthOneCandidateCuIndex)->depth == 1) {
            depthNCost = local_cu_array[depthOneCandidateCuIndex].early_cost + depthNRate;
            if (endDepth < 2) {
                local_cu_array[depthOneCandidateCuIndex].early_split_flag = EB_FALSE;
                local_cu_array[depthOneCandidateCuIndex].early_cost = depthNCost;
            }
            else {
                // Compute depth N+1 cost
                depthNPlusOneCost = local_cu_array[depthTwoCandidateCuIndex].early_cost +
                    local_cu_array[leftCuIndex].early_cost +
                    local_cu_array[topCuIndex].early_cost +
                    local_cu_array[topLeftCuIndex].early_cost +
                    depthNPlusOneRate;

                // Inter depth comparison: depth 1 vs depth 2
                if (depthNCost <= depthNPlusOneCost) {
                    // If the cost is low enough to warrant not spliting further:
                    // 1. set the split flag of the candidate pu for merging to false
                    // 2. update the last pu index
                    local_cu_array[depthOneCandidateCuIndex].early_split_flag = EB_FALSE;
                    local_cu_array[depthOneCandidateCuIndex].early_cost = depthNCost;
                }
                else {
                    // If the cost is not low enough:
                    // update the cost of the candidate pu for merging
                    // this update is required for the next inter depth decision
                    local_cu_array[depthOneCandidateCuIndex].early_cost = depthNPlusOneCost;
                }
            }
        }
    }

    // Stage 2: Inter depth decision: depth 0 vs depth 1

    // Walks to the last coded 32x32 block for merging
    // Stage 2 isn't performed in I slices since the abcense of 64x64 candidates
    if (GROUP_OF_4_32x32_BLOCKS(get_coded_unit_stats(depthOneCandidateCuIndex)->origin_x, get_coded_unit_stats(depthOneCandidateCuIndex)->origin_y) &&
        (group_of16x16_blocks_count == 4)) {
        group_of16x16_blocks_count = 0;

        // From the last coded pu index, get the indices of the left, top, and top left pus
        leftCuIndex = depthOneCandidateCuIndex - DEPTH_ONE_STEP;
        topCuIndex = leftCuIndex - DEPTH_ONE_STEP;
        topLeftCuIndex = topCuIndex - DEPTH_ONE_STEP;

        // From the top left index, get the index of the candidate pu for merging
        depthZeroCandidateCuIndex = topLeftCuIndex - 1;

        if (get_coded_unit_stats(depthZeroCandidateCuIndex)->depth == 0) {
            // Compute depth N cost
            depthNCost = (&local_cu_array[depthZeroCandidateCuIndex])->early_cost + depthNRate;
            if (endDepth < 1)
                (&local_cu_array[depthZeroCandidateCuIndex])->early_split_flag = EB_FALSE;
            else {
                // Compute depth N+1 cost
                depthNPlusOneCost = local_cu_array[depthOneCandidateCuIndex].early_cost +
                    local_cu_array[leftCuIndex].early_cost +
                    local_cu_array[topCuIndex].early_cost +
                    local_cu_array[topLeftCuIndex].early_cost +
                    depthNPlusOneRate;

                // Inter depth comparison: depth 0 vs depth 1
                if (depthNCost <= depthNPlusOneCost) {
                    // If the cost is low enough to warrant not spliting further:
                    // 1. set the split flag of the candidate pu for merging to false
                    // 2. update the last pu index
                    (&local_cu_array[depthZeroCandidateCuIndex])->early_split_flag = EB_FALSE;
                }
            }
        }
    }

    context_ptr->group_of8x8_blocks_count = group_of8x8_blocks_count;
    context_ptr->group_of16x16_blocks_count = group_of16x16_blocks_count;
}
#if PREDICT_NSQ_SHAPE
/// compute the cost of curr depth, and the depth above
void   mdc_compute_depth_costs(
    ModeDecisionConfigurationContext    *context_ptr,
    SequenceControlSet                  *sequence_control_set_ptr,
    uint32_t                             curr_depth_mds,
    uint32_t                             above_depth_mds,
    uint32_t                             step,
    uint64_t                            *above_depth_cost,
    uint64_t                            *curr_depth_cost)
{
    uint64_t       above_non_split_rate = 0;
    uint64_t       above_split_rate = 0;
    UNUSED(sequence_control_set_ptr);
    /*
    ___________
    |     |     |
    |blk0 |blk1 |
    |-----|-----|
    |blk2 |blk3 |
    |_____|_____|
    */
    // current depth blocks
    uint32_t       curr_depth_blk0_mds = curr_depth_mds - 3 * step;
    uint32_t       curr_depth_blk1_mds = curr_depth_mds - 2 * step;
    uint32_t       curr_depth_blk2_mds = curr_depth_mds - 1 * step;
   // uint32_t       curr_depth_blk3_mds = curr_depth_mds;

    // Rate of not spliting the current depth (Depth != 4) in case the children were omitted by MDC
    uint64_t       curr_non_split_rate_blk0 = 0;
    uint64_t       curr_non_split_rate_blk1 = 0;
    uint64_t       curr_non_split_rate_blk2 = 0;
    uint64_t       curr_non_split_rate_blk3 = 0;


    // Compute above depth  cost
    *above_depth_cost = context_ptr->local_cu_array[above_depth_mds].early_cost + above_non_split_rate;


    // Compute current depth  cost
    *curr_depth_cost =
        context_ptr->local_cu_array[curr_depth_mds].early_cost + curr_non_split_rate_blk3 +
        context_ptr->local_cu_array[curr_depth_mds - 1 * step].early_cost + curr_non_split_rate_blk2 +
        context_ptr->local_cu_array[curr_depth_mds - 2 * step].early_cost + curr_non_split_rate_blk1 +
        context_ptr->local_cu_array[curr_depth_mds - 3 * step].early_cost + curr_non_split_rate_blk0 +
        above_split_rate;
}
uint32_t mdc_d2_inter_depth_block_decision(
    PictureControlSet                         *picture_control_set_ptr,
    ModeDecisionConfigurationContext          *context_ptr,
    EbMdcLeafData                             *results_ptr,
    uint32_t                                   blk_mds,
    uint32_t                                   sb_index) {


    uint32_t                last_cu_index;
    uint64_t                parent_depth_cost = 0, current_depth_cost = 0;
    SequenceControlSet     *sequence_control_set_ptr = (SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr;
    EbBool                  last_depth_flag;
    const BlockGeom        *blk_geom;

    last_depth_flag = context_ptr->local_cu_array[blk_mds].early_split_flag == EB_FALSE ? EB_TRUE : EB_FALSE;

    last_cu_index = blk_mds;
    blk_geom = get_blk_geom_mds(blk_mds);
    uint32_t    parent_depth_idx_mds = blk_mds;
    uint32_t    current_depth_idx_mds = blk_mds;

    if (last_depth_flag) {
        while (blk_geom->is_last_quadrant) {
            //get parent idx
            parent_depth_idx_mds = current_depth_idx_mds - parent_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth];
            if (picture_control_set_ptr->slice_type == I_SLICE && parent_depth_idx_mds == 0 && sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128)
                parent_depth_cost = MAX_MODE_COST;
            else
                mdc_compute_depth_costs(context_ptr, sequence_control_set_ptr, current_depth_idx_mds, parent_depth_idx_mds, ns_depth_offset[sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128][blk_geom->depth], &parent_depth_cost, &current_depth_cost);
            if (!sequence_control_set_ptr->sb_geom[sb_index].block_is_allowed[parent_depth_idx_mds])
                parent_depth_cost = MAX_MODE_COST;
            if (parent_depth_cost <= current_depth_cost) {
                context_ptr->local_cu_array[parent_depth_idx_mds].early_split_flag = EB_FALSE;
                context_ptr->local_cu_array[parent_depth_idx_mds].early_cost = parent_depth_cost;
                results_ptr[parent_depth_idx_mds].early_split_flag = context_ptr->local_cu_array[parent_depth_idx_mds].early_split_flag;
                last_cu_index = parent_depth_idx_mds;
            }
            else {
                context_ptr->local_cu_array[parent_depth_idx_mds].early_cost = current_depth_cost;
            }

            //setup next parent inter depth
            blk_geom = get_blk_geom_mds(parent_depth_idx_mds);
            current_depth_idx_mds = parent_depth_idx_mds;
        }
    }

    return last_cu_index;
}

uint64_t  mdc_d1_non_square_block_decision(
    SequenceControlSet                       *sequence_control_set_ptr,
    ModeDecisionConfigurationContext         *context_ptr
)
{
    UNUSED(sequence_control_set_ptr);
    //compute total cost for the whole block partition
    uint64_t tot_cost = 0;
    uint32_t first_blk_idx = context_ptr->mds_idx - (context_ptr->blk_geom->totns - 1);//index of first block in this partition
    uint32_t blk_it;

    for (blk_it = 0; blk_it < context_ptr->blk_geom->totns; blk_it++)
    {
        tot_cost += context_ptr->local_cu_array[first_blk_idx + blk_it].early_cost;
    }
#if 0 //SPLIT_RATE_FIX
    if (context_ptr->blk_geom->bsize > BLOCK_4X4) {
        uint64_t split_cost = 0;
        uint32_t parent_depth_idx_mds = context_ptr->blk_geom->sqi_mds;
        av1_split_flag_rate(
            sequence_control_set_ptr,
            context_ptr,
            &context_ptr->md_cu_arr_nsq[parent_depth_idx_mds],
            0,
            from_shape_to_part[context_ptr->blk_geom->shape],
            &split_cost,
            context_ptr->lambda,
            context_ptr->md_rate_estimation_ptr,
            sequence_control_set_ptr->max_sb_depth);

        tot_cost += split_cost;
    }
#endif
#if 1
    if (context_ptr->blk_geom->shape == PART_N || tot_cost < context_ptr->local_cu_array[context_ptr->blk_geom->sqi_mds].early_cost)
    {
        //store best partition cost in parent square
        context_ptr->local_cu_array[context_ptr->blk_geom->sqi_mds].early_cost = tot_cost;
        context_ptr->local_cu_array[context_ptr->blk_geom->sqi_mds].part = from_shape_to_part[context_ptr->blk_geom->shape];
        context_ptr->local_cu_array[context_ptr->blk_geom->sqi_mds].best_d1_blk = first_blk_idx;
    }
#endif
    return tot_cost;
}

uint8_t find_shape_index(PART shape, PART nsq_shape_table[10]) {
    uint8_t i;
    for (i = 0; i < 10; i++)
        if (shape == nsq_shape_table[i]) return i;

    return 0;
}
uint8_t find_depth_index(uint8_t shape, uint8_t depth_table[NUMBER_OF_DEPTH]) {
    uint8_t i;
    for (i = 0; i < NUMBER_OF_DEPTH; i++)
        if (shape == depth_table[i]) return i;

    return 0;
}

uint8_t get_depth(
    uint8_t sq_size) {
    uint8_t depth = sq_size == 128 ? 0 :
        sq_size == 64 ? 1 :
        sq_size == 32 ? 2 :
        sq_size == 16 ? 3 :
        sq_size == 8 ? 4 : 5;

    return depth;
}
#if ADD_MDC_FULL_COST
static INLINE void set_dc_sign(int32_t *cul_level, int32_t dc_val) {
    if (dc_val < 0)
        *cul_level |= 1 << COEFF_CONTEXT_BITS;
    else if (dc_val > 0)
        *cul_level += 2 << COEFF_CONTEXT_BITS;
}
#if ADD_MDC_FULL_COST
    extern void av1_quantize_b_facade_II(
        const TranLow *coeff_ptr,
        int32_t stride,
        int32_t                width,
        int32_t                height,
        intptr_t n_coeffs,
        const MacroblockPlane *p,
        TranLow *qcoeff_ptr,
        TranLow *dqcoeff_ptr,
        uint16_t *eob_ptr,
        const ScanOrder *sc,
        const QuantParam *qparam);
#endif
 int32_t mdc_av1_quantize_inv_quantize(
    PictureControlSet                *picture_control_set_ptr,
    ModeDecisionConfigurationContext *md_context,
    int32_t                          *coeff,
    const uint32_t                    coeff_stride,
    int32_t                          *quant_coeff,
    int32_t                          *recon_coeff,
    uint32_t                          qp,
    uint32_t                          width,
    uint32_t                          height,
    TxSize                            txsize,
    uint16_t                         *eob,
    EbAsm                             asm_type,
    uint32_t                         *count_non_zero_coeffs,
    uint32_t                          component_type,
    uint32_t                          bit_increment,
    TxType                            tx_type,
    ModeDecisionCandidateBuffer      *candidateBuffer,
    int16_t                           txb_skip_context,
    int16_t                           dc_sign_context,
    PredictionMode                    pred_mode)
{
    UNUSED(md_context);
    UNUSED(asm_type);
    UNUSED(bit_increment);
    UNUSED(txb_skip_context);
    UNUSED(dc_sign_context);
    MacroblockPlane      candidate_plane ;
    const QmVal *qMatrix = picture_control_set_ptr->parent_pcs_ptr->gqmatrix[NUM_QM_LEVELS - 1][0][txsize];
    const QmVal *iqMatrix = picture_control_set_ptr->parent_pcs_ptr->giqmatrix[NUM_QM_LEVELS - 1][0][txsize];
    uint32_t qIndex = picture_control_set_ptr->parent_pcs_ptr->delta_q_present_flag ? quantizer_to_qindex[qp] : picture_control_set_ptr->parent_pcs_ptr->base_qindex;
    if (component_type == COMPONENT_LUMA) {
        candidate_plane.quant_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.y_quant[qIndex];
        candidate_plane.quant_fp_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.y_quant_fp[qIndex];
        candidate_plane.round_fp_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.y_round_fp[qIndex];
        candidate_plane.quant_shift_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.y_quant_shift[qIndex];
        candidate_plane.zbin_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.y_zbin[qIndex];
        candidate_plane.round_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.y_round[qIndex];
        candidate_plane.dequant_QTX = picture_control_set_ptr->parent_pcs_ptr->deqMd.y_dequant_QTX[qIndex];
    }
    if (component_type == COMPONENT_CHROMA_CB) {
        candidate_plane.quant_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.u_quant[qIndex];
        candidate_plane.quant_fp_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.u_quant_fp[qIndex];
        candidate_plane.round_fp_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.u_round_fp[qIndex];
        candidate_plane.quant_shift_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.u_quant_shift[qIndex];
        candidate_plane.zbin_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.u_zbin[qIndex];
        candidate_plane.round_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.u_round[qIndex];
        candidate_plane.dequant_QTX = picture_control_set_ptr->parent_pcs_ptr->deqMd.u_dequant_QTX[qIndex];
    }
    if (component_type == COMPONENT_CHROMA_CR) {
        candidate_plane.quant_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.v_quant[qIndex];
        candidate_plane.quant_fp_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.v_quant_fp[qIndex];
        candidate_plane.round_fp_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.v_round_fp[qIndex];
        candidate_plane.quant_shift_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.v_quant_shift[qIndex];
        candidate_plane.zbin_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.v_zbin[qIndex];
        candidate_plane.round_QTX = picture_control_set_ptr->parent_pcs_ptr->quantsMd.v_round[qIndex];
        candidate_plane.dequant_QTX = picture_control_set_ptr->parent_pcs_ptr->deqMd.v_dequant_QTX[qIndex];
    }
    const ScanOrder *const scan_order = &av1_scan_orders[txsize][tx_type];

    const int32_t n_coeffs = av1_get_max_eob(txsize);

    QuantParam qparam;

    qparam.log_scale = av1_get_tx_scale(txsize);
    qparam.tx_size = txsize;
    qparam.qmatrix = qMatrix;
    qparam.iqmatrix = iqMatrix;
   // EbBool is_inter = (pred_mode >= NEARESTMV);

    av1_quantize_b_facade_II(
        (TranLow*)coeff,
        coeff_stride,
        width,
        height,
        n_coeffs,
        &candidate_plane,
        quant_coeff,
        (TranLow*)recon_coeff,
        eob,
        scan_order,
        &qparam);

    *count_non_zero_coeffs = *eob;
    // Derive cul_level
    int32_t cul_level = 0;
    const int16_t *const scan = scan_order->scan;
    for (int32_t c = 0; c < *eob; ++c) {
        const int16_t pos = scan[c];
        const int32_t v = quant_coeff[pos];
        int32_t level = ABS(v);
        cul_level += level;
    }
    cul_level = AOMMIN(COEFF_CONTEXT_MASK, cul_level);
    // DC value
    set_dc_sign(&cul_level, quant_coeff[0]);
    return cul_level;
}

 EbErrorType mdc_av1_tu_estimate_coeff_bits(
    struct ModeDecisionConfigurationContext *context,
    uint8_t                                  allow_update_cdf,
    FRAME_CONTEXT                           *ec_ctx,
    PictureControlSet                       *picture_control_set_ptr,
    struct ModeDecisionCandidateBuffer      *candidate_buffer_ptr,
    uint32_t                                 tu_origin_index,
    uint32_t                                 tu_chroma_origin_index,
    EntropyCoder                            *entropy_coder_ptr,
    EbPictureBufferDesc                     *coeff_buffer_sb,
    uint32_t                                 y_eob,
    uint32_t                                 cb_eob,
    uint32_t                                 cr_eob,
    uint64_t                                *y_tu_coeff_bits,
    uint64_t                                *cb_tu_coeff_bits,
    uint64_t                                *cr_tu_coeff_bits,
    TxSize                                   txsize,
    TxSize                                   txsize_uv,
    TxType                                   tx_type,
    TxType                                   tx_type_uv,
    COMPONENT_TYPE                           component_type,
    EbAsm                                    asm_type)
{

     UNUSED(tu_chroma_origin_index);
     UNUSED(entropy_coder_ptr);
     UNUSED(cb_eob);
     UNUSED(cr_eob);
     UNUSED(cb_tu_coeff_bits);
     UNUSED(cr_tu_coeff_bits);
     UNUSED(txsize_uv);
     UNUSED(tx_type_uv);
     UNUSED(asm_type);
    EbErrorType return_error = EB_ErrorNone;
    int32_t *coeff_buffer;
    int16_t  luma_txb_skip_context = context->luma_txb_skip_context;
    int16_t  luma_dc_sign_context  = context->luma_dc_sign_context;
    EbBool reducedTransformSetFlag = picture_control_set_ptr->parent_pcs_ptr->reduced_tx_set_used ? EB_TRUE : EB_FALSE;
    //Estimate the rate of the transform type and coefficient for Luma
    if (component_type == COMPONENT_LUMA || component_type == COMPONENT_ALL) {
        if (y_eob) {
            coeff_buffer = (int32_t*)&coeff_buffer_sb->buffer_y[tu_origin_index * sizeof(int32_t)];
            *y_tu_coeff_bits = av1_cost_coeffs_txb(
                allow_update_cdf,
                ec_ctx,
                candidate_buffer_ptr,
                coeff_buffer,
                (uint16_t)y_eob,
                PLANE_TYPE_Y,
                txsize,
                tx_type,
                luma_txb_skip_context,
                luma_dc_sign_context,
                reducedTransformSetFlag);
        }
        else {
            *y_tu_coeff_bits = av1_cost_skip_txb(
                allow_update_cdf,
                ec_ctx,
                candidate_buffer_ptr,
                txsize,
                PLANE_TYPE_Y,
                luma_txb_skip_context);
        }
    }
    return return_error;
}
void mdc_full_loop(
    ModeDecisionCandidateBuffer       *candidate_buffer,
    ModeDecisionConfigurationContext  *context_ptr,
    PictureControlSet                 *picture_control_set_ptr,
    uint32_t                           qp,
    uint32_t                          *y_count_non_zero_coeffs,
    uint64_t                          *y_coeff_bits,
    uint64_t                          *y_full_distortion)
{
    uint32_t                          tu_origin_index;
    uint64_t                          y_full_cost;
    SequenceControlSet                *sequence_control_set_ptr = (SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr;
    EbAsm                             asm_type = sequence_control_set_ptr->encode_context_ptr->asm_type;
    uint64_t                          y_tu_coeff_bits;
    uint64_t                          tuFullDistortion[3][DIST_CALC_TOTAL];
    context_ptr->three_quad_energy = 0;
    uint32_t  txb_1d_offset = 0;
    uint32_t txb_itr = 0;
    assert(asm_type >= 0 && asm_type < ASM_TYPE_TOTAL);
    uint8_t  tx_depth = candidate_buffer->candidate_ptr->tx_depth;
    uint16_t txb_count = context_ptr->blk_geom->txb_count[tx_depth];
    for (txb_itr = 0; txb_itr < txb_count; txb_itr++){
        uint16_t tx_org_x = context_ptr->blk_geom->tx_org_x[tx_depth][txb_itr];
        uint16_t tx_org_y = context_ptr->blk_geom->tx_org_y[tx_depth][txb_itr];
        int32_t cropped_tx_width = MIN(context_ptr->blk_geom->tx_width[tx_depth][txb_itr], sequence_control_set_ptr->seq_header.max_frame_width - (context_ptr->sb_origin_x + tx_org_x));
        int32_t cropped_tx_height = MIN(context_ptr->blk_geom->tx_height[tx_depth][txb_itr], sequence_control_set_ptr->seq_header.max_frame_height - (context_ptr->sb_origin_y + tx_org_y));
        context_ptr->luma_txb_skip_context = 0;
        context_ptr->luma_dc_sign_context = 0;
#if ADD_NEIGHBOR
        get_txb_ctx(
            sequence_control_set_ptr,
            COMPONENT_LUMA,
            context_ptr->mdc_luma_dc_sign_level_coeff_neighbor_array,
            context_ptr->sb_origin_x + tx_org_x,
            context_ptr->sb_origin_y + tx_org_y,
            context_ptr->blk_geom->bsize,
            context_ptr->blk_geom->txsize[tx_depth][txb_itr],
            &context_ptr->luma_txb_skip_context,
            &context_ptr->luma_dc_sign_context);
#endif


        tu_origin_index = tx_org_x + (tx_org_y * candidate_buffer->residual_ptr->stride_y);
        y_tu_coeff_bits = 0;

        // Y: T Q iQ
        av1_estimate_transform(
            &(((int16_t*)candidate_buffer->residual_ptr->buffer_y)[tu_origin_index]),
            candidate_buffer->residual_ptr->stride_y,
            &(((int32_t*)context_ptr->trans_quant_buffers_ptr->tu_trans_coeff2_nx2_n_ptr->buffer_y)[txb_1d_offset]),
            NOT_USED_VALUE,
            context_ptr->blk_geom->txsize[tx_depth][txb_itr],
            &context_ptr->three_quad_energy,
            context_ptr->transform_inner_array_ptr,
            0,
            candidate_buffer->candidate_ptr->transform_type[txb_itr],
            PLANE_TYPE_Y,
            DEFAULT_SHAPE);

        candidate_buffer->candidate_ptr->quantized_dc[0][txb_itr] = mdc_av1_quantize_inv_quantize(
            picture_control_set_ptr,
            context_ptr,
            &(((int32_t*)context_ptr->trans_quant_buffers_ptr->tu_trans_coeff2_nx2_n_ptr->buffer_y)[txb_1d_offset]),
            NOT_USED_VALUE,
            &(((int32_t*)candidate_buffer->residual_quant_coeff_ptr->buffer_y)[txb_1d_offset]),
            &(((int32_t*)candidate_buffer->recon_coeff_ptr->buffer_y)[txb_1d_offset]),
            qp,
            context_ptr->blk_geom->tx_width[tx_depth][txb_itr],
            context_ptr->blk_geom->tx_height[tx_depth][txb_itr],
            context_ptr->blk_geom->txsize[tx_depth][txb_itr],
            &candidate_buffer->candidate_ptr->eob[0][txb_itr],
            asm_type,
            &(y_count_non_zero_coeffs[txb_itr]),
            COMPONENT_LUMA,
            BIT_INCREMENT_8BIT,
            candidate_buffer->candidate_ptr->transform_type[txb_itr],
            candidate_buffer,
            context_ptr->luma_txb_skip_context,
            context_ptr->luma_dc_sign_context,
            candidate_buffer->candidate_ptr->pred_mode);

        if (context_ptr->spatial_sse_full_loop) {
            EbPictureBufferDesc          *input_picture_ptr = picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr;
            uint32_t input_tu_origin_index = (context_ptr->sb_origin_x + tx_org_x + input_picture_ptr->origin_x) + ((context_ptr->sb_origin_y + tx_org_y + input_picture_ptr->origin_y) * input_picture_ptr->stride_y);
            uint32_t y_has_coeff           = y_count_non_zero_coeffs[txb_itr] > 0;

            if (y_has_coeff) {
                (void)context_ptr;
                uint8_t     *pred_buffer = &(candidate_buffer->prediction_ptr->buffer_y[tu_origin_index]);
                uint8_t     *rec_buffer = &(candidate_buffer->recon_ptr->buffer_y[tu_origin_index]);
                uint32_t j;
                for (j = 0; j < context_ptr->blk_geom->tx_height[tx_depth][txb_itr]; j++)
                    memcpy(rec_buffer + j * candidate_buffer->recon_ptr->stride_y, pred_buffer + j * candidate_buffer->prediction_ptr->stride_y, context_ptr->blk_geom->tx_width[tx_depth][txb_itr]);

                av1_inv_transform_recon8bit(
                    &(((int32_t*)candidate_buffer->recon_coeff_ptr->buffer_y)[txb_1d_offset]),
                    rec_buffer,
                    candidate_buffer->recon_ptr->stride_y,
                    context_ptr->blk_geom->txsize[tx_depth][txb_itr],
                    candidate_buffer->candidate_ptr->transform_type[txb_itr],
                    PLANE_TYPE_Y,
                    (uint16_t)candidate_buffer->candidate_ptr->eob[0][txb_itr]);
            }
            else {
                picture_copy8_bit(
                    candidate_buffer->prediction_ptr,
                    tu_origin_index,
                    0,
                    candidate_buffer->recon_ptr,
                    tu_origin_index,
                    0,
                    context_ptr->blk_geom->tx_width[tx_depth][txb_itr],
                    context_ptr->blk_geom->tx_height[tx_depth][txb_itr],
                    0,
                    0,
                    PICTURE_BUFFER_DESC_Y_FLAG,
                    asm_type);
            }

            tuFullDistortion[0][DIST_CALC_PREDICTION] = spatial_full_distortion(
                input_picture_ptr->buffer_y + input_tu_origin_index,
                input_picture_ptr->stride_y,
                candidate_buffer->prediction_ptr->buffer_y + tu_origin_index,
                candidate_buffer->prediction_ptr->stride_y,
                cropped_tx_width,
                cropped_tx_height,
                Log2f(context_ptr->blk_geom->tx_width[tx_depth][txb_itr]) - 2);

            tuFullDistortion[0][DIST_CALC_RESIDUAL] = spatial_full_distortion(
                input_picture_ptr->buffer_y + input_tu_origin_index,
                input_picture_ptr->stride_y,
                &(((uint8_t*)candidate_buffer->recon_ptr->buffer_y)[tu_origin_index]),
                candidate_buffer->recon_ptr->stride_y,
                cropped_tx_width,
                cropped_tx_height,
                Log2f(context_ptr->blk_geom->tx_width[tx_depth][txb_itr]) - 2);

            tuFullDistortion[0][DIST_CALC_PREDICTION] <<= 4;
            tuFullDistortion[0][DIST_CALC_RESIDUAL] <<= 4;
        }
        else {
            // LUMA DISTORTION
            picture_full_distortion32_bits(
                context_ptr->trans_quant_buffers_ptr->tu_trans_coeff2_nx2_n_ptr,
                txb_1d_offset,
                0,
                candidate_buffer->recon_coeff_ptr,
                txb_1d_offset,
                0,
                context_ptr->blk_geom->tx_width[tx_depth][txb_itr],
                context_ptr->blk_geom->tx_height[tx_depth][txb_itr],
                NOT_USED_VALUE,
                NOT_USED_VALUE,
                tuFullDistortion[0],
                NOT_USED_VALUE,
                NOT_USED_VALUE,
                y_count_non_zero_coeffs[txb_itr],
                0,
                0,
                COMPONENT_LUMA);

            tuFullDistortion[0][DIST_CALC_RESIDUAL] += context_ptr->three_quad_energy;
            tuFullDistortion[0][DIST_CALC_PREDICTION] += context_ptr->three_quad_energy;
            //assert(context_ptr->three_quad_energy == 0 && context_ptr->cu_stats->size < 64);
            TxSize tx_size = context_ptr->blk_geom->txsize[tx_depth][txb_itr];
            int32_t shift = (MAX_TX_SCALE - av1_get_tx_scale(tx_size)) * 2;
            tuFullDistortion[0][DIST_CALC_RESIDUAL] = RIGHT_SIGNED_SHIFT(tuFullDistortion[0][DIST_CALC_RESIDUAL], shift);
            tuFullDistortion[0][DIST_CALC_PREDICTION] = RIGHT_SIGNED_SHIFT(tuFullDistortion[0][DIST_CALC_PREDICTION], shift);
        }
        //LUMA-ONLY
        mdc_av1_tu_estimate_coeff_bits(
            context_ptr,
            0,//allow_update_cdf,
            NULL,//FRAME_CONTEXT *ec_ctx,
            picture_control_set_ptr,
            candidate_buffer,
            txb_1d_offset,
            0,
            context_ptr->coeff_est_entropy_coder_ptr,
            candidate_buffer->residual_quant_coeff_ptr,
            y_count_non_zero_coeffs[txb_itr],
            0,
            0,
            &y_tu_coeff_bits,
            &y_tu_coeff_bits,
            &y_tu_coeff_bits,
            context_ptr->blk_geom->txsize[tx_depth][txb_itr],
            context_ptr->blk_geom->txsize_uv[tx_depth][txb_itr],
            candidate_buffer->candidate_ptr->transform_type[txb_itr],
            candidate_buffer->candidate_ptr->transform_type_uv,
            COMPONENT_LUMA,
            asm_type);

        av1_tu_calc_cost_luma(
            context_ptr->luma_txb_skip_context,
            candidate_buffer->candidate_ptr,
            txb_itr,
            context_ptr->blk_geom->txsize[tx_depth][0],
            y_count_non_zero_coeffs[txb_itr],
            tuFullDistortion[0],      //gets updated inside based on cbf decision
            &y_tu_coeff_bits,            //gets updated inside based on cbf decision
            &y_full_cost,
            context_ptr->full_lambda);

        (*y_coeff_bits) += y_tu_coeff_bits;
        y_full_distortion[DIST_CALC_RESIDUAL] += tuFullDistortion[0][DIST_CALC_RESIDUAL];
        y_full_distortion[DIST_CALC_PREDICTION] += tuFullDistortion[0][DIST_CALC_PREDICTION];
        txb_1d_offset += context_ptr->blk_geom->tx_width[tx_depth][txb_itr] * context_ptr->blk_geom->tx_height[tx_depth][txb_itr];
    }
}
extern void av1_set_ref_frame(MvReferenceFrame *rf,
    int8_t ref_frame_type);
EbErrorType mdc_inter_pu_prediction_av1(
    ModeDecisionConfigurationContext     *context_ptr,
    PictureControlSet                    *picture_control_set_ptr,
    ModeDecisionCandidateBuffer          *candidate_buffer_ptr,
    EbAsm                                   asm_type)
{
    UNUSED(asm_type);
    picture_control_set_ptr;
    EbErrorType           return_error = EB_ErrorNone;
    EbPictureBufferDesc  *ref_pic_list0;
    EbPictureBufferDesc  *ref_pic_list1 = NULL;
    //ModeDecisionCandidate *const candidate_ptr = candidate_buffer_ptr->candidate_ptr;

    Mv mv_0;
    Mv mv_1;
    mv_0.x = candidate_buffer_ptr->candidate_ptr->motion_vector_xl0;
    mv_0.y = candidate_buffer_ptr->candidate_ptr->motion_vector_yl0;
    mv_1.x = candidate_buffer_ptr->candidate_ptr->motion_vector_xl1;
    mv_1.y = candidate_buffer_ptr->candidate_ptr->motion_vector_yl1;
    MvUnit mv_unit;
    mv_unit.pred_direction = candidate_buffer_ptr->candidate_ptr->prediction_direction[0];
    mv_unit.mv[0] = mv_0;
    mv_unit.mv[1] = mv_1;
    //SequenceControlSet* sequence_control_set_ptr = ((SequenceControlSet*)(picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr));
     int8_t ref_idx_l0 = candidate_buffer_ptr->candidate_ptr->ref_frame_index_l0;
     int8_t ref_idx_l1 = candidate_buffer_ptr->candidate_ptr->ref_frame_index_l1;
     // MRP_MD_UNI_DIR_BIPRED
     MvReferenceFrame rf[2];
     av1_set_ref_frame(rf, candidate_buffer_ptr->candidate_ptr->ref_frame_type);
     uint8_t list_idx0, list_idx1;
     list_idx0 = get_list_idx(rf[0]);
     if (rf[1] == NONE_FRAME)
         list_idx1 = get_list_idx(rf[0]);
     else
         list_idx1 = get_list_idx(rf[1]);
     assert(list_idx0 < MAX_NUM_OF_REF_PIC_LIST);
     assert(list_idx1 < MAX_NUM_OF_REF_PIC_LIST);
     if (ref_idx_l0 >= 0)
         ref_pic_list0 = ((EbReferenceObject*)picture_control_set_ptr->ref_pic_ptr_array[list_idx0][ref_idx_l0]->object_ptr)->reference_picture;
     else
         ref_pic_list0 = (EbPictureBufferDesc*)EB_NULL;
     if (ref_idx_l1 >= 0)
         ref_pic_list1 = ((EbReferenceObject*)picture_control_set_ptr->ref_pic_ptr_array[list_idx1][ref_idx_l1]->object_ptr)->reference_picture;
     else
         ref_pic_list1 = (EbPictureBufferDesc*)EB_NULL;

     candidate_buffer_ptr->candidate_ptr->interp_filters = 0;

     av1_inter_prediction(
         picture_control_set_ptr,
         candidate_buffer_ptr->candidate_ptr->interp_filters,
         context_ptr->mdc_cu_ptr,
         candidate_buffer_ptr->candidate_ptr->ref_frame_type,
         &mv_unit,
         candidate_buffer_ptr->candidate_ptr->use_intrabc,
         candidate_buffer_ptr->candidate_ptr->compound_idx,
         &candidate_buffer_ptr->candidate_ptr->interinter_comp,
#if II_ED
        NULL,
        NULL,//ep_luma_recon_neighbor_array,
        NULL,//ep_cb_recon_neighbor_array ,
        NULL,//ep_cr_recon_neighbor_array ,
        0,//cu_ptr->is_interintra_used,
        0,//cu_ptr->interintra_mode,
        0,//cu_ptr->use_wedge_interintra,
        0,//cu_ptr->interintra_wedge_index,
#endif
         context_ptr->cu_origin_x,
         context_ptr->cu_origin_y,
         context_ptr->blk_geom->bwidth,
         context_ptr->blk_geom->bheight,
         ref_pic_list0,
         ref_pic_list1,
         candidate_buffer_ptr->prediction_ptr,
         context_ptr->blk_geom->origin_x,
         context_ptr->blk_geom->origin_y,
         0); // No chroma



    return return_error;
}
#if ADD_NEIGHBOR
void mdc_update_neighbor_arrays(
    PictureControlSet     *picture_control_set_ptr,
    ModeDecisionConfigurationContext   *context_ptr,
    uint32_t               index_mds,
    EbBool                 intraMdOpenLoop){
    uint32_t bwdith = context_ptr->blk_geom->bwidth;
    uint32_t bheight = context_ptr->blk_geom->bheight;
    uint32_t origin_x = context_ptr->cu_origin_x;
    uint32_t origin_y = context_ptr->cu_origin_y;
    uint32_t cu_origin_x_uv = context_ptr->round_origin_x >> 1;
    uint32_t cu_origin_y_uv = context_ptr->round_origin_y >> 1;
    uint32_t bwdith_uv = context_ptr->blk_geom->bwidth_uv;
    uint32_t bwheight_uv = context_ptr->blk_geom->bheight_uv;
//    uint8_t modeType = context_ptr->cu_ptr->prediction_mode_flag;
 //   uint8_t intra_luma_mode = (uint8_t)context_ptr->cu_ptr->pred_mode;
  //  uint8_t chroma_mode = (uint8_t)context_ptr->cu_ptr->prediction_unit_array->intra_chroma_mode;
  //  uint8_t skip_flag = (uint8_t)context_ptr->cu_ptr->skip_flag;

   // context_ptr->mv_unit.pred_direction = (uint8_t)(context_ptr->md_cu_arr_nsq[index_mds].prediction_unit_array[0].inter_pred_direction_index);
  //  context_ptr->mv_unit.mv[REF_LIST_0].mv_union = context_ptr->md_cu_arr_nsq[index_mds].prediction_unit_array[0].mv[REF_LIST_0].mv_union;
  //  context_ptr->mv_unit.mv[REF_LIST_1].mv_union = context_ptr->md_cu_arr_nsq[index_mds].prediction_unit_array[0].mv[REF_LIST_1].mv_union;
#if ATB_DC_CONTEXT_SUPPORT_1
#if !DC_SIGN_CONTEXT_FIX
    uint8_t u_has_coeff = context_ptr->cu_ptr->transform_unit_array[0].u_has_coeff;
    int32_t cbDcCoeff = (int32_t)context_ptr->cu_ptr->quantized_dc[1][0];
    uint8_t v_has_coeff = context_ptr->cu_ptr->transform_unit_array[0].v_has_coeff;
    int32_t crDcCoeff = (int32_t)context_ptr->cu_ptr->quantized_dc[2][0];
#endif
#else
    uint8_t                    y_has_coeff = context_ptr->cu_ptr->transform_unit_array[0].y_has_coeff;
    int32_t                   lumaDcCoeff = (int32_t)context_ptr->cu_ptr->quantized_dc[0];
    uint8_t                    u_has_coeff = context_ptr->cu_ptr->transform_unit_array[0].u_has_coeff;
    int32_t                   cbDcCoeff = (int32_t)context_ptr->cu_ptr->quantized_dc[1];
    uint8_t                    v_has_coeff = context_ptr->cu_ptr->transform_unit_array[0].v_has_coeff;
    int32_t                   crDcCoeff = (int32_t)context_ptr->cu_ptr->quantized_dc[2];
#endif
   // uint8_t                    inter_pred_direction_index = (uint8_t)context_ptr->cu_ptr->prediction_unit_array->inter_pred_direction_index;
  //  uint8_t                    ref_frame_type = (uint8_t)context_ptr->cu_ptr->prediction_unit_array[0].ref_frame_type;
    uint8_t                    tx_depth = 0;
    uint16_t txb_count = context_ptr->blk_geom->txb_count[tx_depth];
    for (uint8_t txb_itr = 0; txb_itr < txb_count; txb_itr++){
        uint8_t dc_sign_level_coeff = (int32_t)context_ptr->mdc_cu_ptr->quantized_dc[0][txb_itr];
        neighbor_array_unit_mode_write(
            context_ptr->mdc_luma_dc_sign_level_coeff_neighbor_array,
            (uint8_t*)&dc_sign_level_coeff,
            context_ptr->sb_origin_x + context_ptr->blk_geom->tx_org_x[tx_depth][txb_itr],
            context_ptr->sb_origin_y + context_ptr->blk_geom->tx_org_y[tx_depth][txb_itr],
            context_ptr->blk_geom->tx_width[tx_depth][txb_itr],
            context_ptr->blk_geom->tx_height[tx_depth][txb_itr],
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }
#if 0

#if OPT_LOSSLESS_0
    if (picture_control_set_ptr->parent_pcs_ptr->interpolation_search_level != IT_SEARCH_OFF)
#endif
    neighbor_array_unit_mode_write32(
        context_ptr->interpolation_type_neighbor_array,
        context_ptr->cu_ptr->interp_filters,
        origin_x,
        origin_y,
        bwdith,
        bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    {
        struct PartitionContext partition;
        partition.above = partition_context_lookup[context_ptr->blk_geom->bsize].above;
        partition.left = partition_context_lookup[context_ptr->blk_geom->bsize].left;

        neighbor_array_unit_mode_write(
            context_ptr->leaf_partition_neighbor_array,
            (uint8_t*)(&partition), // NaderM
            origin_x,
            origin_y,
            bwdith,
            bheight,
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

        // Mode Type Update
        neighbor_array_unit_mode_write(
            context_ptr->mode_type_neighbor_array,
            &modeType,
            origin_x,
            origin_y,
            bwdith,
            bheight,
            NEIGHBOR_ARRAY_UNIT_FULL_MASK);
#if OPT_LOSSLESS_0
        if (picture_control_set_ptr->parent_pcs_ptr->skip_sub_blks)
#endif
#if M8_SKIP_BLK
        // Intra Luma Mode Update
        neighbor_array_unit_mode_write(
            context_ptr->leaf_depth_neighbor_array,
            (uint8_t*)&context_ptr->blk_geom->bsize,//(uint8_t*)luma_mode,
            origin_x,
            origin_y,
            bwdith,
            bheight,
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#endif
        // Intra Luma Mode Update
        neighbor_array_unit_mode_write(
            context_ptr->intra_luma_mode_neighbor_array,
            &intra_luma_mode,//(uint8_t*)luma_mode,
            origin_x,
            origin_y,
            bwdith,
            bheight,
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);



    // Hsan: chroma mode rate estimation is kept even for chroma blind
    if (context_ptr->blk_geom->has_uv) {
        // Intra Chroma Mode Update
        neighbor_array_unit_mode_write(
            context_ptr->intra_chroma_mode_neighbor_array,
            &chroma_mode,
            cu_origin_x_uv,
            cu_origin_y_uv,
            bwdith_uv,
            bwheight_uv,
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }

    neighbor_array_unit_mode_write(
        context_ptr->skip_flag_neighbor_array,
        &skip_flag,
        origin_x,
        origin_y,
        bwdith,
        bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

#if !OPT_LOSSLESS_0
    //  Update skip_coeff_neighbor_array,
    neighbor_array_unit_mode_write(
        context_ptr->skip_coeff_neighbor_array,
        &skipCoeff,
        origin_x,
        origin_y,
        bwdith,
        bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#endif

    if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
        //  Update chroma CB cbf and Dc context
        {
#if DC_SIGN_CONTEXT_FIX
            uint8_t dc_sign_level_coeff = (int32_t)context_ptr->cu_ptr->quantized_dc[1][0];
#else
            uint8_t dcSignCtx = 0;
            if (cbDcCoeff > 0)
                dcSignCtx = 2;
            else if (cbDcCoeff < 0)
                dcSignCtx = 1;
            else
                dcSignCtx = 0;
            uint8_t dc_sign_level_coeff = (uint8_t)((dcSignCtx << COEFF_CONTEXT_BITS) | u_has_coeff);
            if (!u_has_coeff)
                dc_sign_level_coeff = 0;
#endif
            neighbor_array_unit_mode_write(
                context_ptr->cb_dc_sign_level_coeff_neighbor_array,
                (uint8_t*)&dc_sign_level_coeff,
                cu_origin_x_uv,
                cu_origin_y_uv,
                bwdith_uv,
                bwheight_uv,
                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        }

        //  Update chroma CR cbf and Dc context
        {
#if DC_SIGN_CONTEXT_FIX
            uint8_t dc_sign_level_coeff = (int32_t)context_ptr->cu_ptr->quantized_dc[2][0];
#else
            uint8_t dcSignCtx = 0;
            if (crDcCoeff > 0)
                dcSignCtx = 2;
            else if (crDcCoeff < 0)
                dcSignCtx = 1;
            else
                dcSignCtx = 0;
            uint8_t dc_sign_level_coeff = (uint8_t)((dcSignCtx << COEFF_CONTEXT_BITS) | v_has_coeff);
            if (!v_has_coeff)
                dc_sign_level_coeff = 0;
#endif
            neighbor_array_unit_mode_write(
                context_ptr->cr_dc_sign_level_coeff_neighbor_array,
                (uint8_t*)&dc_sign_level_coeff,
                cu_origin_x_uv,
                cu_origin_y_uv,
                bwdith_uv,
                bwheight_uv,
                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        }
    }

#if ATB_RATE
    neighbor_array_unit_mode_write(
        context_ptr->txfm_context_array,
        &context_ptr->cu_ptr->tx_depth,
        origin_x,
        origin_y,
        bwdith,
        bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#endif

    // Update the Inter Pred Type Neighbor Array

    neighbor_array_unit_mode_write(
        context_ptr->inter_pred_dir_neighbor_array,
        &inter_pred_direction_index,
        origin_x,
        origin_y,
        bwdith,
        bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    // Update the refFrame Type Neighbor Array
    neighbor_array_unit_mode_write(
        context_ptr->ref_frame_type_neighbor_array,
        &ref_frame_type,
        origin_x,
        origin_y,
        bwdith,
        bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    if (intraMdOpenLoop == EB_FALSE)
    {
        update_recon_neighbor_array(
            context_ptr->luma_recon_neighbor_array,
            context_ptr->cu_ptr->neigh_top_recon[0],
            context_ptr->cu_ptr->neigh_left_recon[0],
            origin_x,
            origin_y,
            context_ptr->blk_geom->bwidth,
            context_ptr->blk_geom->bheight);
#if ATB_MD
        if (picture_control_set_ptr->parent_pcs_ptr->atb_mode) {
            update_recon_neighbor_array(
                picture_control_set_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
                context_ptr->cu_ptr->neigh_top_recon[0],
                context_ptr->cu_ptr->neigh_left_recon[0],
                origin_x,
                origin_y,
                context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight);
        }
#endif
    }

    if (intraMdOpenLoop == EB_FALSE) {
        if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
            update_recon_neighbor_array(
                context_ptr->cb_recon_neighbor_array,
                context_ptr->cu_ptr->neigh_top_recon[1],
                context_ptr->cu_ptr->neigh_left_recon[1],
                cu_origin_x_uv,
                cu_origin_y_uv,
                bwdith_uv,
                bwheight_uv);
            update_recon_neighbor_array(
                context_ptr->cr_recon_neighbor_array,
                context_ptr->cu_ptr->neigh_top_recon[2],
                context_ptr->cu_ptr->neigh_left_recon[2],
                cu_origin_x_uv,
                cu_origin_y_uv,
                bwdith_uv,
                bwheight_uv);
        }
    }
#endif
    return;
}

void mdc_update_all_neighbour_arrays(
    PictureControlSet                *picture_control_set_ptr,
    ModeDecisionConfigurationContext *context_ptr,
    uint32_t                          lastCuIndex_mds,
    uint32_t                          sb_origin_x,
    uint32_t                          sb_origin_y)
{
    context_ptr->blk_geom = get_blk_geom_mds(lastCuIndex_mds);
    context_ptr->cu_origin_x = sb_origin_x + context_ptr->blk_geom->origin_x;
    context_ptr->cu_origin_y = sb_origin_y + context_ptr->blk_geom->origin_y;
    context_ptr->round_origin_x = ((context_ptr->cu_origin_x >> 3) << 3);
    context_ptr->round_origin_y = ((context_ptr->cu_origin_y >> 3) << 3);

    context_ptr->cu_ptr = &context_ptr->local_cu_array[lastCuIndex_mds];

    mdc_update_neighbor_arrays(
        picture_control_set_ptr,
        context_ptr,
        lastCuIndex_mds,
        picture_control_set_ptr->intra_md_open_loop_flag);
#if 0
    update_mi_map(
        context_ptr,
        context_ptr->cu_ptr,
        context_ptr->cu_origin_x,
        context_ptr->cu_origin_y,
        context_ptr->blk_geom,
        0,
        picture_control_set_ptr);
#endif
}

void mdc_update_all_neighbour_arrays_multiple(
    PictureControlSet                *picture_control_set_ptr,
    ModeDecisionConfigurationContext               *context_ptr,
    uint32_t                            blk_mds,
    uint32_t                            sb_origin_x,
    uint32_t                            sb_origin_y){
    context_ptr->blk_geom = get_blk_geom_mds(blk_mds);

    uint32_t blk_it;
    for (blk_it = 0; blk_it < context_ptr->blk_geom->totns; blk_it++)
    {
        mdc_update_all_neighbour_arrays(
            picture_control_set_ptr,
            context_ptr,
            blk_mds + blk_it,
            sb_origin_x,
            sb_origin_y);
    }
}
void mdc_copy_neighbour_arrays(
    PictureControlSet                   *picture_control_set_ptr,
    ModeDecisionConfigurationContext    *context_ptr,
    uint32_t                            src_idx,
    uint32_t                            dst_idx,
    uint32_t                            blk_mds,
    uint32_t                            sb_org_x,
    uint32_t                            sb_org_y)
{
    (void)*context_ptr;

    const BlockGeom * blk_geom = get_blk_geom_mds(blk_mds);

    uint32_t                            blk_org_x = sb_org_x + blk_geom->origin_x;
    uint32_t                            blk_org_y = sb_org_y + blk_geom->origin_y;
    uint32_t                            blk_org_x_uv = (blk_org_x >> 3 << 3) >> 1;
    uint32_t                            blk_org_y_uv = (blk_org_y >> 3 << 3) >> 1;
    uint32_t                            bwidth_uv = blk_geom->bwidth_uv;
    uint32_t                            bheight_uv = blk_geom->bheight_uv;

    //neighbor_array_unit_reset(picture_control_set_ptr->md_luma_dc_sign_level_coeff_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->mdc_luma_dc_sign_level_coeff_neighbor_array[src_idx],
        picture_control_set_ptr->mdc_luma_dc_sign_level_coeff_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#if 0

    copy_neigh_arr(
        picture_control_set_ptr->md_intra_luma_mode_neighbor_array[src_idx],
        picture_control_set_ptr->md_intra_luma_mode_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    //neighbor_array_unit_reset(picture_control_set_ptr->md_intra_chroma_mode_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_intra_chroma_mode_neighbor_array[src_idx],
        picture_control_set_ptr->md_intra_chroma_mode_neighbor_array[dst_idx],
        blk_org_x_uv,
        blk_org_y_uv,
        bwidth_uv,
        bheight_uv,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    //neighbor_array_unit_reset(picture_control_set_ptr->md_skip_flag_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_skip_flag_neighbor_array[src_idx],
        picture_control_set_ptr->md_skip_flag_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    //neighbor_array_unit_reset(picture_control_set_ptr->md_mode_type_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_mode_type_neighbor_array[src_idx],
        picture_control_set_ptr->md_mode_type_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_FULL_MASK);

    //neighbor_array_unit_reset(picture_control_set_ptr->md_leaf_depth_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_leaf_depth_neighbor_array[src_idx],
        picture_control_set_ptr->md_leaf_depth_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    copy_neigh_arr(
        picture_control_set_ptr->mdleaf_partition_neighbor_array[src_idx],
        picture_control_set_ptr->mdleaf_partition_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    //neighbor_array_unit_reset(picture_control_set_ptr->md_luma_recon_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_luma_recon_neighbor_array[src_idx],
        picture_control_set_ptr->md_luma_recon_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_FULL_MASK);

#if ATB_MD
    if (picture_control_set_ptr->parent_pcs_ptr->atb_mode) {
        copy_neigh_arr(
            picture_control_set_ptr->md_tx_depth_1_luma_recon_neighbor_array[src_idx],
            picture_control_set_ptr->md_tx_depth_1_luma_recon_neighbor_array[dst_idx],
            blk_org_x,
            blk_org_y,
            blk_geom->bwidth,
            blk_geom->bheight,
            NEIGHBOR_ARRAY_UNIT_FULL_MASK);
    }
#endif

    if (blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
        //neighbor_array_unit_reset(picture_control_set_ptr->md_cb_recon_neighbor_array[depth]);

        copy_neigh_arr(
            picture_control_set_ptr->md_cb_recon_neighbor_array[src_idx],
            picture_control_set_ptr->md_cb_recon_neighbor_array[dst_idx],
            blk_org_x_uv,
            blk_org_y_uv,
            bwidth_uv,
            bheight_uv,
            NEIGHBOR_ARRAY_UNIT_FULL_MASK);

        //neighbor_array_unit_reset(picture_control_set_ptr->md_cr_recon_neighbor_array[depth]);
        copy_neigh_arr(
            picture_control_set_ptr->md_cr_recon_neighbor_array[src_idx],
            picture_control_set_ptr->md_cr_recon_neighbor_array[dst_idx],
            blk_org_x_uv,
            blk_org_y_uv,
            bwidth_uv,
            bheight_uv,
            NEIGHBOR_ARRAY_UNIT_FULL_MASK);
    }
#if !REMOVE_SKIP_COEFF_NEIGHBOR_ARRAY
    //neighbor_array_unit_reset(picture_control_set_ptr->md_skip_coeff_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_skip_coeff_neighbor_array[src_idx],
        picture_control_set_ptr->md_skip_coeff_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#endif
    //neighbor_array_unit_reset(picture_control_set_ptr->md_luma_dc_sign_level_coeff_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_luma_dc_sign_level_coeff_neighbor_array[src_idx],
        picture_control_set_ptr->md_luma_dc_sign_level_coeff_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

#if ATB_DC_CONTEXT_SUPPORT_2
    copy_neigh_arr(
        picture_control_set_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[src_idx],
        picture_control_set_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#endif

    if (blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
        copy_neigh_arr(
            picture_control_set_ptr->md_cb_dc_sign_level_coeff_neighbor_array[src_idx],
            picture_control_set_ptr->md_cb_dc_sign_level_coeff_neighbor_array[dst_idx],
            blk_org_x_uv,
            blk_org_y_uv,
            bwidth_uv,
            bheight_uv,
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        //neighbor_array_unit_reset(picture_control_set_ptr->md_cr_dc_sign_level_coeff_neighbor_array[depth]);

        copy_neigh_arr(
            picture_control_set_ptr->md_cr_dc_sign_level_coeff_neighbor_array[src_idx],
            picture_control_set_ptr->md_cr_dc_sign_level_coeff_neighbor_array[dst_idx],
            blk_org_x_uv,
            blk_org_y_uv,
            bwidth_uv,
            bheight_uv,
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }

#if ATB_RATE
    //neighbor_array_unit_reset(picture_control_set_ptr->md_txfm_context_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_txfm_context_array[src_idx],
        picture_control_set_ptr->md_txfm_context_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#endif
    //neighbor_array_unit_reset(picture_control_set_ptr->md_inter_pred_dir_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_inter_pred_dir_neighbor_array[src_idx],
        picture_control_set_ptr->md_inter_pred_dir_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    //neighbor_array_unit_reset(picture_control_set_ptr->md_ref_frame_type_neighbor_array[depth]);
    copy_neigh_arr(
        picture_control_set_ptr->md_ref_frame_type_neighbor_array[src_idx],
        picture_control_set_ptr->md_ref_frame_type_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    copy_neigh_arr_32(
        picture_control_set_ptr->md_interpolation_type_neighbor_array[src_idx],
        picture_control_set_ptr->md_interpolation_type_neighbor_array[dst_idx],
        blk_org_x,
        blk_org_y,
        blk_geom->bwidth,
        blk_geom->bheight,
        NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#endif
}
#endif
uint64_t mdc_av1_full_cost(
    ModeDecisionConfigurationContext     *context_ptr,
    uint64_t                             *y_distortion,
    uint64_t                             *y_coeff_bits,
    uint64_t                              lambda){

    //EbErrorType return_error = EB_ErrorNone;
    // Luma and chroma rate
    uint64_t lumaRate = 0;
    uint64_t coeffRate = 0;

    // Luma and chroma SSE
    uint64_t luma_sse;
    uint64_t totalDistortion;
    uint64_t rate;

    lumaRate += context_ptr->candidate_buffer->candidate_ptr->fast_luma_rate;

    // Coeff rate
    coeffRate = (*y_coeff_bits);
    luma_sse = y_distortion[0];
    totalDistortion = luma_sse;
    rate = lumaRate + coeffRate;
    // Assign full cost
    uint64_t full_cost = RDCOST(lambda, rate, totalDistortion);

    return full_cost;
}
#endif
EB_EXTERN EbErrorType nsq_prediction_shape(
    SequenceControlSet                *sequence_control_set_ptr,
    PictureControlSet                 *picture_control_set_ptr,
    ModeDecisionConfigurationContext  *context_ptr,
    MdcLcuData                        *mdcResultTbPtr,
    LargestCodingUnit                 *sb_ptr,
    uint32_t                           sb_originx,
    uint32_t                           sb_originy,
    uint32_t                           sb_index) {
    EbErrorType                          return_error = EB_ErrorNone;

    uint32_t                                cuIdx;
    uint32_t leaf_idx;
    uint32_t start_idx, end_idx;
    //ModeDecisionCandidateBuffer            *bestCandidateBuffers[5];

    uint32_t                                leaf_count = mdcResultTbPtr->leaf_count;
    EbMdcLeafData *              leaf_data_array = mdcResultTbPtr->leaf_data_array;
    MdcpLocalCodingUnit *local_cu_array = context_ptr->local_cu_array;

    MdcpLocalCodingUnit   *cu_ptr;
    //CU Loop
    cuIdx = 0;  //index over mdc array
    start_idx = 0;
    uint64_t nsq_cost[NUMBER_OF_SHAPES] = { MAX_CU_COST, MAX_CU_COST,MAX_CU_COST,MAX_CU_COST,MAX_CU_COST,
        MAX_CU_COST, MAX_CU_COST,MAX_CU_COST,MAX_CU_COST,MAX_CU_COST };
    PART nsq_shape_table[NUMBER_OF_SHAPES] = { PART_N, PART_H, PART_V, PART_HA, PART_HB,
        PART_VA, PART_VB, PART_H4, PART_V4, PART_S };
    uint32_t blk_idx_mds = 0;
    uint32_t  d1_blocks_accumlated = 0;
#if DEPTH_RANKING
    uint64_t depth_cost[NUMBER_OF_DEPTH] = { 0 };
    uint8_t depth_table[NUMBER_OF_DEPTH] = { 0, 1, 2 , 3 ,4 ,5 };
#endif
#if ADD_SAD_FOR_128X128
    uint64_t me_128x128 = 0;
#endif
#if ADD_MDC_FULL_COST
    context_ptr->coeff_est_entropy_coder_ptr = picture_control_set_ptr->coeff_est_entropy_coder_ptr;
#endif
    uint64_t tot_me_sb;

    do {
        EbMdcLeafData * leaf_data_ptr = &mdcResultTbPtr->leaf_data_array[cuIdx];
        blk_idx_mds = leaf_data_array[cuIdx].mds_idx;
        context_ptr->mds_idx = blk_idx_mds;
        const BlockGeom * blk_geom = context_ptr->blk_geom = get_blk_geom_mds(blk_idx_mds);
        uint32_t cu_origin_x = sb_originx + blk_geom->origin_x;
        uint32_t cu_origin_y = sb_originy + blk_geom->origin_y;
        if (!(cu_origin_x < sequence_control_set_ptr->seq_header.max_frame_width && cu_origin_y < sequence_control_set_ptr->seq_header.max_frame_height))
        {
            cuIdx++;
            continue;
        }
        cu_ptr = &local_cu_array[cuIdx];

#if ADD_MDC_FULL_COST
        context_ptr->round_origin_x = ((context_ptr->cu_origin_x >> 3) << 3);
        context_ptr->round_origin_y = ((context_ptr->cu_origin_y >> 3) << 3);
        EbPictureBufferDesc  *input_picture_ptr = picture_control_set_ptr->parent_pcs_ptr->enhanced_picture_ptr;
        context_ptr->cu_size_log2 = blk_geom->bwidth_log2;
        context_ptr->cu_origin_x = sb_originx + blk_geom->origin_x;
        context_ptr->cu_origin_y = sb_originy + blk_geom->origin_y;
        context_ptr->sb_origin_x = sb_originx;
        context_ptr->sb_origin_y = sb_originy;
        uint64_t      y_full_distortion[DIST_CALC_TOTAL] = { 0 };
        uint32_t      count_non_zero_coeffs[MAX_NUM_OF_TU_PER_CU] = { 0 };
        uint64_t      y_coeff_bits = 0;
        const uint32_t       input_origin_index = (context_ptr->cu_origin_y + input_picture_ptr->origin_y) * input_picture_ptr->stride_y + (context_ptr->cu_origin_x + input_picture_ptr->origin_x);
        const uint32_t       cu_origin_index = blk_geom->origin_x + blk_geom->origin_y * SB_STRIDE_Y;

        context_ptr->candidate_buffer->candidate_ptr = &context_ptr->fast_candidate_array[0];
        EbAsm asm_type = sequence_control_set_ptr->encode_context_ptr->asm_type;
        cu_ptr->best_d1_blk = blk_idx_mds;
#if ADD_NEIGHBOR
        if (leaf_data_ptr->tot_d1_blocks != 1) {
            if (blk_geom->shape == PART_N) {
                mdc_copy_neighbour_arrays(
                    picture_control_set_ptr,
                    context_ptr,
                    0,//src_idx,
                    1,//dst_idx,
                    blk_idx_mds,
                    sb_originx,
                    sb_originy);
            }
        }
#endif
#endif
        if (picture_control_set_ptr->slice_type != I_SLICE) {
            uint32_t geom_offset_x = 0;
            uint32_t geom_offset_y = 0;
            uint32_t me_sb_addr;
            if (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128) {
                uint32_t me_sb_size = sequence_control_set_ptr->sb_sz;
                uint32_t me_pic_width_in_sb = (sequence_control_set_ptr->seq_header.max_frame_width + sequence_control_set_ptr->sb_sz - 1) / me_sb_size;
                uint32_t me_pic_height_in_sb = (sequence_control_set_ptr->seq_header.max_frame_height + sequence_control_set_ptr->sb_sz - 1) / me_sb_size;
                tot_me_sb = me_pic_width_in_sb * me_pic_height_in_sb;
                uint32_t me_sb_x = (cu_origin_x / me_sb_size);
                uint32_t me_sb_y = (cu_origin_y / me_sb_size);
                me_sb_addr = me_sb_x + me_sb_y * me_pic_width_in_sb;
                geom_offset_x = (me_sb_x & 0x1) * me_sb_size;
                geom_offset_y = (me_sb_y & 0x1) * me_sb_size;
#if ADD_SAD_FOR_128X128
                uint64_t sb_6x6_index;
                uint64_t sb_6x6_dist_0 = 0;
                uint64_t sb_6x6_dist_1 = 0;
                uint64_t sb_6x6_dist_2 = 0;
                uint64_t sb_6x6_dist_3 = 0;
                if (blk_geom->sq_size == 128) {
                    sb_6x6_index = me_sb_addr;
                    SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_6x6_index];
                    if (sb_params->is_complete_sb) {
                        MeLcuResults *me_results_64x64 = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_6x6_index];
                        const MeCandidate *me_block_results_64x64 = me_results_64x64->me_candidate[0];
                        sb_6x6_dist_0 = me_block_results_64x64[0].distortion;
                    }
                    if (blk_geom->bsize == BLOCK_128X128 || blk_geom->bsize == BLOCK_128X64) {
                        sb_6x6_index = MIN(tot_me_sb - 1,me_sb_addr + 1);
                        SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_6x6_index];
                        if (sb_params->is_complete_sb) {
                            MeLcuResults *me_results_64x64 = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_6x6_index];
                            const MeCandidate *me_block_results_64x64 = me_results_64x64->me_candidate[0];
                            sb_6x6_dist_1 = me_block_results_64x64[0].distortion;
                        }
                    }
                    if (blk_geom->bsize == BLOCK_128X128 || blk_geom->bsize == BLOCK_64X128) {
                        sb_6x6_index = MIN(tot_me_sb - 1, me_sb_addr + me_pic_width_in_sb);

                        SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_6x6_index];
                        if (sb_params->is_complete_sb) {
                            MeLcuResults *me_results_64x64 = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_6x6_index];
                            const MeCandidate *me_block_results_64x64 = me_results_64x64->me_candidate[0];
                            sb_6x6_dist_2 = me_block_results_64x64[0].distortion;
                        }
                    }
                    if (blk_geom->bsize == BLOCK_128X128) {
                        sb_6x6_index = MIN(tot_me_sb - 1, me_sb_addr + me_pic_width_in_sb + 1);
                        SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_6x6_index];
                        if (sb_params->is_complete_sb) {
                            MeLcuResults *me_results_64x64 = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_6x6_index];
                            const MeCandidate *me_block_results_64x64 = me_results_64x64->me_candidate[0];
                            sb_6x6_dist_3 = me_block_results_64x64[0].distortion;
                        }
                    }
                    if (blk_geom->bsize == BLOCK_128X128)
                        me_128x128 = sb_6x6_dist_0 + sb_6x6_dist_1 + sb_6x6_dist_2 + sb_6x6_dist_3;
                    if (blk_geom->bsize == BLOCK_128X64) {
                        me_128x128 = sb_6x6_dist_0 + sb_6x6_dist_1 + sb_6x6_dist_2 + sb_6x6_dist_3;
                        if (blk_geom->bsize == BLOCK_64X128) {
                            me_128x128 = sb_6x6_dist_0 + sb_6x6_dist_1 + sb_6x6_dist_2 + sb_6x6_dist_3;
                        }
                    }
                }

#endif
            }
            else
                me_sb_addr = sb_index;

            uint32_t max_number_of_pus_per_sb;

            max_number_of_pus_per_sb = picture_control_set_ptr->parent_pcs_ptr->max_number_of_pus_per_sb;

            uint32_t me_block_offset =
                (blk_geom->bwidth == 4 || blk_geom->bheight == 4 || blk_geom->bwidth == 128 || blk_geom->bheight == 128) ?
                0 :
                get_me_info_index(max_number_of_pus_per_sb, context_ptr->blk_geom, geom_offset_x, geom_offset_y);

            MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[me_sb_addr];
            EbBool allow_bipred = (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4) ? EB_FALSE : EB_TRUE;
            EbBool is_compound_enabled = (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
            const MeCandidate *me_block_results = me_results->me_candidate[me_block_offset];
            uint8_t total_me_cnt = me_results->total_me_candidate_index[me_block_offset];
            uint8_t me_index = 0;
            for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; me_candidate_index++) {
                const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
                if (is_compound_enabled) {
                    if (allow_bipred) {
                        if (me_block_results_ptr->direction == 2) {
                            me_index = me_candidate_index;
                            break;
                        }
                    }
                    else {
                        if (me_block_results_ptr->direction == 0) {
                            me_index = me_candidate_index;
                            break;
                        }
                    }
                }
                else {
                    if (me_block_results_ptr->direction == 0) {
                        me_index = me_candidate_index;
                        break;
                    }
                }
            }
            // Check best ME candidate
            // Initialize the mdc candidate (only av1 rate estimation inputs)
            // Hsan: mode, direction, .. could be modified toward better early inter depth decision (e.g. NEARESTMV instead of NEWMV)
            context_ptr->mdc_candidate_ptr->md_rate_estimation_ptr = context_ptr->md_rate_estimation_ptr;
            context_ptr->mdc_candidate_ptr->type = INTER_MODE;
            context_ptr->mdc_candidate_ptr->merge_flag = EB_FALSE;
            context_ptr->mdc_candidate_ptr->prediction_direction[0] = (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0) ?
                UNI_PRED_LIST_0 :
#if MD_INJECTION
                me_block_results[me_index].direction;
#else
                mePuResult->distortion_direction[0].direction;
#endif
            // Hsan: what's the best mode for rate simulation
            context_ptr->mdc_candidate_ptr->inter_mode = NEARESTMV;
            context_ptr->mdc_candidate_ptr->pred_mode = NEARESTMV;
            context_ptr->mdc_candidate_ptr->motion_mode = SIMPLE_TRANSLATION;
            context_ptr->mdc_candidate_ptr->is_new_mv = 1;
            context_ptr->mdc_candidate_ptr->is_zero_mv = 0;
            context_ptr->mdc_candidate_ptr->drl_index = 0;

            context_ptr->mdc_candidate_ptr->motion_vector_xl0 = me_results->me_mv_array[me_block_offset][0].x_mv << 1;
            context_ptr->mdc_candidate_ptr->motion_vector_yl0 = me_results->me_mv_array[me_block_offset][0].y_mv << 1;

            context_ptr->mdc_candidate_ptr->motion_vector_xl1 = me_results->me_mv_array[me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2)].x_mv << 1;
            context_ptr->mdc_candidate_ptr->motion_vector_yl1 = me_results->me_mv_array[me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2)].y_mv << 1;

            context_ptr->mdc_candidate_ptr->ref_mv_index = 0;
            context_ptr->mdc_candidate_ptr->pred_mv_weight = 0;
            if (context_ptr->mdc_candidate_ptr->prediction_direction[0] == BI_PRED) {
                context_ptr->mdc_candidate_ptr->ref_frame_type = LAST_BWD_FRAME;
                context_ptr->mdc_candidate_ptr->is_compound = 1;
            }
            else if (context_ptr->mdc_candidate_ptr->prediction_direction[0] == UNI_PRED_LIST_0) {
                context_ptr->mdc_candidate_ptr->ref_frame_type = LAST_FRAME;
                context_ptr->mdc_candidate_ptr->is_compound = 0;
            }
            else { // context_ptr->mdc_candidate_ptr->prediction_direction[0]
                context_ptr->mdc_candidate_ptr->ref_frame_type = BWDREF_FRAME;
                context_ptr->mdc_candidate_ptr->is_compound = 0;
            }
            context_ptr->mdc_candidate_ptr->motion_vector_pred_x[REF_LIST_0] = 0;
            context_ptr->mdc_candidate_ptr->motion_vector_pred_y[REF_LIST_0] = 0;
            // Initialize the ref mv
            memset(context_ptr->mdc_ref_mv_stack, 0, sizeof(CandidateMv));
            //blk_geom = get_blk_geom_mds(pa_to_ep_block_index[cuIdx]);
            // Initialize mdc cu (only av1 rate estimation inputs)
            context_ptr->mdc_cu_ptr->is_inter_ctx = 0;
            context_ptr->mdc_cu_ptr->skip_flag_context = 0;
            context_ptr->mdc_cu_ptr->inter_mode_ctx[context_ptr->mdc_candidate_ptr->ref_frame_type] = 0;
            context_ptr->mdc_cu_ptr->reference_mode_context = 0;
            context_ptr->mdc_cu_ptr->compoud_reference_type_context = 0;
            av1_zero(context_ptr->mdc_cu_ptr->av1xd->neighbors_ref_counts); // Hsan: neighbor not generated @ open loop partitioning => assumes always (0,0)

#if ADD_MDC_FULL_COST
            //const uint8_t inter_direction = me_block_results[me_index].direction;
            const uint8_t list0_ref_index = me_block_results[me_index].ref_idx_l0;
            const uint8_t list1_ref_index = me_block_results[me_index].ref_idx_l1;
            context_ptr->candidate_buffer->candidate_ptr->use_intrabc = 0;
            context_ptr->candidate_buffer->candidate_ptr->motion_mode = SIMPLE_TRANSLATION;
            context_ptr->candidate_buffer->candidate_ptr->md_rate_estimation_ptr = context_ptr->md_rate_estimation_ptr;
            context_ptr->candidate_buffer->candidate_ptr->type = INTER_MODE;
            context_ptr->candidate_buffer->candidate_ptr->merge_flag = EB_FALSE;
            context_ptr->candidate_buffer->candidate_ptr->prediction_direction[0] = me_block_results[me_index].direction;
            context_ptr->candidate_buffer->candidate_ptr->motion_mode = SIMPLE_TRANSLATION;
            context_ptr->candidate_buffer->candidate_ptr->is_new_mv = 1;
            context_ptr->candidate_buffer->candidate_ptr->is_zero_mv = 0;
            context_ptr->candidate_buffer->candidate_ptr->drl_index = 0;
            int16_t to_inject_mv_x_l0 = me_results->me_mv_array[me_block_offset][list0_ref_index].x_mv << 1;
            int16_t to_inject_mv_y_l0 = me_results->me_mv_array[me_block_offset][list0_ref_index].y_mv << 1;
            int16_t to_inject_mv_x_l1 = me_results->me_mv_array[me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results[me_index].ref1_list << 2) : (me_block_results[me_index].ref1_list << 1)) + list1_ref_index].x_mv << 1;
            int16_t to_inject_mv_y_l1 = me_results->me_mv_array[me_block_offset][((sequence_control_set_ptr->mrp_mode == 0) ? (me_block_results[me_index].ref1_list << 2) : (me_block_results[me_index].ref1_list << 1)) + list1_ref_index].y_mv << 1;
            context_ptr->candidate_buffer->candidate_ptr->motion_vector_xl0 = to_inject_mv_x_l0;
            context_ptr->candidate_buffer->candidate_ptr->motion_vector_yl0 = to_inject_mv_y_l0;
            context_ptr->candidate_buffer->candidate_ptr->motion_vector_xl1 = to_inject_mv_x_l1;
            context_ptr->candidate_buffer->candidate_ptr->motion_vector_yl1 = to_inject_mv_y_l1;
            context_ptr->candidate_buffer->candidate_ptr->ref_mv_index = 0;
            context_ptr->candidate_buffer->candidate_ptr->pred_mv_weight = 0;
            if (context_ptr->candidate_buffer->candidate_ptr->prediction_direction[0] == 0) {
                context_ptr->candidate_buffer->candidate_ptr->inter_mode = NEARESTMV;
                context_ptr->candidate_buffer->candidate_ptr->pred_mode = NEARESTMV;
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_type = svt_get_ref_frame_type(REF_LIST_0, list0_ref_index);
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_index_l0 = list0_ref_index;
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_index_l1 = -1;
                context_ptr->candidate_buffer->candidate_ptr->is_compound = 0;
            }
            else if (context_ptr->candidate_buffer->candidate_ptr->prediction_direction[0] == 1) {
                context_ptr->candidate_buffer->candidate_ptr->inter_mode = NEARESTMV;
                context_ptr->candidate_buffer->candidate_ptr->pred_mode = NEARESTMV;
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_type = svt_get_ref_frame_type(REF_LIST_1, list1_ref_index);
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_index_l0 = -1;
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_index_l1 = list1_ref_index;
                context_ptr->candidate_buffer->candidate_ptr->is_compound = 0;
            }
            else if (context_ptr->candidate_buffer->candidate_ptr->prediction_direction[0] == 2) {
                MvReferenceFrame rf[2];
                rf[0] = svt_get_ref_frame_type(me_block_results[me_index].ref0_list, list0_ref_index);
                rf[1] = svt_get_ref_frame_type(me_block_results[me_index].ref1_list, list1_ref_index);
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_type = av1_ref_frame_type(rf);
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_index_l0 = list0_ref_index;
                context_ptr->candidate_buffer->candidate_ptr->ref_frame_index_l1 = list1_ref_index;
                context_ptr->candidate_buffer->candidate_ptr->inter_mode = NEW_NEWMV;
                context_ptr->candidate_buffer->candidate_ptr->pred_mode = NEW_NEWMV;
                context_ptr->candidate_buffer->candidate_ptr->is_compound = 1;
            }
            else {
                printf("mdc invalid pred_direction");
            }
            context_ptr->candidate_buffer->candidate_ptr->motion_vector_pred_x[REF_LIST_0] = 0;
            context_ptr->candidate_buffer->candidate_ptr->motion_vector_pred_y[REF_LIST_0] = 0;
            context_ptr->candidate_buffer->candidate_ptr->motion_vector_pred_x[REF_LIST_1] = 0;
            context_ptr->candidate_buffer->candidate_ptr->motion_vector_pred_y[REF_LIST_1] = 0;
            context_ptr->candidate_buffer->candidate_ptr->interp_filters = 0;
            context_ptr->candidate_buffer->candidate_ptr->compound_idx = 0;
            context_ptr->candidate_buffer->candidate_ptr->interinter_comp.type = COMPOUND_AVERAGE;
            context_ptr->mdc_cu_ptr->is_inter_ctx = 0;
            context_ptr->mdc_cu_ptr->skip_flag_context = 0;
            context_ptr->mdc_cu_ptr->inter_mode_ctx[context_ptr->candidate_buffer->candidate_ptr->ref_frame_type] = 0;
            context_ptr->mdc_cu_ptr->reference_mode_context = 0;
            context_ptr->mdc_cu_ptr->compoud_reference_type_context = 0;
            av1_zero(context_ptr->mdc_cu_ptr->av1xd->neighbors_ref_counts);
            uint16_t txb_count = context_ptr->blk_geom->txb_count[0];
            for (uint16_t txb_itr = 0; txb_itr < txb_count; txb_itr++) {
                context_ptr->candidate_buffer->candidate_ptr->transform_type[txb_itr] = DCT_DCT;
            }
            mdc_inter_pu_prediction_av1(
                context_ptr,
                picture_control_set_ptr,
                context_ptr->candidate_buffer,
                asm_type);

            //Y Residual
            ResidualKernel(
                &(input_picture_ptr->buffer_y[input_origin_index]),
                input_picture_ptr->stride_y,
                &(context_ptr->candidate_buffer->prediction_ptr->buffer_y[cu_origin_index]),
                context_ptr->candidate_buffer->prediction_ptr->stride_y/* 64*/,
                &(((int16_t*)context_ptr->candidate_buffer->residual_ptr->buffer_y)[cu_origin_index]),
                context_ptr->candidate_buffer->residual_ptr->stride_y,
                context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight);

            context_ptr->candidate_buffer->candidate_ptr->tx_depth = 0;
            context_ptr->spatial_sse_full_loop = 0;
#if ADD_NEIGHBOR
             context_ptr->mdc_luma_dc_sign_level_coeff_neighbor_array = picture_control_set_ptr->mdc_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
#endif
            mdc_full_loop(
                context_ptr->candidate_buffer,
                context_ptr,
                picture_control_set_ptr,
                context_ptr->qp,
                count_non_zero_coeffs,
                &y_coeff_bits,
                y_full_distortion);
            //txb_count = context_ptr->blk_geom->txb_count[context_ptr->candidate_buffer->candidate_ptr->tx_depth];
            for (uint8_t txb_itr = 0; txb_itr < txb_count; txb_itr++) {
                context_ptr->mdc_cu_ptr->quantized_dc[0][txb_itr] = context_ptr->candidate_buffer->candidate_ptr->quantized_dc[0][txb_itr];
            }
#endif
            // Fast Cost Calc
#if! ADD_MDC_FULL_COST
            cu_ptr->early_cost = av1_inter_fast_cost(
                context_ptr->mdc_cu_ptr,
                context_ptr->mdc_candidate_ptr,
                context_ptr->qp,
#if MD_INJECTION
#if ADD_SAD_FOR_128X128
                blk_geom->sq_size == 128 ? me_128x128 : me_block_results[me_index].distortion,
#else
                me_block_results[me_index].distortion,
#endif
#else
                mePuResult->distortion_direction[0].distortion,
#endif
                (uint64_t)0,
                context_ptr->lambda,
                0,
                picture_control_set_ptr,
                context_ptr->mdc_ref_mv_stack,
                blk_geom,
                (sb_originy + blk_geom->origin_y) >> MI_SIZE_LOG2,
                (sb_originx + blk_geom->origin_x) >> MI_SIZE_LOG2,
#if MRP_COST_EST
                0,
#endif
                DC_PRED,        // Hsan: neighbor not generated @ open loop partitioning
                DC_PRED);       // Hsan: neighbor not generated @ open loop partitioning
#endif
#if ADD_MDC_FULL_COST
            mdc_av1_inter_fast_cost(
                context_ptr->mdc_cu_ptr,
                context_ptr->candidate_buffer->candidate_ptr,
                context_ptr->qp,
                blk_geom->sq_size == 128 ? me_128x128 : me_block_results[me_index].distortion,
                context_ptr->lambda,
                0,//use_ssd,
                picture_control_set_ptr,
                context_ptr->mdc_ref_mv_stack,
                blk_geom);

            cu_ptr->early_cost = mdc_av1_full_cost(
                context_ptr,
                y_full_distortion,
                &y_coeff_bits,
                context_ptr->full_lambda);
#endif

        }

#if ADD_MDC_INTRA
        // Check best Intra OIS Candidate
#if FIX_CRASH
        if (sequence_control_set_ptr->seq_header.sb_size == BLOCK_64X64 && sequence_control_set_ptr->sb_geom[sb_ptr->index].is_complete_sb && context_ptr->blk_geom->sq_size > 4 && context_ptr->blk_geom->sq_size < 128 && context_ptr->blk_geom->shape == PART_N) {
#else
        if (context_ptr->blk_geom->sq_size > 4 && context_ptr->blk_geom->sq_size < 128 && context_ptr->blk_geom->shape == PART_N) {
#endif
            context_ptr->mdc_cu_ptr->is_inter_ctx = 0;
            context_ptr->mdc_cu_ptr->skip_flag_context = 0;
            context_ptr->mdc_cu_ptr->inter_mode_ctx[context_ptr->mdc_candidate_ptr->ref_frame_type] = 0;
            uint8_t         intra_candidate_counter;
            uint8_t         intra_mode;
            uint32_t        can_total_cnt = 0;
            EbBool          disable_cfl_flag = EB_TRUE;// (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE : EB_FALSE;
            OisSbResults    *ois_sb_results_ptr = picture_control_set_ptr->parent_pcs_ptr->ois_sb_results[sb_ptr->index];
            OisCandidate    *ois_blk_ptr = ois_sb_results_ptr->ois_candidate_array[ep_to_pa_block_index[context_ptr->blk_geom->blkidx_mds]];
            uint8_t          total_intra_luma_mode = MIN(3, ois_sb_results_ptr->total_ois_intra_candidate[ep_to_pa_block_index[context_ptr->blk_geom->blkidx_mds]]);
            uint64_t temp_intra_cost = MAX_CU_COST;
            uint64_t intra_cost = MAX_CU_COST;
            for (intra_candidate_counter = 0; intra_candidate_counter < total_intra_luma_mode; ++intra_candidate_counter) {
                intra_mode = ois_blk_ptr[can_total_cnt].intra_mode;
                assert(intra_mode < INTRA_MODES);
                if (av1_is_directional_mode((PredictionMode)intra_mode)) {
                    int32_t angle_delta = ois_blk_ptr[can_total_cnt].angle_delta;
                    context_ptr->mdc_candidate_ptr->type = INTRA_MODE;
                    context_ptr->mdc_candidate_ptr->intra_luma_mode = intra_mode;
                    context_ptr->mdc_candidate_ptr->distortion_ready = 1;
                    context_ptr->mdc_candidate_ptr->me_distortion = ois_blk_ptr[can_total_cnt].distortion;
                    context_ptr->mdc_candidate_ptr->use_intrabc = 0;
                    context_ptr->mdc_candidate_ptr->is_directional_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)intra_mode);
#if !SEARCH_UV_CLEAN_UP
                    context_ptr->mdc_candidate_ptr->use_angle_delta = use_angle_delta ? context_ptr->mdc_candidate_ptr->is_directional_mode_flag : 0;
#endif
                    context_ptr->mdc_candidate_ptr->angle_delta[PLANE_TYPE_Y] = angle_delta;
                    context_ptr->mdc_candidate_ptr->intra_chroma_mode = disable_cfl_flag ? intra_luma_to_chroma[intra_mode] : UV_CFL_PRED;

                    context_ptr->mdc_candidate_ptr->cfl_alpha_signs = 0;
                    context_ptr->mdc_candidate_ptr->cfl_alpha_idx = 0;
                    context_ptr->mdc_candidate_ptr->is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)context_ptr->mdc_candidate_ptr->intra_chroma_mode);
                    context_ptr->mdc_candidate_ptr->angle_delta[PLANE_TYPE_UV] = 0;
                    context_ptr->mdc_candidate_ptr->transform_type[0] = DCT_DCT;
                    context_ptr->mdc_candidate_ptr->transform_type_uv = DCT_DCT;
                    context_ptr->mdc_candidate_ptr->ref_frame_type = INTRA_FRAME;
                    context_ptr->mdc_candidate_ptr->pred_mode = (PredictionMode)intra_mode;
                    context_ptr->mdc_candidate_ptr->motion_mode = SIMPLE_TRANSLATION;

                }
                else {
                    context_ptr->mdc_candidate_ptr->type = INTRA_MODE;
                    context_ptr->mdc_candidate_ptr->intra_luma_mode = intra_mode;
                    context_ptr->mdc_candidate_ptr->distortion_ready = 1;
                    context_ptr->mdc_candidate_ptr->me_distortion = ois_blk_ptr[can_total_cnt].distortion;
                    context_ptr->mdc_candidate_ptr->use_intrabc = 0;
                    context_ptr->mdc_candidate_ptr->is_directional_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)intra_mode);
#if !SEARCH_UV_CLEAN_UP
                    context_ptr->mdc_candidate_ptr->use_angle_delta = context_ptr->mdc_candidate_ptr->is_directional_mode_flag;
#endif
                    context_ptr->mdc_candidate_ptr->angle_delta[PLANE_TYPE_Y] = 0;
                    context_ptr->mdc_candidate_ptr->intra_chroma_mode = disable_cfl_flag ? intra_luma_to_chroma[intra_mode] : UV_CFL_PRED;

                    context_ptr->mdc_candidate_ptr->cfl_alpha_signs = 0;
                    context_ptr->mdc_candidate_ptr->cfl_alpha_idx = 0;
                    context_ptr->mdc_candidate_ptr->is_directional_chroma_mode_flag = (uint8_t)av1_is_directional_mode((PredictionMode)context_ptr->mdc_candidate_ptr->intra_chroma_mode);
                    context_ptr->mdc_candidate_ptr->angle_delta[PLANE_TYPE_UV] = 0;
                    context_ptr->mdc_candidate_ptr->transform_type[0] = DCT_DCT;
                    context_ptr->mdc_candidate_ptr->transform_type_uv = DCT_DCT;
                    context_ptr->mdc_candidate_ptr->ref_frame_type = INTRA_FRAME;
                    context_ptr->mdc_candidate_ptr->pred_mode = (PredictionMode)intra_mode;
                    context_ptr->mdc_candidate_ptr->motion_mode = SIMPLE_TRANSLATION;

                }
                // Fast Cost Calc
                temp_intra_cost = av1_intra_fast_cost(
                    context_ptr->mdc_cu_ptr,
                    context_ptr->mdc_candidate_ptr,
                    context_ptr->qp,
#if MD_INJECTION
                    context_ptr->mdc_candidate_ptr->me_distortion,
#else
                    mePuResult->distortion_direction[0].distortion,
#endif
                    (uint64_t)0,
                    context_ptr->lambda,
                    0,
                    picture_control_set_ptr,
                    context_ptr->mdc_ref_mv_stack,
                    blk_geom,
                    (sb_originy + blk_geom->origin_y) >> MI_SIZE_LOG2,
                    (sb_originx + blk_geom->origin_x) >> MI_SIZE_LOG2,
#if MRP_COST_EST
                    0,
#endif
                    DC_PRED,        // Hsan: neighbor not generated @ open loop partitioning
                    DC_PRED);       // Hsan: neighbor not generated @ open loop partitioning

                if (temp_intra_cost < intra_cost)
                    intra_cost = temp_intra_cost;
            }
            cu_ptr->early_cost = intra_cost;
        }
#endif


        //NADER - FORCE DEPTH
        /*SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_index];
        if (sb_params->is_complete_sb) {
            if (context_ptr->blk_geom->sq_size == 32)
                cu_ptr->early_cost = 0;
        }
        else {
            if (context_ptr->blk_geom->sq_size == 4)
                cu_ptr->early_cost = 0;
        }*/
        if (blk_geom->nsi + 1 == blk_geom->totns) {
            nsq_cost[context_ptr->blk_geom->shape] = mdc_d1_non_square_block_decision(sequence_control_set_ptr, context_ptr);
        }

        d1_blocks_accumlated = blk_geom->shape == PART_N ? 1 : d1_blocks_accumlated + 1;
#if ADD_NEIGHBOR
        if (blk_geom->shape != PART_N) {
            if (blk_geom->nsi + 1 < blk_geom->totns)
                mdc_update_all_neighbour_arrays(
                    picture_control_set_ptr,
                    context_ptr,
                    blk_idx_mds,
                    sb_originx,
                    sb_originy);
            else
                mdc_copy_neighbour_arrays(      //restore [1] in [0] after done last ns block
                    picture_control_set_ptr,
                    context_ptr,
                    1, 0,
                    blk_geom->sqi_mds,
                    sb_originx,
                    sb_originy);
        }
#endif

        if (d1_blocks_accumlated == leaf_data_ptr->tot_d1_blocks)
        {
            end_idx = cuIdx + 1;
            //Sorting
            {
                uint32_t i, j, index;
                for (i = 0; i < 10 - 1; ++i) {
                    for (j = i + 1; j < 10; ++j) {
                        if (nsq_cost[nsq_shape_table[j]] < nsq_cost[nsq_shape_table[i]]) {
                            index = nsq_shape_table[i];
                            nsq_shape_table[i] = nsq_shape_table[j];
                            nsq_shape_table[j] = index;
                        }
                    }
                }
            }

#if DEPTH_RANKING
            depth_cost[get_depth(context_ptr->blk_geom->sq_size)] += nsq_cost[nsq_shape_table[0]];
#endif
            // Assign ranking # to each block
            for (leaf_idx = start_idx; leaf_idx < end_idx; leaf_idx++) {
                EbMdcLeafData * current_depth_leaf_data = &mdcResultTbPtr->leaf_data_array[leaf_idx];
                //uint32_t idx_mds = leaf_data_array[leaf_idx].mds_idx;
                const BlockGeom * bepth_blk_geom = get_blk_geom_mds(leaf_data_array[leaf_idx].mds_idx);
                current_depth_leaf_data->open_loop_ranking = find_shape_index(bepth_blk_geom->shape, nsq_shape_table);
#if COMBINE_MDC_NSQ_TABLE
                current_depth_leaf_data->ol_best_nsq_shape1 = nsq_shape_table[0];
                current_depth_leaf_data->ol_best_nsq_shape2 = nsq_shape_table[1];
                current_depth_leaf_data->ol_best_nsq_shape3 = nsq_shape_table[2];
                current_depth_leaf_data->ol_best_nsq_shape4 = nsq_shape_table[3];
                current_depth_leaf_data->ol_best_nsq_shape5 = nsq_shape_table[4];
                current_depth_leaf_data->ol_best_nsq_shape6 = nsq_shape_table[5];
                current_depth_leaf_data->ol_best_nsq_shape7 = nsq_shape_table[6];
                current_depth_leaf_data->ol_best_nsq_shape8 = nsq_shape_table[7];
#endif
            }

            //Reset nsq table
            //memset(nsq_cost, MAX_CU_COST,NUMBER_OF_SHAPES*sizeof(uint64_t));
            for (int cost_idx = 0; cost_idx < NUMBER_OF_SHAPES; cost_idx++)
                nsq_cost[cost_idx] = MAX_CU_COST;
            for (int sh = 0; sh < NUMBER_OF_SHAPES; sh++)
                nsq_shape_table[sh] = (PART)sh;

            start_idx = end_idx;

            uint32_t  last_cu_index = mdc_d2_inter_depth_block_decision(
                picture_control_set_ptr,
                context_ptr,
                leaf_data_ptr,
                blk_geom->sqi_mds,//input is parent square,
                sb_index);
            if (last_cu_index)
                last_cu_index = 0;
#if ADD_NEIGHBOR
             if (context_ptr->local_cu_array[last_cu_index].early_split_flag == EB_FALSE)
            {
                mdc_update_all_neighbour_arrays_multiple(
                    picture_control_set_ptr,
                    context_ptr,
                    context_ptr->local_cu_array[last_cu_index].best_d1_blk,
                    sb_originx,
                    sb_originy);
            }
#endif

        }
        cuIdx++;
    } while (cuIdx < leaf_count);// End of CU loop

#if DEPTH_RANKING
    if (sequence_control_set_ptr->seq_header.sb_size == BLOCK_64X64)
        depth_cost[0] = MAX_CU_COST;
    //Sorting
    {
        uint32_t i, j, index;
        for (i = 0; i < NUMBER_OF_DEPTH - 1; ++i) {
            for (j = i + 1; j < NUMBER_OF_DEPTH; ++j) {
                if (depth_cost[depth_table[j]] < depth_cost[depth_table[i]]) {
                    index = depth_table[i];
                    depth_table[i] = depth_table[j];
                    depth_table[j] = index;
                }
            }
        }
    }
    for (uint8_t depth_idx = 0; depth_idx < NUMBER_OF_DEPTH; depth_idx++)
        sb_ptr->depth_ranking[depth_idx] = find_depth_index(depth_idx, depth_table);
#endif
    return return_error;
}
#endif
void PredictionPartitionLoop(
    SequenceControlSet                   *sequence_control_set_ptr,
    PictureControlSet                    *picture_control_set_ptr,
    uint32_t                                sb_index,
    uint32_t                                tbOriginX,
    uint32_t                                tbOriginY,
    uint32_t                                startDepth,
    uint32_t                                endDepth,
    ModeDecisionConfigurationContext     *context_ptr){
    MdcpLocalCodingUnit *local_cu_array = context_ptr->local_cu_array;
    MdcpLocalCodingUnit   *cu_ptr;

    SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_index];
    uint32_t      cuIndexInRaterScan;
    uint32_t      cu_index = 0;
    uint32_t      start_index = 0;

    (void)tbOriginX;
    (void)tbOriginY;

    const CodedUnitStats *cuStatsPtr;

    for (cu_index = start_index; cu_index < CU_MAX_COUNT; ++cu_index)

    {
        local_cu_array[cu_index].selected_cu = EB_FALSE;
        local_cu_array[cu_index].stop_split = EB_FALSE;

        cu_ptr = &local_cu_array[cu_index];
        cuIndexInRaterScan = md_scan_to_raster_scan[cu_index];
        if (sb_params->raster_scan_cu_validity[cuIndexInRaterScan])
        {
            uint32_t depth;
            cuStatsPtr = get_coded_unit_stats(cu_index);

            depth = cuStatsPtr->depth;
            cu_ptr->early_split_flag = (depth < endDepth) ? EB_TRUE : EB_FALSE;

            if (depth >= startDepth && depth <= endDepth) {
                //reset the flags here:   all CU splitFalg=TRUE. default: we always split. interDepthDecision will select where  to stop splitting(ie setting the flag to False)

                if (picture_control_set_ptr->slice_type != I_SLICE) {
#if MD_INJECTION
                    const MeLcuResults *me_results = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_index];
                    const MeCandidate *me_block_results = me_results->me_candidate[cuIndexInRaterScan];
                    uint8_t total_me_cnt = me_results->total_me_candidate_index[cuIndexInRaterScan];
                    uint8_t me_index = 0;
                    for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt; me_candidate_index++) {
                        const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
                        if (picture_control_set_ptr->parent_pcs_ptr->reference_mode == SINGLE_REFERENCE) {
                            if (me_block_results_ptr->direction == 0) {
                                me_index = me_candidate_index;
                                break;
                            }
                        }
                        else {
                            if (me_block_results_ptr->direction == 2) {
                                me_index = me_candidate_index;
                                break;
                            }
                        }
                    }

                    //const MeCandidate_t *me_results = &me_block_results[me_index];

#else
                    MeCuResults * mePuResult = &picture_control_set_ptr->parent_pcs_ptr->me_results[sb_index][cuIndexInRaterScan];
#endif
                    // Initialize the mdc candidate (only av1 rate estimation inputs)
                    // Hsan: mode, direction, .. could be modified toward better early inter depth decision (e.g. NEARESTMV instead of NEWMV)
                    context_ptr->mdc_candidate_ptr->md_rate_estimation_ptr = context_ptr->md_rate_estimation_ptr;
                    context_ptr->mdc_candidate_ptr->type = INTER_MODE;
                    context_ptr->mdc_candidate_ptr->merge_flag = EB_FALSE;
                    context_ptr->mdc_candidate_ptr->prediction_direction[0] = (picture_control_set_ptr->parent_pcs_ptr->temporal_layer_index == 0) ?
                        UNI_PRED_LIST_0 :
#if MD_INJECTION
                        me_block_results[me_index].direction;
#else
                        mePuResult->distortion_direction[0].direction;
#endif
                    // Hsan: what's the best mode for rate simulation
                    context_ptr->mdc_candidate_ptr->inter_mode = NEARESTMV;
                    context_ptr->mdc_candidate_ptr->pred_mode = NEARESTMV;
                    context_ptr->mdc_candidate_ptr->motion_mode = SIMPLE_TRANSLATION;
                    context_ptr->mdc_candidate_ptr->is_new_mv = 1;
                    context_ptr->mdc_candidate_ptr->is_zero_mv = 0;
                    context_ptr->mdc_candidate_ptr->drl_index = 0;
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
                    context_ptr->mdc_candidate_ptr->motion_vector_xl0 = me_results->me_mv_array[cuIndexInRaterScan][0].x_mv << 1;
                    context_ptr->mdc_candidate_ptr->motion_vector_yl0 = me_results->me_mv_array[cuIndexInRaterScan][0].y_mv << 1;
#else
                    context_ptr->mdc_candidate_ptr->motion_vector_xl0 = me_block_results[me_index].x_mv_l0 << 1;
                    context_ptr->mdc_candidate_ptr->motion_vector_yl0 = me_block_results[me_index].y_mv_l0 << 1;
#endif
#else
                    context_ptr->mdc_candidate_ptr->motion_vector_xl0 = mePuResult->x_mv_l0 << 1;
                    context_ptr->mdc_candidate_ptr->motion_vector_yl0 = mePuResult->y_mv_l0 << 1;
#endif
#if MD_INJECTION
#if MEMORY_FOOTPRINT_OPT_ME_MV
                    context_ptr->mdc_candidate_ptr->motion_vector_xl1 = me_results->me_mv_array[cuIndexInRaterScan][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2)].x_mv << 1;
                    context_ptr->mdc_candidate_ptr->motion_vector_yl1 = me_results->me_mv_array[cuIndexInRaterScan][((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2)].y_mv << 1;
#else
                    context_ptr->mdc_candidate_ptr->motion_vector_xl1 = me_block_results[me_index].x_mv_l1 << 1;
                    context_ptr->mdc_candidate_ptr->motion_vector_yl1 = me_block_results[me_index].y_mv_l1 << 1;
#endif
#else
                    context_ptr->mdc_candidate_ptr->motion_vector_xl1 = mePuResult->x_mv_l1 << 1;
                    context_ptr->mdc_candidate_ptr->motion_vector_yl1 = mePuResult->y_mv_l1 << 1;
#endif
                    context_ptr->mdc_candidate_ptr->ref_mv_index = 0;
                    context_ptr->mdc_candidate_ptr->pred_mv_weight = 0;
                    if (context_ptr->mdc_candidate_ptr->prediction_direction[0] == BI_PRED) {
                        context_ptr->mdc_candidate_ptr->ref_frame_type = LAST_BWD_FRAME;
                        context_ptr->mdc_candidate_ptr->is_compound = 1;
                    }
                    else if (context_ptr->mdc_candidate_ptr->prediction_direction[0] == UNI_PRED_LIST_0) {
                        context_ptr->mdc_candidate_ptr->ref_frame_type = LAST_FRAME;
                        context_ptr->mdc_candidate_ptr->is_compound = 0;
                    }
                    else { // context_ptr->mdc_candidate_ptr->prediction_direction[0]
                        context_ptr->mdc_candidate_ptr->ref_frame_type = BWDREF_FRAME;
                        context_ptr->mdc_candidate_ptr->is_compound = 0;
                    }
                    context_ptr->mdc_candidate_ptr->motion_vector_pred_x[REF_LIST_0] = 0;
                    context_ptr->mdc_candidate_ptr->motion_vector_pred_y[REF_LIST_0] = 0;
                    // Initialize the ref mv
                    memset(context_ptr->mdc_ref_mv_stack,0,sizeof(CandidateMv));
                    context_ptr->blk_geom = get_blk_geom_mds(pa_to_ep_block_index[cu_index]);
                    // Initialize mdc cu (only av1 rate estimation inputs)
                    context_ptr->mdc_cu_ptr->is_inter_ctx = 0;
                    context_ptr->mdc_cu_ptr->skip_flag_context = 0;
                    context_ptr->mdc_cu_ptr->inter_mode_ctx[context_ptr->mdc_candidate_ptr->ref_frame_type] = 0;
                    context_ptr->mdc_cu_ptr->reference_mode_context = 0;
                    context_ptr->mdc_cu_ptr->compoud_reference_type_context = 0;
                    av1_zero(context_ptr->mdc_cu_ptr->av1xd->neighbors_ref_counts); // Hsan: neighbor not generated @ open loop partitioning => assumes always (0,0)

                    // Fast Cost Calc
                    cu_ptr->early_cost = av1_inter_fast_cost(
                        context_ptr->mdc_cu_ptr,
                        context_ptr->mdc_candidate_ptr,
                        context_ptr->qp,
#if MD_INJECTION
                        me_block_results[me_index].distortion,
#else
                        mePuResult->distortion_direction[0].distortion,
#endif
                        (uint64_t) 0,
                        context_ptr->lambda,
                        0,
                        picture_control_set_ptr,
                        context_ptr->mdc_ref_mv_stack,
                        context_ptr->blk_geom,
                        (tbOriginY + context_ptr->blk_geom->origin_y) >> MI_SIZE_LOG2,
                        (tbOriginX + context_ptr->blk_geom->origin_x) >> MI_SIZE_LOG2,
#if MRP_COST_EST
                        0,
#endif
                        DC_PRED,        // Hsan: neighbor not generated @ open loop partitioning
                        DC_PRED);       // Hsan: neighbor not generated @ open loop partitioning
                }

                if (endDepth == 2)
                    context_ptr->group_of8x8_blocks_count = depth == 2 ? incrementalCount[cuIndexInRaterScan] : 0;
                if (endDepth == 1)
                    context_ptr->group_of16x16_blocks_count = depth == 1 ? incrementalCount[cuIndexInRaterScan] : 0;
                MdcInterDepthDecision(
                    context_ptr,
                    cuStatsPtr->origin_x,
                    cuStatsPtr->origin_y,
                    endDepth,
                    cu_index);
            }
            else
                cu_ptr->early_cost = ~0u;
        }
    }// End CU Loop
}

EbErrorType early_mode_decision_lcu(
    SequenceControlSet                   *sequence_control_set_ptr,
    PictureControlSet                    *picture_control_set_ptr,
    LargestCodingUnit                    *sb_ptr,
    uint32_t                                  sb_index,
    ModeDecisionConfigurationContext     *context_ptr){
    EbErrorType    return_error = EB_ErrorNone;
    uint32_t       tbOriginX    = sb_ptr->origin_x;
    uint32_t       tbOriginY    = sb_ptr->origin_y;

    uint32_t      startDepth = DEPTH_64;

    uint32_t      endDepth =  DEPTH_8 ;
    context_ptr->group_of8x8_blocks_count = 0;
    context_ptr->group_of16x16_blocks_count = 0;

    PredictionPartitionLoop(
        sequence_control_set_ptr,
        picture_control_set_ptr,
        sb_index,
        tbOriginX,
        tbOriginY,
        startDepth,
        endDepth,
        context_ptr
    );

    RefinementPredictionLoop(
        sequence_control_set_ptr,
        picture_control_set_ptr,
#if !MEMORY_FOOTPRINT_OPT
        sb_ptr,
#endif
        sb_index,
        context_ptr);

    ForwardCuToModeDecision(
        sequence_control_set_ptr,
        picture_control_set_ptr,
        sb_index,
        context_ptr);

    return return_error;
}
