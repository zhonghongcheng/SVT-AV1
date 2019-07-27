/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include "EbModeDecisionConfiguration.h"
#include "EbRateDistortionCost.h"
#include "EbUtility.h"
#include "EbModeDecisionProcess.h"
#include "EbDefinitions.h"
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
    uint32_t       curr_depth_blk3_mds = curr_depth_mds;

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


    uint32_t                last_cu_index, /*d0_idx_mds,*/ d1_idx_mds, d2_idx_mds/*, top_left_idx_mds*/;
    uint64_t                parent_depth_cost = 0, current_depth_cost = 0;
    SequenceControlSet     *sequence_control_set_ptr = (SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr;
    EbBool                  last_depth_flag;
    const BlockGeom        *blk_geom;

    last_depth_flag = context_ptr->local_cu_array[blk_mds].early_split_flag == EB_FALSE ? EB_TRUE : EB_FALSE;

    d1_idx_mds = blk_mds;
    d2_idx_mds = blk_mds;
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
    uint32_t start_idx,end_idx;
    //ModeDecisionCandidateBuffer            *bestCandidateBuffers[5];

    uint32_t                                leaf_count = mdcResultTbPtr->leaf_count;
    EbMdcLeafData *              leaf_data_array = mdcResultTbPtr->leaf_data_array;
    MdcpLocalCodingUnit *local_cu_array     = context_ptr->local_cu_array;

    MdcpLocalCodingUnit   *cu_ptr;
    //CU Loop
    cuIdx = 0;  //index over mdc array
    start_idx = 0;
    uint64_t nsq_cost[NUMBER_OF_SHAPES] = { MAX_CU_COST, MAX_CU_COST,MAX_CU_COST,MAX_CU_COST,MAX_CU_COST,
                              MAX_CU_COST, MAX_CU_COST,MAX_CU_COST,MAX_CU_COST,MAX_CU_COST};
    PART nsq_shape_table[NUMBER_OF_SHAPES] = { PART_N, PART_H, PART_V, PART_HA, PART_HB,
                               PART_VA, PART_VB, PART_H4, PART_V4, PART_S};
    uint32_t blk_idx_mds = 0;
    uint32_t  d1_blocks_accumlated = 0;
#if DEPTH_RANKING
    uint64_t depth_cost[NUMBER_OF_DEPTH] = { 0 };
    uint8_t depth_table[NUMBER_OF_DEPTH] = {0, 1, 2 , 3 ,4 ,5};
#endif
#if ADD_SAD_FOR_128X128
    uint64_t me_128x128 = 0;
#endif

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
        if (picture_control_set_ptr->slice_type != I_SLICE) {
            uint32_t geom_offset_x = 0;
            uint32_t geom_offset_y = 0;
            uint32_t me_sb_addr;
            if (sequence_control_set_ptr->seq_header.sb_size == BLOCK_128X128) {
                uint32_t me_sb_size = sequence_control_set_ptr->sb_sz;
                uint32_t me_pic_width_in_sb = (sequence_control_set_ptr->seq_header.max_frame_width + sequence_control_set_ptr->sb_sz - 1) / me_sb_size;
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
                        sb_6x6_index = me_sb_addr + 1;
                        SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_6x6_index];
                        if (sb_params->is_complete_sb) {
                            MeLcuResults *me_results_64x64 = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_6x6_index];
                            const MeCandidate *me_block_results_64x64 = me_results_64x64->me_candidate[0];
                            sb_6x6_dist_1 = me_block_results_64x64[0].distortion;
                        }
                    }
                    if (blk_geom->bsize == BLOCK_128X128 || blk_geom->bsize == BLOCK_64X128) {
                        sb_6x6_index = me_sb_addr + me_pic_width_in_sb;
                        SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_6x6_index];
                        if (sb_params->is_complete_sb) {
                            MeLcuResults *me_results_64x64 = picture_control_set_ptr->parent_pcs_ptr->me_results[sb_6x6_index];
                            const MeCandidate *me_block_results_64x64 = me_results_64x64->me_candidate[0];
                            sb_6x6_dist_2 = me_block_results_64x64[0].distortion;
                        }
                    }
                    if (blk_geom->bsize == BLOCK_128X128) {
                        sb_6x6_index = me_sb_addr + me_pic_width_in_sb + 1;
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


            const MeCandidate *me_block_results = me_results->me_candidate[me_block_offset];
            uint8_t total_me_cnt = me_results->total_me_candidate_index[me_block_offset];
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

            // Fast Cost Calc
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
        }
#if ADD_MDC_INTRA
            // Check best Intra OIS Candidate
        if(context_ptr->blk_geom->sq_size > 4 && context_ptr->blk_geom->shape == PART_N){
            context_ptr->mdc_cu_ptr->is_inter_ctx = 0;
            context_ptr->mdc_cu_ptr->skip_flag_context = 0;
            context_ptr->mdc_cu_ptr->inter_mode_ctx[context_ptr->mdc_candidate_ptr->ref_frame_type] = 0;
            uint8_t         intra_candidate_counter;
            uint8_t         intra_mode;
            uint32_t        can_total_cnt = 0;
            EbBool          disable_cfl_flag = EB_TRUE;// (MAX(context_ptr->blk_geom->bheight, context_ptr->blk_geom->bwidth) > 32) ? EB_TRUE : EB_FALSE;
            OisSbResults    *ois_sb_results_ptr = picture_control_set_ptr->parent_pcs_ptr->ois_sb_results[sb_ptr->index];
            OisCandidate    *ois_blk_ptr = ois_sb_results_ptr->ois_candidate_array[ep_to_pa_block_index[context_ptr->blk_geom->blkidx_mds]];
            uint8_t          total_intra_luma_mode = MIN(1, ois_sb_results_ptr->total_ois_intra_candidate[ep_to_pa_block_index[context_ptr->blk_geom->blkidx_mds]]);

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
            }
            // Fast Cost Calc
            uint64_t intra_cost = av1_intra_fast_cost(
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

            if (intra_cost < cu_ptr->early_cost)
                cu_ptr->early_cost = intra_cost;
        }
#endif


        if (blk_geom->nsi + 1 == blk_geom->totns) {
            nsq_cost[context_ptr->blk_geom->shape] = mdc_d1_non_square_block_decision(sequence_control_set_ptr, context_ptr);
        }

        d1_blocks_accumlated = blk_geom->shape == PART_N ? 1 : d1_blocks_accumlated + 1;

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
            depth_cost[get_depth(context_ptr->blk_geom->sq_size)] += nsq_cost[0];
#endif
            // Assign ranking # to each block
            for (leaf_idx = start_idx; leaf_idx < end_idx; leaf_idx++) {
                EbMdcLeafData * current_depth_leaf_data = &mdcResultTbPtr->leaf_data_array[leaf_idx];
                uint32_t idx_mds = leaf_data_array[leaf_idx].mds_idx;
               const BlockGeom * bepth_blk_geom = get_blk_geom_mds(leaf_data_array[leaf_idx].mds_idx);
               current_depth_leaf_data->open_loop_ranking = find_shape_index(bepth_blk_geom->shape, nsq_shape_table);
            }

            //Reset nsq table
            memset(nsq_cost, MAX_CU_COST,NUMBER_OF_SHAPES*sizeof(uint64_t));
            for (int sh = 0; sh < NUMBER_OF_SHAPES; sh++)
                nsq_shape_table[sh] = (PART) sh;

            start_idx = end_idx;

            uint32_t  last_cu_index = mdc_d2_inter_depth_block_decision(
                picture_control_set_ptr,
                context_ptr,
                leaf_data_ptr,
                blk_geom->sqi_mds,//input is parent square,
                sb_index);
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
    for(uint8_t depth_idx = 0;depth_idx < NUMBER_OF_DEPTH; depth_idx++ )
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
