/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#ifndef EBAVCSTYLEMCP_H
#define EBAVCSTYLEMCP_H

#include "EbAvcStyleMcp_SSE2.h"
#include "EbAvcStyleMcp_SSSE3.h"

#include "EbPictureOperators.h"

#include "EbMcp.h"

#include "EbDefinitions.h"
#include "EbPictureBufferDesc.h"
#include "EbPictureControlSet.h"

#ifdef __cplusplus
extern "C" {
#endif

    void estimate_bi_pred_interpolation_avc_luma(
        EbPictureBufferDesc *ref_pic_list0,
        EbPictureBufferDesc *ref_pic_list1,
        uint32_t                 ref_list0_pos_x,
        uint32_t                 ref_list0_pos_y,
        uint32_t                 ref_list1_pos_x,
        uint32_t                 ref_list1_pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *bi_dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,
        uint32_t                 component_mask,
        EbByte                ref_list0_temp_dst,
        EbByte                ref_list1_temp_dst,
        EbByte                first_pass_if_temp_dst,
        EbBool                sub_sample_pred_flag,
        EbAsm                 asm_type);

    void estimate_uni_pred_interpolation_avc_luma(
        EbPictureBufferDesc *ref_pic,
        uint32_t                 pos_x,
        uint32_t                 pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,
        uint32_t                 component_mask,
        EbByte                temp_buf,
        EbBool                sub_sample_pred_flag,
        EbAsm                 asm_type);

    void estimate_bi_pred_interpolation_unpacked_avc_style(
        EbPictureBufferDesc *ref_pic_list0,
        EbPictureBufferDesc *ref_pic_list1,
        uint32_t                 ref_list0_pos_x,
        uint32_t                 ref_list0_pos_y,
        uint32_t                 ref_list1_pos_x,
        uint32_t                 ref_list1_pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *bi_dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,
        uint32_t                 component_mask,
        EbByte                ref_list0_temp_dst,
        EbByte                ref_list1_temp_dst,
        EbByte                first_pass_if_temp_dst,
        EbBool                sub_sample_pred_flag,
        EbAsm                 asm_type);

    void estimate_uni_pred_interpolation_unpacked_avc_style(
        EbPictureBufferDesc *ref_pic,
        uint32_t                 pos_x,
        uint32_t                 pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,
        uint32_t                 component_mask,
        EbByte                temp_buf,
        EbBool                sub_sample_pred_flag,
        EbAsm                 asm_type);

    void estimate_uni_pred_interpolation_avc_chroma_ref10_bit(
        EbPictureBufferDesc *ref_frame_pic_list0,
        uint32_t                 pos_x,
        uint32_t                 pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,          //input parameter, please refer to the detailed explanation above.
        uint32_t                 component_mask,
        EbByte                temp_buf,
        EbBool                sub_pred,
        EbAsm                 asm_type);

    void estimate_bi_pred_interpolation_avc_chroma_ref10_bit(
        EbPictureBufferDesc *ref_frame_pic_list0,
        EbPictureBufferDesc *ref_frame_pic_list1,
        uint32_t                 ref_list0_pos_x,
        uint32_t                 ref_list0_pos_y,
        uint32_t                 ref_list1_pos_x,
        uint32_t                 ref_list1_pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *bi_dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,
        uint32_t                 component_mask,
        EbByte                ref_list0_temp_dst,
        EbByte                ref_list1_temp_dst,
        EbByte                first_pass_if_temp_dst,
        EbBool                sub_pred);

    void estimate_uni_pred_interpolation_avc_lumaRef10Bit(
        EbPictureBufferDesc *ref_frame_pic_list0,
        uint32_t                 pos_x,
        uint32_t                 pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,          //input parameter, please refer to the detailed explanation above.
        uint32_t                 component_mask,
        EbByte                temp_buf,
        EbBool                sub_pred,
        EbBool                sub_pred_chroma,
        EbAsm                 asm_type);

    void estimate_bi_pred_interpolation_avc_luma_ref10_bit(
        EbPictureBufferDesc *ref_frame_pic_list0,
        EbPictureBufferDesc *ref_frame_pic_list1,
        uint32_t                 ref_list0_pos_x,
        uint32_t                 ref_list0_pos_y,
        uint32_t                 ref_list1_pos_x,
        uint32_t                 ref_list1_pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *bi_dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,
        uint32_t                 component_mask,
        EbByte                ref_list0_temp_dst,
        EbByte                ref_list1_temp_dst,
        EbByte                first_pass_if_temp_dst,
        EbBool                sub_pred,
        EbBool                sub_pred_chroma);

    void uni_pred_i_free_ref8_bit(
        EbPictureBufferDesc *ref_pic,
        uint32_t                 pos_x,
        uint32_t                 pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,          //input parameter, please refer to the detailed explanation above.
        uint32_t                 component_mask,
        EbByte                temp_buf,
        EbBool                sub_sample_pred_flag,
        EbBool                subSamplePredFlagChroma,
        EbAsm                 asm_type);
    void bi_pred_i_free_ref8_bit(
        EbPictureBufferDesc *ref_pic_list0,
        EbPictureBufferDesc *ref_pic_list1,
        uint32_t                 ref_list0_pos_x,
        uint32_t                 ref_list0_pos_y,
        uint32_t                 ref_list1_pos_x,
        uint32_t                 ref_list1_pos_y,
        uint32_t                 pu_width,
        uint32_t                 pu_height,
        EbPictureBufferDesc *bi_dst,
        uint32_t                 dst_luma_index,
        uint32_t                 dst_chroma_index,
        uint32_t                 component_mask,
        EbByte                ref_list0_temp_dst,
        EbByte                ref_list1_temp_dst,
        EbByte                first_pass_if_temp_dst,
        EbBool                sub_sample_pred_flag,
        EbBool                subSamplePredFlagChroma,
        EbAsm                 asm_type);

#ifdef __cplusplus
}
#endif
#endif //EBAVCSTYLEMCP_H
