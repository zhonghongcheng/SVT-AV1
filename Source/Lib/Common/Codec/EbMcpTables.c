/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include "EbMcp.h"

void uni_pred_luma_if_helper(
    EbByte               ref_pic,
    uint32_t                src_stride,
    EbByte               dst,
    uint32_t                dst_stride,
    uint32_t                pu_width,
    uint32_t                pu_height,
    int16_t               *first_pass_if_dst,
    uint8_t             choice) {

    switch (choice) {

    case 0:
        luma_interpolation_copy_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 1:
        luma_interpolation_filter_posa_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 2:
        luma_interpolation_filter_posb_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 3:
        luma_interpolation_filter_posc_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 4:
        luma_interpolation_filter_posd_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 5:
        luma_interpolation_filter_pose_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 6:
        luma_interpolation_filter_posf_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 7:
        luma_interpolation_filter_posg_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 8:
        luma_interpolation_filter_posh_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 9:
        luma_interpolation_filter_posi_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 10:
        luma_interpolation_filter_posj_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 11:
        luma_interpolation_filter_posk_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 12:
        luma_interpolation_filter_posn_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 13:
        luma_interpolation_filter_posp_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 14:
        luma_interpolation_filter_posq_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    case 15:
        luma_interpolation_filter_posr_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst); break;
    }
}

void bi_pred_luma_if_helper(
    EbByte               ref_pic,
    uint32_t                src_stride,
    int16_t               *dst,
    uint32_t                pu_width,
    uint32_t                pu_height,
    int16_t               *first_pass_if_dst,
    uint8_t             choice) {

    switch (choice) {

    case 0:
        luma_interpolation_copy_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 1:
        luma_interpolation_filter_posa_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 2:
        luma_interpolation_filter_posb_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 3:
        luma_interpolation_filter_posc_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 4:
        luma_interpolation_filter_posd_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 5:
        luma_interpolation_filter_pose_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 6:
        luma_interpolation_filter_posf_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 7:
        luma_interpolation_filter_posg_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 8:
        luma_interpolation_filter_posh_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 9:
        luma_interpolation_filter_posi_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 10:
        luma_interpolation_filter_posj_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 11:
        luma_interpolation_filter_posk_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 12:
        luma_interpolation_filter_posn_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 13:
        luma_interpolation_filter_posp_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 14:
        luma_interpolation_filter_posq_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    case 15:
        luma_interpolation_filter_posr_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst); break;
    }
}

void uni_pred_chroma_if_helper(
    EbByte               ref_pic,
    uint32_t                src_stride,
    EbByte               dst,
    uint32_t                dst_stride,
    uint32_t                pu_width,
    uint32_t                pu_height,
    int16_t               *first_pass_if_dst,
    uint32_t                frac_pos_x,
    uint32_t                frac_pos_y,
    uint8_t             choice) {

    switch (choice) {

    case 0:
        chroma_interpolation_copy_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 1:
        chroma_interpolation_filter_one_d_horizontal_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 2:
        chroma_interpolation_filter_one_d_horizontal_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 3:
        chroma_interpolation_filter_one_d_horizontal_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 4:
        chroma_interpolation_filter_one_d_horizontal_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 5:
        chroma_interpolation_filter_one_d_horizontal_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 6:
        chroma_interpolation_filter_one_d_horizontal_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 7:
        chroma_interpolation_filter_one_d_horizontal_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 8:
        chroma_interpolation_filter_one_d_vertical_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 9:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 10:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 11:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 12:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 13:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 14:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 15:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 16:
        chroma_interpolation_filter_one_d_vertical_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 17:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 18:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 19:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 20:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 21:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 22:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 23:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 24:
        chroma_interpolation_filter_one_d_vertical_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 25:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 26:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 27:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 28:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 29:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 30:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 31:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 32:
        chroma_interpolation_filter_one_d_vertical_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 33:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 34:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 35:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 36:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 37:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 38:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 39:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 40:
        chroma_interpolation_filter_one_d_vertical_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 41:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 42:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 43:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 44:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 45:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 46:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 47:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 48:
        chroma_interpolation_filter_one_d_vertical_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 49:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 50:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 51:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 52:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 53:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 54:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 55:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 56:
        chroma_interpolation_filter_one_d_vertical_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 57:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 58:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 59:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 60:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 61:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 62:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 63:
        chroma_interpolation_filter_two_d_ssse3(ref_pic, src_stride, dst, dst_stride, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    }
}

void bi_pred_chroma_if_helper(
    EbByte               ref_pic,
    uint32_t                src_stride,
    int16_t               *dst,
    uint32_t                pu_width,
    uint32_t                pu_height,
    int16_t               *first_pass_if_dst,
    uint32_t                frac_pos_x,
    uint32_t                frac_pos_y,
    uint8_t             choice) {

    switch (choice) {

    case 0:
        chroma_interpolation_copy_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 1:
        chroma_interpolation_filter_one_d_out_raw_horizontal_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 2:
        chroma_interpolation_filter_one_d_out_raw_horizontal_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 3:
        chroma_interpolation_filter_one_d_out_raw_horizontal_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 4:
        chroma_interpolation_filter_one_d_out_raw_horizontal_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 5:
        chroma_interpolation_filter_one_d_out_raw_horizontal_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 6:
        chroma_interpolation_filter_one_d_out_raw_horizontal_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 7:
        chroma_interpolation_filter_one_d_out_raw_horizontal_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 8:
        chroma_interpolation_filter_one_d_out_raw_vertical_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 9:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 10:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 11:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 12:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 13:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 14:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 15:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 16:
        chroma_interpolation_filter_one_d_out_raw_vertical_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 17:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 18:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 19:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 20:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 21:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 22:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 23:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 24:
        chroma_interpolation_filter_one_d_out_raw_vertical_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 25:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 26:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 27:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 28:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 29:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 30:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 31:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 32:
        chroma_interpolation_filter_one_d_out_raw_vertical_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 33:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 34:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 35:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 36:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 37:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 38:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 39:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 40:
        chroma_interpolation_filter_one_d_out_raw_vertical_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 41:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 42:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 43:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 44:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 45:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 46:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 47:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 48:
        chroma_interpolation_filter_one_d_out_raw_vertical_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 49:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 50:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 51:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 52:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 53:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 54:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 55:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 56:
        chroma_interpolation_filter_one_d_out_raw_vertical_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 57:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 58:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 59:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 60:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 61:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 62:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    case 63:
        chroma_interpolation_filter_two_d_out_raw_ssse3(ref_pic, src_stride, dst, pu_width, pu_height, first_pass_if_dst, frac_pos_x, frac_pos_y); break;
    }
}
