/*
 * Copyright (c) 2018, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#ifndef AOM_DSP_X86_CONVOLVE_AVX2_H_
#define AOM_DSP_X86_CONVOLVE_AVX2_H_

#include "convolve.h"
#include "EbInterPrediction.h"
#include "EbMemory_AVX2.h"
#include "synonyms.h"

 // filters for 16
DECLARE_ALIGNED(32, static const uint8_t, filt1_global_avx2[32]) = {
  0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8,
  0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8
};

DECLARE_ALIGNED(32, static const uint8_t, filt2_global_avx2[32]) = {
  2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10,
  2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10
};

DECLARE_ALIGNED(32, static const uint8_t, filt3_global_avx2[32]) = {
  4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12,
  4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12
};

DECLARE_ALIGNED(32, static const uint8_t, filt4_global_avx2[32]) = {
  6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14,
  6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14
};

static INLINE EbBool is_convolve_2tap(const int16_t *const filter) {
    return (EbBool)((InterpKernel *)filter == bilinear_filters);
}

static INLINE EbBool is_convolve_4tap(const int16_t *const filter) {
    return (EbBool)(((InterpKernel *)filter == sub_pel_filters_4) ||
        ((InterpKernel *)filter == sub_pel_filters_4smooth));
}

static INLINE EbBool is_convolve_6tap(const int16_t *const filter) {
    return (EbBool)(((InterpKernel *)filter == sub_pel_filters_8) ||
        ((InterpKernel *)filter == sub_pel_filters_8smooth));
}

static INLINE int32_t get_convolve_tap(const int16_t *const filter) {
    if (is_convolve_2tap(filter)) return 2;
    else if (is_convolve_4tap(filter)) return 4;
    else if (is_convolve_6tap(filter)) return 6;
    else return 8;
}

static INLINE void prepare_half_coeffs_2tap_ssse3(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m128i *const coeffs /* [4] */) {
    const int16_t *const filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);
    const __m128i coeffs_8 = _mm_cvtsi32_si128(*(const int32_t *)(filter + 3));

    // right shift all filter co-efficients by 1 to reduce the bits required.
    // This extra right shift will be taken care of at the end while rounding
    // the result.
    // Since all filter co-efficients are even, this change will not affect the
    // end result
    assert(_mm_test_all_zeros(_mm_and_si128(coeffs_8, _mm_set1_epi16(1)),
        _mm_set1_epi16((short)0xffff)));

    const __m128i coeffs_1 = _mm_srai_epi16(coeffs_8, 1);

    // coeffs 3 4 3 4 3 4 3 4
    *coeffs = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0200u));
}

static INLINE void prepare_half_coeffs_4tap_ssse3(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m128i *const coeffs /* [2] */) {
    const int16_t *const filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);
    const __m128i coeffs_8 = _mm_load_si128((__m128i *)filter);

    // right shift all filter co-efficients by 1 to reduce the bits required.
    // This extra right shift will be taken care of at the end while rounding
    // the result.
    // Since all filter co-efficients are even, this change will not affect the
    // end result
    assert(_mm_test_all_zeros(_mm_and_si128(coeffs_8, _mm_set1_epi16(1)),
        _mm_set1_epi16((short)0xffff)));

    const __m128i coeffs_1 = _mm_srai_epi16(coeffs_8, 1);

    // coeffs 2 3 2 3 2 3 2 3
    coeffs[0] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0604u));
    // coeffs 4 5 4 5 4 5 4 5
    coeffs[1] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0a08u));
}

static INLINE void prepare_half_coeffs_6tap_ssse3(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m128i *const coeffs /* [4] */) {
    const int16_t *const filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);
    const __m128i coeffs_8 = _mm_load_si128((__m128i *)filter);

    // right shift all filter co-efficients by 1 to reduce the bits required.
    // This extra right shift will be taken care of at the end while rounding
    // the result.
    // Since all filter co-efficients are even, this change will not affect the
    // end result
    assert(_mm_test_all_zeros(_mm_and_si128(coeffs_8, _mm_set1_epi16(1)),
        _mm_set1_epi16((short)0xffff)));

    const __m128i coeffs_1 = _mm_srai_epi16(coeffs_8, 1);

    // coeffs 1 2 1 2 1 2 1 2
    coeffs[0] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0402u));
    // coeffs 3 4 3 4 3 4 3 4
    coeffs[1] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0806u));
    // coeffs 5 6 5 6 5 6 5 6
    coeffs[2] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0C0Au));
}

static INLINE void prepare_half_coeffs_8tap_ssse3(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m128i *const coeffs /* [4] */) {
    const int16_t *const filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);
    const __m128i coeffs_8 = _mm_load_si128((__m128i *)filter);

    // right shift all filter co-efficients by 1 to reduce the bits required.
    // This extra right shift will be taken care of at the end while rounding
    // the result.
    // Since all filter co-efficients are even, this change will not affect the
    // end result
    assert(_mm_test_all_zeros(_mm_and_si128(coeffs_8, _mm_set1_epi16(1)),
        _mm_set1_epi16((short)0xffff)));

    const __m128i coeffs_1 = _mm_srai_epi16(coeffs_8, 1);

    // coeffs 0 1 0 1 0 1 0 1
    coeffs[0] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0200u));
    // coeffs 2 3 2 3 2 3 2 3
    coeffs[1] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0604u));
    // coeffs 4 5 4 5 4 5 4 5
    coeffs[2] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0a08u));
    // coeffs 6 7 6 7 6 7 6 7
    coeffs[3] = _mm_shuffle_epi8(coeffs_1, _mm_set1_epi16(0x0e0cu));
}

static INLINE void prepare_half_coeffs_2tap_avx2(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m256i *const coeffs /* [4] */) {
    const int16_t *const filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);
    const __m128i coeffs_8 = _mm_cvtsi32_si128(*(const int32_t *)(filter + 3));
    const __m256i filter_coeffs = _mm256_broadcastsi128_si256(coeffs_8);

    // right shift all filter co-efficients by 1 to reduce the bits required.
    // This extra right shift will be taken care of at the end while rounding
    // the result.
    // Since all filter co-efficients are even, this change will not affect the
    // end result
    assert(_mm_test_all_zeros(_mm_and_si128(coeffs_8, _mm_set1_epi16(1)),
        _mm_set1_epi16((short)0xffff)));

    const __m256i coeffs_1 = _mm256_srai_epi16(filter_coeffs, 1);

    // coeffs 3 4 3 4 3 4 3 4
    *coeffs = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0200u));
}

static INLINE void prepare_half_coeffs_4tap_avx2(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m256i *const coeffs /* [3] */) {
    const int16_t *const filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);
    const __m128i coeffs_8 = _mm_load_si128((__m128i *)filter);
    const __m256i filter_coeffs = _mm256_broadcastsi128_si256(coeffs_8);

    // right shift all filter co-efficients by 1 to reduce the bits required.
    // This extra right shift will be taken care of at the end while rounding
    // the result.
    // Since all filter co-efficients are even, this change will not affect the
    // end result
    assert(_mm_test_all_zeros(_mm_and_si128(coeffs_8, _mm_set1_epi16(1)),
        _mm_set1_epi16((short)0xffff)));

    const __m256i coeffs_1 = _mm256_srai_epi16(filter_coeffs, 1);

    // coeffs 2 3 2 3 2 3 2 3
    coeffs[0] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0604u));
    // coeffs 4 5 4 5 4 5 4 5
    coeffs[1] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0a08u));
}

static INLINE void prepare_half_coeffs_6tap_avx2(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m256i *const coeffs /* [3] */) {
    const int16_t *const filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);
    const __m128i coeffs_8 = _mm_load_si128((__m128i *)filter);
    const __m256i filter_coeffs = _mm256_broadcastsi128_si256(coeffs_8);

    // right shift all filter co-efficients by 1 to reduce the bits required.
    // This extra right shift will be taken care of at the end while rounding
    // the result.
    // Since all filter co-efficients are even, this change will not affect the
    // end result
    assert(_mm_test_all_zeros(_mm_and_si128(coeffs_8, _mm_set1_epi16(1)),
        _mm_set1_epi16((short)0xffff)));

    const __m256i coeffs_1 = _mm256_srai_epi16(filter_coeffs, 1);

    // coeffs 1 2 1 2 1 2 1 2
    coeffs[0] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0402u));
    // coeffs 3 4 3 4 3 4 3 4
    coeffs[1] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0806u));
    // coeffs 5 6 5 6 5 6 5 6
    coeffs[2] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0C0Au));
}

static INLINE void prepare_half_coeffs_8tap_avx2(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m256i *const coeffs /* [4] */) {
    const int16_t *const filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);
    const __m128i coeffs_8 = _mm_load_si128((__m128i *)filter);
    const __m256i filter_coeffs = _mm256_broadcastsi128_si256(coeffs_8);

    // right shift all filter co-efficients by 1 to reduce the bits required.
    // This extra right shift will be taken care of at the end while rounding
    // the result.
    // Since all filter co-efficients are even, this change will not affect the
    // end result
    assert(_mm_test_all_zeros(_mm_and_si128(coeffs_8, _mm_set1_epi16(1)),
        _mm_set1_epi16((short)0xffff)));

    const __m256i coeffs_1 = _mm256_srai_epi16(filter_coeffs, 1);

    // coeffs 0 1 0 1 0 1 0 1
    coeffs[0] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0200u));
    // coeffs 2 3 2 3 2 3 2 3
    coeffs[1] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0604u));
    // coeffs 4 5 4 5 4 5 4 5
    coeffs[2] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0a08u));
    // coeffs 6 7 6 7 6 7 6 7
    coeffs[3] = _mm256_shuffle_epi8(coeffs_1, _mm256_set1_epi16(0x0e0cu));
}

static INLINE void prepare_coeffs_2tap_avx2(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m256i *const coeffs /* [4] */) {
    const int16_t *filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);

    const __m128i coeff_8 = _mm_cvtsi32_si128(*(const int32_t *)(filter + 3));
    const __m256i coeff = _mm256_broadcastsi128_si256(coeff_8);

    // coeffs 3 4 3 4 3 4 3 4
    coeffs[0] = _mm256_shuffle_epi32(coeff, 0x00);
}

static INLINE void prepare_coeffs_8tap_avx2(
    const InterpFilterParams *const filter_params, const int32_t subpel_q4,
    __m256i *const coeffs /* [4] */) {
    const int16_t *filter = av1_get_interp_filter_subpel_kernel(
        *filter_params, subpel_q4 & SUBPEL_MASK);

    const __m128i coeff_8 = _mm_load_si128((__m128i *)filter);
    const __m256i coeff = _mm256_broadcastsi128_si256(coeff_8);

    // coeffs 0 1 0 1 0 1 0 1
    coeffs[0] = _mm256_shuffle_epi32(coeff, 0x00);
    // coeffs 2 3 2 3 2 3 2 3
    coeffs[1] = _mm256_shuffle_epi32(coeff, 0x55);
    // coeffs 4 5 4 5 4 5 4 5
    coeffs[2] = _mm256_shuffle_epi32(coeff, 0xaa);
    // coeffs 6 7 6 7 6 7 6 7
    coeffs[3] = _mm256_shuffle_epi32(coeff, 0xff);
}

static INLINE __m128i convolve_2tap_ssse3(const __m128i *const s,
    const __m128i *const coeffs) {
    return _mm_maddubs_epi16(s[0], coeffs[0]);
}

static INLINE __m128i convolve_4tap_ssse3(const __m128i *const s,
    const __m128i *const coeffs) {
    const __m128i res_23 = _mm_maddubs_epi16(s[0], coeffs[0]);
    const __m128i res_45 = _mm_maddubs_epi16(s[1], coeffs[1]);
    return _mm_add_epi16(res_23, res_45);
}

static INLINE __m128i convolve_6tap_ssse3(const __m128i *const s,
    const __m128i *const coeffs) {
    const __m128i res_12 = _mm_maddubs_epi16(s[0], coeffs[0]);
    const __m128i res_34 = _mm_maddubs_epi16(s[1], coeffs[1]);
    const __m128i res_56 = _mm_maddubs_epi16(s[2], coeffs[2]);
    const __m128i res_1256 = _mm_add_epi16(res_12, res_56);
    return _mm_add_epi16(res_1256, res_34);
}

static INLINE __m128i convolve_8tap_ssse3(const __m128i *const s,
    const __m128i *const coeffs) {
    const __m128i res_01 = _mm_maddubs_epi16(s[0], coeffs[0]);
    const __m128i res_23 = _mm_maddubs_epi16(s[1], coeffs[1]);
    const __m128i res_45 = _mm_maddubs_epi16(s[2], coeffs[2]);
    const __m128i res_67 = _mm_maddubs_epi16(s[3], coeffs[3]);
    const __m128i res_0145 = _mm_add_epi16(res_01, res_45);
    const __m128i res_2367 = _mm_add_epi16(res_23, res_67);
    return _mm_add_epi16(res_0145, res_2367);
}

static INLINE __m256i convolve_2tap_avx2(const __m256i *const s,
    const __m256i *const coeffs) {
    return _mm256_maddubs_epi16(s[0], coeffs[0]);
}

static INLINE __m256i convolve_4tap_avx2(const __m256i *const s,
    const __m256i *const coeffs) {
    const __m256i res_23 = _mm256_maddubs_epi16(s[0], coeffs[0]);
    const __m256i res_45 = _mm256_maddubs_epi16(s[1], coeffs[1]);
    return _mm256_add_epi16(res_23, res_45);
}

static INLINE __m256i convolve_6tap_avx2(const __m256i *const s,
    const __m256i *const coeffs) {
    const __m256i res_01 = _mm256_maddubs_epi16(s[0], coeffs[0]);
    const __m256i res_23 = _mm256_maddubs_epi16(s[1], coeffs[1]);
    const __m256i res_45 = _mm256_maddubs_epi16(s[2], coeffs[2]);
    const __m256i res_0145 = _mm256_add_epi16(res_01, res_45);
    return _mm256_add_epi16(res_0145, res_23);
}

static INLINE __m256i convolve_8tap_avx2(const __m256i *const s,
    const __m256i *const coeffs) {
    const __m256i res_01 = _mm256_maddubs_epi16(s[0], coeffs[0]);
    const __m256i res_23 = _mm256_maddubs_epi16(s[1], coeffs[1]);
    const __m256i res_45 = _mm256_maddubs_epi16(s[2], coeffs[2]);
    const __m256i res_67 = _mm256_maddubs_epi16(s[3], coeffs[3]);
    const __m256i res_0145 = _mm256_add_epi16(res_01, res_45);
    const __m256i res_2367 = _mm256_add_epi16(res_23, res_67);
    return _mm256_add_epi16(res_0145, res_2367);
}

static INLINE __m256i convolve_2tap(const __m256i *const s,
    const __m256i *const coeffs) {
    return _mm256_madd_epi16(s[0], coeffs[0]);
}

static INLINE __m256i convolve_4tap(const __m256i *const s,
    const __m256i *const coeffs) {
    const __m256i res_1 = _mm256_madd_epi16(s[0], coeffs[0]);
    const __m256i res_2 = _mm256_madd_epi16(s[1], coeffs[1]);
    return _mm256_add_epi32(res_1, res_2);
}

static INLINE __m256i convolve_8tap(const __m256i *const s,
    const __m256i *const coeffs) {
    const __m256i res_01 = _mm256_madd_epi16(s[0], coeffs[0]);
    const __m256i res_23 = _mm256_madd_epi16(s[1], coeffs[1]);
    const __m256i res_45 = _mm256_madd_epi16(s[2], coeffs[2]);
    const __m256i res_67 = _mm256_madd_epi16(s[3], coeffs[3]);
    const __m256i res_0123 = _mm256_add_epi32(res_01, res_23);
    const __m256i res_4567 = _mm256_add_epi32(res_45, res_67);
    return _mm256_add_epi32(res_0123, res_4567);
}

static INLINE __m256i convolve_x_2tap(const __m256i data,
    const __m256i *const coeffs, const __m256i *const filt) {
    const __m256i s = _mm256_shuffle_epi8(data, filt[0]);
    return convolve_2tap_avx2(&s, coeffs);
}

static INLINE __m256i convolve_x_4tap(const __m256i data,
    const __m256i *const coeffs, const __m256i *const filt) {
    __m256i s[2];

    s[0] = _mm256_shuffle_epi8(data, filt[0]);
    s[1] = _mm256_shuffle_epi8(data, filt[1]);

    return convolve_4tap_avx2(s, coeffs);
}

static INLINE __m256i convolve_x_6tap_avx2(const __m256i data,
    const __m256i *const coeffs, const __m256i *const filt) {
    __m256i s[3];

    s[0] = _mm256_shuffle_epi8(data, filt[0]);
    s[1] = _mm256_shuffle_epi8(data, filt[1]);
    s[2] = _mm256_shuffle_epi8(data, filt[2]);

    return convolve_6tap_avx2(s, coeffs);
}

static INLINE __m256i convolve_x_8tap_avx2(const __m256i data,
    const __m256i *const coeffs, const __m256i *const filt) {
    __m256i s[4];

    s[0] = _mm256_shuffle_epi8(data, filt[0]);
    s[1] = _mm256_shuffle_epi8(data, filt[1]);
    s[2] = _mm256_shuffle_epi8(data, filt[2]);
    s[3] = _mm256_shuffle_epi8(data, filt[3]);

    return convolve_8tap_avx2(s, coeffs);
}

static INLINE __m128i convolve_x_round_sse2(const __m128i src) {
    const __m128i round = _mm_set1_epi16(34);
    const __m128i dst = _mm_add_epi16(src, round);
    return _mm_srai_epi16(dst, 6);
}

static INLINE __m256i convolve_x_round_avx2(const __m256i src) {
    const __m256i round = _mm256_set1_epi16(34);
    const __m256i dst = _mm256_add_epi16(src, round);
    return _mm256_srai_epi16(dst, 6);
}

static INLINE __m128i convolve_y_round_sse2(const __m128i src) {
    const __m128i round = _mm_set1_epi16(32);
    const __m128i dst = _mm_add_epi16(src, round);
    return _mm_srai_epi16(dst, 6);
}

static INLINE __m256i convolve_y_round_avx2(const __m256i src) {
    const __m256i round = _mm256_set1_epi16(32);
    const __m256i dst = _mm256_add_epi16(src, round);
    return _mm256_srai_epi16(dst, FILTER_BITS - 1);
}

static INLINE void convolve_store_2x2_sse2(const __m128i res,
    uint8_t *const dst, const int32_t dst_stride)
{
    const __m128i d = _mm_packus_epi16(res, res);
    *(int16_t *)dst = (int16_t)_mm_cvtsi128_si32(d);
    *(int16_t *)(dst + dst_stride) = (int16_t)_mm_extract_epi16(d, 1);
}

static INLINE void convolve_store_4x2_sse2(const __m128i res,
    uint8_t *const dst, const int32_t dst_stride)
{
    const __m128i d = _mm_packus_epi16(res, res);
    xx_storel_32(dst, d);
    *(int32_t *)(dst + dst_stride) = _mm_extract_epi32(d, 1);
}

static INLINE void convolve_store_8x2_avx2(const __m256i res,
    uint8_t *const dst, const int32_t dst_stride)
{
    const __m256i d = _mm256_packus_epi16(res, res);
    const __m128i d0 = _mm256_castsi256_si128(d);
    const __m128i d1 = _mm256_extracti128_si256(d, 1);
    _mm_storel_epi64((__m128i *)dst, d0);
    _mm_storel_epi64((__m128i *)(dst + dst_stride), d1);
}

static INLINE void convolve_store_16x2_avx2(const __m256i res0,
    const __m256i res1, uint8_t *const dst, const int32_t dst_stride)
{
    const __m256i d = _mm256_packus_epi16(res0, res1);
    const __m128i d0 = _mm256_castsi256_si128(d);
    const __m128i d1 = _mm256_extracti128_si256(d, 1);
    _mm_storeu_si128((__m128i *)dst, d0);
    _mm_storeu_si128((__m128i *)(dst + dst_stride), d1);
}

static INLINE void convolve_store_32_avx2(const __m256i res0,
    const __m256i res1, uint8_t *const dst)
{
    const __m256i d = _mm256_packus_epi16(res0, res1);
    _mm256_storeu_si256((__m256i *)dst, d);
}

static INLINE __m128i convolve_x_8_4tap_kernel_ssse3(const __m128i src[2],
    const __m128i coeffs[2]) {
    const __m128i res = convolve_4tap_ssse3(src, coeffs);
    const __m128i r = convolve_x_round_sse2(res);
    return _mm_packus_epi16(r, r);
}

static INLINE void convolve_x_32_2tap_avx2(const uint8_t *const src,
    const __m256i *const coeffs, uint8_t *const dst) {
    const __m256i s0 = _mm256_loadu_si256((__m256i *)src);
    const __m256i s1 = _mm256_loadu_si256((__m256i *)(src + 1));
    const __m256i ss0 = _mm256_unpacklo_epi8(s0, s1);
    const __m256i ss1 = _mm256_unpackhi_epi8(s0, s1);
    const __m256i res0 = convolve_2tap_avx2(&ss0, coeffs);
    const __m256i res1 = convolve_2tap_avx2(&ss1, coeffs);
    const __m256i r0 = convolve_x_round_avx2(res0);
    const __m256i r1 = convolve_x_round_avx2(res1);
    convolve_store_32_avx2(r0, r1, dst);
}

static AOM_FORCE_INLINE void convolve_x_16x2_6tap_avx2(
    const uint8_t *const src, const int32_t src_stride,
    const __m256i *const coeffs, const __m256i *const filt, uint8_t *const dst,
    const int32_t dst_stride) {
    const __m128i s0_128 = _mm_loadu_si128((__m128i *)src);
    const __m128i s1_128 = _mm_loadu_si128((__m128i *)(src + src_stride));
    const __m128i s2_128 = _mm_loadu_si128((__m128i *)(src + 8));
    const __m128i s3_128 = _mm_loadu_si128((__m128i *)(src + src_stride + 8));
    const __m256i s0_256 = _mm256_setr_m128i(s0_128, s1_128);
    const __m256i s1_256 = _mm256_setr_m128i(s2_128, s3_128);
    const __m256i res0 = convolve_x_6tap_avx2(s0_256, coeffs, filt);
    const __m256i res1 = convolve_x_6tap_avx2(s1_256, coeffs, filt);
    const __m256i r0 = convolve_x_round_avx2(res0);
    const __m256i r1 = convolve_x_round_avx2(res1);
    convolve_store_16x2_avx2(r0, r1, dst, dst_stride);
}

static AOM_FORCE_INLINE void convolve_x_16x2_8tap_avx2(
    const uint8_t *const src, const int32_t src_stride,
    const __m256i *const coeffs, const __m256i *const filt, uint8_t *const dst,
    const int32_t dst_stride) {
    const __m128i s0_128 = _mm_loadu_si128((__m128i *)src);
    const __m128i s1_128 = _mm_loadu_si128((__m128i *)(src + src_stride));
    const __m128i s2_128 = _mm_loadu_si128((__m128i *)(src + 8));
    const __m128i s3_128 = _mm_loadu_si128((__m128i *)(src + src_stride + 8));
    const __m256i s0_256 = _mm256_setr_m128i(s0_128, s1_128);
    const __m256i s1_256 = _mm256_setr_m128i(s2_128, s3_128);
    const __m256i res0 = convolve_x_8tap_avx2(s0_256, coeffs, filt);
    const __m256i res1 = convolve_x_8tap_avx2(s1_256, coeffs, filt);
    const __m256i r0 = convolve_x_round_avx2(res0);
    const __m256i r1 = convolve_x_round_avx2(res1);
    convolve_store_16x2_avx2(r0, r1, dst, dst_stride);
}

static INLINE void convolve_x_32_2tap_avg(const uint8_t *const src,
    uint8_t *const dst) {
    const __m256i s0 = _mm256_loadu_si256((__m256i *)src);
    const __m256i s1 = _mm256_loadu_si256((__m256i *)(src + 1));
    const __m256i d = _mm256_avg_epu8(s0, s1);
    _mm256_storeu_si256((__m256i *)dst, d);
}

static INLINE void convolve_y_32_2tap_avg(const __m256i s0, const __m256i s1,
    uint8_t *const dst) {
    const __m256i d = _mm256_avg_epu8(s0, s1);
    _mm256_storeu_si256((__m256i *)dst, d);
}

static INLINE void convolve_y_32_2tap_avx2(const __m256i s0, const __m256i s1,
    const __m256i *const coeffs, uint8_t *const dst)
{
    const __m256i ss0 = _mm256_unpacklo_epi8(s0, s1);
    const __m256i ss1 = _mm256_unpackhi_epi8(s0, s1);
    const __m256i res0 = convolve_2tap_avx2(&ss0, coeffs);
    const __m256i res1 = convolve_2tap_avx2(&ss1, coeffs);
    const __m256i r0 = convolve_y_round_avx2(res0);
    const __m256i r1 = convolve_y_round_avx2(res1);
    convolve_store_32_avx2(r0, r1, dst);
}

static INLINE void convolve_y_32_6tap_avx2(const __m256i src[6],
    const __m256i *const coeffs, uint8_t *const dst)
{
    const __m256i res0 = convolve_6tap_avx2(src + 0, coeffs);
    const __m256i res1 = convolve_6tap_avx2(src + 3, coeffs);
    const __m256i r0 = convolve_y_round_avx2(res0);
    const __m256i r1 = convolve_y_round_avx2(res1);
    convolve_store_32_avx2(r0, r1, dst);
}

static INLINE void convolve_y_32_8tap_avx2(const __m256i src[8],
    const __m256i *const coeffs, uint8_t *const dst)
{
    const __m256i res0 = convolve_8tap_avx2(src + 0, coeffs);
    const __m256i res1 = convolve_8tap_avx2(src + 4, coeffs);
    const __m256i r0 = convolve_y_round_avx2(res0);
    const __m256i r1 = convolve_y_round_avx2(res1);
    convolve_store_32_avx2(r0, r1, dst);
}

static INLINE void add_store_aligned_256(ConvBufType *const dst,
    const __m256i *const res,
    const int32_t do_average) {
    __m256i d;
    if (do_average) {
        d = _mm256_load_si256((__m256i *)dst);
        d = _mm256_add_epi32(d, *res);
        d = _mm256_srai_epi32(d, 1);
    }
    else
        d = *res;
    _mm256_store_si256((__m256i *)dst, d);
}

static INLINE __m256i comp_avg(const __m256i *const data_ref_0,
    const __m256i *const res_unsigned,
    const __m256i *const wt,
    const int32_t use_jnt_comp_avg) {
    __m256i res;
    if (use_jnt_comp_avg) {
        const __m256i data_lo = _mm256_unpacklo_epi16(*data_ref_0, *res_unsigned);
        const __m256i data_hi = _mm256_unpackhi_epi16(*data_ref_0, *res_unsigned);

        const __m256i wt_res_lo = _mm256_madd_epi16(data_lo, *wt);
        const __m256i wt_res_hi = _mm256_madd_epi16(data_hi, *wt);

        const __m256i res_lo = _mm256_srai_epi32(wt_res_lo, DIST_PRECISION_BITS);
        const __m256i res_hi = _mm256_srai_epi32(wt_res_hi, DIST_PRECISION_BITS);

        res = _mm256_packs_epi32(res_lo, res_hi);
    }
    else {
        const __m256i wt_res = _mm256_add_epi16(*data_ref_0, *res_unsigned);
        res = _mm256_srai_epi16(wt_res, 1);
    }
    return res;
}

static INLINE __m256i convolve_rounding(const __m256i *const res_unsigned,
    const __m256i *const offset_const,
    const __m256i *const round_const,
    const int32_t round_shift) {
    const __m256i res_signed = _mm256_sub_epi16(*res_unsigned, *offset_const);
    const __m256i res_round = _mm256_srai_epi16(
        _mm256_add_epi16(res_signed, *round_const), round_shift);
    return res_round;
}

static INLINE __m256i highbd_comp_avg(const __m256i *const data_ref_0,
    const __m256i *const res_unsigned,
    const __m256i *const wt0,
    const __m256i *const wt1,
    const int32_t use_jnt_comp_avg) {
    __m256i res;
    if (use_jnt_comp_avg) {
        const __m256i wt0_res = _mm256_mullo_epi32(*data_ref_0, *wt0);
        const __m256i wt1_res = _mm256_mullo_epi32(*res_unsigned, *wt1);
        const __m256i wt_res = _mm256_add_epi32(wt0_res, wt1_res);
        res = _mm256_srai_epi32(wt_res, DIST_PRECISION_BITS);
    }
    else {
        const __m256i wt_res = _mm256_add_epi32(*data_ref_0, *res_unsigned);
        res = _mm256_srai_epi32(wt_res, 1);
    }
    return res;
}

static INLINE __m256i highbd_convolve_rounding(
    const __m256i *const res_unsigned, const __m256i *const offset_const,
    const __m256i *const round_const, const int32_t round_shift) {
    const __m256i res_signed = _mm256_sub_epi32(*res_unsigned, *offset_const);
    const __m256i res_round = _mm256_srai_epi32(
        _mm256_add_epi32(res_signed, *round_const), round_shift);

    return res_round;
}

#define CONVOLVE_SR_HORIZONTAL_FILTER_2TAP                                     \
    for (i = 0; i < (im_h - 2); i += 2) {                                      \
        __m256i data = _mm256_castsi128_si256(                                 \
            _mm_loadu_si128((__m128i *)&src_ptr[(i * src_stride) + j]));       \
                                                                               \
        data = _mm256_inserti128_si256(                                        \
            data,                                                              \
            _mm_loadu_si128(                                                   \
            (__m128i *)&src_ptr[(i * src_stride) + j + src_stride]),           \
            1);                                                                \
        __m256i res = convolve_x_2tap(data, coeffs_h, filt);                   \
                                                                               \
        res = _mm256_sra_epi16(_mm256_add_epi16(res, round_const_h),           \
            round_shift_h);                                                    \
        _mm256_store_si256((__m256i *)&im_block[i * im_stride], res);          \
    }                                                                          \
                                                                               \
    __m256i data_1 = _mm256_castsi128_si256(                                   \
        _mm_loadu_si128((__m128i *)&src_ptr[(i * src_stride) + j]));           \
                                                                               \
    __m256i res = convolve_x_2tap(data_1, coeffs_h, filt);                     \
    res =                                                                      \
        _mm256_sra_epi16(_mm256_add_epi16(res, round_const_h), round_shift_h); \
    _mm256_store_si256((__m256i *)&im_block[i * im_stride], res);

#define CONVOLVE_SR_HORIZONTAL_FILTER_4TAP                                     \
    for (i = 0; i < (im_h - 2); i += 2) {                                      \
        __m256i data = _mm256_castsi128_si256(                                 \
            _mm_loadu_si128((__m128i *)&src_ptr[(i * src_stride) + j]));       \
                                                                               \
        data = _mm256_inserti128_si256(                                        \
            data,                                                              \
            _mm_loadu_si128(                                                   \
            (__m128i *)&src_ptr[(i * src_stride) + j + src_stride]),           \
            1);                                                                \
        __m256i res = convolve_x_4tap(data, coeffs_h + 1, filt);               \
                                                                               \
        res = _mm256_sra_epi16(_mm256_add_epi16(res, round_const_h),           \
            round_shift_h);                                                    \
        _mm256_store_si256((__m256i *)&im_block[i * im_stride], res);          \
    }                                                                          \
                                                                               \
    __m256i data_1 = _mm256_castsi128_si256(                                   \
        _mm_loadu_si128((__m128i *)&src_ptr[(i * src_stride) + j]));           \
                                                                               \
    __m256i res = convolve_x_4tap(data_1, coeffs_h + 1, filt);                 \
    res =                                                                      \
        _mm256_sra_epi16(_mm256_add_epi16(res, round_const_h), round_shift_h); \
    _mm256_store_si256((__m256i *)&im_block[i * im_stride], res);

#define CONVOLVE_SR_HORIZONTAL_FILTER_8TAP                                     \
  for (i = 0; i < (im_h - 2); i += 2) {                                        \
    __m256i data = _mm256_castsi128_si256(                                     \
        _mm_loadu_si128((__m128i *)&src_ptr[(i * src_stride) + j]));           \
    data = _mm256_inserti128_si256(                                            \
        data,                                                                  \
        _mm_loadu_si128(                                                       \
            (__m128i *)&src_ptr[(i * src_stride) + j + src_stride]),           \
        1);                                                                    \
                                                                               \
    __m256i res = convolve_x_8tap_avx2(data, coeffs_h, filt);                  \
    res =                                                                      \
        _mm256_sra_epi16(_mm256_add_epi16(res, round_const_h), round_shift_h); \
    _mm256_store_si256((__m256i *)&im_block[i * im_stride], res);              \
  }                                                                            \
                                                                               \
  __m256i data_1 = _mm256_castsi128_si256(                                     \
      _mm_loadu_si128((__m128i *)&src_ptr[(i * src_stride) + j]));             \
                                                                               \
  __m256i res = convolve_x_8tap_avx2(data_1, coeffs_h, filt);                  \
                                                                               \
  res = _mm256_sra_epi16(_mm256_add_epi16(res, round_const_h), round_shift_h); \
                                                                               \
  _mm256_store_si256((__m256i *)&im_block[i * im_stride], res);

#endif
