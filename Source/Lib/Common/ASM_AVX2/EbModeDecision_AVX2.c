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
#include "EbDefinitions.h"
#include <immintrin.h>

#if ESTIMATE_INTRA
// Indices are sign, integer, and fractional part of the gradient value
static const uint8_t gradient_to_angle_bin[2][7][16] = {
  {
      { 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1 },
      { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
      { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
      { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
      { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
      { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
  },
  {
      { 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4 },
      { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3 },
      { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 },
      { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 },
      { 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3 },
      { 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
      { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
  },
};

static INLINE __m256i __m256i_div_epi32(const __m256i *a, const __m256i *b)
{
    __m256 d_f = _mm256_div_ps(_mm256_cvtepi32_ps(*a), _mm256_cvtepi32_ps(*b));
    //Integer devide round down
    return _mm256_cvtps_epi32(_mm256_floor_ps(d_f));
}

static INLINE void get_gradient_hist_avx2_internal(const __m256i *src1,
    const __m256i *src2, const __m256i *src3, int16_t *dy_mask_array,
    int16_t *quot_array, int16_t *remd_array, int16_t * sn_array,
    int32_t *temp_array) {

    const __m256i zero = _mm256_setzero_si256();
    const __m256i val_15_i16 = _mm256_set1_epi16(15);
    const __m256i val_6_i16 = _mm256_set1_epi16(6);
    __m256i dx, dy;
    __m256i tmp1_32, tmp2_32;
    __m256i dx1_32, dx2_32;
    __m256i dy1_32, dy2_32;
    __m256i sn;
    __m256i remd;
    __m256i quot;
    __m256i dy_mask;

    dx = _mm256_sub_epi16(*src1, *src2);
    dy = _mm256_sub_epi16(*src1, *src3);

    //sn = (dx > 0) ^ (dy > 0);
    sn = _mm256_xor_si256(dx, dy);  //result is 0 or 0xFFFF
    sn = _mm256_srli_epi16(sn, 15);  //change output from 0xFFFF to 1

    //mask which shows where are zeros in dy register 0/1
    dy_mask = _mm256_srli_epi16(_mm256_cmpeq_epi16(dy, zero), 15);

    //dx = abs(dx); dy = abs(dy);
    dx = _mm256_abs_epi16(dx);
    dy = _mm256_abs_epi16(dy);

    _mm256_add_epi16(dy, dy_mask);

    //  temp = dx * dx + dy * dy;
    dx1_32 = _mm256_cvtepi16_epi32(_mm256_castsi256_si128(dx)); //dx
    dy1_32 = _mm256_cvtepi16_epi32(_mm256_castsi256_si128(dy)); //dy

    tmp1_32 = _mm256_add_epi32(
        _mm256_mullo_epi32(dx1_32, dx1_32),
        _mm256_mullo_epi32(dy1_32, dy1_32));

    dx2_32 = _mm256_cvtepi16_epi32(_mm256_extracti128_si256(dx, 1));
    dy2_32 = _mm256_cvtepi16_epi32(_mm256_extracti128_si256(dy, 1));

    tmp2_32 = _mm256_add_epi32(
        _mm256_mullo_epi32(dx2_32, dx2_32),
        _mm256_mullo_epi32(dy2_32, dy2_32));

    /* Code:
     quot16 = (dx << 4) / dy;
     quot = quot16 >> 4;
     remd = = (quot16 & (15));
    Equivalent of:
     quot = dx / dy;
     remd = (dx % dy) * 16 / dy;*/

     //quot16 = (dx << 4) / dy;
    dx1_32 = _mm256_slli_epi32(dx1_32, 4);
    dx2_32 = _mm256_slli_epi32(dx2_32, 4);
    const __m256i d1_i32 = __m256i_div_epi32(&dx1_32, &dy1_32);
    const __m256i d2_i32 = __m256i_div_epi32(&dx2_32, &dy2_32);
    __m256i quot16 = _mm256_permute4x64_epi64(
        _mm256_packs_epi32(d1_i32, d2_i32), 0xD8);

    quot = _mm256_srli_epi16(quot16, 4);

    //remd = (quot16 & (15));
    remd = _mm256_and_si256(quot16, val_15_i16);

    //AOMMIN(remdA, 15)
    remd = _mm256_min_epi16(remd, val_15_i16);
    //AOMMIN(quotA, 6)
    quot = _mm256_min_epi16(quot, val_6_i16);

    _mm256_store_si256((__m256i *)dy_mask_array, dy_mask);
    _mm256_store_si256((__m256i *)quot_array, quot);
    _mm256_store_si256((__m256i *)remd_array, remd);
    _mm256_store_si256((__m256i *)sn_array, sn);
    _mm256_store_si256((__m256i *)temp_array, tmp1_32);
    _mm256_store_si256((__m256i *)&temp_array[8], tmp2_32);
}

void av1_get_gradient_hist_avx2(const uint8_t *src, int src_stride, int rows,
    int cols, uint64_t *hist) {
    src += src_stride;

    __m128i tmp_src;
    __m256i src1; //src[c]
    __m256i src2; //src[c-1]
    __m256i src3; //src[c - src_stride]

    DECLARE_ALIGNED(64, int16_t, dy_mask_array[16]);
    DECLARE_ALIGNED(64, int16_t, quot_array[16]);
    DECLARE_ALIGNED(64, int16_t, remd_array[16]);
    DECLARE_ALIGNED(64, int16_t, sn_array[16]);
    DECLARE_ALIGNED(64, int32_t, temp_array[16]);

    if (cols < 8) { //i.e cols ==4
        for (int r = 1; r < rows; r += 4) {
            if ((r + 3) >= rows) {
                tmp_src = _mm_set_epi32(
                    0,
                    *(uint32_t*)(src + 1),
                    *(uint32_t*)(src + 1 + src_stride),
                    *(uint32_t*)(src + 1 + 2 * src_stride));
                src1 = _mm256_cvtepu8_epi16(tmp_src);

                tmp_src = _mm_set_epi32(
                    0,
                    *(uint32_t*)(src),
                    *(uint32_t*)(src + src_stride),
                    *(uint32_t*)(src + 2 * src_stride));
                src2 = _mm256_cvtepu8_epi16(tmp_src);

                tmp_src = _mm_set_epi32(
                    0,
                    *(uint32_t*)(src + 1 - src_stride),
                    *(uint32_t*)(src + 1),
                    *(uint32_t*)(src + 1 + src_stride));
                src3 = _mm256_cvtepu8_epi16(tmp_src);
            }
            else {
                tmp_src = _mm_set_epi32(
                    *(uint32_t*)(src + 1),
                    *(uint32_t*)(src + 1 + src_stride),
                    *(uint32_t*)(src + 1 + 2 * src_stride),
                    *(uint32_t*)(src + 1 + 3 * src_stride));
                src1 = _mm256_cvtepu8_epi16(tmp_src);

                tmp_src = _mm_set_epi32(
                    *(uint32_t*)(src),
                    *(uint32_t*)(src + src_stride),
                    *(uint32_t*)(src + 2 * src_stride),
                    *(uint32_t*)(src + 3 * src_stride));
                src2 = _mm256_cvtepu8_epi16(tmp_src);

                tmp_src = _mm_set_epi32(
                    *(uint32_t*)(src + 1 - src_stride),
                    *(uint32_t*)(src + 1),
                    *(uint32_t*)(src + 1 + src_stride),
                    *(uint32_t*)(src + 1 + 2 * src_stride));
                src3 = _mm256_cvtepu8_epi16(tmp_src);
            }

            get_gradient_hist_avx2_internal(&src1, &src2, &src3, dy_mask_array,
                quot_array, remd_array, sn_array, temp_array);

            if ((r + 3) >= rows) {
                for (int w = 0; w < 11; ++w) {
                    if (w == 3 || w == 7)
                        continue;
                    if (dy_mask_array[w] != 1) {
                        int index = gradient_to_angle_bin[sn_array[w]]
                            [quot_array[w]][remd_array[w]];
                        hist[index] += temp_array[w];
                    }
                    else {
                        hist[2] += temp_array[w];
                    }
                }
            }
            else {
                for (int w = 0; w < 15; ++w) {
                    if (w == 3 || w == 7 || w == 11)
                        continue;
                    if (dy_mask_array[w] != 1) {
                        int index = gradient_to_angle_bin[sn_array[w]]
                            [quot_array[w]][remd_array[w]];
                        hist[index] += temp_array[w];
                    }
                    else {
                        hist[2] += temp_array[w];
                    }
                }
            }
            src += 4 * src_stride;
        }
    }
    else if (cols < 16) { //i.e cols ==8
        for (int r = 1; r < rows; r += 2) {
            if ((r + 1) >= rows) {
                tmp_src = _mm_set1_epi64x(*(uint64_t*)(src + 1));
                src1 = _mm256_cvtepu8_epi16(tmp_src);

                tmp_src = _mm_set1_epi64x(*(uint64_t*)(src));
                src2 = _mm256_cvtepu8_epi16(tmp_src);

                tmp_src = _mm_set1_epi64x(*(uint64_t*)(src + 1 - src_stride));
                src3 = _mm256_cvtepu8_epi16(tmp_src);
            }
            else {
                tmp_src = _mm_set_epi64x(*(uint64_t*)(src + 1 + src_stride),
                    *(uint64_t*)(src + 1));
                src1 = _mm256_cvtepu8_epi16(tmp_src);

                tmp_src = _mm_set_epi64x(*(uint64_t*)(src + src_stride),
                    *(uint64_t*)(src));
                src2 = _mm256_cvtepu8_epi16(tmp_src);

                tmp_src = _mm_set_epi64x(*(uint64_t*)(src + 1),
                    *(uint64_t*)(src + 1 - src_stride));
                src3 = _mm256_cvtepu8_epi16(tmp_src);
            }

            get_gradient_hist_avx2_internal(&src1, &src2, &src3, dy_mask_array,
                quot_array, remd_array, sn_array, temp_array);

            if ((r + 1) >= rows) {
                for (int w = 0; w < 7; ++w) {
                    if (dy_mask_array[w] != 1) {
                        int index = gradient_to_angle_bin[sn_array[w]]
                            [quot_array[w]][remd_array[w]];
                        hist[index] += temp_array[w];
                    }
                    else {
                        hist[2] += temp_array[w];
                    }
                }
            }
            else {
                for (int w = 0; w < 15; ++w) {
                    if (w == 7)
                        continue;
                    if (dy_mask_array[w] != 1) {
                        int index = gradient_to_angle_bin[sn_array[w]]
                            [quot_array[w]][remd_array[w]];
                        hist[index] += temp_array[w];
                    }
                    else {
                        hist[2] += temp_array[w];
                    }
                }
            }
            src += 2 * src_stride;
        }
    }
    else {
        for (int r = 1; r < rows; ++r) {
            int c = 1;
            for (; cols - c >= 15; c += 16) {

                //read too many [1:16], while max is 15
                src1 = _mm256_cvtepu8_epi16(
                    _mm_loadu_si128((__m128i const*)&src[c]));
                src2 = _mm256_cvtepu8_epi16(
                    _mm_loadu_si128((__m128i const*)&src[c - 1]));
                src3 = _mm256_cvtepu8_epi16(
                    _mm_loadu_si128((__m128i const*)&src[c - src_stride]));

                get_gradient_hist_avx2_internal(&src1, &src2, &src3,
                    dy_mask_array, quot_array, remd_array, sn_array, temp_array);

                int max = 16;
                if (c + 16 > cols) {
                    max = 15;
                }

                for (int w = 0; w < max; ++w) {

                    if (dy_mask_array[w] != 1) {
                        int index = gradient_to_angle_bin[sn_array[w]]
                            [quot_array[w]][remd_array[w]];
                        hist[index] += temp_array[w];
                    }
                    else {
                        hist[2] += temp_array[w];
                    }
                }
            }
            src += src_stride;
        }
    }
}

#endif

