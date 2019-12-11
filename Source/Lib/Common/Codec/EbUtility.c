/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include <time.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <stdlib.h>
#include <sys/time.h>
#endif

#include "EbDefinitions.h"
#include "EbUtility.h"
#include "EbTime.h"
/********************************************************************************************
* faster memcopy for <= 64B blocks, great w/ inlining and size known at compile time (or w/ PGO)
* THIS NEEDS TO STAY IN A HEADER FOR BEST PERFORMANCE
********************************************************************************************/

#include <immintrin.h>

#if defined(__GNUC__) && !defined(__clang__) && !defined(__ICC__)
__attribute__((optimize("unroll-loops")))
#endif
static void eb_memcpy_small(void* dst_ptr, void const* src_ptr, size_t size) {
    const char* src = (const char*)src_ptr;
    char*       dst = (char*)dst_ptr;
    size_t      i = 0;

#ifdef _INTEL_COMPILER
#pragma unroll
#endif
    while ((i + 16) <= size)
    {
        _mm_storeu_ps((float*)(dst + i), _mm_loadu_ps((const float*)(src + i)));
        i += 16;
    }

    if ((i + 8) <= size)
    {
        _mm_store_sd((double*)(dst + i), _mm_load_sd((const double*)(src + i)));
        i += 8;
    }

    for (; i < size; ++i)
        dst[i] = src[i];
}
#define EB_MIN(a,b)             (((a) < (b)) ? (a) : (b))
static void eb_memcpy_sse(void* dst_ptr, void const* src_ptr, size_t size) {
    const char* src = (const char*)src_ptr;
    char*       dst = (char*)dst_ptr;
    size_t      i = 0;
    size_t align_cnt = EB_MIN((64 - ((size_t)dst & 63)), size);

    // align dest to a $line
    if (align_cnt != 64) {
        eb_memcpy_small(dst, src, align_cnt);
        dst += align_cnt;
        src += align_cnt;
        size -= align_cnt;
    }

    // copy a $line at a time
    // dst aligned to a $line
    size_t cline_cnt = (size & ~(size_t)63);
    for (i = 0; i < cline_cnt; i += 64) {
        __m128 c0 = _mm_loadu_ps((const float*)(src + i));
        __m128 c1 = _mm_loadu_ps((const float*)(src + i + sizeof(c0)));
        __m128 c2 = _mm_loadu_ps((const float*)(src + i + sizeof(c0) * 2));
        __m128 c3 = _mm_loadu_ps((const float*)(src + i + sizeof(c0) * 3));

        _mm_storeu_ps((float*)(dst + i), c0);
        _mm_storeu_ps((float*)(dst + i + sizeof(c0)), c1);
        _mm_storeu_ps((float*)(dst + i + sizeof(c0) * 2), c2);
        _mm_storeu_ps((float*)(dst + i + sizeof(c0) * 3), c3);
    }

    // copy the remainder
    if (i < size)
        eb_memcpy_small(dst + i, src + i, size - i);
}
void eb_memcpy(void  *dst_ptr, void  *src_ptr, size_t size) {
    if (size > 64)
        eb_memcpy_sse(dst_ptr, src_ptr, size);
    else
        eb_memcpy_small(dst_ptr, src_ptr, size);
}
/*****************************************
 * Z-Order
 *****************************************/
static TxSize blocksize_to_txsize[BlockSizeS_ALL] = {
      TX_4X4    ,      // BLOCK_4X4
      TX_4X8    ,      // BLOCK_4X8
      TX_8X4    ,      // BLOCK_8X4
      TX_8X8    ,      // BLOCK_8X8
      TX_8X16   ,      // BLOCK_8X16
      TX_16X8   ,      // BLOCK_16X8
      TX_16X16  ,      // BLOCK_16X16
      TX_16X32  ,      // BLOCK_16X32
      TX_32X16  ,      // BLOCK_32X16
      TX_32X32  ,      // BLOCK_32X32
      TX_32X64  ,      // BLOCK_32X64
      TX_64X32  ,      // BLOCK_64X32
      TX_64X64  ,      // BLOCK_64X64
      TX_64X64  ,      // BLOCK_64X128
      TX_64X64  ,      // BLOCK_128X64
      TX_64X64  ,      // BLOCK_128X128
      TX_4X16   ,      // BLOCK_4X16
      TX_16X4   ,      // BLOCK_16X4
      TX_8X32   ,      // BLOCK_8X32
      TX_32X8   ,      // BLOCK_32X8
      TX_16X64  ,      // BLOCK_16X64
      TX_64X16         // BLOCK_64X16
};
EbErrorType z_order_increment(
    uint32_t *x_loc,   // x location, level agnostic
    uint32_t *y_loc)   // y location, level agnostic
{
    EbErrorType return_error = EB_ErrorNone;
    uint32_t mask;

    // The basic idea of this function is to increment an x,y coordinate
    // that has had its size removed to the next z-coding order location.
    //
    // In a four quadrant partition, the z coding order is [0,0], [1,0], [0,1], [1,1]
    // Some observations (only looking at one bit position or the LSB) are:
    //  1. X is always toggled (achieved with X ^= 0x1)
    //  2. Y can be toggled with (Y = Y ^ X)
    //  3. Recall that a value XOR'ed with 1 toggles, and XOR'ed with 0 stays the same
    //
    //  Extending this logic is somewhat trickier. The two main observations to make are
    //  4. The LSB of X and Y are always progressed.
    //  5. Every other bit-position, N, other than the LSB are progressed in their state
    //     when the N-1 bit position resets back to [0,0].
    //
    //  From 5, we can infer the need of a "progression mask" of the form 0x1, 0x3, 0x7, 0xF, etc.
    //  The first step of contructing the mask is to find which bit positions are ready to
    //  reset (found by X & Y) and setting the LSB of the mask to 1 (the LSB always progresses).
    //  The second step is to eliminate all ones from the mask above the lowest-ordered zero bit.
    //  Note we can achieve more precision in the second mask step by more masking-out operations,
    //  but for a 64 -> 4 (5 steps), the precision below is sufficient.
    //
    //  Finally, X and Y are progressed only at the bit-positions in the mask.

    mask = ((*x_loc & *y_loc) << 1) | 0x1;
    mask &= (mask << 1) | 0x01;
    mask &= (mask << 2) | 0x03;
    mask &= (mask << 4) | 0x0F;
    mask &= (mask << 8) | 0xFF;

    *y_loc ^= *x_loc & mask;
    *x_loc ^= mask;

    return return_error;
}

/*****************************************
 * Z-Order Increment with Level
 *   This is the main function for progressing
 *   through a treeblock's coding units. To get
 *   the true CU size, multiple the x_loc, y_loc
 *   by the smallest CU size.
 *****************************************/
void ZOrderIncrementWithLevel(
    uint32_t *x_loc,   // x location, units of smallest block size
    uint32_t *y_loc,   // y location, units of smallest block size
    uint32_t *level,  // level, number of block size-steps from the smallest block size
    //   (e.g. if 8x8 = level 0, 16x16 = level 1, 32x32 == level 2, 64x64 == level 3)
    uint32_t *index)  // The CU index, can be used to index a lookup table (see get_coded_unit_stats)
{
    uint32_t mask;

    // The basic idea of this function is to increment an x,y coordinate
    // that has had its size removed to the next z-coding order location.
    //
    // In a four quadrant partition, the z coding order is [0,0], [1,0], [0,1], [1,1]
    // Some observations (only looking at one bit position or the LSB) are:
    //  1. X is always toggled (achieved with X ^= 0x1)
    //  2. Y can be toggled with (Y = Y ^ X)
    //  3. Recall that a value XOR'ed with 1 toggles, and XOR'ed with 0 stays the same
    //
    //  Extending this logic is somewhat trickier. The two main observations to make are
    //  4. The LSB of X and Y are always progressed.
    //  5. Every other bit-position, N, other than the LSB are progressed in their state
    //     when the N-1 bit position resets back to [0,0].
    //
    //  From 5, we can infer the need of a "progression mask" of the form 0x1, 0x3, 0x7, 0xF, etc.
    //  The first step of contructing the mask is to find which bit positions are ready to
    //  reset (found by X & Y) and setting the LSB of the mask to 1 (the LSB always progresses).
    //  The second step is to eliminate all ones from the mask above the lowest-ordered zero bit.
    //  Note we can achieve more precision in the second mask step by more masking-out operations,
    //  but for a 64 -> 4 (5 steps), the precision below is sufficient.
    //
    //  Finally, X and Y are progressed only at the bit-positions in the mask.

    // Seed the mask
    mask = ((*x_loc & *y_loc) << 1) | 0x1;

    // This step zero-outs the mask if level is not zero.
    //   The purpose of this is step further down the tree
    //   if not already at the bottom of the tree
    //   Equivalent to: mask = (level > 0) ? mask : 0;
    mask &= (uint32_t)(-(*level == 0));

    // Construct the mask
    mask &= (mask << 1) | 0x01;
    mask &= (mask << 2) | 0x03;
    mask &= (mask << 4) | 0x0F;
    mask &= (mask << 8) | 0xFF;

    // Decrement the level if not already at the bottom of the tree
    //  Equivalent to level = (level > 0) ? level - 1 : 0;
    *level = (*level - 1) & -(*level > 0);

    // If at one of the "corner" positions where the mask > 1, we
    //   need to increase the level since larger blocks are processed
    //   before smaller blocks.  Note that by using mask, we are protected
    //   against inadvertently incrementing the level if not already at
    //   the bottom of the tree.  The level increment should really be
    //   Log2f(mask >> 1), but since there are only 3 valid positions,
    //   we are using a cheesy Log2f approximation
    //   Equivalent to: level += (mask > 3) ? 2 : mask >> 1;

    *level += ((2 ^ (mask >> 1)) & -(mask > 3)) ^ (mask >> 1);

    // Increment the x_loc, y_loc.  Note that this only occurs when
    //   we are at the bottom of the tree.
    *y_loc ^= *x_loc & mask;
    *x_loc ^= mask;

    // Increment the index. Note that the natural progression of this
    //   block aligns with how leafs are stored in the accompanying
    //   CU data structures.
    ++(*index);

    return;
}

static CodedUnitStats CodedUnitStatsArray[] = {
    //   Depth       Size      SizeLog2     OriginX    OriginY   cu_num_in_depth   Index
        {0,           64,         6,           0,         0,        0     ,   0    },   // 0
        {1,           32,         5,           0,         0,        0     ,   1    },   // 1
        {2,           16,         4,           0,         0,        0     ,   1    },   // 2
        {3,            8,         3,           0,         0,        0     ,   1    },   // 3
        {3,            8,         3,           8,         0,        1     ,   1    },   // 4
        {3,            8,         3,           0,         8,        8     ,   1    },   // 5
        {3,            8,         3,           8,         8,        9     ,   1    },   // 6
        {2,           16,         4,          16,         0,        1     ,   1    },   // 7
        {3,            8,         3,          16,         0,        2     ,   1    },   // 8
        {3,            8,         3,          24,         0,        3     ,   1    },   // 9
        {3,            8,         3,          16,         8,        10    ,   1     },  // 10
        {3,            8,         3,          24,         8,        11    ,   1     },  // 11
        {2,           16,         4,           0,        16,        4     ,   1    },   // 12
        {3,            8,         3,           0,        16,        16    ,   1     },  // 13
        {3,            8,         3,           8,        16,        17    ,   1     },  // 14
        {3,            8,         3,           0,        24,        24    ,   1     },  // 15
        {3,            8,         3,           8,        24,        25    ,   1     },  // 16
        {2,           16,         4,          16,        16,        5     ,   1    },   // 17
        {3,            8,         3,          16,        16,        18    ,   1     },  // 18
        {3,            8,         3,          24,        16,        19    ,   1     },  // 19
        {3,            8,         3,          16,        24,        26    ,   1     },  // 20
        {3,            8,         3,          24,        24,        27    ,   1     },  // 21
        {1,           32,         5,          32,         0,        1     ,   2    },   // 22
        {2,           16,         4,          32,         0,        2     ,   2    },   // 23
        {3,            8,         3,          32,         0,        4     ,   2    },   // 24
        {3,            8,         3,          40,         0,        5     ,   2    },   // 25
        {3,            8,         3,          32,         8,        12    ,   2     },  // 26
        {3,            8,         3,          40,         8,        13    ,   2     },  // 27
        {2,           16,         4,          48,         0,        3     ,   2    },   // 28
        {3,            8,         3,          48,         0,        6     ,   2    },   // 29
        {3,            8,         3,          56,         0,        7     ,   2    },   // 30
        {3,            8,         3,          48,         8,        14    ,   2     },  // 31
        {3,            8,         3,          56,         8,        15    ,   2     },  // 32
        {2,           16,         4,          32,        16,        6     ,   2    },   // 33
        {3,            8,         3,          32,        16,        20    ,   2     },  // 34
        {3,            8,         3,          40,        16,        21    ,   2     },  // 35
        {3,            8,         3,          32,        24,        28    ,   2     },  // 36
        {3,            8,         3,          40,        24,        29    ,   2     },  // 37
        {2,           16,         4,          48,        16,        7     ,   2    },   // 38
        {3,            8,         3,          48,        16,        22    ,   2     },  // 39
        {3,            8,         3,          56,        16,        23    ,   2     },  // 40
        {3,            8,         3,          48,        24,        30    ,   2     },  // 41
        {3,            8,         3,          56,        24,        31    ,   2     },  // 42
        {1,           32,         5,           0,        32,        2     ,   3    },   // 43
        {2,           16,         4,           0,        32,        8     ,   3    },   // 44
        {3,            8,         3,           0,        32,        32    ,   3     },  // 45
        {3,            8,         3,           8,        32,        33    ,   3     },  // 46
        {3,            8,         3,           0,        40,        40    ,   3     },  // 47
        {3,            8,         3,           8,        40,        41    ,   3     },  // 48
        {2,           16,         4,          16,        32,        9     ,   3    },   // 49
        {3,            8,         3,          16,        32,        34    ,   3     },  // 50
        {3,            8,         3,          24,        32,        35    ,   3     },  // 51
        {3,            8,         3,          16,        40,        42    ,   3     },  // 52
        {3,            8,         3,          24,        40,        43    ,   3     },  // 53
        {2,           16,         4,           0,        48,        12    ,   3     },  // 54
        {3,            8,         3,           0,        48,        48    ,   3     },  // 55
        {3,            8,         3,           8,        48,        49    ,   3     },  // 56
        {3,            8,         3,           0,        56,        56    ,   3     },  // 57
        {3,            8,         3,           8,        56,        57    ,   3     },  // 58
        {2,           16,         4,          16,        48,        13    ,   3     },  // 59
        {3,            8,         3,          16,        48,        50    ,   3     },  // 60
        {3,            8,         3,          24,        48,        51    ,   3     },  // 61
        {3,            8,         3,          16,        56,        58    ,   3     },  // 62
        {3,            8,         3,          24,        56,        59    ,   3     },  // 63
        {1,           32,         5,          32,        32,        3     ,   4     },  // 64
        {2,           16,         4,          32,        32,        10    ,   4     },  // 65
        {3,            8,         3,          32,        32,        36    ,   4     },  // 66
        {3,            8,         3,          40,        32,        37    ,   4     },  // 67
        {3,            8,         3,          32,        40,        44    ,   4     },  // 68
        {3,            8,         3,          40,        40,        45    ,   4     },  // 69
        {2,           16,         4,          48,        32,        11    ,   4     },  // 70
        {3,            8,         3,          48,        32,        38    ,   4     },  // 71
        {3,            8,         3,          56,        32,        39    ,   4     },  // 72
        {3,            8,         3,          48,        40,        46    ,   4     },  // 73
        {3,            8,         3,          56,        40,        47    ,   4     },  // 74
        {2,           16,         4,          32,        48,        14    ,   4     },  // 75
        {3,            8,         3,          32,        48,        52    ,   4     },  // 76
        {3,            8,         3,          40,        48,        53    ,   4     },  // 77
        {3,            8,         3,          32,        56,        60    ,   4     },  // 78
        {3,            8,         3,          40,        56,        61    ,   4     },  // 79
        {2,           16,         4,          48,        48,        15    ,   4     },  // 80
        {3,            8,         3,          48,        48,        54    ,   4     },  // 81
        {3,            8,         3,          56,        48,        55    ,   4     },  // 82
        {3,            8,         3,          48,        56,        62    ,   4     },  // 83
        {3,            8,         3,          56,        56,        63    ,   4     }   // 84
};

/**************************************************************
 * Get Coded Unit Statistics
 **************************************************************/
const CodedUnitStats* get_coded_unit_stats(const uint32_t cuIdx)
{
    //ASSERT(cuIdx < CU_MAX_COUNT && "get_coded_unit_stats: Out-of-range CU Idx\n");
    if (cuIdx == 255)
        printf("Invalid CuIndex\n");

    return &CodedUnitStatsArray[cuIdx];
}

static const TransformUnitStats TransformUnitStatsArray[] = {
    //
    //        depth
    //       /
    //      /       offset_x (units of the current depth)
    //     /       /
    //    /       /       offset_y (units of the current depth)
    //   /       /       /
    {0,     0,      0},     // 0
    {1,     0,      0},     // 1
    {1,     2,      0},     // 2
    {1,     0,      2},     // 3
    {1,     2,      2},     // 4
    {2,     0,      0},     // 5
    {2,     1,      0},     // 6
    {2,     0,      1},     // 7
    {2,     1,      1},     // 8
    {2,     2,      0},     // 9
    {2,     3,      0},     // 10
    {2,     2,      1},     // 11
    {2,     3,      1},     // 12
    { 2,    0,        2},     // 13
    { 2,    1,        2},     // 14
    { 2,    0,        3},     // 15
    { 2,    1,        3},     // 16
    { 2,    2,        2},     // 17
    { 2,    3,        2},     // 18
    { 2,    2,        3},     // 19
    { 2,    3,        3},    // 20
    {0xFF,  0xFF,   0xFF}   // Invalid
};

/**************************************************************
 * Get Transform Unit Statistics
 **************************************************************/
const TransformUnitStats* get_transform_unit_stats(const uint32_t tuIdx)
{
    return &TransformUnitStatsArray[tuIdx];
}

/*****************************************
 * Integer Log 2
 *  This is a quick adaptation of a Number
 *  Leading Zeros (NLZ) algorithm to get
 *  the log2f of an integer
 *****************************************/
 /*uint32_t Log2f(uint32_t x)
 {
     uint32_t y;
     int32_t n = 32, c = 16;

     do {
         y = x >> c;
         if (y > 0) {
             n -= c;
             x = y;
         }
         c >>= 1;
     } while (c > 0);

     return 32 - n;
 }*/

 /*****************************************
  * Long Log 2
  *  This is a quick adaptation of a Number
  *  Leading Zeros (NLZ) algorithm to get
  *  the log2f of a 64-bit number
  *****************************************/
inline uint64_t Log2f64(uint64_t x)
{
    uint64_t y;
    int64_t n = 64, c = 32;

    do {
        y = x >> c;
        if (y > 0) {
            n -= c;
            x = y;
        }
        c >>= 1;
    } while (c > 0);

    return 64 - n;
}

/*****************************************
 * Endian Swap
 *****************************************/
uint32_t endian_swap(uint32_t ui)
{
    uint32_t ul2;

    ul2 = ui >> 24;
    ul2 |= (ui >> 8) & 0x0000ff00;
    ul2 |= (ui << 8) & 0x00ff0000;
    ul2 |= ui << 24;

    return ul2;
}

uint64_t log2f_high_precision(uint64_t x, uint8_t precision)
{
    uint64_t sigBitLocation = Log2f64(x);
    uint64_t Remainder = x - ((uint64_t)1 << (uint8_t)sigBitLocation);
    uint64_t result;

    result = (sigBitLocation << precision) + ((Remainder << precision) / ((uint64_t)1 << (uint8_t)sigBitLocation));

    return result;
}

// concatenate two linked list, and return the pointer to the new concatenated list
EbLinkedListNode* concat_eb_linked_list(EbLinkedListNode* a, EbLinkedListNode* b)
{
    if (a)
    {
        while (a->next)
            a = a->next;
        a->next = b;
        return a;
    }
    else
        return b;
}

// split a linked list
EbLinkedListNode* split_eb_linked_list(EbLinkedListNode* input, EbLinkedListNode** restLL, EbBool(*predicate_func)(EbLinkedListNode*))
{
    EbLinkedListNode* llTruePtr = (EbLinkedListNode *)EB_NULL;    // list of nodes satifying predicate_func(node) == TRUE
    EbLinkedListNode* llRestPtr = (EbLinkedListNode *)EB_NULL;    // list of nodes satifying predicate_func(node) != TRUE

    while (input)
    {
        EbLinkedListNode* next = input->next;
        input->next = (EbLinkedListNode *)EB_NULL;
        if (predicate_func(input))
            llTruePtr = concat_eb_linked_list(input, llTruePtr);
        else
            llRestPtr = concat_eb_linked_list(input, llRestPtr);
        input = next;
    }

    *restLL = llRestPtr;
    return llTruePtr;
}

static const MiniGopStats MiniGopStatsArray[] = {
    //    hierarchical_levels    start_index    end_index    Lenght    mini_gop_index
    { 5,  0, 31, 32 },    // 0
    { 4,  0, 15, 16 },    // 1
    { 3,  0,  7,  8 },    // 2
    { 2,  0,  3,  4 },    // 3
    { 2,  4,  7,  4 },    // 4
    { 3,  8, 15,  8 },    // 5
    { 2,  8, 11,  4 },    // 6
    { 2, 12, 15,  4 },    // 7
    { 4, 16, 31, 16 },    // 8
    { 3, 16, 23,  8 },    // 9
    { 2, 16, 19,  4 },    // 10
    { 2, 20, 23,  4 },    // 11
    { 3, 24, 31,  8 },    // 12
    { 2, 24, 27,  4 },    // 13
    { 2, 28, 31,  4 }    // 14
};

/**************************************************************
* Get Mini GOP Statistics
**************************************************************/
const MiniGopStats* get_mini_gop_stats(const uint32_t mini_gop_index)
{
    return &MiniGopStatsArray[mini_gop_index];
}

uint32_t ns_quarter_off_mult[9/*Up to 9 part*/][2/*x+y*/][4/*Up to 4 ns blocks per part*/] =
{
    //9 means not used.

    //          |   x   |     |   y   |

    /*P=0*/  {  {0,9,9,9}  ,  {0,9,9,9}  },
    /*P=1*/  {  {0,0,9,9}  ,  {0,2,9,9}  },
    /*P=2*/  {  {0,2,9,9}  ,  {0,0,9,9}  },
    /*P=3*/  {  {0,2,0,9}  ,  {0,0,2,9}  },
    /*P=4*/  {  {0,0,2,9}  ,  {0,2,2,9}  },
    /*P=5*/  {  {0,0,2,9}  ,  {0,2,0,9}  },
    /*P=6*/  {  {0,2,2,9}  ,  {0,0,2,9}  },
    /*P=7*/  {  {0,0,0,0}  ,  {0,1,2,3}  },
    /*P=8*/  {  {0,1,2,3}  ,  {0,0,0,0}  }
};

uint32_t ns_quarter_size_mult[9/*Up to 9 part*/][2/*h+v*/][4/*Up to 4 ns blocks per part*/] =
{
    //9 means not used.

    //          |   h   |     |   v   |

    /*P=0*/  {  {4,9,9,9}  ,  {4,9,9,9}  },
    /*P=1*/  {  {4,4,9,9}  ,  {2,2,9,9}  },
    /*P=2*/  {  {2,2,9,9}  ,  {4,4,9,9}  },
    /*P=3*/  {  {2,2,4,9}  ,  {2,2,2,9}  },
    /*P=4*/  {  {4,2,2,9}  ,  {2,2,2,9}  },
    /*P=5*/  {  {2,2,2,9}  ,  {2,2,4,9}  },
    /*P=6*/  {  {2,2,2,9}  ,  {4,2,2,9}  },
    /*P=7*/  {  {4,4,4,4}  ,  {1,1,1,1}  },
    /*P=8*/  {  {1,1,1,1}  ,  {4,4,4,4}  }
};

BlockSize hvsize_to_bsize[/*H*/6][/*V*/6] =
{
    {  BLOCK_4X4,       BLOCK_4X8,     BLOCK_4X16,      BLOCK_INVALID,   BLOCK_INVALID,   BLOCK_INVALID      },
    {  BLOCK_8X4,       BLOCK_8X8,     BLOCK_8X16,      BLOCK_8X32,      BLOCK_INVALID,   BLOCK_INVALID      },
    {  BLOCK_16X4,      BLOCK_16X8,    BLOCK_16X16,     BLOCK_16X32,     BLOCK_16X64,     BLOCK_INVALID   },
    {  BLOCK_INVALID,   BLOCK_32X8,    BLOCK_32X16,     BLOCK_32X32,     BLOCK_32X64,     BLOCK_INVALID   },
    {  BLOCK_INVALID,   BLOCK_INVALID, BLOCK_64X16,     BLOCK_64X32,     BLOCK_64X64,     BLOCK_64X128    },
    {  BLOCK_INVALID,   BLOCK_INVALID, BLOCK_INVALID,   BLOCK_INVALID,   BLOCK_128X64,    BLOCK_128X128   }
};

uint32_t  max_sb = 64;
uint32_t  max_depth = 5;
uint32_t  max_part = 9;
uint32_t  max_num_active_blocks;

//data could be  organized in 2 forms: depth scan (dps) or MD scan (mds):
//dps: all depth0 - all depth1 - all depth2 - all depth3.
//     within a depth: square blk0 in raster scan (followed by all its ns blcoks),
//     square blk1 in raster scan (followed by all its ns blcoks), etc
//mds: top-down and Z scan.
BlockGeom blk_geom_dps[MAX_NUM_BLOCKS_ALLOC];  //to access geom info of a particular block : use this table if you have the block index in depth scan
BlockGeom blk_geom_mds[MAX_NUM_BLOCKS_ALLOC];  //to access geom info of a particular block : use this table if you have the block index in md    scan

uint32_t search_matching_from_dps(
    uint32_t depth,
    uint32_t part,
    uint32_t x,
    uint32_t y)
{
    uint32_t found = 0;
    uint32_t it;
    uint32_t matched = 0xFFFF;
    for (it = 0; it < max_num_active_blocks; it++)
    {
        if (blk_geom_dps[it].depth == depth && blk_geom_dps[it].shape == part && blk_geom_dps[it].origin_x == x && blk_geom_dps[it].origin_y == y)
        {
            if (found == 0)
            {
                matched = it;
                found = 1;
            }
            else {
                matched = 0xFFFF;
                break;
            }
        }
    }

    if (matched == 0xFFFF)
        printf(" \n\n PROBLEM\n\n ");

    return matched;
}
uint32_t search_matching_from_mds(
    uint32_t depth,
    uint32_t part,
    uint32_t x,
    uint32_t y)
{
    uint32_t found = 0;
    uint32_t it;
    uint32_t matched = 0xFFFF;
    for (it = 0; it < max_num_active_blocks; it++)
    {
        if (blk_geom_mds[it].depth == depth && blk_geom_mds[it].shape == part && blk_geom_mds[it].origin_x == x && blk_geom_mds[it].origin_y == y)
        {
            if (found == 0)
            {
                matched = it;
                found = 1;
            }
            else {
                matched = 0xFFFF;
                break;
            }
        }
    }

    if (matched == 0xFFFF)
        printf(" \n\n PROBLEM\n\n ");

    return matched;
}

static INLINE TxSize av1_get_tx_size(
    BlockSize  sb_type,
    int32_t plane/*, const MacroBlockD *xd*/) {
    //const MbModeInfo *mbmi = xd->mi[0];
    // if (xd->lossless[mbmi->segment_id]) return TX_4X4;
    if (plane == 0) return blocksize_to_txsize[sb_type];
    // const MacroblockdPlane *pd = &xd->plane[plane];

    uint32_t subsampling_x = plane > 0 ? 1 : 0;
    uint32_t subsampling_y = plane > 0 ? 1 : 0;
    return av1_get_max_uv_txsize(/*mbmi->*/sb_type, subsampling_x, subsampling_y);
    UNUSED(plane);
}

void md_scan_all_blks(uint32_t *idx_mds, uint32_t sq_size, uint32_t x, uint32_t y, int32_t is_last_quadrant, uint8_t quad_it)
{
    //the input block is the parent square block of size sq_size located at pos (x,y)

    uint32_t part_it, nsq_it, d1_it, sqi_mds;

    uint32_t halfsize = sq_size / 2;
    uint32_t quartsize = sq_size / 4;

    uint32_t max_part_updated = sq_size == 128 ? MIN(max_part, 7) :
        sq_size == 8 ? MIN(max_part, 3) :

        sq_size == 4 ? 1 : max_part;

    d1_it = 0;
    sqi_mds = *idx_mds;

    for (part_it = 0; part_it < max_part_updated; part_it++)
    {
        uint32_t tot_num_ns_per_part =
            part_it < 1 ? 1 :
            part_it < 3 ? 2 :
            part_it < 7 ? 3 : 4;

        for (nsq_it = 0; nsq_it < tot_num_ns_per_part; nsq_it++)
        {
            blk_geom_mds[*idx_mds].depth = sq_size == max_sb / 1 ? 0 :
                sq_size == max_sb / 2 ? 1 :
                sq_size == max_sb / 4 ? 2 :
                sq_size == max_sb / 8 ? 3 :
                sq_size == max_sb / 16 ? 4 : 5;

            blk_geom_mds[*idx_mds].sq_size = sq_size;
            blk_geom_mds[*idx_mds].is_last_quadrant = is_last_quadrant;
            blk_geom_mds[*idx_mds].quadi = quad_it;

            blk_geom_mds[*idx_mds].shape = (PART)part_it;
            blk_geom_mds[*idx_mds].origin_x = x + quartsize * ns_quarter_off_mult[part_it][0][nsq_it];
            blk_geom_mds[*idx_mds].origin_y = y + quartsize * ns_quarter_off_mult[part_it][1][nsq_it];

            blk_geom_mds[*idx_mds].d1i = d1_it++;
            blk_geom_mds[*idx_mds].sqi_mds = sqi_mds;
            blk_geom_mds[*idx_mds].totns = tot_num_ns_per_part;
            blk_geom_mds[*idx_mds].nsi = nsq_it;

            uint32_t matched = search_matching_from_dps(
                blk_geom_mds[*idx_mds].depth,
                blk_geom_mds[*idx_mds].shape,
                blk_geom_mds[*idx_mds].origin_x,
                blk_geom_mds[*idx_mds].origin_y);

            blk_geom_mds[*idx_mds].blkidx_dps = blk_geom_dps[matched].blkidx_dps;

            blk_geom_mds[*idx_mds].bwidth = quartsize * ns_quarter_size_mult[part_it][0][nsq_it];
            blk_geom_mds[*idx_mds].bheight = quartsize * ns_quarter_size_mult[part_it][1][nsq_it];
            blk_geom_mds[*idx_mds].bwidth_log2 = Log2f(blk_geom_mds[*idx_mds].bwidth);
            blk_geom_mds[*idx_mds].bheight_log2 = Log2f(blk_geom_mds[*idx_mds].bheight);
            blk_geom_mds[*idx_mds].bsize = hvsize_to_bsize[blk_geom_mds[*idx_mds].bwidth_log2 - 2][blk_geom_mds[*idx_mds].bheight_log2 - 2];
            blk_geom_mds[*idx_mds].bwidth_uv = MAX(4, blk_geom_mds[*idx_mds].bwidth >> 1);
            blk_geom_mds[*idx_mds].bheight_uv = MAX(4, blk_geom_mds[*idx_mds].bheight >> 1);
            blk_geom_mds[*idx_mds].has_uv = 1;

            if (blk_geom_mds[*idx_mds].bwidth == 4 && blk_geom_mds[*idx_mds].bheight == 4)
                blk_geom_mds[*idx_mds].has_uv = is_last_quadrant ? 1 : 0;

            else
                if ((blk_geom_mds[*idx_mds].bwidth >> 1) < blk_geom_mds[*idx_mds].bwidth_uv || (blk_geom_mds[*idx_mds].bheight >> 1) < blk_geom_mds[*idx_mds].bheight_uv) {
                    int32_t num_blk_same_uv = 1;
                    if (blk_geom_mds[*idx_mds].bwidth >> 1 < 4)
                        num_blk_same_uv *= 2;
                    if (blk_geom_mds[*idx_mds].bheight >> 1 < 4)
                        num_blk_same_uv *= 2;
                    //if (blk_geom_mds[*idx_mds].nsi % 2 == 0)
                    //if (blk_geom_mds[*idx_mds].nsi != (blk_geom_mds[*idx_mds].totns-1) )
                    if (blk_geom_mds[*idx_mds].nsi != (num_blk_same_uv - 1) && blk_geom_mds[*idx_mds].nsi != (2 * num_blk_same_uv - 1))
                        blk_geom_mds[*idx_mds].has_uv = 0;
                }

            blk_geom_mds[*idx_mds].bsize_uv = get_plane_block_size(blk_geom_mds[*idx_mds].bsize, 1, 1);
            uint16_t   txb_itr = 0;
            // tx_depth 1 geom settings
            uint8_t tx_depth = 0;
            blk_geom_mds[*idx_mds].txb_count[tx_depth] = blk_geom_mds[*idx_mds].bsize == BLOCK_128X128 ? 4 :
                blk_geom_mds[*idx_mds].bsize == BLOCK_128X64 || blk_geom_mds[*idx_mds].bsize == BLOCK_64X128 ? 2 : 1;
            for (txb_itr = 0; txb_itr < blk_geom_mds[*idx_mds].txb_count[tx_depth]; txb_itr++) {
                blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(blk_geom_mds[*idx_mds].bsize, 0);
                blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = av1_get_tx_size(blk_geom_mds[*idx_mds].bsize, 1);
                if (blk_geom_mds[*idx_mds].bsize == BLOCK_128X128)
                {
                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = (txb_itr == 0 || txb_itr == 2) ? blk_geom_mds[*idx_mds].origin_x : blk_geom_mds[*idx_mds].origin_x + 64;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = (txb_itr == 0 || txb_itr == 1) ? blk_geom_mds[*idx_mds].origin_y : blk_geom_mds[*idx_mds].origin_y + 64;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_128X64)
                {
                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = (txb_itr == 0) ? blk_geom_mds[*idx_mds].origin_x : blk_geom_mds[*idx_mds].origin_x + 64;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_64X128)
                {
                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = (txb_itr == 0) ? blk_geom_mds[*idx_mds].origin_y : blk_geom_mds[*idx_mds].origin_y + 64;
                }
                else
                {
                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y;
                }
                /*if (blk_geom_mds[*idx_mds].bsize == BLOCK_16X8)
                    printf("");*/
                blk_geom_mds[*idx_mds].tx_boff_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] - blk_geom_mds[*idx_mds].origin_x;
                blk_geom_mds[*idx_mds].tx_boff_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] - blk_geom_mds[*idx_mds].origin_y;
                blk_geom_mds[*idx_mds].tx_width[tx_depth][txb_itr] = tx_size_wide[blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr]];
                blk_geom_mds[*idx_mds].tx_height[tx_depth][txb_itr] = tx_size_high[blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr]];
                blk_geom_mds[*idx_mds].tx_width_uv[tx_depth][txb_itr] = tx_size_wide[blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr]];
                blk_geom_mds[*idx_mds].tx_height_uv[tx_depth][txb_itr] = tx_size_high[blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr]];
            }
            // tx_depth 1 geom settings
            tx_depth = 1;
            blk_geom_mds[*idx_mds].txb_count[tx_depth] = blk_geom_mds[*idx_mds].bsize == BLOCK_128X128 ? 4 :
                blk_geom_mds[*idx_mds].bsize == BLOCK_128X64 || blk_geom_mds[*idx_mds].bsize == BLOCK_64X128 ? 2 : 1;

            if (blk_geom_mds[*idx_mds].bsize == BLOCK_64X64 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_32X32 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_16X16 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_8X8)
            {
                blk_geom_mds[*idx_mds].txb_count[tx_depth] = 4;
            }

            if (blk_geom_mds[*idx_mds].bsize == BLOCK_64X32 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_32X64 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_32X16 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_16X32 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_16X8 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_8X16)
            {
                blk_geom_mds[*idx_mds].txb_count[tx_depth] = 2;
            }
            if (blk_geom_mds[*idx_mds].bsize == BLOCK_64X16 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_16X64 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_32X8 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_8X32 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_16X4 ||
                blk_geom_mds[*idx_mds].bsize == BLOCK_4X16)
            {
                blk_geom_mds[*idx_mds].txb_count[tx_depth] = 4;
            }
            for (txb_itr = 0; txb_itr < blk_geom_mds[*idx_mds].txb_count[tx_depth]; txb_itr++) {
                if (blk_geom_mds[*idx_mds].bsize == BLOCK_64X64)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_32X32, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[4] = { 0,32,0,32 };
                    uint8_t offsety[4] = { 0,0,32,32 };
                    //   0  1
                    //   2  3
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_64X32)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_32X32, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[2] = { 0,32 };
                    uint8_t offsety[2] = { 0,0 };
                    //   0  1
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_32X64)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_32X32, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[2] = { 0,0 };
                    uint8_t offsety[2] = { 0,32 };
                    //   0  1
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_32X32)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_16X16, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[4] = { 0,16,0,16 };
                    uint8_t offsety[4] = { 0,0,16,16 };
                    //   0  1
                    //   2  3
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_32X16)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_16X16, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[2] = { 0,16 };
                    uint8_t offsety[2] = { 0,0 };
                    //   0  1
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_16X32)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_16X16, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[2] = { 0,0 };
                    uint8_t offsety[2] = { 0,16 };
                    //   0  1
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_16X16)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_8X8, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[4] = { 0,8,0,8 };
                    uint8_t offsety[4] = { 0,0,8,8 };
                    //   0  1
                    //   2  3
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_16X8)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_8X8, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[2] = { 0,8 };
                    uint8_t offsety[2] = { 0,0 };
                    //   0  1
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                    //printf("");
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_8X16)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_8X8, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[2] = { 0,0 };
                    uint8_t offsety[2] = { 0,8 };
                    //   0  1
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_8X8)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_4X4, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    uint8_t offsetx[4] = { 0,4,0,4 };
                    uint8_t offsety[4] = { 0,0,4,4 };
                    //   0  1
                    //   2  3
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_64X16)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_16X16, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    //   0  1 2 3
                    uint8_t offsetx[4] = { 0,16, 32, 48 };
                    uint8_t offsety[4] = { 0,0,  0,   0 };
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_16X64)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_16X16, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    //   0  1 2 3
                    uint8_t offsetx[4] = { 0,0,  0, 0 };
                    uint8_t offsety[4] = { 0,16,32, 48 };
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_32X8)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_8X8, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    //   0  1 2 3
                    uint8_t offsetx[4] = { 0,8, 16, 24 };
                    uint8_t offsety[4] = { 0,0,  0,   0 };
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_8X32)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_8X8, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    //   0  1 2 3
                    uint8_t offsetx[4] = { 0,0,  0, 0 };
                    uint8_t offsety[4] = { 0,8,16, 24 };
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_16X4)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_4X4, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    //   0  1 2 3
                    uint8_t offsetx[4] = { 0,4, 8, 12 };
                    uint8_t offsety[4] = { 0,0,  0,   0 };
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else if (blk_geom_mds[*idx_mds].bsize == BLOCK_4X16)
                {
                    blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(BLOCK_4X4, 0);
                    blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                    //   0  1 2 3
                    uint8_t offsetx[4] = { 0,0,  0, 0 };
                    uint8_t offsety[4] = { 0,4,8, 12 };
                    uint8_t tbx = offsetx[txb_itr];
                    uint8_t tby = offsety[txb_itr];

                    blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x + tbx;
                    blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y + tby;
                }
                else
                {
                    if (blk_geom_mds[*idx_mds].bsize == BLOCK_128X128)
                    {
                        blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(blk_geom_mds[*idx_mds].bsize, 0);
                        blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];

                        blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = (txb_itr == 0 || txb_itr == 2) ? blk_geom_mds[*idx_mds].origin_x : blk_geom_mds[*idx_mds].origin_x + 64;
                        blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = (txb_itr == 0 || txb_itr == 1) ? blk_geom_mds[*idx_mds].origin_y : blk_geom_mds[*idx_mds].origin_y + 64;
                    }
                    else if (blk_geom_mds[*idx_mds].bsize == BLOCK_128X64)
                    {
                        blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(blk_geom_mds[*idx_mds].bsize, 0);
                        blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];

                        blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = (txb_itr == 0) ? blk_geom_mds[*idx_mds].origin_x : blk_geom_mds[*idx_mds].origin_x + 64;
                        blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y;
                    }
                    else if (blk_geom_mds[*idx_mds].bsize == BLOCK_64X128)
                    {
                        blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(blk_geom_mds[*idx_mds].bsize, 0);
                        blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                        blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x;
                        blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = (txb_itr == 0) ? blk_geom_mds[*idx_mds].origin_y : blk_geom_mds[*idx_mds].origin_y + 64;
                    }
                    else
                    {
                        blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr] = av1_get_tx_size(blk_geom_mds[*idx_mds].bsize, 0);
                        blk_geom_mds[*idx_mds].txsize_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].txsize_uv[0][0];
                        blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_x;
                        blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].origin_y;
                    }
                }
                blk_geom_mds[*idx_mds].tx_boff_x[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].tx_org_x[tx_depth][txb_itr] - blk_geom_mds[*idx_mds].origin_x;
                blk_geom_mds[*idx_mds].tx_boff_y[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].tx_org_y[tx_depth][txb_itr] - blk_geom_mds[*idx_mds].origin_y;
                blk_geom_mds[*idx_mds].tx_width[tx_depth][txb_itr] = tx_size_wide[blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr]];
                blk_geom_mds[*idx_mds].tx_height[tx_depth][txb_itr] = tx_size_high[blk_geom_mds[*idx_mds].txsize[tx_depth][txb_itr]];
                blk_geom_mds[*idx_mds].tx_width_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].tx_width_uv[0][0];
                blk_geom_mds[*idx_mds].tx_height_uv[tx_depth][txb_itr] = blk_geom_mds[*idx_mds].tx_height_uv[0][0];
            }
            blk_geom_mds[*idx_mds].blkidx_mds = (*idx_mds);
            (*idx_mds) = (*idx_mds) + 1;
        }
    }

    uint32_t min_size = max_sb >> (max_depth - 1);
    if (halfsize >= min_size)
    {
        md_scan_all_blks(idx_mds, halfsize, x, y, 0,0);
        md_scan_all_blks(idx_mds, halfsize, x + halfsize, y, 0,1);
        md_scan_all_blks(idx_mds, halfsize, x, y + halfsize, 0,2);
        md_scan_all_blks(idx_mds, halfsize, x + halfsize, y + halfsize, 1,3);
    }
}

void depth_scan_all_blks()
{
    uint32_t depth_it, sq_it_y, sq_it_x, part_it, nsq_it;
    uint32_t sq_orgx, sq_orgy;
    uint32_t  depth_scan_idx = 0;

    for (depth_it = 0; depth_it < max_depth; depth_it++)
    {
        uint32_t  tot_num_sq = 1 << depth_it;
        uint32_t  sq_size = depth_it == 0 ? max_sb :
            depth_it == 1 ? max_sb / 2 :
            depth_it == 2 ? max_sb / 4 :
            depth_it == 3 ? max_sb / 8 :
            depth_it == 4 ? max_sb / 16 : max_sb / 32;

        uint32_t max_part_updated = sq_size == 128 ? MIN(max_part, 7) :
            sq_size == 8 ? MIN(max_part, 3) :
            sq_size == 4 ? 1 : max_part;

        for (sq_it_y = 0; sq_it_y < tot_num_sq; sq_it_y++)
        {
            sq_orgy = sq_it_y * sq_size;

            for (sq_it_x = 0; sq_it_x < tot_num_sq; sq_it_x++)
            {
                sq_orgx = sq_it_x * sq_size;

                for (part_it = 0; part_it < max_part_updated; part_it++)
                {
                    uint32_t tot_num_ns_per_part = part_it < 1 ? 1 :
                        part_it < 3 ? 2 :
                        part_it < 7 ? 3 : 4;

                    for (nsq_it = 0; nsq_it < tot_num_ns_per_part; nsq_it++)
                    {
                        blk_geom_dps[depth_scan_idx].blkidx_dps = depth_scan_idx;
                        blk_geom_dps[depth_scan_idx].depth = depth_it;
                        blk_geom_dps[depth_scan_idx].shape = (PART)part_it;
                        blk_geom_dps[depth_scan_idx].origin_x = sq_orgx + (sq_size / 4) *ns_quarter_off_mult[part_it][0][nsq_it];
                        blk_geom_dps[depth_scan_idx].origin_y = sq_orgy + (sq_size / 4) *ns_quarter_off_mult[part_it][1][nsq_it];

                        depth_scan_idx++;
                    }
                }
            }
        }
    }
}

void finish_depth_scan_all_blks()
{
    uint32_t do_print = 0;
    uint32_t min_size = max_sb >> (max_depth - 1);
    FILE * fp = NULL;
    if (do_print)
        FOPEN(fp, "e:\\test\\data.csv", "w");

    uint32_t depth_it, sq_it_y, sq_it_x, part_it, nsq_it;

    uint32_t  depth_scan_idx = 0;

    for (depth_it = 0; depth_it < max_depth; depth_it++)
    {
        uint32_t  tot_num_sq = 1 << depth_it;
        uint32_t  sq_size = depth_it == 0 ? max_sb :
            depth_it == 1 ? max_sb / 2 :
            depth_it == 2 ? max_sb / 4 :
            depth_it == 3 ? max_sb / 8 :
            depth_it == 4 ? max_sb / 16 : max_sb / 32;

        uint32_t max_part_updated = sq_size == 128 ? MIN(max_part, 7) :
            sq_size == 8 ? MIN(max_part, 3) :
            sq_size == 4 ? 1 : max_part;

        if (do_print)
        {
            fprintf(fp, "\n\n\n");
            printf("\n\n\n");
        }

        for (sq_it_y = 0; sq_it_y < tot_num_sq; sq_it_y++)
        {
            if (do_print)
            {
                for (uint32_t i = 0; i < sq_size / min_size; i++)
                {
                    fprintf(fp, "\n ");
                    printf("\n ");
                }
            }

            for (sq_it_x = 0; sq_it_x < tot_num_sq; sq_it_x++)
            {
                for (part_it = 0; part_it < max_part_updated; part_it++)
                {
                    uint32_t tot_num_ns_per_part = part_it < 1 ? 1 :
                        part_it < 3 ? 2 :
                        part_it < 7 ? 3 : 4;

                    for (nsq_it = 0; nsq_it < tot_num_ns_per_part; nsq_it++)
                    {
                        uint32_t matched = search_matching_from_mds(
                            blk_geom_dps[depth_scan_idx].depth,
                            blk_geom_dps[depth_scan_idx].shape,
                            blk_geom_dps[depth_scan_idx].origin_x,
                            blk_geom_dps[depth_scan_idx].origin_y);

                        blk_geom_dps[depth_scan_idx].blkidx_mds = blk_geom_mds[matched].blkidx_mds;

                        if (do_print && part_it == 0)
                        {
                            fprintf(fp, "%i", blk_geom_dps[depth_scan_idx].blkidx_mds);
                            printf("%i", blk_geom_dps[depth_scan_idx].blkidx_mds);

                            for (uint32_t i = 0; i < sq_size / min_size; i++)
                            {
                                fprintf(fp, ",");
                                printf(",");
                            }
                        }
                        depth_scan_idx++;
                    }
                }
            }
        }
    }

    if (do_print)
        fclose(fp);
}

uint32_t count_total_num_of_active_blks()
{
    uint32_t depth_it, sq_it_y, sq_it_x, part_it, nsq_it;

    uint32_t  depth_scan_idx = 0;

    for (depth_it = 0; depth_it < max_depth; depth_it++)
    {
        uint32_t  tot_num_sq = 1 << depth_it;
        uint32_t  sq_size = depth_it == 0 ? max_sb :
            depth_it == 1 ? max_sb / 2 :
            depth_it == 2 ? max_sb / 4 :
            depth_it == 3 ? max_sb / 8 :
            depth_it == 4 ? max_sb / 16 : max_sb / 32;

        uint32_t max_part_updated = sq_size == 128 ? MIN(max_part, 7) :
            sq_size == 8 ? MIN(max_part, 3) :
            sq_size == 4 ? 1 : max_part;

        for (sq_it_y = 0; sq_it_y < tot_num_sq; sq_it_y++)
        {
            for (sq_it_x = 0; sq_it_x < tot_num_sq; sq_it_x++)
            {
                for (part_it = 0; part_it < max_part_updated; part_it++)
                {
                    uint32_t tot_num_ns_per_part = part_it < 1 ? 1 :
                        part_it < 3 ? 2 :
                        part_it < 7 ? 3 : 4;

                    for (nsq_it = 0; nsq_it < tot_num_ns_per_part; nsq_it++)
                        depth_scan_idx++;
                }
            }
        }
    }

    return depth_scan_idx;
}
void log_redundancy_similarity(uint32_t  max_block_count)
{
    uint32_t blk_it, s_it;

    for (blk_it = 0; blk_it < max_block_count; blk_it++)
    {
        BlockGeom * cur_geom = &blk_geom_mds[blk_it];
        cur_geom->similar = 0;
        cur_geom->redund = 0;
        cur_geom->redund_list.list_size = 0;
        cur_geom->similar_list.list_size = 0;

        for (s_it = 0; s_it < max_block_count; s_it++)
        {
            BlockGeom * search_geom = &blk_geom_mds[s_it];

            if (cur_geom->bsize == search_geom->bsize  && cur_geom->origin_x == search_geom->origin_x &&
                cur_geom->origin_y == search_geom->origin_y && s_it != blk_it)
            {
                //one block could have similar and redundant blocks
                cur_geom->similar = 1;
                cur_geom->similar_list.blk_mds_table[cur_geom->similar_list.list_size] = search_geom->blkidx_mds;
                cur_geom->similar_list.list_size++;
                if (cur_geom->nsi == 0 && search_geom->nsi==0)
                {
                    cur_geom->redund = 1;
                    cur_geom->redund_list.blk_mds_table[cur_geom->redund_list.list_size] = search_geom->blkidx_mds;
                    cur_geom->redund_list.list_size++;
                }
            }
        }
    }
}
void build_blk_geom(int32_t use_128x128)
{
    max_sb = use_128x128 ? 128 : 64;
    max_depth = use_128x128 ? 6 : 5;
    uint32_t  max_block_count = use_128x128 ? BLOCK_MAX_COUNT_SB_128 : BLOCK_MAX_COUNT_SB_64;

    //(0)compute total number of blocks using the information provided
    max_num_active_blocks = count_total_num_of_active_blks();
    if (max_num_active_blocks != max_block_count)
        printf(" \n\n Error %i blocks\n\n ", max_num_active_blocks);

    //(1) Construct depth scan blk_geom_dps
    depth_scan_all_blks();

    //(2) Construct md scan blk_geom_mds:  use info from dps
    uint32_t idx_mds = 0;
    md_scan_all_blks(&idx_mds, max_sb, 0, 0, 0,0);

    //(3) Fill more info from mds to dps - print using dps
    finish_depth_scan_all_blks();

    log_redundancy_similarity(max_block_count);
}

//need to finish filling dps by inherting data from mds
const BlockGeom * Get_blk_geom_dps(uint32_t bidx_dps)
{
    return &blk_geom_dps[bidx_dps];
}
const BlockGeom * get_blk_geom_mds(uint32_t bidx_mds)
{
    return &blk_geom_mds[bidx_mds];
}

uint32_t get_mds_idx(uint32_t orgx, uint32_t orgy, uint32_t size, uint32_t use_128x128)
{
    uint32_t max_block_count = use_128x128 ? BLOCK_MAX_COUNT_SB_128 : BLOCK_MAX_COUNT_SB_64;
    uint32_t mds = 0;

    for (uint32_t blk_it = 0; blk_it < max_block_count; blk_it++){
        BlockGeom * cur_geom = &blk_geom_mds[blk_it];

        if ((uint32_t)cur_geom->sq_size == size && cur_geom->origin_x == orgx &&
            cur_geom->origin_y == orgy && cur_geom->shape == PART_N) {
            mds = cur_geom->blkidx_mds;
            break;
        }
    }
    return mds;
}
