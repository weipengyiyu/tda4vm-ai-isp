/*
 *
 * Copyright (c) 2020 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "iss_sensor_sc2210.h"

/* 定义增益表条目结构 */
typedef struct {
    uint32_t gain_q10;  /* 增益值 * 1024 (Q10 格式) */
    uint8_t reg_h;      /* 高 8 位寄存器值 (ANA GAIN / DIG GAIN) */
    uint8_t reg_l;      /* 低 8 位寄存器值 (ANA FINE / DIG FINE) */
} sc2210_gain_entry_t;

/**
 * @brief SC2210 模拟增益查找表 (基于 Datasheet 表 2-5)
 * 覆盖范围: 1.0x (~0dB) 到 54.07x (~34.66dB)
 */
static const sc2210_gain_entry_t gSC2210AnaGainTable[] = {
    /* --- ANA GAIN = 0x03 (图 1) --- */
    {1024,  0x03, 0x40}, {1040,  0x03, 0x41}, {1056,  0x03, 0x42}, {1072,  0x03, 0x43},
    {1088,  0x03, 0x44}, {1104,  0x03, 0x45}, {1120,  0x03, 0x46}, {1136,  0x03, 0x47},
    {1152,  0x03, 0x48}, {1168,  0x03, 0x49}, {1184,  0x03, 0x4A}, {1200,  0x03, 0x4B},
    {1216,  0x03, 0x4C}, {1232,  0x03, 0x4D}, {1248,  0x03, 0x4E}, {1264,  0x03, 0x4F},
    {1280,  0x03, 0x50}, {1296,  0x03, 0x51}, {1312,  0x03, 0x52}, {1328,  0x03, 0x53},
    {1344,  0x03, 0x54}, {1360,  0x03, 0x55}, {1376,  0x03, 0x56}, {1392,  0x03, 0x57},
    {1408,  0x03, 0x58}, {1424,  0x03, 0x59}, {1440,  0x03, 0x5A}, {1456,  0x03, 0x5B},
    {1472,  0x03, 0x5C}, {1488,  0x03, 0x5D}, {1504,  0x03, 0x5E}, {1520,  0x03, 0x5F},
    {1536,  0x03, 0x60}, {1552,  0x03, 0x61}, {1568,  0x03, 0x62}, {1584,  0x03, 0x63},
    {1600,  0x03, 0x64}, {1616,  0x03, 0x65}, {1632,  0x03, 0x66}, {1648,  0x03, 0x67},
    {1664,  0x03, 0x68}, {1680,  0x03, 0x69}, {1696,  0x03, 0x6A}, {1712,  0x03, 0x6B},
    {1728,  0x03, 0x6C}, {1744,  0x03, 0x6D}, {1760,  0x03, 0x6E}, {1776,  0x03, 0x6F},
    {1792,  0x03, 0x70}, {1808,  0x03, 0x71}, {1824,  0x03, 0x72}, {1840,  0x03, 0x73},
    {1856,  0x03, 0x74}, {1872,  0x03, 0x75}, {1888,  0x03, 0x76}, {1904,  0x03, 0x77},
    {1920,  0x03, 0x78}, {1936,  0x03, 0x79}, {1952,  0x03, 0x7A}, {1968,  0x03, 0x7B},
    {1984,  0x03, 0x7C}, {2000,  0x03, 0x7D}, {2016,  0x03, 0x7E}, {2032,  0x03, 0x7F},

    /* --- ANA GAIN = 0x07 (图 1 下半部分 & 图 2 上半部分) --- */
    {2048,  0x07, 0x40}, {2080,  0x07, 0x41}, {2112,  0x07, 0x42}, {2144,  0x07, 0x43},
    {2176,  0x07, 0x44}, {2208,  0x07, 0x45}, {2240,  0x07, 0x46}, {2272,  0x07, 0x47},
    {2304,  0x07, 0x48}, {2336,  0x07, 0x49}, {2368,  0x07, 0x4A}, {2400,  0x07, 0x4B},
    {2432,  0x07, 0x4C}, {2464,  0x07, 0x4D}, {2496,  0x07, 0x4E}, {2528,  0x07, 0x4F},
    {2560,  0x07, 0x50}, {2592,  0x07, 0x51}, {2624,  0x07, 0x52}, {2656,  0x07, 0x53},
    {2688,  0x07, 0x54}, {2720,  0x07, 0x55}, {2752,  0x07, 0x56}, {2784,  0x07, 0x57},
    {2816,  0x07, 0x58}, {2848,  0x07, 0x59}, {2880,  0x07, 0x5A}, {2912,  0x07, 0x5B},
    {2944,  0x07, 0x5C}, {2976,  0x07, 0x5D}, {3008,  0x07, 0x5E}, {3040,  0x07, 0x5F},
    {3072,  0x07, 0x60}, {3104,  0x07, 0x61}, {3136,  0x07, 0x62}, {3168,  0x07, 0x63},
    {3200,  0x07, 0x64}, {3232,  0x07, 0x65}, {3264,  0x07, 0x66}, {3296,  0x07, 0x67},
    {3328,  0x07, 0x68}, {3360,  0x07, 0x69}, {3392,  0x07, 0x6A}, {3424,  0x07, 0x6B},
    {3456,  0x07, 0x6C}, {3488,  0x07, 0x6D}, {3520,  0x07, 0x6E}, {3552,  0x07, 0x6F},
    {3584,  0x07, 0x70}, {3616,  0x07, 0x71}, {3648,  0x07, 0x72}, {3680,  0x07, 0x73},
    {3712,  0x07, 0x74}, {3744,  0x07, 0x75}, {3776,  0x07, 0x76}, {3808,  0x07, 0x77},
    {3840,  0x07, 0x78}, {3872,  0x07, 0x79}, {3904,  0x07, 0x7A}, {3936,  0x07, 0x7B},
    {3968,  0x07, 0x7C}, {4000,  0x07, 0x7D}, {4032,  0x07, 0x7E}, {4064,  0x07, 0x7F},

    /* --- ANA GAIN = 0x23 (图 2 下半部分 & 图 3 上半部分) --- */
    {4096,  0x23, 0x40}, {4144,  0x23, 0x41}, {4192,  0x23, 0x42}, {4240,  0x23, 0x43},
    {4288,  0x23, 0x44}, {4336,  0x23, 0x45}, {4384,  0x23, 0x46}, {4432,  0x23, 0x47},
    {4480,  0x23, 0x48}, {4528,  0x23, 0x49}, {4576,  0x23, 0x4A}, {4624,  0x23, 0x4B},
    {4672,  0x23, 0x4C}, {4720,  0x23, 0x4D}, {4768,  0x23, 0x4E}, {4816,  0x23, 0x4F},
    {4864,  0x23, 0x50}, {4912,  0x23, 0x51}, {4960,  0x23, 0x52}, {5008,  0x23, 0x53},
    {5056,  0x23, 0x54}, {5104,  0x23, 0x55}, {5152,  0x23, 0x56}, {5200,  0x23, 0x57},
    {5248,  0x23, 0x58}, {5296,  0x23, 0x59}, {5344,  0x23, 0x5A}, {5392,  0x23, 0x5B},
    {5440,  0x23, 0x5C}, {5488,  0x23, 0x5D}, {5536,  0x23, 0x5E}, {5584,  0x23, 0x5F},
    {5632,  0x23, 0x60}, {5680,  0x23, 0x61}, {5728,  0x23, 0x62}, {5776,  0x23, 0x63},
    {5824,  0x23, 0x64}, {5872,  0x23, 0x65}, {5920,  0x23, 0x66}, {5968,  0x23, 0x67},
    {6016,  0x23, 0x68}, {6064,  0x23, 0x69}, {6112,  0x23, 0x6A}, {6160,  0x23, 0x6B},
    {6208,  0x23, 0x6C}, {6256,  0x23, 0x6D}, {6304,  0x23, 0x6E}, {6352,  0x23, 0x6F},
    {6400,  0x23, 0x70}, {6448,  0x23, 0x71}, {6496,  0x23, 0x72}, {6544,  0x23, 0x73},
    {6592,  0x23, 0x74}, {6640,  0x23, 0x75}, {6688,  0x23, 0x76}, {6736,  0x23, 0x77},
    {6784,  0x23, 0x78}, {6832,  0x23, 0x79}, {6880,  0x23, 0x7A}, {6928,  0x23, 0x7B},
    {6976,  0x23, 0x7C}, {7024,  0x23, 0x7D}, {7072,  0x23, 0x7E}, {7120,  0x23, 0x7F},

    /* --- ANA GAIN = 0x27 (图 3 下半部分 & 图 4 上半部分) --- */
    {7168,  0x27, 0x40}, {7216,  0x27, 0x41}, {7264,  0x27, 0x42}, {7312,  0x27, 0x43},
    {7360,  0x27, 0x44}, {7408,  0x27, 0x45}, {7456,  0x27, 0x46}, {7504,  0x27, 0x47},
    {7552,  0x27, 0x48}, {7600,  0x27, 0x49}, {7648,  0x27, 0x4A}, {7696,  0x27, 0x4B},
    {7744,  0x27, 0x4C}, {7792,  0x27, 0x4D}, {7840,  0x27, 0x4E}, {7888,  0x27, 0x4F},
    {7936,  0x27, 0x50}, {7984,  0x27, 0x51}, {8032,  0x27, 0x52}, {8080,  0x27, 0x53},
    {8128,  0x27, 0x54}, {8176,  0x27, 0x55}, {8224,  0x27, 0x56}, {8272,  0x27, 0x57},
    {8320,  0x27, 0x58}, {8368,  0x27, 0x59}, {8416,  0x27, 0x5A}, {8464,  0x27, 0x5B},
    {8512,  0x27, 0x5C}, {8560,  0x27, 0x5D}, {8608,  0x27, 0x5E}, {8656,  0x27, 0x5F},
    {8704,  0x27, 0x60}, {8752,  0x27, 0x61}, {8800,  0x27, 0x62}, {8848,  0x27, 0x63},
    {8896,  0x27, 0x64}, {8944,  0x27, 0x65}, {8992,  0x27, 0x66}, {9040,  0x27, 0x67},
    {9088,  0x27, 0x68}, {9136,  0x27, 0x69}, {9184,  0x27, 0x6A}, {9232,  0x27, 0x6B},
    {9280,  0x27, 0x6C}, {9328,  0x27, 0x6D}, {9376,  0x27, 0x6E}, {9424,  0x27, 0x6F},
    {9472,  0x27, 0x70}, {9520,  0x27, 0x71}, {9568,  0x27, 0x72}, {9616,  0x27, 0x73},
    {9664,  0x27, 0x74}, {9712,  0x27, 0x75}, {9760,  0x27, 0x76}, {9808,  0x27, 0x77},
    {9856,  0x27, 0x78}, {9904,  0x27, 0x79}, {9952,  0x27, 0x7A}, {10000, 0x27, 0x7B},
    {10048, 0x27, 0x7C}, {10096, 0x27, 0x7D}, {10144, 0x27, 0x7E}, {10192, 0x27, 0x7F},

    /* --- ANA GAIN = 0x2F (图 4 下半部分 & 图 5 上半部分) --- */
    {10240, 0x2F, 0x40}, {10288, 0x2F, 0x41}, {10336, 0x2F, 0x42}, {10384, 0x2F, 0x43},
    {10432, 0x2F, 0x44}, {10480, 0x2F, 0x45}, {10528, 0x2F, 0x46}, {10576, 0x2F, 0x47},
    {10624, 0x2F, 0x48}, {10672, 0x2F, 0x49}, {10720, 0x2F, 0x4A}, {10768, 0x2F, 0x4B},
    {10816, 0x2F, 0x4C}, {10864, 0x2F, 0x4D}, {10912, 0x2F, 0x4E}, {10960, 0x2F, 0x4F},
    {11008, 0x2F, 0x50}, {11056, 0x2F, 0x51}, {11104, 0x2F, 0x52}, {11152, 0x2F, 0x53},
    {11200, 0x2F, 0x54}, {11248, 0x2F, 0x55}, {11296, 0x2F, 0x56}, {11344, 0x2F, 0x57},
    {11392, 0x2F, 0x58}, {11440, 0x2F, 0x59}, {11488, 0x2F, 0x5A}, {11536, 0x2F, 0x5B},
    {11584, 0x2F, 0x5C}, {11632, 0x2F, 0x5D}, {11680, 0x2F, 0x5E}, {11728, 0x2F, 0x5F},
    {11776, 0x2F, 0x60}, {11824, 0x2F, 0x61}, {11872, 0x2F, 0x62}, {11920, 0x2F, 0x63},
    {11968, 0x2F, 0x64}, {12016, 0x2F, 0x65}, {12064, 0x2F, 0x66}, {12112, 0x2F, 0x67},
    {12160, 0x2F, 0x68}, {12208, 0x2F, 0x69}, {12256, 0x2F, 0x6A}, {12304, 0x2F, 0x6B},
    {12352, 0x2F, 0x6C}, {12400, 0x2F, 0x6D}, {12448, 0x2F, 0x6E}, {12496, 0x2F, 0x6F},
    {12544, 0x2F, 0x70}, {12592, 0x2F, 0x71}, {12640, 0x2F, 0x72}, {12688, 0x2F, 0x73},
    {12736, 0x2F, 0x74}, {12784, 0x2F, 0x75}, {12832, 0x2F, 0x76}, {12880, 0x2F, 0x77},
    {12928, 0x2F, 0x78}, {12976, 0x2F, 0x79}, {13024, 0x2F, 0x7A}, {13072, 0x2F, 0x7B},
    {13120, 0x2F, 0x7C}, {13168, 0x2F, 0x7D}, {13216, 0x2F, 0x7E}, {13264, 0x2F, 0x7F},

    /* --- ANA GAIN = 0x3F (图 5 下半部分) --- */
    {13312, 0x3F, 0x40}, {13360, 0x3F, 0x41}, {13408, 0x3F, 0x42}, {13456, 0x3F, 0x43},
    {13504, 0x3F, 0x44}, {13552, 0x3F, 0x45}, {13600, 0x3F, 0x46}, {13648, 0x3F, 0x47},
    {13696, 0x3F, 0x48}, {13744, 0x3F, 0x49}, {13792, 0x3F, 0x4A}, {13840, 0x3F, 0x4B},
    {13888, 0x3F, 0x4C}, {13936, 0x3F, 0x4D}, {13984, 0x3F, 0x4E}, {14032, 0x3F, 0x4F},
    {14080, 0x3F, 0x50}, {14128, 0x3F, 0x51}, {14176, 0x3F, 0x52}, {14224, 0x3F, 0x53},
    {14272, 0x3F, 0x54}, {14320, 0x3F, 0x55}, {14368, 0x3F, 0x56}, {14416, 0x3F, 0x57},
    {14464, 0x3F, 0x58}, {14512, 0x3F, 0x59}, {14560, 0x3F, 0x5A}, {14608, 0x3F, 0x5B},
    {14656, 0x3F, 0x5C}, {14704, 0x3F, 0x5D}, {14752, 0x3F, 0x5E}, {14800, 0x3F, 0x5F},
    {14848, 0x3F, 0x60}, {14896, 0x3F, 0x61}, {14944, 0x3F, 0x62}, {14992, 0x3F, 0x63},
    {15040, 0x3F, 0x64}, {15088, 0x3F, 0x65}, {15136, 0x3F, 0x66}, {15184, 0x3F, 0x67},
    {15232, 0x3F, 0x68}, {15280, 0x3F, 0x69}, {15328, 0x3F, 0x6A}, {15376, 0x3F, 0x6B},
    {15424, 0x3F, 0x6C}, {15472, 0x3F, 0x6D}, {15520, 0x3F, 0x6E}, {15568, 0x3F, 0x6F},
    {15616, 0x3F, 0x70}, {15664, 0x3F, 0x71}, {15712, 0x3F, 0x72}, {15760, 0x3F, 0x73},
    {15808, 0x3F, 0x74}, {15856, 0x3F, 0x75}, {15904, 0x3F, 0x76}, {15952, 0x3F, 0x77},
    {16000, 0x3F, 0x78}, {16048, 0x3F, 0x79}, {16096, 0x3F, 0x7A}, {16144, 0x3F, 0x7B},
    {16192, 0x3F, 0x7C}, {16240, 0x3F, 0x7D}, {16288, 0x3F, 0x7E}, {16336, 0x3F, 0x7F},
    
    /* 最后一项 (图 5 右下角) */
    {55367, 0x3F, 0x7F} 
};

#define SC2210_ANA_GAIN_TBL_SIZE (sizeof(gSC2210AnaGainTable) / sizeof(gSC2210AnaGainTable[0]))


/**
 * @brief SC2210 数字增益查找表 (基于 Datasheet 表 2-6)
 * 覆盖范围: 1.0x (~0dB) 到 31.5x (~29.97dB)
 */
 static const sc2210_gain_entry_t gSC2210DigGainTable[] = {
    /* --- DIG GAIN = 0x00 (图 6) --- */
    {1024,  0x00, 0x80}, {1056,  0x00, 0x84}, {1088,  0x00, 0x88}, {1120,  0x00, 0x8C},
    {1152,  0x00, 0x90}, {1184,  0x00, 0x94}, {1216,  0x00, 0x98}, {1248,  0x00, 0x9C},
    {1280,  0x00, 0xA0}, {1312,  0x00, 0xA4}, {1344,  0x00, 0xA8}, {1376,  0x00, 0xAC},
    {1408,  0x00, 0xB0}, {1440,  0x00, 0xB4}, {1472,  0x00, 0xB8}, {1504,  0x00, 0xBC},
    {1536,  0x00, 0xC0}, {1568,  0x00, 0xC4}, {1600,  0x00, 0xC8}, {1632,  0x00, 0xCC},
    {1664,  0x00, 0xD0}, {1696,  0x00, 0xD4}, {1728,  0x00, 0xD8}, {1760,  0x00, 0xDC},
    {1792,  0x00, 0xE0}, {1824,  0x00, 0xE4}, {1856,  0x00, 0xE8}, {1888,  0x00, 0xEC},
    {1920,  0x00, 0xF0}, {1952,  0x00, 0xF4}, {1984,  0x00, 0xF8}, {2016,  0x00, 0xFC},

    /* --- DIG GAIN = 0x01 (图 6 下半部分) --- */
    {2048,  0x01, 0x80}, {2112,  0x01, 0x84}, {2176,  0x01, 0x88}, {2240,  0x01, 0x8C},
    {2304,  0x01, 0x90}, {2368,  0x01, 0x94}, {2432,  0x01, 0x98}, {2496,  0x01, 0x9C},
    {2560,  0x01, 0xA0}, {2624,  0x01, 0xA4}, {2688,  0x01, 0xA8}, {2752,  0x01, 0xAC},
    {2816,  0x01, 0xB0}, {2880,  0x01, 0xB4}, {2944,  0x01, 0xB8}, {3008,  0x01, 0xBC},
    {3072,  0x01, 0xC0}, {3136,  0x01, 0xC4}, {3200,  0x01, 0xC8}, {3264,  0x01, 0xCC},
    {3328,  0x01, 0xD0}, {3392,  0x01, 0xD4}, {3456,  0x01, 0xD8}, {3520,  0x01, 0xDC},
    {3584,  0x01, 0xE0}, {3648,  0x01, 0xE4}, {3712,  0x01, 0xE8}, {3776,  0x01, 0xEC},
    {3840,  0x01, 0xF0}, {3904,  0x01, 0xF4}, {3968,  0x01, 0xF8}, {4032,  0x01, 0xFC},

    /* --- DIG GAIN = 0x03 (图 6 底部 & 图 7 顶部) --- */
    {4096,  0x03, 0x80}, {4160,  0x03, 0x84}, {4224,  0x03, 0x88}, {4288,  0x03, 0x8C},
    {4352,  0x03, 0x90}, {4416,  0x03, 0x94}, {4480,  0x03, 0x98}, {4544,  0x03, 0x9C},
    {4608,  0x03, 0xA0}, {4672,  0x03, 0xA4}, {4736,  0x03, 0xA8}, {4800,  0x03, 0xAC},
    {4864,  0x03, 0xB0}, {4928,  0x03, 0xB4}, {4992,  0x03, 0xB8}, {5056,  0x03, 0xBC},
    {5120,  0x03, 0xC0}, {5184,  0x03, 0xC4}, {5248,  0x03, 0xC8}, {5312,  0x03, 0xCC},
    {5376,  0x03, 0xD0}, {5440,  0x03, 0xD4}, {5504,  0x03, 0xD8}, {5568,  0x03, 0xDC},
    {5632,  0x03, 0xE0}, {5696,  0x03, 0xE4}, {5760,  0x03, 0xE8}, {5824,  0x03, 0xEC},
    {5888,  0x03, 0xF0}, {5952,  0x03, 0xF4}, {6016,  0x03, 0xF8}, {6080,  0x03, 0xFC},

    /* --- DIG GAIN = 0x07 (图 7) --- */
    {6144,  0x07, 0x80}, {6272,  0x07, 0x84}, {6400,  0x07, 0x88}, {6528,  0x07, 0x8C},
    {6656,  0x07, 0x90}, {6784,  0x07, 0x94}, {6912,  0x07, 0x98}, {7040,  0x07, 0x9C},
    {7168,  0x07, 0xA0}, {7296,  0x07, 0xA4}, {7424,  0x07, 0xA8}, {7552,  0x07, 0xAC},
    {7680,  0x07, 0xB0}, {7808,  0x07, 0xB4}, {7936,  0x07, 0xB8}, {8064,  0x07, 0xBC},
    {8192,  0x07, 0xC0}, {8320,  0x07, 0xC4}, {8448,  0x07, 0xC8}, {8576,  0x07, 0xCC},
    {8704,  0x07, 0xD0}, {8832,  0x07, 0xD4}, {8960,  0x07, 0xD8}, {9088,  0x07, 0xDC},
    {9216,  0x07, 0xE0}, {9344,  0x07, 0xE4}, {9472,  0x07, 0xE8}, {9600,  0x07, 0xEC},
    {9728,  0x07, 0xF0}, {9856,  0x07, 0xF4}, {9984,  0x07, 0xF8}, {10112, 0x07, 0xFC},

    /* --- DIG GAIN = 0x0F (图 7 底部 & 图 8) --- */
    {10240, 0x0F, 0x80}, {10496, 0x0F, 0x84}, {10752, 0x0F, 0x88}, {11008, 0x0F, 0x8C},
    {11264, 0x0F, 0x90}, {11520, 0x0F, 0x94}, {11776, 0x0F, 0x98}, {12032, 0x0F, 0x9C},
    {12288, 0x0F, 0xA0}, {12544, 0x0F, 0xA4}, {12800, 0x0F, 0xA8}, {13056, 0x0F, 0xAC},
    {13312, 0x0F, 0xB0}, {13568, 0x0F, 0xB4}, {13824, 0x0F, 0xB8}, {14080, 0x0F, 0xBC},
    {14336, 0x0F, 0xC0}, {14592, 0x0F, 0xC4}, {14848, 0x0F, 0xC8}, {15104, 0x0F, 0xCC},
    {15360, 0x0F, 0xD0}, {15616, 0x0F, 0xD4}, {15872, 0x0F, 0xD8}, {16128, 0x0F, 0xDC},
    {16384, 0x0F, 0xE0}, {16640, 0x0F, 0xE4}, {16896, 0x0F, 0xE8}, {17152, 0x0F, 0xEC},
    {17408, 0x0F, 0xF0}, {17664, 0x0F, 0xF4}, {17920, 0x0F, 0xF8}, {18176, 0x0F, 0xFC},

    /* 最后几项 (图 8) */
    {27648, 0x0F, 0xD8}, {28160, 0x0F, 0xDC}, {28672, 0x0F, 0xE0}, {29184, 0x0F, 0xE4},
    {29696, 0x0F, 0xE8}, {30208, 0x0F, 0xEC}, {30720, 0x0F, 0xF0}, {31232, 0x0F, 0xF4},
    {31744, 0x0F, 0xF8}, {32256, 0x0F, 0xFC}
};

#define SC2210_DIG_GAIN_TBL_SIZE (sizeof(gSC2210DigGainTable) / sizeof(gSC2210DigGainTable[0]))

    
static IssSensor_CreateParams  SC2210CreatePrms = {
    SENSOR_SC2210,     /*sensor name*/
    0x6,                             /*i2cInstId*/
    {
        0x30, 0x30, 0x30, 0x30,               /* #define SENSOR_I2C_ADDR    0x60 */
        0,0,0,0
    },/*i2cAddrSensor*/
    {
        0, 0, 0, 0,
        0, 0, 0, 0
    },/*i2cAddrSer*/
    /*IssSensor_Info*/
    {
        {
            1920,                               /*width 1920 2304*/
            1080,                               /*height 1080 1092*/
            1,                                  /*num_exposures*/
            vx_false_e,                         /*line_interleaved*/
            {
                {TIVX_RAW_IMAGE_16_BIT, 11},    /*dataFormat and MSB [0] TIVX_RAW_IMAGE_P12_BIT TIVX_RAW_IMAGE_16_BIT Unpacked 16-bit: 1920*1080*2 = 4,147,200 字节*/
            },
            0,                                  /*meta_height_before*/
            0,                                  /*meta_height_after*/
        },
        ISS_SENSOR_SC2210_FEATURES,              /*features*/
        ALGORITHMS_ISS_AEWB_MODE_AEWB,          /*aewbMode ALGORITHMS_ISS_AEWB_MODE_AEWB ALGORITHMS_ISS_AEWB_MODE_NONE ALGORITHMS_ISS_AEWB_MODE_AEWB*/
        30,                                     /*fps*/
        4,                                      /*numDataLanes*/
        {1, 2, 3, 4},                           /*dataLanesMap*/
        {0, 0, 0, 0},                           /*dataLanesPolarity*/
        CSIRX_LANE_BAND_SPEED_400_TO_480_MBPS, /*CSIRX_LANE_BAND_SPEED_400_TO_480_MBPS=1920*1080*25*16*2/4; CSIRX_LANE_BAND_SPEED_720_TO_800_MBPS:csi_laneBandSpeed 1920*1080*60*16*1.5/4 = 746,496,000 bits/秒;1 Mbps = 1,000,000 bits/s 746.496 Mbps */
    },
    2,                                  /*numChan 这里跟i2cAddrSensor有关联*/
    2210,                                /*dccId 5233*/
};


static IssSensorFxns           SC2210SensorFxns = {
    SC2210_Probe,
    SC2210_Config,
    SC2210_StreamOn,
    SC2210_StreamOff,
    SC2210_PowerOn,
    SC2210_PowerOff,
    SC2210_GetExpParams,
    SC2210_SetAeParams,
    SC2210_GetDccParams,
    SC2210_InitAewbConfig,
    SC2210_GetIspConfig,
    SC2210_ReadWriteReg,
    SC2210_GetExpPrgFxn,
    SC2210_deinit,
    SC2210_GetWBPrgFxn,
    SC2210_SetAwbParams
};

static IssSensorIntfParams     SC2210SensorIntfPrms = {
    0,             /*sensorBroadcast*/
    0,             /*enableFsin*/
    0,			   /*numCamerasStreaming*/
};

IssSensors_Handle SC2210SensorHandle = {
    1,                                 /*isUsed*/
    &SC2210CreatePrms,                /*CreatePrms*/
    &SC2210SensorFxns,                /*SensorFxns*/
    &SC2210SensorIntfPrms,            /*SensorIntfPrms*/
};

/*
 * \brief DCC Parameters of SC2210
 */
//IssCapture_CmplxIoLaneCfg           SC2210Csi2CmplxIoLaneCfg;
extern IssSensors_Handle * gIssSensorTable[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
static uint16_t sp1hGainRegValueOld[ISS_SENSORS_MAX_CHANNEL];
static uint16_t redGain_prev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t greenGain_prev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t blueGain_prev[ISS_SENSORS_MAX_CHANNEL];

int32_t IssSensor_SC2210_Init()
{
    int32_t status;
    int32_t chId;
    status = IssSensor_Register(&SC2210SensorHandle);
    if(0 != status)
    {
        printf("IssSensor_SC2210_Init failed \n");
    }
    printf("IssSensor_SC2210_Init is ok \n");

    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t SC2210_Probe(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    return (status);
}

static int32_t OVC3C_RunSensorConfigScript(uint32_t i2cInstId, uint8_t sensorI2cAddr, const I2cParams *sensorCfg, uint16_t sensor_cfg_script_len)
{
    int32_t status = 0;
    uint16_t regAddr;
    uint8_t regValue;
    uint8_t checkValue = 0;
    uint16_t delayMilliSec;
    uint32_t regCnt;
    uint32_t errregCnt;

    if(NULL != sensorCfg)
    {
        regCnt = 0;
        errregCnt = 0;
        regAddr  = sensorCfg[regCnt].nRegAddr;
        regValue = sensorCfg[regCnt].nRegValue;
        delayMilliSec = sensorCfg[regCnt].nDelay;
        issLogPrintf(" sensorCfg[%d].nRegAddr = 0x%x\n", regCnt, sensorCfg[regCnt].nRegAddr);

        issLogPrintf(" Configuring OVC3C imager sensorI2cAddr = 0x%x.. Please wait till it finishes \n", sensorI2cAddr);
        while(regCnt<sensor_cfg_script_len)
        {
            status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, regAddr, regValue, 1u);

            if (0 != status)
            {
                errregCnt++;
                // issLogPrintf(" \n \n OVC3C: Sensor Reg Write Failed for regAddr 0x%x \n \n", regAddr);
            }

            if(delayMilliSec > 0)
            {
               appLogWaitMsecs(delayMilliSec);
               issLogPrintf(" \n \n OVC3C: regAddr 0x%x, delayMilliSec=%d ms \n \n", regAddr, delayMilliSec);
            }

            regCnt++;
            regAddr  = sensorCfg[regCnt].nRegAddr;
            regValue = sensorCfg[regCnt].nRegValue;
            delayMilliSec = sensorCfg[regCnt].nDelay;
            // appLogWaitMsecs(2);
        }
        /*Wait 100ms after the init is done*/
        appLogWaitMsecs(100);
        issLogPrintf(" OVC3C config done ,config total = %d, errregCnt = %d \n ", regCnt, errregCnt);
    }
    else
    {
        issLogPrintf(" OVC3C config script is NULL \n");
    }
    return status;
}

static uint32_t SC2210FeaturesEnabled;
static int32_t SC2210_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams * pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);
    uint8_t  i2cAddrSensor = pCreatePrms->i2cAddrSensor[chId];
    uint32_t i2cInstId     = pCreatePrms->i2cInstId;
    uint32_t sensorConfigSize = 0;
    
    appLogWaitMsecs(100);

    sensorConfigSize = sizeof(sc2210_linear_4lane_12bit_1920x1080_30fps_init_all) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, i2cAddrSensor, sc2210_linear_4lane_12bit_1920x1080_30fps_init_all, sensorConfigSize);
    if (status != 0)
    {
        issLogPrintf("Config error : OVC3C_RunSensorConfigScript is error !!!\n");
        return status;
    }
    issLogPrintf("Config : OVC3C_RunSensorConfigScript is ok !!!\n");
    appLogWaitMsecs(100);
    
    return (status);
}

static int32_t SC2210_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    issLogPrintf("SC2210_StreamOn  -------------------> start  chId:%d\n", chId);
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams * pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    uint8_t checkValue = 0;
    uint8_t  i2cAddrSensor = pCreatePrms->i2cAddrSensor[chId];
    uint32_t i2cInstId     = pCreatePrms->i2cInstId;

    status = SC2210_WriteReg(i2cInstId, i2cAddrSensor, 0x0100, 0x01, 1u);
    if (status != 0)
    {
        issLogPrintf("SC2210_StreamOn: SC2210_WriteReg 0x0100 error \n");
        return status;
    }
 
    issLogPrintf("----------->SC2210_StreamOn SC2210_WriteReg streaming is ok \n");
    return (status);
}

static int32_t SC2210_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    return status;
}

static int32_t SC2210_PowerOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    printf("SC2210_PowerOn : chId = 0x%x \n", chId);
    issLogPrintf("SC2210_PowerOn  & sensor  is reset -------------------> end\n");

    return status;
}

static int32_t SC2210_PowerOff(uint32_t chId, void *pSensorHdl)
{
    printf("--------------> SC2210_PowerOff \n");
    return (0);
}

static void SC2210_deinit (uint32_t chId, void *pSensorHdl)
{
    printf("-------------------------------------------> SC2210_deinit \n");
    return;
}

static int32_t SC2210_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg)
{
    int32_t status = 0;

    uint8_t regValue = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;

    assert(NULL != pReg);

    if (1u == readWriteFlag)
    {
        /*write*/
        regValue = pReg->nRegValue;
        status = SC2210_WriteReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, regValue, 1u);
    }
    else
    {
        /*read*/
        status = SC2210_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue, 1u);

        if (0 == status)
        {
            pReg->nRegValue = regValue;
        }
    }
    return (status);
}

static int32_t SC2210_ReadReg(uint8_t     i2cInstId,
                            uint8_t         i2cAddr,
                            uint16_t        regAddr,
                            uint8_t         *regVal,
                            uint32_t        numRegs)
{
    int32_t  status = -1;
    I2C_Handle sensorI2cHandle = NULL;
    static uint8_t sensorI2cByteOrder = 255U;
    getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
    if(NULL == sensorI2cHandle)
    {
        printf("Sensor I2C Handle is NULL \n");
        return -1;
    }
    status = Board_i2c16BitRegRd(sensorI2cHandle, i2cAddr, regAddr, regVal, numRegs, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
    if(0 != status)
    {
        issLogPrintf("Error : I2C Timeout while reading from SC2210 register 0x%x \n", regAddr);
    }
    return (status);
}

static int32_t SC2210_WriteReg(uint8_t    i2cInstId,
                             uint8_t       i2cAddr,
                             uint16_t         regAddr,
                             uint8_t          regVal,
                             uint32_t      numRegs)
{
    int32_t  status = -1;
    I2C_Handle sensorI2cHandle = NULL;
    static uint8_t sensorI2cByteOrder = 255U;
    getIssSensorI2cInfo(&sensorI2cByteOrder, &sensorI2cHandle);
    if(NULL == sensorI2cHandle)
    {
        printf("Sensor I2C Handle is NULL \n");
        return -1;
    }
    status = Board_i2c16BitRegWr(sensorI2cHandle, i2cAddr, regAddr, &regVal, numRegs, sensorI2cByteOrder, SENSOR_I2C_TIMEOUT);
    if(0 != status)
    {
        printf("Error : I2C Timeout while i2cAddr:%x writing 0x%x to SC2210 register 0x%x \n", i2cAddr, regVal, regAddr);
    }

    return (status);
}

/**
 * @brief SC2210 Set AE Params (最终版：完全信任 AE 算法，无硬编码阈值)
 * 
 * 策略说明：
 * 1. 移除了所有人为设定的增益拆分阈值（如 8.0x 或 54.0x）。
 * 2. AE 请求的 targetTotalGainQ10 将直接用于模拟增益查表。
 * 3. 数字增益默认固定为 1.0x (1024)。
 * 4. 仅当 AE 请求值超过查找表最大项时，才强制钳位到最大值，防止数组越界。
 */
static int32_t SC2210_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t  status = 0;
    IssSensors_Handle *pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams *pCreatePrms;
    
    uint8_t anaGainRegH = 0, anaGainRegL = 0;
    uint8_t digGainRegH = 0, digGainRegL = 0;
    uint16_t exposureLines = 0;
    uint32_t vmax = 1125; 
    uint32_t i2cInstId;
    uint8_t  sensorI2cAddr;
    uint16_t cnt;
    
    assert(pSenHandle != NULL);
    pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    i2cInstId     = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    // ------------------------------------------------------------------------
    // 1. 获取参数
    // ------------------------------------------------------------------------
    uint32_t exposureTimeNs = pExpPrms->exposureTime[ISS_SENSOR_EXPOSURE_LONG];
    // AE 算法计算的总增益目标
    uint32_t targetTotalGainQ10 = pExpPrms->analogGain[ISS_SENSOR_EXPOSURE_LONG];

    // ------------------------------------------------------------------------
    // 2. 读取 VMAX (Frame Length)
    // ------------------------------------------------------------------------
    uint8_t val_h, val_l;
    if (SC2210_ReadReg(i2cInstId, sensorI2cAddr, 0x320e, &val_h, 1u) == 0 &&
        SC2210_ReadReg(i2cInstId, sensorI2cAddr, 0x320f, &val_l, 1u) == 0) {
        vmax = ((uint32_t)val_h << 8) | val_l;
    } else {
        vmax = 1350; 
        // printf("[SC2210] Warning: Read VMAX failed, using default %d\n", vmax);
    }

    // ------------------------------------------------------------------------
    // 3. 计算曝光行数
    // ------------------------------------------------------------------------
    if (vmax > 0 && pCreatePrms->sensorInfo.fps > 0) {
        uint64_t numerator = (uint64_t)exposureTimeNs * pCreatePrms->sensorInfo.fps * vmax;
        exposureLines = (uint16_t)((numerator + 500000000ULL) / 1000000000ULL);
    } else {
        exposureLines = 4;
    }

    // 安全限制
    if (exposureLines < 4) exposureLines = 4;
    if (exposureLines > (vmax - 4)) exposureLines = vmax - 4;

    // ------------------------------------------------------------------------
    // 4. 【核心修改】增益处理：完全信任 AE，无中间拆分逻辑
    // ------------------------------------------------------------------------
    uint32_t finalAnaGainQ10;
    uint32_t finalDigGainQ10;

    // 获取查找表中的最大物理增益值 (数组最后一项)
    uint32_t maxTableGainQ10 = gSC2210AnaGainTable[SC2210_ANA_GAIN_TBL_SIZE - 1].gain_q10;

    // 逻辑：
    // 如果 AE 请求 <= 表内最大值：直接使用 AE 请求值作为模拟增益。
    // 如果 AE 请求 > 表内最大值：钳位到表内最大值 (保护硬件)，数字增益仍保持 1.0x。
    // 注意：这里不再计算 "剩余增益给数字增益"，因为我们要把决策权完全交给 AE。
    // 如果 AE 需要数字增益，它应该通过调整总增益请求或在 ISP 端处理。
    
    if (targetTotalGainQ10 > maxTableGainQ10) {
        finalAnaGainQ10 = maxTableGainQ10;
        printf("[SC2210] Warning: AE Gain (%d) exceeds Max Table Gain (%d). Clamping AnaGain.\n", 
               targetTotalGainQ10, maxTableGainQ10);
    } else {
        finalAnaGainQ10 = targetTotalGainQ10;
    }

    // 数字增益固定为 1.0x (1024)，除非未来有特殊需求再开启
    finalDigGainQ10 = 1024U;

    // ------------------------------------------------------------------------
    // 5. 查表 - 模拟增益
    // ------------------------------------------------------------------------
    // 寻找第一个 >= 目标值的条目
    for (cnt = 0; cnt < SC2210_ANA_GAIN_TBL_SIZE; cnt++) {
        if (finalAnaGainQ10 <= gSC2210AnaGainTable[cnt].gain_q10) {
            anaGainRegH = gSC2210AnaGainTable[cnt].reg_h;
            anaGainRegL = gSC2210AnaGainTable[cnt].reg_l;
            break;
        }
    }
    // 如果超出范围（理论上已被上面的钳位逻辑阻止，但保留以防万一）
    if (cnt == SC2210_ANA_GAIN_TBL_SIZE) {
        anaGainRegH = gSC2210AnaGainTable[SC2210_ANA_GAIN_TBL_SIZE - 1].reg_h;
        anaGainRegL = gSC2210AnaGainTable[SC2210_ANA_GAIN_TBL_SIZE - 1].reg_l;
    }

    // ------------------------------------------------------------------------
    // 6. 查表 - 数字增益 (固定 1.0x)
    // ------------------------------------------------------------------------
    // 即使固定为 1024，也走一遍查表以确保寄存器值正确对应 1.0x
    for (cnt = 0; cnt < SC2210_DIG_GAIN_TBL_SIZE; cnt++) {
        if (finalDigGainQ10 <= gSC2210DigGainTable[cnt].gain_q10) {
            digGainRegH = gSC2210DigGainTable[cnt].reg_h;
            digGainRegL = gSC2210DigGainTable[cnt].reg_l;
            break;
        }
    }
    if (cnt == SC2210_DIG_GAIN_TBL_SIZE) {
        digGainRegH = gSC2210DigGainTable[SC2210_DIG_GAIN_TBL_SIZE - 1].reg_h;
        digGainRegL = gSC2210DigGainTable[SC2210_DIG_GAIN_TBL_SIZE - 1].reg_l;
    }

    // ------------------------------------------------------------------------
    // 7. 调试打印
    // ------------------------------------------------------------------------
    printf("[SC2210] AE Update: Time=%dns -> Lines=%d, ReqGain=%d, AnaGain=%d(0x%x), DigGain=%d(0x%x)\n",
            exposureTimeNs, exposureLines, targetTotalGainQ10, 
            finalAnaGainQ10, ((uint16_t)anaGainRegH << 8) | anaGainRegL,
            finalDigGainQ10, ((uint16_t)digGainRegH << 8) | digGainRegL);

    // ------------------------------------------------------------------------
    // 8. 写入寄存器 (Group Hold)
    // ------------------------------------------------------------------------
    // 8.1 Start Group 0 Pack
    status = SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3800, 0x00, 1u);
    
    // 8.2 Write Exposure
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3e00, (exposureLines >> 12) & 0x0F, 1u);
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3e01, (exposureLines >> 4) & 0xFF, 1u);
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3e02, (exposureLines & 0x0F) << 4, 1u);
    
    // 8.3 Write Analog Gain
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3e08, anaGainRegH, 1u);
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3e09, anaGainRegL, 1u);
    
    // 8.4 Write Digital Gain
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3e06, digGainRegH, 1u);
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3e07, digGainRegL, 1u);
    
    // 8.5 End Group 0 Pack
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3800, 0x10, 1u);
    
    // 8.6 Launch Group 0 Immediately
    status |= SC2210_WriteReg(i2cInstId, sensorI2cAddr, 0x3800, 0x60, 1u);

    if (status != 0) {
        printf("[SC2210] Error writing registers\n");
    }

    return 0;
}

static int32_t SC2210_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms)
{
    int32_t status = 0;
    uint8_t count = 0;
    p_ae_dynPrms->targetBrightnessRange.min = DEFAULT_TARGET_BRIGHTNESS_MIN;
    p_ae_dynPrms->targetBrightnessRange.max = DEFAULT_TARGET_BRIGHTNESS_MAX;
    p_ae_dynPrms->targetBrightness = DEFAULT_TARGET_BRIGHTNESS;
    p_ae_dynPrms->threshold = DEFAULT_TARGET_THRESHOLD;
    p_ae_dynPrms->exposureTimeStepSize = DEFAULT_EXPOSURE_TIME;
    p_ae_dynPrms->enableBlc = DEFAULT_ENABLE_BLC;

    /*Limit max exposure time to 8.3ms to minimize motion artifacts*/
    p_ae_dynPrms->exposureTimeRange[count].min = 100;
    p_ae_dynPrms->exposureTimeRange[count].max = 100000; // 33333
    p_ae_dynPrms->analogGainRange[count].min = 1024;
    p_ae_dynPrms->analogGainRange[count].max = 55296;//1024*54 
    p_ae_dynPrms->digitalGainRange[count].min = 256;
    p_ae_dynPrms->digitalGainRange[count].max = 25600;
    count++;

    p_ae_dynPrms->numAeDynParams = count;
    
    printf("-------------------------------------------> SC2210_GetExpPrgFxn \n");
    return status;
}

static int32_t SC2210_GetWBPrgFxn(uint32_t chId, void *pSensorHdl, IssAwbDynamicParams *p_awb_dynPrms)
{
    int32_t status = 0;

    p_awb_dynPrms->redGainRange.min = 512;
    p_awb_dynPrms->redGainRange.max = 2048;

    p_awb_dynPrms->greenGainRange.min = 512;
    p_awb_dynPrms->greenGainRange.max = 2048;

    p_awb_dynPrms->blueGainRange.min = 512;
    p_awb_dynPrms->blueGainRange.max = 2048;
  
    p_awb_dynPrms->sensor_pre_gain = 0;      // 不使用传感器预增益

    printf("-------------------------------------------> SC2210_GetWBPrgFxn \n");
    return status;
}

static int32_t SC2210_SetAwbParams(void *pSensorHdl, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = 0;




    printf("-------------------------------------------> SC2210_SetAwbParams \n");
    return (status);
}

static int32_t SC2210_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    printf("-------------------------------------------> SC2210_GetDccParams \n");
    return (status);
}

static int32_t SC2210_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;
    printf("-------------------------------------------> SC2210_GetExpParams \n");
    return (status);
}

static void SC2210_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    printf("-------------------------------------------> SC2210_InitAewbConfig \n");
    return;
}

static void SC2210_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    printf("-------------------------------------------> SC2210_GetIspConfig \n");
    return;
}
