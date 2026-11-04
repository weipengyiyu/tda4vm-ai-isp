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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <TI/tivx.h>
#include <TI/tivx_fileio.h>
#include <TI/tivx_target_kernel.h>
#include "tivx_kernels_target_utils.h"
#include <tivx_fileio_write_raw_image_hdr_host.h>

#include <TI/tivx_ext_raw_image.h>

#define WRITE_FILE_OUTPUT

/*! \brief 最大支持通道数 */
#define MAX_CHANNELS 3

/*! \brief HDR 数据内存缓冲区
 *  使用静态内存代替 vx_raw_image，因为 target kernel 无法创建 OpenVX 对象
 *  结构: hdr_buf[ch][y][x] = 收集的 HDR 数据
 *  每个通道的数据依次排列
 */
#define HDR_BUF_MAGIC 0x48445248  /* "HDRH" magic number */
typedef struct {
    uint32_t magic;
    uint32_t width;
    uint32_t height;
    uint32_t BPP;
    uint32_t stride;
    uint32_t ch_count;  /* 已收集的通道数 */
    uint8_t *buf_ptr;  /* 数据缓冲区 */
} hdr_mem_buf_t;

static hdr_mem_buf_t hdr_mem = {0};
/*! \brief 标记 hdr_mem 是否已初始化 */
static vx_bool hdr_mem_initialized = vx_false_e;
/*! \brief 已处理的帧计数（用于判断是否所有通道都已处理） */
static vx_uint32 hdr_frame_counter = 0;

typedef struct {
    tivxFileIOWriteCmd cmd;
    vx_uint32 frame_counter;
    vx_uint32 skip_counter;
    vx_uint32 ch_num;

} tivxWriteRawImageParams;

static tivx_target_kernel vx_write_raw_image_kernel = NULL;

static vx_int32 inst_id = 0;

/* 前向声明 */
static void deinitHdrMem(void);
static vx_status initHdrMem(uint32_t width, uint32_t height, uint32_t BPP, uint32_t stride);
static vx_status copyExposureToHdrMem(tivx_obj_desc_raw_image_t *in_raw_img_desc, 
                                       vx_uint32 exp_id, 
                                       vx_uint32 ch_num);

static vx_status tivxKernelWriteRawImageHdrCmd
(
    tivxWriteRawImageParams *prms,
    const tivx_obj_desc_user_data_object_t *usr_data_obj
)
{
    vx_status status = (vx_status)VX_SUCCESS;

    if (NULL != usr_data_obj)
    {
        void  *target_ptr = tivxMemShared2TargetPtr(&usr_data_obj->mem_ptr);

        tivxMemBufferMap(target_ptr, usr_data_obj->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY);

        if (sizeof(tivxFileIOWriteCmd) ==  usr_data_obj->mem_size)
        {
            memcpy(&prms->cmd, target_ptr, sizeof(tivxFileIOWriteCmd));
        }
        prms->skip_counter = 0;
        tivxMemBufferUnmap(target_ptr, usr_data_obj->mem_size,
            (vx_enum)VX_MEMORY_TYPE_HOST, (vx_enum)VX_READ_ONLY);
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR,
            "tivxKernelWriteRawImageCmd: Data Object is NULL\n");
        status = (vx_status)VX_ERROR_INVALID_PARAMETERS;
    }

    return (status);
}

static vx_status VX_CALLBACK tivxKernelWriteRawImageHdrControl
(
    tivx_target_kernel_instance kernel,
    uint32_t node_cmd_id,
    tivx_obj_desc_t *obj_desc[],
    uint16_t num_params,
    void *priv_arg
)
{
    vx_status status = VX_SUCCESS;

    tivxWriteRawImageParams *prms = NULL;

    if (NULL == obj_desc[TIVX_KERNEL_WRITE_RAW_IMAGE_HDR_INPUT_IDX])
    {
        printf("Input Image handle is NULL!\n");
        status = VX_FAILURE;
    }

    if(status==VX_SUCCESS)
    {
        uint32_t size;

        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&prms, &size);
        if ((VX_SUCCESS != status) || (NULL == prms) ||
            (sizeof(tivxWriteRawImageParams) != size))
        {
            status = VX_FAILURE;
        }
    }

    if(status==VX_SUCCESS)
    {
        switch (node_cmd_id)
        {
            case TIVX_FILEIO_CMD_SET_FILE_WRITE:
            {
                tivxKernelWriteRawImageHdrCmd(prms,
                    (tivx_obj_desc_user_data_object_t *)obj_desc[0U]);
                break;
            }
            default:
            {
                VX_PRINT(VX_ZONE_ERROR,
                    "tivxKernelWriteRawImageHdrControl: Invalid Command Id\n");
                status = (vx_status)VX_FAILURE;
                break;
            }
        }
    }

    return (status);
}

static vx_status VX_CALLBACK tivxKernelWriteRawImageHdrCreate
(
  tivx_target_kernel_instance kernel,
  tivx_obj_desc_t *obj_desc[],
  vx_uint16 num_params,
  void *priv_arg
)
{
  tivxWriteRawImageParams * prms = NULL;

  prms = tivxMemAlloc(sizeof(tivxWriteRawImageParams), TIVX_MEM_EXTERNAL);
  if(prms == NULL)
  {
      printf("Unable to allcate memory for tivxWriteRawImageParams\n");
      return VX_FAILURE;
  }
  prms->ch_num = inst_id;
  prms->frame_counter   = 0;
  prms->skip_counter    = 0;
  prms->cmd.start_frame = -1;
  prms->cmd.num_frames  = 0;
  prms->cmd.num_skip    = 0;

  inst_id++;
  tivxSetTargetKernelInstanceContext(kernel, prms,  sizeof(tivxWriteRawImageParams));

  printf("Input Image handle HDR        %d created................ch_num = %d\n", inst_id, prms->ch_num);
  return(VX_SUCCESS);
}

static vx_status VX_CALLBACK tivxKernelWriteRawImageHdrDelete(
    tivx_target_kernel_instance kernel, tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params, void *priv_arg)
{
    vx_status status = VX_SUCCESS;

    if (NULL == obj_desc[TIVX_KERNEL_WRITE_RAW_IMAGE_HDR_INPUT_IDX])
    {
        printf("Input Image handle is NULL!\n");
        status = VX_FAILURE;
    }

    if (VX_SUCCESS == status)
    {
        uint32_t size;
        tivxWriteRawImageParams *prms = NULL;

        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&prms, &size);

        tivxMemFree(prms, sizeof(tivxWriteRawImageParams), TIVX_MEM_EXTERNAL);
    }

    /* 释放 HDR 内存资源（只在最后一个实例删除时释放） */
    if (inst_id > 0)
    {
        inst_id--;
    }
    if (inst_id == 0)
    {
        deinitHdrMem();
        printf("[HDR] HDR memory released\n");
    }

    return (status);
}

/*!
 * \brief 初始化 HDR 内存缓冲区（只执行一次）
 *
 * 使用静态内存缓冲区代替 vx_raw_image
 * 因为 target kernel 无法获取有效的 vx_context 来创建 OpenVX 对象
 *
 * @param width   图像宽度
 * @param height  图像高度
 * @param BPP     每像素字节数
 * @param stride  行跨度
 * @return vx_status 操作状态
 */
static vx_status initHdrMem(uint32_t width, uint32_t height, uint32_t BPP, uint32_t stride)
{
    vx_status status = VX_SUCCESS;
    uint32_t buf_size;

    if (hdr_mem_initialized == vx_true_e)
    {
        /* 已经初始化，无需再次分配 */
        return VX_SUCCESS;
    }

    /* 计算缓冲区大小: width * BPP * height * MAX_CHANNELS */
    buf_size = width * BPP * height * MAX_CHANNELS;

    /* 使用 tivxMemAlloc 分配内存（与 TIOVX 内存管理一致） */
    hdr_mem.buf_ptr = (uint8_t *)tivxMemAlloc(buf_size, TIVX_MEM_EXTERNAL);
    if (hdr_mem.buf_ptr == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to allocate HDR memory!\n");
        return VX_FAILURE;
    }

    /* 初始化缓冲区为 0 */
    memset(hdr_mem.buf_ptr, 0, buf_size);

    /* 保存参数 */
    hdr_mem.magic   = HDR_BUF_MAGIC;
    hdr_mem.width    = width;
    hdr_mem.height   = height;
    hdr_mem.BPP      = BPP;
    hdr_mem.stride   = stride;
    hdr_mem.ch_count = 0;

    hdr_mem_initialized = vx_true_e;
    printf("[HDR] HDR memory allocated: %dx%d, BPP=%d, stride=%d, size=%d bytes\n", 
           width, height, BPP, stride, buf_size);

    return status;
}

/*!
 * \brief 释放 HDR 内存缓冲区
 */
static void deinitHdrMem(void)
{
    if (hdr_mem.buf_ptr != NULL)
    {
        tivxMemFree(hdr_mem.buf_ptr, 
                    hdr_mem.width * hdr_mem.BPP * hdr_mem.height * MAX_CHANNELS,
                    TIVX_MEM_EXTERNAL);
        hdr_mem.buf_ptr = NULL;
    }
    hdr_mem.magic   = 0;
    hdr_mem.width   = 0;
    hdr_mem.height  = 0;
    hdr_mem.BPP     = 0;
    hdr_mem.stride  = 0;
    hdr_mem.ch_count = 0;
    hdr_mem_initialized = vx_false_e;
}

/*!
 * \brief 将输入 RAW 图像的指定曝光数据复制到 HDR 内存缓冲区
 *
 * 数据转置原理:
 *   输入格式: [exp0_ch0, exp1_ch0, exp2_ch0, ...] (Frame-Channel 排列)
 *             每个节点处理一个通道的所有曝光帧
 *   输出格式: [exp0_ch0, exp0_ch1, exp0_ch2, ...] (按 exp0 排列所有通道)
 *             ch_num 决定数据存放到 HDR 缓冲区的哪个位置
 *
 * @param in_raw_img_desc  输入 RAW 图像描述符
 * @param exp_id          输入图像的曝光索引（通常为 0，只处理第一帧）
 * @param ch_num          通道号（0, 1, 2），决定数据在 HDR 缓冲区中的存放位置
 * @return vx_status      操作状态
 */
static vx_status copyExposureToHdrMem(tivx_obj_desc_raw_image_t *in_raw_img_desc, 
                                       vx_uint32 exp_id, 
                                       vx_uint32 ch_num)
{
    vx_status status = VX_SUCCESS;
    void *in_ptr = NULL;
    uint32_t row;
    vx_uint32 width, height, BPP, stride;

    if (hdr_mem.buf_ptr == NULL || hdr_mem.magic != HDR_BUF_MAGIC)
    {
        VX_PRINT(VX_ZONE_ERROR, "hdr_mem is not initialized!\n");
        return VX_FAILURE;
    }

    /* 获取输入图像的维度信息 */
    width  = in_raw_img_desc->imagepatch_addr[exp_id].dim_x;
    height = in_raw_img_desc->imagepatch_addr[exp_id].dim_y;
    BPP    = in_raw_img_desc->imagepatch_addr[exp_id].stride_x;
    stride = in_raw_img_desc->imagepatch_addr[exp_id].stride_y;

    /* 使用 tivxMemShared2TargetPtr 获取输入图像指针（与原始 fileio kernel 一致） */
    in_ptr = tivxMemShared2TargetPtr(&in_raw_img_desc->mem_ptr[exp_id]);
    status = tivxMemBufferMap(in_ptr, in_raw_img_desc->mem_size[exp_id], 
                               VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
    if (status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to map input raw image!\n");
        return status;
    }

    /* 复制数据到 HDR 内存缓冲区
     * HDR 缓冲区布局: [ch0_data][ch1_data][ch2_data]
     * 每个通道数据连续排列，大小为 width * BPP * height
     */
    {
        uint8_t *pInRow  = (uint8_t *)in_ptr;
        uint8_t *pOutRow;
        uint32_t line_size = width * BPP;  /* 每行有效数据大小 */
        
        /* 计算当前通道在 HDR 缓冲区中的起始位置 */
        /* ch0: offset = 0
         * ch1: offset = width * BPP * height
         * ch2: offset = 2 * width * BPP * height
         */
        pOutRow = hdr_mem.buf_ptr + (ch_num * width * BPP * height);

        for (row = 0; row < height; row++)
        {
            memcpy(pOutRow, pInRow, line_size);
            pInRow  += stride;
            pOutRow += width * BPP;
        }

        /* 更新已收集的通道计数 */
        if (ch_num >= hdr_mem.ch_count)
        {
            hdr_mem.ch_count = ch_num + 1;
        }
    }

    /* 取消输入映射 */
    tivxMemBufferUnmap(in_ptr, in_raw_img_desc->mem_size[exp_id], 
                       VX_MEMORY_TYPE_HOST, VX_READ_ONLY);

    return status;
}

/*!
 * \brief 将收集完成的 HDR 数据填入输出 RAW 图像描述符
 *
 * @param out_hdr_raw_img_desc  输出 RAW 图像描述符
 * @return vx_status            操作状态
 */
static vx_status fillOutputHdrImage(tivx_obj_desc_raw_image_t *out_hdr_raw_img_desc)
{
    vx_status status = VX_SUCCESS;
    uint8_t *out_ptr = NULL;
    uint32_t ch, row;
    vx_uint32 width, height, BPP;
    vx_uint32 exp_id = 0;  /* 输出使用 exp_id = 0 */

    if (out_hdr_raw_img_desc == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "out_hdr_raw_img_desc is NULL!\n");
        return VX_FAILURE;
    }

    if (hdr_mem.buf_ptr == NULL || hdr_mem.magic != HDR_BUF_MAGIC)
    {
        VX_PRINT(VX_ZONE_ERROR, "hdr_mem is not initialized!\n");
        return VX_FAILURE;
    }

    /* 获取输出图像的维度信息 */
    width  = out_hdr_raw_img_desc->imagepatch_addr[exp_id].dim_x;
    height = out_hdr_raw_img_desc->imagepatch_addr[exp_id].dim_y;
    BPP    = out_hdr_raw_img_desc->imagepatch_addr[exp_id].stride_x;

    // printf("[HDR] fillOutputHdrImage: width=%d, height=%d, BPP=%d, ch_count=%d\n",
    //        width, height, BPP, hdr_mem.ch_count);

    /* 获取输出图像的内存指针 */
    out_ptr = (uint8_t *)tivxMemShared2TargetPtr(&out_hdr_raw_img_desc->mem_ptr[exp_id]);
    
    // printf("[HDR] === fillOutputHdrImage DEBUG ===\n");
    // printf("[HDR] out_ptr address: %p\n", out_ptr);
    // printf("[HDR] out_hdr_raw_img_desc->mem_size[0]: %d bytes\n", out_hdr_raw_img_desc->mem_size[exp_id]);
    
    /* 验证 out_ptr 是否有效 */
    if (out_ptr == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "ERROR: out_ptr is NULL after tivxMemShared2TargetPtr!\n");
        return VX_FAILURE;
    }
    
    // /* 打印 HDR 缓冲区中的数据样本，用于验证 */
    // {
    //     uint32_t sample_size = 16;
    //     uint8_t *pCh0Sample = hdr_mem.buf_ptr;
    //     uint8_t *pCh1Sample = hdr_mem.buf_ptr + (width * BPP * height);
    //     uint8_t *pCh2Sample = hdr_mem.buf_ptr + (2 * width * BPP * height);
        
    //     printf("[HDR] HDR buf first 16 bytes:\n");
    //     printf("[HDR]   ch0: ");
    //     for (uint32_t i = 0; i < sample_size && i < width * BPP; i++) {
    //         printf("%02X ", pCh0Sample[i]);
    //     }
    //     printf("\n");
    //     printf("[HDR]   ch1: ");
    //     for (uint32_t i = 0; i < sample_size && i < width * BPP; i++) {
    //         printf("%02X ", pCh1Sample[i]);
    //     }
    //     printf("\n");
    //     printf("[HDR]   ch2: ");
    //     for (uint32_t i = 0; i < sample_size && i < width * BPP; i++) {
    //         printf("%02X ", pCh2Sample[i]);
    //     }
    //     printf("\n");
    // }
    
    status = tivxMemBufferMap(out_ptr, out_hdr_raw_img_desc->mem_size[exp_id],
                                VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY);
    if (status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to map output raw image!\n");
        return status;
    }

    /* 将 HDR 缓冲区数据填入输出
     * HDR 布局: [ch0_data][ch1_data][ch2_data] (每个通道 width*BPP*height)
     * 输出布局: 每行包含所有通道的数据
     *   Row 0: [ch0_pixel0...ch0_pixelN][ch1_pixel0...ch1_pixelN][ch2_pixel0...ch2_pixelN]
     *   ...
     * 输出 height = 原始 height (只使用 exp0)
     */
    {
        uint32_t ch_data_size = width * BPP * height;  /* 单个通道数据大小 */
        uint32_t out_stride = out_hdr_raw_img_desc->imagepatch_addr[exp_id].stride_y;
        uint32_t line_size = width * BPP;  /* 每通道每行数据大小 */
        
        for (row = 0; row < height; row++)
        {
            uint8_t *pOutRow = out_ptr + (row * out_stride);
            
            /* 依次复制每个通道的对应行 */
            for (ch = 0; ch < hdr_mem.ch_count; ch++)
            {
                uint8_t *pChData = hdr_mem.buf_ptr + (ch * ch_data_size) + (row * line_size);
                memcpy(pOutRow, pChData, line_size);
                pOutRow += line_size;
            }
        }
    }

    /* 取消输出映射 */
    tivxMemBufferUnmap(out_ptr, out_hdr_raw_img_desc->mem_size[exp_id],
                        VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY);

    /* 重新映射以验证输出数据 */
    out_ptr = (uint8_t *)tivxMemShared2TargetPtr(&out_hdr_raw_img_desc->mem_ptr[exp_id]);
    status = tivxMemBufferMap(out_ptr, out_hdr_raw_img_desc->mem_size[exp_id],
                                VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
    if (status == VX_SUCCESS)
    {
        // /* 打印输出数据的前 32 字节，验证数据是否正确 */
        // printf("[HDR] OUTPUT verification - first 32 bytes:\n");
        // printf("[HDR]   ");
        // for (uint32_t i = 0; i < 32; i++) {
        //     printf("%02X ", out_ptr[i]);
        //     if ((i + 1) % 16 == 0) printf("\n[HDR]   ");
        // }
        // printf("\n");
        
        // /* 验证输出宽度是否正确 (应该是 width * ch_count) */
        // uint32_t expected_width = width * hdr_mem.ch_count;
        // printf("[HDR] OUTPUT: width=%d, height=%d, ch_count=%d, expected_total_width=%d\n",
        //        width, height, hdr_mem.ch_count, expected_width);
        
        // /* 验证 stride */
        // uint32_t actual_stride = out_hdr_raw_img_desc->imagepatch_addr[exp_id].stride_y;
        // printf("[HDR] OUTPUT stride_y: %d, expected: %d\n", actual_stride, width * hdr_mem.ch_count * BPP);
        
        // /* 计算并验证数据总量 */
        // uint32_t total_bytes = actual_stride * height;
        // printf("[HDR] OUTPUT total bytes: %d, mem_size: %d\n", total_bytes, out_hdr_raw_img_desc->mem_size[exp_id]);
        
        // if (total_bytes == out_hdr_raw_img_desc->mem_size[exp_id])
        // {
        //     printf("[HDR] ✓ Output data size verification PASSED!\n");
        // }
        // else
        // {
        //     printf("[HDR] ✗ Output data size verification FAILED! (expected %d, got %d)\n",
        //            out_hdr_raw_img_desc->mem_size[exp_id], total_bytes);
        // }
        
        tivxMemBufferUnmap(out_ptr, out_hdr_raw_img_desc->mem_size[exp_id],
                            VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
    }

    // printf("[HDR] ✓ HDR image filled to output successfully!\n");
    // printf("[HDR] =========================================\n");

    return status;
}

static vx_status VX_CALLBACK tivxKernelWriteRawImageHdrProcess
(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg
)
{
    vx_status status = VX_SUCCESS;
    uint32_t frame_id = 0;

    tivxWriteRawImageParams *prms = NULL;

    /* 获取 kernel 上下文 */
    if (NULL == obj_desc[TIVX_KERNEL_WRITE_RAW_IMAGE_HDR_INPUT_IDX])
    {
        printf("Input Image handle is NULL!\n");
        status = VX_FAILURE;
    }

    if (status == VX_SUCCESS)
    {
        uint32_t size;

        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&prms, &size);
        if ((VX_SUCCESS != status) || (NULL == prms) ||
            (sizeof(tivxWriteRawImageParams) != size))
        {
            status = VX_FAILURE;
        }
    }
    tivx_obj_desc_raw_image_t *out_hdr_raw_img_desc = 
        (tivx_obj_desc_raw_image_t *)obj_desc[TIVX_KERNEL_WRITE_RAW_IMAGE_HDR_OUTPUT_IDX];
        
    if (status == VX_SUCCESS)
    {
        tivx_obj_desc_raw_image_t *in_raw_img_desc = 
            (tivx_obj_desc_raw_image_t *)obj_desc[TIVX_KERNEL_WRITE_RAW_IMAGE_HDR_INPUT_IDX];

        /* 延迟初始化 HDR 内存（在第一个节点调用时）
         * 使用第一个输入图像的参数作为模板
         */
        if (hdr_mem_initialized == vx_false_e)
        {
            uint32_t width  = in_raw_img_desc->imagepatch_addr[0].dim_x;
            uint32_t height = in_raw_img_desc->imagepatch_addr[0].dim_y;
            uint32_t BPP    = in_raw_img_desc->imagepatch_addr[0].stride_x;
            uint32_t stride = in_raw_img_desc->imagepatch_addr[0].stride_y;
            
            status = initHdrMem(width, height, BPP, stride);
            if (status != VX_SUCCESS)
            {
                return status;
            }
        }

        /* 
         * 核心逻辑：每次只处理 num_exposures = 0 (第一帧曝光)
         * 根据 prms->ch_num 将数据存入 hdr_mem 的对应位置
         *
         * 数据流示意：
         *   节点 0 (ch_num=0): 读取 exp0_ch0 -> 存入 hdr_mem[ch0]
         *   节点 1 (ch_num=1): 读取 exp0_ch1 -> 存入 hdr_mem[ch1]
         *   节点 2 (ch_num=2): 读取 exp0_ch2 -> 存入 hdr_mem[ch2]
         */
        vx_uint32 fixed_exp_id = 0;  /* 固定只处理第一个曝光帧 */
        
        /* 验证通道号和曝光索引的有效性 */
        if (prms->ch_num < MAX_CHANNELS && fixed_exp_id < in_raw_img_desc->params.num_exposures)
        {
            /* 调用复制函数：将当前通道的 exp0 数据复制到 HDR 内存 */
            status = copyExposureToHdrMem(in_raw_img_desc, fixed_exp_id, prms->ch_num);
            
            // if (status == VX_SUCCESS)
            // {
            //     printf("[HDR] Node ch_num=%d: copied exp%d to HDR mem (ch_count=%d)\n", 
            //            prms->ch_num, fixed_exp_id, hdr_mem.ch_count);
            // }
            
            /* 
             * 当 ch_num == 2 (最后一个通道) 时，所有通道数据已收集完成
             * 此时将 hdr_mem 中的数据填入输出图像描述符
             */
            if (prms->ch_num == (MAX_CHANNELS - 1) && hdr_mem.ch_count == MAX_CHANNELS)
            {
                // printf("[HDR] All channels collected, filling output HDR image...\n");
                status = fillOutputHdrImage(out_hdr_raw_img_desc);
                if (status != VX_SUCCESS)
                {
                    VX_PRINT(VX_ZONE_ERROR, "Failed to fill output HDR image!\n");
                }
                
                /* 重置收集状态，准备下一帧 */
                hdr_frame_counter++;
                hdr_mem.ch_count = 0;
            }
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Invalid ch_num=%d or exp_id=%d\n", 
                     prms->ch_num, fixed_exp_id);
            status = VX_FAILURE;
        }

        /* 更新帧计数 */
        prms->skip_counter++;
        prms->skip_counter = prms->skip_counter % (prms->cmd.num_skip + 1);
        prms->frame_counter++;
        
        /* 调试：显示当前帧信息 */
        frame_id = prms->frame_counter - 1;
        printf("[HDR] Process: frame=%d, ch=%d, exp_id=%d\n", 
               frame_id, prms->ch_num, fixed_exp_id);
    }

    return status;
}

void tivxAddTargetKernelWriteRawImageHdr()
{
    char target_name[TIVX_TARGET_MAX_NAME];
    vx_enum self_cpu;

    self_cpu = tivxGetSelfCpuId();

    if ((self_cpu == TIVX_CPU_ID_MPU_0))
    {
        strncpy(target_name, TIVX_TARGET_MPU_0, TIVX_TARGET_MAX_NAME);

        vx_write_raw_image_kernel = tivxAddTargetKernelByName
                                      (
                                        TIVX_KERNEL_WRITE_RAW_IMAGE_HDR_NAME,
                                        target_name,
                                        tivxKernelWriteRawImageHdrProcess,
                                        tivxKernelWriteRawImageHdrCreate,
                                        tivxKernelWriteRawImageHdrDelete,
                                        tivxKernelWriteRawImageHdrControl,
                                        NULL
                                      );
    }
}

void tivxRemoveTargetKernelWriteRawImageHdr()
{
    vx_status status = VX_SUCCESS;

    status = tivxRemoveTargetKernel(vx_write_raw_image_kernel);
    if (status == VX_SUCCESS)
    {
        vx_write_raw_image_kernel = NULL;
    }
}
