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
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <TI/tivx.h>
#include <TI/tivx_img_proc.h>
#include <TI/tivx_target_kernel.h>
#include "tivx_kernels_target_utils.h"

#include <tivx_oc_pre_proc_host.h>

#include <utils/mem/include/app_mem.h>

/* 调试打印开关 - 生产环境可关闭 */
#define OC_PRE_PROC_DEBUG_PRINT     (1)
#define OC_PRE_PROC_PRINT_MAX_BAYER (4)  /* 打印前4个Bayer单元用于验证 */

static tivx_target_kernel vx_oc_pre_proc_target_kernel = NULL;

static vx_status VX_CALLBACK tivxKernelOCPreProcCreate
(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg
)
{
    tivxOCPreProcParams * prms = NULL;

    prms = tivxMemAlloc(sizeof(tivxOCPreProcParams), TIVX_MEM_EXTERNAL);
    if (NULL == prms)
    {
        return VX_FAILURE;
    }

    tivxSetTargetKernelInstanceContext(kernel, prms, sizeof(tivxOCPreProcParams));

    return VX_SUCCESS;
}

static vx_status VX_CALLBACK tivxKernelOCPreProcDelete(
    tivx_target_kernel_instance kernel, 
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params, 
    void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    uint32_t i;

    for (i = 0U; i < num_params; i++)
    {
        if (NULL == obj_desc[i])
        {
            status = VX_FAILURE;
            break;
        }
    }

    if (VX_SUCCESS == status)
    {
        uint32_t size;
        tivxOCPreProcParams *prms = NULL;

        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&prms, &size);

        if ((VX_SUCCESS == status) && (NULL != prms))
        {
            tivxMemFree(prms, sizeof(tivxOCPreProcParams), TIVX_MEM_EXTERNAL);
        }
    }

    return status;
}


static vx_status VX_CALLBACK tivxKernelOCPreProcProcess(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg)
{
    vx_status status = VX_SUCCESS;
    vx_int32 i;
    tivxOCPreProcParams *prms = NULL;

    /* 参数验证 */
    for (i = 0U; i < num_params; i++)
    {
        if (NULL == obj_desc[i])
        {
            VX_PRINT(VX_ZONE_ERROR, "Invalid obj_desc[%d] = NULL\n", i);
            status = VX_FAILURE;
            break;
        }
    }

    if (status == VX_SUCCESS)
    {
        uint32_t size;
        status = tivxGetTargetKernelInstanceContext(kernel, (void **)&prms, &size);
        if ((VX_SUCCESS != status) || (NULL == prms) || (sizeof(tivxOCPreProcParams) != size))
        {
            status = VX_FAILURE;
        }
    }

    if (VX_SUCCESS == status)
    {
        /* 获取对象描述 */
        tivx_obj_desc_user_data_object_t* config_desc = 
            (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_OC_PRE_PROC_CONFIG_IDX];
        void *config_target_ptr = tivxMemShared2TargetPtr(&config_desc->mem_ptr);
        
        tivx_obj_desc_raw_image_t *in_img_desc = 
            (tivx_obj_desc_raw_image_t *)obj_desc[TIVX_KERNEL_OC_PRE_PROC_INPUT_IMAGE_IDX];
        void* in_img_target_ptr = tivxMemShared2TargetPtr(&in_img_desc->mem_ptr[0]);
        
        tivx_obj_desc_tensor_t *out_tensor_desc = 
            (tivx_obj_desc_tensor_t *)obj_desc[TIVX_KERNEL_OC_PRE_PROC_OUTPUT_TENSOR_IDX];
        void *out_tensor_target_ptr = tivxMemShared2TargetPtr(&out_tensor_desc->mem_ptr);

        /* 内存映射 */
        if (VX_SUCCESS != tivxMemBufferMap(config_target_ptr, config_desc->mem_size, 
                                            VX_MEMORY_TYPE_HOST, VX_READ_ONLY))
        {
            return VX_FAILURE;
        }
        if (VX_SUCCESS != tivxMemBufferMap(in_img_target_ptr, in_img_desc->mem_size[0], 
                                            VX_MEMORY_TYPE_HOST, VX_READ_ONLY))
        {
            tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, 
                              VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
            VX_PRINT(VX_ZONE_ERROR, "in_img_target_ptr is error ");
            return VX_FAILURE;
        }
        if (VX_SUCCESS != tivxMemBufferMap(out_tensor_target_ptr, out_tensor_desc->mem_size, 
                                            VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY))
        {
            tivxMemBufferUnmap(in_img_target_ptr, in_img_desc->mem_size[0], 
                              VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
            tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, 
                              VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
            VX_PRINT(VX_ZONE_ERROR, "out_tensor_target_ptr is error ");
            return VX_FAILURE;
        }

        uint64_t cur_time = tivxPlatformGetTimeInUsecs();

        vx_uint32 pixel_container = in_img_desc->params.format[0U].pixel_container;
        vx_uint32 msb = in_img_desc->params.format[0U].msb;
        vx_uint32 in_width   = in_img_desc->imagepatch_addr[0].dim_x;
        vx_uint32 in_height  = in_img_desc->imagepatch_addr[0].dim_y;
        vx_uint32 in_stride  = in_img_desc->imagepatch_addr[0].stride_y;
        
        VX_PRINT(VX_ZONE_INFO, 
                 "=== OC Pre-Proc Debug ===\n"
                 "pixel_container=%d (0x%X), msb=%d\n"
                 "Input: %dx%d, stride=%d bytes\n"
                 "TIVX_RAW_IMAGE_16_BIT=%d\n",
                 pixel_container, pixel_container, msb,
                 in_width, in_height, in_stride,
                 TIVX_RAW_IMAGE_16_BIT);

        /* 检查format是否匹配 */
        if (pixel_container != TIVX_RAW_IMAGE_16_BIT)
        {
            VX_PRINT(VX_ZONE_ERROR, "WARNING: pixel_container mismatch!\n");
        }

        if (TIVX_RAW_IMAGE_16_BIT == pixel_container)
        {
            if ((msb <= 12U) && (msb >= 10U))
            {
                if (out_tensor_desc->dimensions[2] == 4) 
                {
                    uint32_t element_size = sizeof(vx_float32);
                    
                    uint16_t *in_base  = (uint16_t *)in_img_target_ptr;
                    float *out_base = (float *)out_tensor_target_ptr;
                    VX_PRINT(VX_ZONE_INFO, 
                        "out_tensor  dims=[%d,%d,%d,%d]\n"
                        "out_tensor  strides字节=[%d,%d,%d,%d]\n"
                        "out_tensor  element_size=%d\n"
                        "out_tensor  mem_size=%d\n",
                        out_tensor_desc->dimensions[0],
                        out_tensor_desc->dimensions[1],
                        out_tensor_desc->dimensions[2],
                        out_tensor_desc->dimensions[3],
                        out_tensor_desc->stride[0],
                        out_tensor_desc->stride[1],
                        out_tensor_desc->stride[2],
                        out_tensor_desc->stride[3],
                        element_size,
                        out_tensor_desc->mem_size);
                    /* 
                    * 关键修复：TIDL CHW布局, 但是dim[0]-W dim[1]-H dim[2]-C
                    * dimensions[0] = Width,  stride[0] = 1 (元素大小)
                    * dimensions[1] = Height, stride[1] = Width (Width*元素大小)
                    * dimensions[2] = Channels, stride[2] = Width*Height (Width*Height*元素大小)
                    * 
                    * 内存索引公式: index = x*stride[0] + y*stride[1] + c*stride[2]
                    */
                    uint32_t tensor_stride_w = out_tensor_desc->stride[0] / element_size; /* Width方向stride 1 */
                    uint32_t tensor_stride_h = out_tensor_desc->stride[1] / element_size; /* Height方向stride 960*/
                    uint32_t tensor_stride_c = out_tensor_desc->stride[2] / element_size; /* Channel方向stride 518400*/

                    
                    float *out_ch0, *out_ch1, *out_ch2, *out_ch3;
                    out_ch0 = out_base;
                    out_ch1 = out_base + tensor_stride_c;
                    out_ch2 = out_base + 2 * tensor_stride_c;
                    out_ch3 = out_base + 3 * tensor_stride_c;
                    uint32_t y, x;
                    uint32_t process_height = (in_height / 2) * 2;//1080
                    uint32_t process_width = (in_width / 2) * 2;//1920
                    const float scale_4c = 1.0/4095.0;
                    // VX_PRINT(VX_ZONE_ERROR, "out_ch0=%p, out_ch1=%p, out_ch2=%p, out_ch3=%p, mem_size=%d, C=%d,H=%d,W=%d, stride[2]=%d,\n process_height=%d,process_width=%d,in_stride=%d,dimensions[0]=%d, dimensions[1]=%d\n",\
                    //      out_ch0,out_ch1,out_ch2,out_ch3,out_tensor_desc->mem_size,tensor_stride_c,tensor_stride_h,tensor_stride_w,out_tensor_desc->stride[2],process_height,process_width,in_stride,out_tensor_desc->dimensions[0],out_tensor_desc->dimensions[1]);
                    for (y = 0; y < process_height; y += 2) //步长为 2
                    {
                        uint32_t out_y = y / 2;
                        uint32_t stride_elements = in_stride / sizeof(uint16_t);
                        uint16_t *pInRowEven = (uint16_t *)(in_base + y * stride_elements);
                        uint16_t *pInRowOdd  = (uint16_t *)(in_base + (y + 1) * stride_elements);
                        
                        for (x = 0; x < process_width; x += 2)//步长为 2
                        {
                            uint32_t out_x = x / 2;
                            uint16_t pixel_00  = pInRowEven[x]      & 0x0FFF;//步长为 2
                            uint16_t pixel_01 = pInRowEven[x + 1]   & 0x0FFF;
                            uint16_t pixel_10 = pInRowOdd[x]        & 0x0FFF;
                            uint16_t pixel_11  = pInRowOdd[x + 1]   & 0x0FFF;
                            
                            /* 分离通道 - 每个通道是独立的WH平面 */
                            uint32_t pixel_idx = out_y * tensor_stride_h + out_x * tensor_stride_w;//out_y * 960 + out_x;
                            out_ch0[pixel_idx] =    (float)pixel_00*scale_4c;
                            out_ch1[pixel_idx] =    (float)pixel_01*scale_4c;
                            out_ch2[pixel_idx] =    (float)pixel_10*scale_4c;
                            out_ch3[pixel_idx] =    (float)pixel_11*scale_4c;
                            // if (pixel_idx >= 518399)
                            //     VX_PRINT(VX_ZONE_ERROR,"out_ch0[%d]=%2.f,out_ch1[%d]=%2.f,out_ch2[%d]=%2.f,out_ch3[%d]=%2.f, pInRowEven=%p, pInRowOdd=%p \n", \
                            //         pixel_idx, out_ch0[pixel_idx], pixel_idx, out_ch1[pixel_idx], pixel_idx, out_ch2[pixel_idx], pixel_idx, out_ch3[pixel_idx], pInRowEven, pInRowOdd);
                            // // VX_PRINT(VX_ZONE_ERROR,"out_ch0[%d]=%2.f,out_ch1[%d]=%2.f,out_ch2[%d]=%2.f,out_ch3[%d]=%2.f \n", pixel_idx, out_ch0[pixel_idx], pixel_idx, out_ch1[pixel_idx], pixel_idx, out_ch2[pixel_idx], pixel_idx, out_ch3[pixel_idx]);
                            // // if ((x%500 == 0) && (y%300 == 0))
                            //     // VX_PRINT(VX_ZONE_ERROR, "x=%d,y=%d,input raw12:%d,%d,%d,%d;tensor output:%.2f,%.2f,%.2f,%.2f\n", x,y,pixel_00,pixel_01,pixel_10,pixel_11,*pOut00,*pOut01,*pOut10,*pOut11);

                        }
                    }
                }
                else if (out_tensor_desc->dimensions[2] == 1)
                {
                    uint32_t element_size = sizeof(vx_float32);
                    
                    uint16_t *in_base  = (uint16_t *)in_img_target_ptr;
                    float *out_base = (float *)out_tensor_target_ptr;

                    /* 
                    * 1C 直接拷贝逻辑:
                    * 遍历输入图像的每一个像素，直接转换为 float 写入输出 Tensor。
                    * 不再进行 2x2 分块、通道分离或归一化。
                    */
                    uint32_t tensor_stride_w = out_tensor_desc->stride[0] / element_size; 
                    uint32_t tensor_stride_h = out_tensor_desc->stride[1] / element_size; 
                    
                    // 确定处理范围：防止输出 Tensor 小于输入图像时越界
                    uint32_t process_height = in_height;
                    uint32_t process_width = in_width;
                    
                    if (out_tensor_desc->dimensions[1] < in_height) process_height = out_tensor_desc->dimensions[1];
                    if (out_tensor_desc->dimensions[0] < in_width)  process_width  = out_tensor_desc->dimensions[0];

                    uint32_t y, x;
                    for (y = 0; y < process_height; y++)
                    {
                        // 计算输入行指针
                        uint32_t stride_elements = in_stride / sizeof(uint16_t);
                        uint16_t *pInRow = (uint16_t *)(in_base + y * stride_elements);
                        
                        // 计算输出行指针
                        float *pOutRow = out_base + (y * tensor_stride_h);

                        for (x = 0; x < process_width; x++)
                        {
                            // 1. 读取 16-bit 像素
                            // 2. 屏蔽高位 (保留低 12 位，根据 msb 判断，这里统一掩码 0x0FFF 以防万一)
                            // 3. 直接转换为 float (无归一化)
                            uint16_t pixel_val = pInRow[x] & 0x0FFF;
                            
                            // 直接写入对应位置
                            pOutRow[x * tensor_stride_w] = (float)pixel_val;
                        }
                    }
                }

            }
            else
            {
                VX_PRINT(VX_ZONE_ERROR, "Unsupported MSB: %d\n", msb);
                status = VX_FAILURE;
            }
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Unsupported pixel format: %d\n", pixel_container);
            status = VX_FAILURE;
        }
        
        cur_time = tivxPlatformGetTimeInUsecs() - cur_time;
        VX_PRINT(VX_ZONE_ERROR, "RAW12 pre-processing completed. Time: %lu us (%.2f ms)\n", 
                 (unsigned long)cur_time, cur_time/1000.0f);
        
        /* 解除映射 */
        tivxMemBufferUnmap(out_tensor_target_ptr, out_tensor_desc->mem_size, 
                          VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY);
        tivxMemBufferUnmap(in_img_target_ptr, in_img_desc->mem_size[0], 
                          VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
        tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, 
                          VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
    }

    return status;
}

void tivxAddTargetKernelOCPreProc()
{
    char target_name[TIVX_TARGET_MAX_NAME];

    if ((vx_status)VX_SUCCESS == tivxKernelsTargetUtilsAssignTargetNameDsp(target_name))
    {
        vx_oc_pre_proc_target_kernel = tivxAddTargetKernelByName
                                        (
                                            TIVX_KERNEL_OC_PRE_PROC_NAME,
                                            target_name,
                                            tivxKernelOCPreProcProcess,
                                            tivxKernelOCPreProcCreate,
                                            tivxKernelOCPreProcDelete,
                                            NULL,
                                            NULL
                                        );
        
        if (NULL != vx_oc_pre_proc_target_kernel)
        {
            VX_PRINT(VX_ZONE_ERROR, "OC Pre-Proc kernel registered successfully on %s\n", target_name);
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to register OC Pre-Proc kernel\n");
        }
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to assign DSP target name\n");
    }
}

void tivxRemoveTargetKernelOCPreProc()
{
    vx_status status = VX_SUCCESS;

    if (NULL != vx_oc_pre_proc_target_kernel)
    {
        status = tivxRemoveTargetKernel(vx_oc_pre_proc_target_kernel);
        if (status == VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "OC Pre-Proc kernel removed successfully\n");
            vx_oc_pre_proc_target_kernel = NULL;
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to remove OC Pre-Proc kernel: %d\n", status);
        }
    }
}
