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

/*
 * 与预处理代码保持一致的内存布局
 */

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <TI/tivx.h>
#include <TI/tivx_img_proc.h>
#include <TI/tivx_target_kernel.h>
#include "tivx_kernels_target_utils.h"

#include <tivx_oc_post_proc_host.h>
#include <utils/mem/include/app_mem.h>

/* 调试打印开关 - 生产环境可关闭 */
#define OC_POST_PROC_DEBUG_PRINT     (1)
#define OC_POST_PROC_PRINT_MAX_BAYER (4)  /* 打印前4个Bayer单元用于验证 */

static tivx_target_kernel vx_oc_post_proc_target_kernel = NULL;

static vx_status VX_CALLBACK tivxKernelOCPostProcCreate
(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg
)
{
    tivxOCPostProcParams * prms = NULL;

    prms = tivxMemAlloc(sizeof(tivxOCPostProcParams), TIVX_MEM_EXTERNAL);
    if (NULL == prms)
    {
        return VX_FAILURE;
    }

    tivxSetTargetKernelInstanceContext(kernel, prms, sizeof(tivxOCPostProcParams));

    return VX_SUCCESS;
}

static vx_status VX_CALLBACK tivxKernelOCPostProcDelete(
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
        tivxOCPostProcParams *prms = NULL;

        status = tivxGetTargetKernelInstanceContext(kernel,
            (void **)&prms, &size);

        if ((VX_SUCCESS == status) && (NULL != prms))
        {
            tivxMemFree(prms, sizeof(tivxOCPostProcParams), TIVX_MEM_EXTERNAL);
        }
    }

    return status;
}

static vx_status VX_CALLBACK tivxKernelOCPostProcProcess
(
    tivx_target_kernel_instance kernel,
    tivx_obj_desc_t *obj_desc[],
    vx_uint16 num_params,
    void *priv_arg
)
{
    vx_status status = VX_SUCCESS;
    vx_int32 i;
    tivxOCPostProcParams *prms = NULL;

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
        if ((VX_SUCCESS != status) || (NULL == prms) || (sizeof(tivxOCPostProcParams) != size))
        {
            status = VX_FAILURE;
        }
    }

    if (VX_SUCCESS == status)
    {
        uint64_t cur_time = tivxPlatformGetTimeInUsecs();

        /* 获取对象描述 */
        tivx_obj_desc_user_data_object_t* config_desc = 
            (tivx_obj_desc_user_data_object_t *)obj_desc[TIVX_KERNEL_OC_POST_PROC_CONFIG_IDX];
        void *config_target_ptr = tivxMemShared2TargetPtr(&config_desc->mem_ptr);
        
        tivx_obj_desc_tensor_t *in_tensor_desc = 
            (tivx_obj_desc_tensor_t *)obj_desc[TIVX_KERNEL_OC_POST_PROC_INPUT_TENSOR_IDX];
        void *in_tensor_target_ptr = tivxMemShared2TargetPtr(&in_tensor_desc->mem_ptr);
        
        tivx_obj_desc_raw_image_t *out_img_desc = 
            (tivx_obj_desc_raw_image_t *)obj_desc[TIVX_KERNEL_OC_POST_PROC_OUT_IMAGE];
        void *out_img_target_ptr = tivxMemShared2TargetPtr(&out_img_desc->mem_ptr[0]);

        /* 内存映射 */
        if (VX_SUCCESS != tivxMemBufferMap(config_target_ptr, config_desc->mem_size, 
                                            VX_MEMORY_TYPE_HOST, VX_READ_ONLY))
        {
            return VX_FAILURE;
        }
        if (VX_SUCCESS != tivxMemBufferMap(in_tensor_target_ptr, in_tensor_desc->mem_size, 
                                            VX_MEMORY_TYPE_HOST, VX_READ_ONLY))
        {
            tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, 
                              VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
            return VX_FAILURE;
        }
        if (VX_SUCCESS != tivxMemBufferMap(out_img_target_ptr, out_img_desc->mem_size[0], 
                                            VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY))
        {
            tivxMemBufferUnmap(in_tensor_target_ptr, in_tensor_desc->mem_size, 
                              VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
            tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, 
                              VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
            return VX_FAILURE;
        }
        // 3. 检查输入tensor前几个值
        float *first_tensor_value = (float *)in_tensor_target_ptr;
        VX_PRINT(VX_ZONE_INFO, "post 20 tensor values:%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", \
            first_tensor_value[10000], first_tensor_value[10001], first_tensor_value[10002], first_tensor_value[10003], first_tensor_value[10004], \
            first_tensor_value[2105], first_tensor_value[4006], first_tensor_value[1007], first_tensor_value[108], first_tensor_value[1009], \
            first_tensor_value[3400], first_tensor_value[7001], first_tensor_value[2002], first_tensor_value[1403], first_tensor_value[1104], \
            first_tensor_value[7205], first_tensor_value[8606], first_tensor_value[30027], first_tensor_value[508], first_tensor_value[1209]);

        /* 获取输出图像维度 */
        // uint32_t out_width  = out_img_desc->imagepatch_addr[0].dim_x;
        // uint32_t out_height = out_img_desc->imagepatch_addr[0].dim_y;
        uint32_t out_stride = out_img_desc->imagepatch_addr[0].stride_y;
        
        /* 
         * 获取输入Tensor维度 (与预处理一致)
         * dimensions[0] = Width (下采样后)
         * dimensions[1] = Height (下采样后)
         * dimensions[2] = Channels (4)
         */
        uint32_t in_w = in_tensor_desc->dimensions[0];
        uint32_t in_h = in_tensor_desc->dimensions[1];
        // uint32_t in_c = in_tensor_desc->dimensions[2];
        uint32_t element_size = sizeof(vx_float32);
        VX_PRINT(VX_ZONE_INFO, 
                "num_dims=%d\n"
                "in_tensor dims=[%d,%d,%d,%d]\n" 
                "in_tensor strides=[%d,%d,%d,%d] (字节)\n"
                "in_tensor element_size=%d\n"
                "in_tensor mem_size=%d\n",
                in_tensor_desc->number_of_dimensions,
                in_tensor_desc->dimensions[0],
                in_tensor_desc->dimensions[1], 
                in_tensor_desc->dimensions[2],
                in_tensor_desc->dimensions[3],
                in_tensor_desc->stride[0],
                in_tensor_desc->stride[1],
                in_tensor_desc->stride[2],
                in_tensor_desc->stride[3],
                element_size,
                in_tensor_desc->mem_size);

        /* 维度验证 */
        // if ((out_width != in_w * 2) || (out_height != in_h * 2))
        // {
        //     VX_PRINT(VX_ZONE_ERROR, 
        //              "[WARNING] 输出尺寸不匹配! 期望: %dx%d, 实际: %dx%d\n",
        //              in_w * 2, in_h * 2, out_width, out_height);
        // }
        
        // if (in_c != 4)
        // {
        //     VX_PRINT(VX_ZONE_ERROR, 
        //              "[ERROR] Tensor通道数不匹配! 期望: C=4, 实际: C=%d\n", in_c);
        //     status = VX_FAILURE;
        // }

        if (VX_SUCCESS == status)
        {
            if (in_tensor_desc->dimensions[2] == 4)
            {
                float    *in_base  = (float *)in_tensor_target_ptr;
                uint8_t *out_base = (uint8_t *)out_img_target_ptr;
                
                /* 清零输出缓冲区 */
                memset(out_base, 0, out_img_desc->mem_size[0]);
                
                /* 
                * 关键修复：TIDL CHW布局, 但是dim[0]-W dim[1]-H dim[2]-C
                * 与预处理代码保持一致
                * 
                * dimensions[0] = Width,  stride[0] = Width方向stride
                * dimensions[1] = Height, stride[1] = Height方向stride
                * dimensions[2] = Channels, stride[2] = Channel方向stride
                *
                */
                uint32_t tensor_stride_w = in_tensor_desc->stride[0] / element_size;
                uint32_t tensor_stride_h = in_tensor_desc->stride[1] / element_size;
                uint32_t tensor_stride_c = in_tensor_desc->stride[2] / element_size;
                
                float *in_ch0, *in_ch1, *in_ch2, *in_ch3;
                
                /* 通道分离存储 - 每个通道是独立的WH平面 */
                in_ch0 = in_base;
                in_ch1 = in_base + tensor_stride_c;
                in_ch2 = in_base + 2 * tensor_stride_c;
                in_ch3 = in_base + 3 * tensor_stride_c;
                uint32_t in_y, in_x;
                // VX_PRINT(VX_ZONE_ERROR, "in_ch0=%p, in_ch1=%p, in_ch2=%p, in_ch3=%p, mem_size=%d, C=%d,H=%d,W=%d, stride[2]=%d,out_stride=%d, in_h=%d, in_w=%d\n", \
                //     in_ch0,in_ch1,in_ch2,in_ch3, in_tensor_desc->mem_size,tensor_stride_c,tensor_stride_h,tensor_stride_w, in_tensor_desc->stride[2],out_stride, in_h, in_w);
                
                /* 按2x2块处理（与预处理对称）*/
                for (in_y = 0; in_y < in_h; in_y++)
                {
                    /* 输出行指针（考虑stride，单位为uint16_t）*/
                    uint32_t out_y = in_y * 2;
                    uint16_t *pOutRow0 = (uint16_t *)(out_base + out_y * out_stride);
                    uint16_t *pOutRow1 = (uint16_t *)(out_base + (out_y + 1) * out_stride);
                    
                    for (in_x = 0; in_x < in_w; in_x++)
                    {
                        uint32_t out_x = in_x * 2;
                        
                        float fval_00, fval_01, fval_10, fval_11;
                        
                        /* 分离通道 - 每个通道是独立的WH平面 */
                        uint32_t pixel_idx = in_x * tensor_stride_w + in_y * tensor_stride_h;
                        fval_00  = in_ch0[pixel_idx];
                        fval_01 = in_ch1[pixel_idx];
                        fval_10 = in_ch2[pixel_idx];
                        fval_11  = in_ch3[pixel_idx];
                        pOutRow0[out_x]     = (uint16_t)(fval_00*4095) & 0x0FFF;   
                        pOutRow0[out_x + 1] = (uint16_t)(fval_01*4095) & 0x0FFF;   
                        pOutRow1[out_x]     = (uint16_t)(fval_10*4095) & 0x0FFF;  
                        pOutRow1[out_x + 1] = (uint16_t)(fval_11*4095) & 0x0FFF;  
                        // pOutRow0[out_x]     = (uint16_t)(fval_00*4095  + 0.5f) & 0x0FFF;   
                        // pOutRow0[out_x + 1] = (uint16_t)(fval_01*4095 + 0.5f) & 0x0FFF;   
                        // pOutRow1[out_x]     = (uint16_t)(fval_10*4095 + 0.5f) & 0x0FFF;  
                        // pOutRow1[out_x + 1] = (uint16_t)(fval_11*4095  + 0.5f) & 0x0FFF;  
                        // if (pixel_idx >= 518399)
                        //     VX_PRINT(VX_ZONE_ERROR, "in_x=%d,in_y=%d,tensor input:%.2f,%.2f,%.2f,%.2f;output raw12:%d,%d,%d,%d, pOutRow0=%p, pOutRow1=%p\n",\
                        //     in_x,in_y,fval_00,fval_01,fval_10,fval_11,pOutRow0[out_x],pOutRow0[out_x+1],pOutRow1[out_x],pOutRow1[out_x+1], pOutRow0, pOutRow1);
                        
                    }
                }
            }
            else if (in_tensor_desc->dimensions[2] == 1)
            {
                /* 定义指针：类型互换 */
                float *in_base = (float *)in_tensor_target_ptr;       // 输入现在是 Float Tensor
                uint16_t *out_base = (uint16_t *)out_img_target_ptr;  // 输出现在是 Uint16 Image

                /* 获取输入 Tensor 的步长 (单位：元素个数) */
                // 假设 in_tensor_desc 是输入 Tensor 的描述符
                uint32_t tensor_stride_w = in_tensor_desc->stride[0] / sizeof(float); 
                uint32_t tensor_stride_h = in_tensor_desc->stride[1] / sizeof(float);

                /* 获取输出 Image 的步长 (单位：字节) */
                // 假设 out_img_desc 是输出 Image 的描述符
                uint32_t out_stride_bytes = out_img_desc->imagepatch_addr[0].stride_y;
                uint32_t out_width  = out_img_desc->imagepatch_addr[0].dim_x;
                uint32_t out_height = out_img_desc->imagepatch_addr[0].dim_y;

                /* 确定处理尺寸 (取输入输出的最小值，防止越界) */
                // 输入 Tensor 的宽高
                uint32_t in_tensor_w = in_tensor_desc->dimensions[0];
                uint32_t in_tensor_h = in_tensor_desc->dimensions[1];

                uint32_t h_limit = (in_tensor_h < out_height) ? in_tensor_h : out_height;
                uint32_t w_limit = (in_tensor_w < out_width)  ? in_tensor_w  : out_width;

                uint32_t y, x;
                for (y = 0; y < h_limit; y++)
                {
                    /* 输入行地址：基于 Tensor 的 element stride */
                    float *pInRow = in_base + (y * tensor_stride_h);
                    
                    /* 输出行地址：基于 Image 的 byte stride */
                    // 先算出这一行有多少个 uint16 元素
                    uint32_t out_stride_elements = out_stride_bytes / sizeof(uint16_t);
                    uint16_t *pOutRow = out_base + (y * out_stride_elements);

                    for (x = 0; x < w_limit; x++)
                    {
                        float val_float = pInRow[x * tensor_stride_w];
                        
                        /* 
                        * 转换逻辑：
                        * 1. (可选) 四舍五入：+ 0.5f
                        * 2. 强制转换为 uint16_t
                        * 3. (可选) 钳位保护：防止 float 超出 0~65535 范围导致溢出
                        */
                        // 推荐：四舍五入模式
                        uint16_t val_uint16 = (uint16_t)(val_float);
                        pOutRow[x] = val_uint16;
                    }
                }
            }

        }

        cur_time = tivxPlatformGetTimeInUsecs() - cur_time;
        VX_PRINT(VX_ZONE_ERROR, 
                 "RAW12 post-processing completed. Time: %lu us (%.2f ms)\n", 
                 (unsigned long)cur_time, cur_time/1000.0f);

        /* 解除映射 */
        tivxMemBufferUnmap(out_img_target_ptr, out_img_desc->mem_size[0], 
                          VX_MEMORY_TYPE_HOST, VX_WRITE_ONLY);
        tivxMemBufferUnmap(in_tensor_target_ptr, in_tensor_desc->mem_size, 
                          VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
        tivxMemBufferUnmap(config_target_ptr, config_desc->mem_size, 
                          VX_MEMORY_TYPE_HOST, VX_READ_ONLY);
    }

    return status;
}

void tivxAddTargetKernelOCPostProc()
{
    char target_name[TIVX_TARGET_MAX_NAME];

    if ((vx_status)VX_SUCCESS == tivxKernelsTargetUtilsAssignTargetNameDsp(target_name))
    {
        vx_oc_post_proc_target_kernel = tivxAddTargetKernelByName
                                        (
                                            TIVX_KERNEL_OC_POST_PROC_NAME,
                                            target_name,
                                            tivxKernelOCPostProcProcess,
                                            tivxKernelOCPostProcCreate,
                                            tivxKernelOCPostProcDelete,
                                            NULL,
                                            NULL
                                        );
        
        if (NULL != vx_oc_post_proc_target_kernel)
        {
            VX_PRINT(VX_ZONE_ERROR, "OC Post-Proc kernel registered successfully on %s\n", target_name);
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to register OC Post-Proc kernel\n");
        }
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Failed to assign DSP target name\n");
    }
}

void tivxRemoveTargetKernelOCPostProc()
{
    vx_status status = VX_SUCCESS;

    if (NULL != vx_oc_post_proc_target_kernel)
    {
        status = tivxRemoveTargetKernel(vx_oc_post_proc_target_kernel);
        if (status == VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "OC Post-Proc kernel removed successfully\n");
            vx_oc_post_proc_target_kernel = NULL;
        }
        else
        {
            VX_PRINT(VX_ZONE_ERROR, "Failed to remove OC Post-Proc kernel: %d\n", status);
        }
    }
}
