/*
 *
 * Copyright (c) 2017 Texas Instruments Incorporated
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

#include "app_pre_proc_module.h"

static void createOutputTensors(vx_context context, vx_user_data_object config, vx_tensor output_tensors[]);
static inline vx_enum get_vx_tensor_datatype(int32_t tidl_datatype);

vx_status app_init_pre_proc(vx_context context, PreProcObj *preProcObj, char *objName)
{
    vx_status status = VX_SUCCESS;

    preProcObj->config = vxCreateUserDataObject(context, "PreProcConfig", sizeof(tivxOCPreProcParams), NULL);
    status = vxGetStatus((vx_reference)preProcObj->config);

    if(status == VX_SUCCESS)
    {
        status = vxCopyUserDataObject(preProcObj->config, 0, sizeof(tivxOCPreProcParams),\
                  &preProcObj->params, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
    }

    if(preProcObj->en_out_pre_proc_write == 1)
    {
        char file_path[TIVX_FILEIO_FILE_PATH_LENGTH];
        char file_prefix[TIVX_FILEIO_FILE_PREFIX_LENGTH];

        strcpy(file_path, preProcObj->output_file_path);
        preProcObj->file_path   = vxCreateArray(context, VX_TYPE_UINT8, TIVX_FILEIO_FILE_PATH_LENGTH);
        vxAddArrayItems(preProcObj->file_path, TIVX_FILEIO_FILE_PATH_LENGTH, &file_path[0], 1);

        strcpy(file_prefix, "pre_proc_output");
        preProcObj->file_prefix = vxCreateArray(context, VX_TYPE_UINT8, TIVX_FILEIO_FILE_PREFIX_LENGTH);
        vxAddArrayItems(preProcObj->file_prefix, TIVX_FILEIO_FILE_PREFIX_LENGTH, &file_prefix[0], 1);

        preProcObj->write_cmd = vxCreateUserDataObject(context, "tivxFileIOWriteCmd", sizeof(tivxFileIOWriteCmd), NULL);
    }
    else
    {
        preProcObj->file_path   = NULL;
        preProcObj->file_prefix = NULL;
        preProcObj->write_node  = NULL;
        preProcObj->write_cmd   = NULL;
    }

    return status;
}

vx_status app_update_pre_proc(vx_context context, PreProcObj *preProcObj, vx_user_data_object config, vx_int32 num_cameras)
{
    vx_status status = VX_SUCCESS;

    vx_map_id map_id_config;
    sTIDL_IOBufDesc_t *ioBufDesc;
    tivxTIDLJ7Params *tidlParams;
    vx_tensor output_tensors[APP_MAX_TENSORS];
    vx_int32 i;

    vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                    (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;
    memcpy(&preProcObj->params.ioBufDesc, ioBufDesc, sizeof(sTIDL_IOBufDesc_t));

//debug
#if 0
    // --- 开始打印所有字段 ---
    printf("=== sTIDL_IOBufDesc_t Dump ===\n");
    printf("numInputBuf: %d\n", ioBufDesc->numInputBuf);
    printf("numOutputBuf: %d\n", ioBufDesc->numOutputBuf);
    printf("numCores: %d\n", ioBufDesc->numCores);
    printf("numVirtualCores: %d\n", ioBufDesc->numVirtualCores);
    printf("numSuperBatches: %d\n", ioBufDesc->numSuperBatches);
    printf("inferenceMode: %d\n", ioBufDesc->inferenceMode);

    int inBufTotal = 4;
    int outBufTotal = 4;

    for (int idx = 0; idx < inBufTotal; idx++) {
        printf("\n--- Input Buffer Index %d ---\n", idx);
        printf("inDataFormat[%d]: %d\n", idx, ioBufDesc->inDataFormat[idx]);
        printf("inResizeType[%d]: %d\n", idx, ioBufDesc->inResizeType[idx]);
        printf("resizeWidth[%d]: %d\n", idx, ioBufDesc->resizeWidth[idx]);
        printf("resizeHeight[%d]: %d\n", idx, ioBufDesc->resizeHeight[idx]);
        printf("inWidth[%d]: %d\n", idx, ioBufDesc->inWidth[idx]);
        printf("inHeight[%d]: %d\n", idx, ioBufDesc->inHeight[idx]);
        printf("inNumChannels[%d]: %d\n", idx, ioBufDesc->inNumChannels[idx]);
        printf("inDIM2[%d]: %d\n", idx, ioBufDesc->inDIM2[idx]);
        printf("inDIM1[%d]: %d\n", idx, ioBufDesc->inDIM1[idx]);
        printf("inChannelPitch[%d]: %d\n", idx, ioBufDesc->inChannelPitch[idx]);
        printf("inNumBatches[%d]: %d\n", idx, ioBufDesc->inNumBatches[idx]);
        printf("inPadL[%d]: %d\n", idx, ioBufDesc->inPadL[idx]);
        printf("inPadT[%d]: %d\n", idx, ioBufDesc->inPadT[idx]);
        printf("inPadR[%d]: %d\n", idx, ioBufDesc->inPadR[idx]);
        printf("inPadB[%d]: %d\n", idx, ioBufDesc->inPadB[idx]);
        printf("inPadCh[%d]: %d\n", idx, ioBufDesc->inPadCh[idx]);
        printf("rawDataInElementType[%d]: %d\n", idx, ioBufDesc->rawDataInElementType[idx]);
        printf("inElementType[%d]: %d\n", idx, ioBufDesc->inElementType[idx]);
        printf("inZeroPoint[%d]: %d\n", idx, ioBufDesc->inZeroPoint[idx]);
        printf("inLayout[%d]: %d\n", idx, ioBufDesc->inLayout[idx]);
        printf("inDataId[%d]: %d\n", idx, ioBufDesc->inDataId[idx]);
        printf("inTensorScale[%d]: %f\n", idx, ioBufDesc->inTensorScale[idx]);
        printf("inDataName[%d]: %s\n", idx, ioBufDesc->inDataName[idx]);
    }

    for (int idx = 0; idx < outBufTotal; idx++) {
        printf("\n--- Output Buffer Index %d ---\n", idx);
        printf("outWidth[%d]: %d\n", idx, ioBufDesc->outWidth[idx]);
        printf("outHeight[%d]: %d\n", idx, ioBufDesc->outHeight[idx]);
        printf("outDIM2[%d]: %d\n", idx, ioBufDesc->outDIM2[idx]);
        printf("outDIM1[%d]: %d\n", idx, ioBufDesc->outDIM1[idx]);
        printf("outNumChannels[%d]: %d\n", idx, ioBufDesc->outNumChannels[idx]);
        printf("outChannelPitch[%d]: %d\n", idx, ioBufDesc->outChannelPitch[idx]);
        printf("outNumBatches[%d]: %d\n", idx, ioBufDesc->outNumBatches[idx]);
        printf("outPadL[%d]: %d\n", idx, ioBufDesc->outPadL[idx]);
        printf("outPadT[%d]: %d\n", idx, ioBufDesc->outPadT[idx]);
        printf("outPadR[%d]: %d\n", idx, ioBufDesc->outPadR[idx]);
        printf("outPadB[%d]: %d\n", idx, ioBufDesc->outPadB[idx]);
        printf("outPadCh[%d]: %d\n", idx, ioBufDesc->outPadCh[idx]);
        printf("outElementType[%d]: %d\n", idx, ioBufDesc->outElementType[idx]);
        printf("outDataId[%d]: %d\n", idx, ioBufDesc->outDataId[idx]);
        printf("outDataName[%d]: %s\n", idx, ioBufDesc->outDataName[idx]);
        printf("outTensorScale[%d]: %f\n", idx, ioBufDesc->outTensorScale[idx]);
        printf("outZeroPoint[%d]: %d\n", idx, ioBufDesc->outZeroPoint[idx]);
        printf("outLayout[%d]: %d\n", idx, ioBufDesc->outLayout[idx]);
        printf("numValidTensorDims[%d]: %d\n", idx, ioBufDesc->numValidTensorDims[idx]);
    }
    printf("=== End of sTIDL_IOBufDesc_t Dump ===\n");
    // --- 打印结束 ---
#endif

    vxUnmapUserDataObject(config, map_id_config);

    preProcObj->num_input_tensors = ioBufDesc->numInputBuf;
    preProcObj->num_output_tensors = ioBufDesc->numOutputBuf;

    createOutputTensors(context, config, output_tensors);

    for(i = 0; i < preProcObj->num_input_tensors; i++)
    {
        preProcObj->output_tensor_arr[i] = vxCreateObjectArray(context, (vx_reference)output_tensors[i], num_cameras);
        vxReleaseTensor(&output_tensors[i]);
    }

    return status;
}

void app_deinit_pre_proc(PreProcObj *preProcObj)
{
    vxReleaseUserDataObject(&preProcObj->config);

    vx_int32 i;
    for(i = 0; i < preProcObj->num_input_tensors; i++)
    {
        vxReleaseObjectArray(&preProcObj->output_tensor_arr[i]);
    }
    if(preProcObj->en_out_pre_proc_write == 1)
    {
        vxReleaseArray(&preProcObj->file_path);
        vxReleaseArray(&preProcObj->file_prefix);
        vxReleaseUserDataObject(&preProcObj->write_cmd);
    }
}

void app_delete_pre_proc(PreProcObj *preProcObj)
{
    if(preProcObj->node != NULL)
    {
        vxReleaseNode(&preProcObj->node);
    }
    if(preProcObj->write_node != NULL)
    {
        vxReleaseNode(&preProcObj->write_node);
    }
}

vx_status app_create_graph_pre_proc(vx_graph graph, PreProcObj *preProcObj, vx_object_array input_arr)
{
    vx_status status = VX_SUCCESS;

    // tivx_raw_image  input   = (tivx_raw_image)vxGetObjectArrayItem((vx_object_array)input_arr, 0);
    tivx_raw_image  input   = (tivx_raw_image)vxGetObjectArrayItem((vx_object_array)input_arr, 0);
    vx_tensor output  = (vx_tensor)vxGetObjectArrayItem((vx_object_array)preProcObj->output_tensor_arr[0], 0);

    preProcObj->node = tivxOCPreProcNode(graph,
                                         preProcObj->config,
                                         input,
                                         output);

    APP_ASSERT_VALID_REF(preProcObj->node);

    status = vxSetNodeTarget(preProcObj->node, VX_TARGET_STRING, TIVX_TARGET_DSP1);
    vxSetReferenceName((vx_reference)preProcObj->node, "pre_proc_node");

    vx_bool replicate[] = {vx_false_e, vx_true_e, vx_true_e};
    vxReplicateNode(graph, preProcObj->node, replicate, 3);

    tivxReleaseRawImage(&input);
    vxReleaseTensor(&output);

    if(preProcObj->en_out_pre_proc_write == 1)
    {
        printf("Adding Pre Proc write node on graph .. \n");
        status = app_create_graph_pre_proc_write_output(graph, preProcObj);
        printf("Pre Proc write node added! \n");
    }

    return(status);
}

vx_status app_create_graph_pre_proc_write_output(vx_graph graph, PreProcObj *preProcObj)
{
    vx_status status = VX_SUCCESS;

    vx_tensor output = (vx_tensor)vxGetObjectArrayItem(preProcObj->output_tensor_arr[0], 0);

    preProcObj->write_node = tivxWriteTensorNode(graph, output, preProcObj->file_path, preProcObj->file_prefix);
    status = vxGetStatus((vx_reference)preProcObj->write_node);
    vxSetNodeTarget(preProcObj->write_node, VX_TARGET_STRING, TIVX_TARGET_MPU_0);
    vxSetReferenceName((vx_reference)preProcObj->write_node, "pre_proc_write_node");

    vx_bool replicate[] = { vx_true_e, vx_false_e, vx_false_e};
    vxReplicateNode(graph, preProcObj->write_node, replicate, 3);

    vxReleaseTensor(&output);

    return (status);
}

vx_status app_send_cmd_pre_proc_write_node(PreProcObj *preProcObj, vx_uint32 start_frame, vx_uint32 num_frames, vx_uint32 num_skip)
{
    vx_status status = VX_SUCCESS;

    tivxFileIOWriteCmd write_cmd;

    write_cmd.start_frame = start_frame;
    write_cmd.num_frames = num_frames;
    write_cmd.num_skip = num_skip;

    status = vxCopyUserDataObject(preProcObj->write_cmd, 0, sizeof(tivxFileIOWriteCmd),\
                  &write_cmd, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    if(status == VX_SUCCESS)
    {
        vx_reference refs[2];

        refs[0] = (vx_reference)preProcObj->write_cmd;

        status = tivxNodeSendCommand(preProcObj->write_node, TIVX_CONTROL_CMD_SEND_TO_ALL_REPLICATED_NODES,
                                 TIVX_FILEIO_CMD_SET_FILE_WRITE,
                                 refs, 1u);

        if(VX_SUCCESS != status)
        {
            printf("Pre proc write node send command failed!\n");
        }

        APP_PRINTF("Pre proc write node send command success!\n");
    }

    return (status);
}

vx_status writePreProcOutput(char* file_name, PreProcObj *preProcObj)
{
    vx_status status = VX_SUCCESS;

    return(status);
}

static void createOutputTensors(vx_context context, vx_user_data_object config, vx_tensor output_tensors[])
{
    vx_size input_sizes[APP_MAX_TENSOR_DIMS] = {0}; // 初始化为0，避免未定义行为
    vx_map_id map_id_config;
    vx_uint32 id;

    tivxTIDLJ7Params *tidlParams;
    sTIDL_IOBufDesc_t *ioBufDesc;

    vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                      (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;

    // 创建 4 维张量：[N, C, H, W] = [1, inNumChannels, inHeight+pad, inWidth+pad]
    // const vx_size num_dims = 3;

    for(id = 0; id < ioBufDesc->numInputBuf; id++) 
    {
        const vx_size num_dims = 4;
        // 按 WHCN 顺序填充 sizes 数组
        input_sizes[0] = ioBufDesc->inWidth[id]  + ioBufDesc->inPadL[id] + ioBufDesc->inPadR[id]; // W
        input_sizes[1] = ioBufDesc->inHeight[id] + ioBufDesc->inPadT[id] + ioBufDesc->inPadB[id]; // H
        input_sizes[2] = ioBufDesc->inNumChannels[id];                     // C
        input_sizes[3] = 1;                     // N
        // input_sizes[0] = 1;  // Batch size N = 1
        // input_sizes[1] = ioBufDesc->inNumChannels[id];                     // C
        // input_sizes[2] = ioBufDesc->inHeight[id] + ioBufDesc->inPadT[id] + ioBufDesc->inPadB[id]; // H
        // input_sizes[3] = ioBufDesc->inWidth[id]  + ioBufDesc->inPadL[id] + ioBufDesc->inPadR[id]; // W
        // // 按WHC顺序填充 sizes 数组
        // input_sizes[0] = ioBufDesc->inWidth[id]  + ioBufDesc->inPadL[id] + ioBufDesc->inPadR[id]; // W
        // input_sizes[1] = ioBufDesc->inHeight[id] + ioBufDesc->inPadT[id] + ioBufDesc->inPadB[id]; // H
        // input_sizes[2] = ioBufDesc->inNumChannels[id];                     // C

        vx_enum data_type = get_vx_tensor_datatype(ioBufDesc->inElementType[id]);
        output_tensors[id] = vxCreateTensor(context, num_dims, input_sizes, data_type, 0);

        printf("input_sizes[0]=%ld,input_sizes[1]=%ld,input_sizes[2]=%ld,input_sizes[3]=%ld, data_type=%d\n", input_sizes[0],input_sizes[1],input_sizes[2],input_sizes[3], data_type);
        // 可选：添加错误检查（推荐）
        if (vxGetStatus((vx_reference)output_tensors[id]) != VX_SUCCESS) {
            // 可以打印日志或设置全局错误标志
            printf("Failed to create tensor for input %d\n", id);
        }
    }

    vxUnmapUserDataObject(config, map_id_config);
}

static inline vx_enum get_vx_tensor_datatype(int32_t tidl_datatype)
{
    vx_enum tiovx_datatype = VX_TYPE_INVALID;

    if(tidl_datatype == TIDL_UnsignedChar)
    {
        tiovx_datatype = VX_TYPE_UINT8;
    }
    else if(tidl_datatype == TIDL_SignedChar)
    {
        tiovx_datatype = VX_TYPE_INT8;
    }
    else if(tidl_datatype == TIDL_UnsignedShort)
    {
        tiovx_datatype = VX_TYPE_UINT16;
    }
    else if(tidl_datatype == TIDL_SignedShort)
    {
        tiovx_datatype = VX_TYPE_INT16;
    }
    else if(tidl_datatype == TIDL_UnsignedWord)
    {
        tiovx_datatype = VX_TYPE_UINT32;
    }
    else if(tidl_datatype == TIDL_SignedWord)
    {
        tiovx_datatype = VX_TYPE_INT32;
    }
    else if(tidl_datatype == TIDL_SinglePrecFloat)
    {
        tiovx_datatype = VX_TYPE_FLOAT32;
    }

    return (tiovx_datatype);
}
