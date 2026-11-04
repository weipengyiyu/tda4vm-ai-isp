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

#include "app_post_proc_module.h"

vx_status app_init_post_proc(vx_context context, PostProcObj *postProcObj, char *objName, vx_int32 num_cameras, vx_int32 bufq_depth)
{
    vx_status status = VX_SUCCESS;

    postProcObj->config = vxCreateUserDataObject(context, "PostProcConfig", sizeof(tivxOCPostProcParams), NULL);
    status = vxGetStatus((vx_reference)postProcObj->config);

    if(status == VX_SUCCESS)
    {
        status = vxCopyUserDataObject(postProcObj->config, 0, sizeof(tivxOCPostProcParams),\
                  &postProcObj->params, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
    }

    if(status == VX_SUCCESS)
    {
        tivx_raw_image_create_params_t raw_params;
        // raw_params.width = 1920/2;
        // raw_params.height = 1080/2;
        raw_params.width = 1920;
        raw_params.height = 1080;
        raw_params.num_exposures = 1;
        raw_params.line_interleaved = vx_false_e;
        raw_params.format[0].pixel_container = TIVX_RAW_IMAGE_16_BIT;
        raw_params.format[0].msb = 11;
        // raw_params.format[1].pixel_container = TIVX_RAW_IMAGE_16_BIT;
        // raw_params.format[1].msb = 11;
        // raw_params.format[2].pixel_container = TIVX_RAW_IMAGE_16_BIT;
        // raw_params.format[2].msb = 11;
        raw_params.meta_height_before = 0;
        raw_params.meta_height_after = 0;
        tivx_raw_image output = tivxCreateRawImage(context, &raw_params);
        status = vxGetStatus((vx_reference)output);
        postProcObj->output_image_arr  = vxCreateObjectArray(context, (vx_reference)output, num_cameras);
        status = vxGetStatus((vx_reference)postProcObj->output_image_arr);
        if (status == VX_SUCCESS) {
            vx_char name[VX_MAX_REFERENCE_NAME];

            snprintf(name, VX_MAX_REFERENCE_NAME, "post_node_raw_image");

            vxSetReferenceName((vx_reference)postProcObj->output_image_arr, name);
        }
        tivxReleaseRawImage(&output);
        printf("post pro width=%d, height=%d \n", raw_params.width, raw_params.height);
    }

    /* 
    #define TIVX_FILEIO_FILE_PATH_LENGTH    (512U)
    #define TIVX_FILEIO_FILE_PREFIX_LENGTH  (256U)
    #define TIVX_FILEIO_CMD_SET_FILE_WRITE  (50) 
    create_post_output
    */
    if(postProcObj->en_out_post_proc_write == 1)
    {
        char file_path[TIVX_FILEIO_FILE_PATH_LENGTH];
        char file_postfix[TIVX_FILEIO_FILE_PREFIX_LENGTH];

        strcpy(file_path, postProcObj->output_file_path);
        postProcObj->file_path   = vxCreateArray(context, VX_TYPE_UINT8, TIVX_FILEIO_FILE_PATH_LENGTH);
        status = vxGetStatus((vx_reference)postProcObj->file_path);
        if(status == VX_SUCCESS) 
        {
            vxSetReferenceName((vx_reference)postProcObj->file_path, "post_write_node_file_path");
            vxAddArrayItems(postProcObj->file_path, TIVX_FILEIO_FILE_PATH_LENGTH, &file_path[0], 1);
        }
        else
        {
            printf("[POST-MODULE] Unable to create file path object for writing post output! \n");
        }

        strcpy(file_postfix, "post_proc_output");
        postProcObj->file_postfix = vxCreateArray(context, VX_TYPE_UINT8, TIVX_FILEIO_FILE_PREFIX_LENGTH);

        status = vxGetStatus((vx_reference)postProcObj->file_postfix);
        if(status == VX_SUCCESS)
        {
            vxSetReferenceName((vx_reference)postProcObj->file_postfix, "post_write_node_file_prefix");
            vxAddArrayItems(postProcObj->file_postfix, TIVX_FILEIO_FILE_PREFIX_LENGTH, &file_postfix[0], 1);
        }
        else
        {
            printf("[POST-MODULE] Unable to create file prefix object for writing post output! \n");
        }


        postProcObj->write_cmd = vxCreateUserDataObject(context, "tivxFileIOWriteCmd", sizeof(tivxFileIOWriteCmd), NULL);
        status = vxGetStatus((vx_reference)postProcObj->write_cmd);
        if(status != VX_SUCCESS)
        {
            printf("[POST-MODULE] Unable to create file write cmd object for writing post output! \n");
        }
        else
        {
            vxSetReferenceName((vx_reference)postProcObj->write_cmd, "post_write_node_write_cmd");
        }
    }
    else
    {
        postProcObj->file_path       = NULL;
        postProcObj->file_postfix    = NULL;
        postProcObj->write_node      = NULL;
        postProcObj->write_cmd       = NULL;
    }

    return status;
}

vx_status app_update_post_proc(vx_context context, PostProcObj *postProcObj, vx_user_data_object config)
{
    vx_status status = VX_SUCCESS;

    vx_map_id map_id_config;
    sTIDL_IOBufDesc_t *ioBufDesc;
    tivxTIDLJ7Params *tidlParams;

    vxMapUserDataObject(config, 0, sizeof(tivxTIDLJ7Params), &map_id_config,
                    (void **)&tidlParams, VX_READ_ONLY, VX_MEMORY_TYPE_HOST, 0);

    ioBufDesc = (sTIDL_IOBufDesc_t *)&tidlParams->ioBufDesc;
    memcpy(&postProcObj->params.ioBufDesc, ioBufDesc, sizeof(sTIDL_IOBufDesc_t));

    postProcObj->num_input_tensors = ioBufDesc->numInputBuf;
    postProcObj->num_output_tensors = ioBufDesc->numOutputBuf;

    vxUnmapUserDataObject(config, map_id_config);

    return status;
}

void app_deinit_post_proc(PostProcObj *postProcObj, vx_int32 bufq_depth)
{
    vxReleaseUserDataObject(&postProcObj->config);
    vxReleaseObjectArray(&postProcObj->output_image_arr);
    if(postProcObj->en_out_post_proc_write == 1)
    {
        vxReleaseArray(&postProcObj->file_path);
        vxReleaseArray(&postProcObj->file_postfix);
        vxReleaseUserDataObject(&postProcObj->write_cmd);
    }
}

void app_delete_post_proc(PostProcObj *postProcObj)
{
    if(postProcObj->node != NULL)
    {
        vxReleaseNode(&postProcObj->node);
    }
    if(postProcObj->write_node != NULL)
    {
        vxReleaseNode(&postProcObj->write_node);
    }
}

vx_status app_create_graph_post_proc(vx_graph graph, PostProcObj *postProcObj, vx_object_array out_args_arr, vx_object_array out_tensor_arr)
{
    vx_status status = VX_SUCCESS;

    vx_user_data_object  in_args  = (vx_user_data_object)vxGetObjectArrayItem((vx_object_array)out_args_arr, 0);
    vx_tensor in_tensor = (vx_tensor)vxGetObjectArrayItem((vx_object_array)out_tensor_arr, 0);
    tivx_raw_image  output_image   = (tivx_raw_image)vxGetObjectArrayItem((vx_object_array)postProcObj->output_image_arr, 0);

    postProcObj->node = tivxOCPostProcNode(graph,
                                           postProcObj->config,
                                           in_args,
                                           in_tensor,
                                           output_image);

    APP_ASSERT_VALID_REF(postProcObj->node);

    status = vxSetNodeTarget(postProcObj->node, VX_TARGET_STRING, TIVX_TARGET_DSP2);
    vxSetReferenceName((vx_reference)postProcObj->node, "post_proc_node");

    vx_bool replicate[] = {vx_false_e, vx_true_e, vx_true_e, vx_true_e};
    vxReplicateNode(graph, postProcObj->node, replicate, 4);

    vxReleaseUserDataObject(&in_args);
    vxReleaseTensor(&in_tensor);
    tivxReleaseRawImage(&output_image);
    if(postProcObj->en_out_post_proc_write == 1)
    {
        printf("Adding Post Proc write node on graph .. \n");
        status = app_create_graph_post_proc_write_output(graph, postProcObj);
        printf("Post Proc write node added! \n");
    }

    return(status);
}

vx_status app_create_graph_post_proc_write_output(vx_graph graph, PostProcObj *postProcObj)
{
    vx_status status = VX_SUCCESS;

    tivx_raw_image raw_img = (tivx_raw_image)vxGetObjectArrayItem(postProcObj->output_image_arr, 0);
    
    postProcObj->write_node = tivxWriteRawImageNode(graph, raw_img, postProcObj->file_path, postProcObj->file_postfix);
    status = vxGetStatus((vx_reference)postProcObj->write_node);
    if(status == VX_SUCCESS)
    {
        vxSetNodeTarget(postProcObj->write_node, VX_TARGET_STRING, TIVX_TARGET_MPU_0);
        vxSetReferenceName((vx_reference)postProcObj->write_node, "post_proc_write_node");
        vx_bool replicate[] = { vx_true_e, vx_false_e, vx_false_e};
        vxReplicateNode(graph, postProcObj->write_node, replicate, 3);
    }
    else
    {
        printf("[POST-MODULE] Unable to create RAW image write node! \n");
    }
    tivxReleaseRawImage(&raw_img);
    return (status);
}

vx_status app_send_cmd_post_proc_write_node(PostProcObj *postProcObj, vx_uint32 start_frame, vx_uint32 num_frames, vx_uint32 num_skip)
{
    vx_status status = VX_SUCCESS;

    tivxFileIOWriteCmd write_cmd;

    write_cmd.start_frame = start_frame;
    write_cmd.num_frames = num_frames;
    write_cmd.num_skip = num_skip;

    // printf("Post proc write node send command 1 \n");
    status = vxCopyUserDataObject(postProcObj->write_cmd, 0, sizeof(tivxFileIOWriteCmd),\
                  &write_cmd, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    if(status == VX_SUCCESS)
    {
        vx_reference refs[2];

        refs[0] = (vx_reference)postProcObj->write_cmd;

        // printf("Post proc write node send command 2 \n");
        status = tivxNodeSendCommand(postProcObj->write_node, TIVX_CONTROL_CMD_SEND_TO_ALL_REPLICATED_NODES,
                                 TIVX_FILEIO_CMD_SET_FILE_WRITE,
                                 refs, 1u);

        if(VX_SUCCESS != status)
        {
            printf("Post proc write node send command failed!\n");
        }

       printf("Post proc write node send command success!\n");
    }

    return (status);
}
