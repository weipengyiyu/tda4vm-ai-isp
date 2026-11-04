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
#include <iss_sensors.h>
#include <iss_sensor_if.h>
#include <iss_sensor_priv.h>
#include <app_remote_service.h>
#include <app_ipc.h>
#include "TI/tivx_mutex.h"

#if defined (SOC_J721E)
#include <ti/board/src/j721e_evm/include/board_cfg.h>
#include <ti/board/src/j721e_evm/include/board_i2c_io_exp.h>
#elif defined (SOC_J721S2)
#include <ti/board/src/j721s2_evm/include/board_cfg.h>
#include <ti/board/src/j721s2_evm/include/board_i2c_io_exp.h>
#elif defined (SOC_J784S4)
#include <ti/board/src/j784s4_evm/include/board_cfg.h>
#include <ti/board/src/j784s4_evm/include/board_i2c_io_exp.h>
#endif

/**
 * @{
 * I2C Addresses for serialisers/Sensors attached to the UB960
 */
#define UB960_SERIALISER_ADDR                (0x18)
#define D3IMX390_SENSOR_ADDR_CM_MODULE       (0x21)
#define D3IMX390_SENSOR_ADDR_RCM_MODULE      (0x1A)
/** @} */

/**
 * @{
 * Generic Alias Addresses for serialisers attached to the UB960 and UB9702 Instance0
 */
#define D3IMX390_INST0_PORT_0_SER_ADDR       (0x74U)
#define D3IMX390_INST0_PORT_1_SER_ADDR       (0x76U)
#define D3IMX390_INST0_PORT_2_SER_ADDR       (0x78U)
#define D3IMX390_INST0_PORT_3_SER_ADDR       (0x7AU)
/** @} */

/**
 * @{
 * Generic Alias Addresses for sensors attached to the UB960 Instance0
 */
#define D3IMX390_INST0_PORT_0_SENSOR_ADDR    (0x40U)
#define D3IMX390_INST0_PORT_1_SENSOR_ADDR    (0x42U)
#define D3IMX390_INST0_PORT_2_SENSOR_ADDR    (0x44U)
#define D3IMX390_INST0_PORT_3_SENSOR_ADDR    (0x46U)
/** @} */

/**
 * @{
 * Generic Alias Addresses for serialisers attached to the UB960 Instance1
 */
#define D3IMX390_INST1_PORT_0_SER_ADDR       (0x6CU)
#define D3IMX390_INST1_PORT_1_SER_ADDR       (0x6EU)
#define D3IMX390_INST1_PORT_2_SER_ADDR       (0x70U)
#define D3IMX390_INST1_PORT_3_SER_ADDR       (0x72U)

/*******************************************************************************
 *  Globals
 *******************************************************************************
 */
IssSensors_Handle *gIssSensorTable[ISS_SENSORS_MAX_SUPPORTED_SENSOR];

#define DS90UB9702

/* Fusion2 board detect flag */
vx_bool gFusion2Det = vx_false_e;
#include <ti/board/src/devices/fpd/ds90ub9702.h>

/*******************************************************************************
 *  Local Functions Declarations
 *******************************************************************************
 */
static int32_t checkForHandle(void* handle);

/*******************************************************************************
 *  Function Definition
 *******************************************************************************
 */
static I2C_Handle gISS_Sensor_I2cHandle = NULL;
static uint8_t gISS_Sensor_ByteOrder = BOARD_I2C_REG_ADDR_MSB_FIRST;
static uint8_t num_sensors_open = 0;
static IssSensors_Handle * g_pSenHndl[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
static uint8_t g_detectedSensors[ISS_SENSORS_MAX_SUPPORTED_SENSOR];

void getIssSensorI2cInfo(uint8_t * byteOrder, I2C_Handle * i2cHndl)
{
    *byteOrder = gISS_Sensor_ByteOrder;
    *i2cHndl = gISS_Sensor_I2cHandle;
}

static int32_t setupI2CInst(uint8_t i2cInst)
{
    int32_t status = -1;
    I2C_Params i2cParams;
    /* Initializes the I2C Parameters */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    /* Configures the I2C instance with the passed parameters*/
    if(NULL == gISS_Sensor_I2cHandle)
    {
       gISS_Sensor_I2cHandle = I2C_open(i2cInst, &i2cParams);
    }

    if(gISS_Sensor_I2cHandle == NULL)
    {
        issLogPrintf(" I2C: ERROR: I2C Open Failed for instance %d !!!\n", i2cInst);
        status = -1;
    }
    else
    {
        status = 0;
    }

    return status;
}


int32_t IssSensor_Init()
{
    int32_t status = -1;
    uint32_t cnt;

    for(cnt=0;cnt<ISS_SENSORS_MAX_SUPPORTED_SENSOR;cnt++)
    {
        g_pSenHndl[cnt] = NULL;
    }

    status = appRemoteServiceRegister(
        IMAGE_SENSOR_REMOTE_SERVICE_NAME,
        ImageSensor_RemoteServiceHandler
    );

    if(status!=0)
    {
        printf(" REMOTE_SERVICE_SENSOR: ERROR: Unable to register remote service sensor handler\n");
           return status;
    }

    for (cnt = 0U; cnt < ISS_SENSORS_MAX_SUPPORTED_SENSOR; cnt ++)
    {
        gIssSensorTable[cnt] = NULL;
        g_detectedSensors[cnt] = 0xFF;
    }

/*
    Call init function of all supported sensors.
    This will register the sensors with the framework
*/
    status |= IssSensor_OVX3C_Init();
    status |= IssSensor_SC2210_Init();
    status |= IssSensor_IMX623_Init();
    // status |= IssSensor_IMX728_Init();
    // status |= IssSensor_IMX390_Init();
    // status |= IssSensor_OVX3C_Init();
    // status |= IssSensor_AR0233_Init();
    // status |= IssSensor_AR0820_Init();
    // status |= IssSensor_rawtestpat_Init();
    // status |= IssSensor_testpat_Init();
    // status |= IssSensor_gw_ar0233_Init();

    return status;
}

int32_t IssSensor_Register(IssSensors_Handle *pSensorHandle)
{
    int32_t status = 0;
    uint32_t cnt = 0U;

    if (NULL == pSensorHandle)
    {
        status = -2;
    }
    else
    {
        /* Find a free entry in the sensor table */
        for (cnt = 0U; cnt < ISS_SENSORS_MAX_SUPPORTED_SENSOR; cnt ++)
        {
            if (NULL == gIssSensorTable[cnt])
            {
                gIssSensorTable[cnt] = pSensorHandle;
                issLogPrintf("Found sensor %s at location %d \n", pSensorHandle->createPrms->name, cnt);
                break;
            }
        }

        if (cnt == ISS_SENSORS_MAX_SUPPORTED_SENSOR)
        {
            printf(" ISS_SENSOR: Could not register sensor \n");
            status = -2;
        }
    }

    return (status);
}

IssSensors_Handle * getSensorHandleFromName(char *name)
{
    uint32_t cnt;
    IssSensors_Handle *pSensorHandle = NULL;
    /* Check For Errors */
    if (NULL == name)
    {
        return NULL;
    }
    else
    {
        for (cnt = 0U; cnt < ISS_SENSORS_MAX_SUPPORTED_SENSOR; cnt ++)
        {
            pSensorHandle = gIssSensorTable[cnt];
            if(NULL == pSensorHandle)
            {
                issLogPrintf("pSensorHandle is NULL \n");
                return NULL;
            }
            if(NULL == pSensorHandle->createPrms)
            {
                issLogPrintf("createPrms is NULL \n");
                return NULL;
            }
            if (0 == strncmp(pSensorHandle->createPrms->name, name, ISS_SENSORS_MAX_NAME))
            {
                break;
            }
        }
    }
    return pSensorHandle;
}

int32_t IssSensor_GetSensorInfo(char *name, IssSensor_CreateParams *sensor_prms)
{
    int32_t status = -1;
    uint32_t cnt;
    IssSensors_Handle *pSensorHandle = NULL;

    /* Check For Errors */
    if ((NULL == sensor_prms) || (NULL == name))
    {
        status = -2;
    }
    else
    {
        for (cnt = 0U; cnt < ISS_SENSORS_MAX_SUPPORTED_SENSOR; cnt ++)
        {
            pSensorHandle = gIssSensorTable[cnt];
            if(NULL == pSensorHandle)
            {
                issLogPrintf("pSensorHandle is NULL \n");
                return -1;
            }
            if(NULL == pSensorHandle->createPrms)
            {
                issLogPrintf("createPrms is NULL \n");
                return -1;
            }
            if (0 == strncmp(pSensorHandle->createPrms->name, name, ISS_SENSORS_MAX_NAME))
            {
                memcpy(sensor_prms, pSensorHandle->createPrms, sizeof(IssSensor_CreateParams));
                status = 0;
                break;
            }
        }
    }
    return (status);
}

IssSensors_Handle * IssSensor_GetSensorHandle(char * name)
{
    uint32_t cnt;
    IssSensors_Handle *pSensorHandle = NULL;
    IssSensor_CreateParams *pCreatePrms = NULL;

    /* Check For Errors */
    if (NULL != name)
    {
        for (cnt = 0U; cnt < ISS_SENSORS_MAX_SUPPORTED_SENSOR; cnt ++)
        {
            pSensorHandle = gIssSensorTable[cnt];
            if(NULL == pSensorHandle)
            {
                return NULL;
            }

            pCreatePrms = pSensorHandle->createPrms;

            if (vx_true_e == pSensorHandle->isUsed)
            {
                if(0 == strncmp(pCreatePrms->name, name, ISS_SENSORS_MAX_NAME))
                {
                    break;
                }
            }
        }
    }
    return (void*)(pSensorHandle);
}

int32_t IssSensor_Delete(void* handle)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;

        memset(pSensorHandle->createPrms, 0, sizeof(IssSensor_CreateParams));
    }

    return (status);
}

int32_t IssSensor_Config(void* handle, uint32_t chId, uint32_t feat)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;
        if(chId < pSensorHandle->createPrms->num_channels)
        {
            if (NULL != pSensorHandle->sensorFxns->config)
            {
                status = pSensorHandle->sensorFxns->config(chId, handle, feat);
            }
        }
        else
        {
            printf("IssSensor_Config Error : Incorrect channel ID %d \n", chId);
            status = -1;
        }
    }

    return (status);
}

/*Probes all the sensors in gIssSensorTable*/
/*If a sensor is detected, the index of the sensor in gIssSensorTable is copied to sensor_id_list*/
static int32_t IssSensor_detect_sensor(uint8_t chId)
{
    int32_t probeStatus = -1;
    int32_t i = 0;
    IssSensors_Handle * pSenHndl;
    int32_t sensor_id_found = -1;

    while ((sensor_id_found < 0) && (i<ISS_SENSORS_MAX_SUPPORTED_SENSOR))
    {
        pSenHndl = gIssSensorTable[i];
        if(NULL !=pSenHndl)
        {
            probeStatus = pSenHndl->sensorFxns->probe(chId, pSenHndl);

            if(probeStatus == 0 )
            {
                /*Sensor found at index i*/
                issLogPrintf("Found sensor %s at port # %d\n", pSenHndl->createPrms->name, chId);
                sensor_id_found = i;
            }
        }
        i++;
    }

    return sensor_id_found;
}

int32_t IssSensor_PowerOn(void* handle, uint32_t chMask)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    num_sensors_open++;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        uint32_t chId = 0;
        pSensorHandle = (IssSensors_Handle *)handle;
        pSensorHandle->sensorIntfPrms->numCamerasStreaming = 0;
        while ((chMask > 0) && (chId < ISS_SENSORS_MAX_SUPPORTED_SENSOR))
        {
            if(chMask & 0x1)
            {
                if (NULL != pSensorHandle->sensorFxns->powerOn)
                {
                    status = pSensorHandle->sensorFxns->powerOn(chId, handle);
                }
            }
            chMask = chMask >> 1;
            chId++;
        }
    }

    return (status);

}

int32_t IssSensor_PowerOff(void* handle, uint32_t chId)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;
        
        if(pSensorHandle->sensorIntfPrms->numCamerasStreaming != 0)
        {
            issLogPrintf("IssSensor_PowerOff : Warning %d cameras are still streaming \n ", pSensorHandle->sensorIntfPrms->numCamerasStreaming);        
        }

        if (NULL != pSensorHandle->sensorFxns->powerOff)
        {
            status = pSensorHandle->sensorFxns->powerOff(chId, handle);
        }
    }

    num_sensors_open--;
    if(0 == num_sensors_open)
    {
        I2C_close(gISS_Sensor_I2cHandle);
        gISS_Sensor_I2cHandle = NULL;
    }

    return (status);

}

int32_t IssSensor_Start(void* handle, uint32_t chId)
{
    int32_t status = -1;
    uint32_t numCamerasEnabled = 0;
    IssSensors_Handle *pSensorHandle = (IssSensors_Handle *)NULL;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;
        if(2U == pSensorHandle->sensorIntfPrms->sensorBroadcast)
        {
            if(0x04 > chId)
            {
                /*Broadcast mode - UB960_0*/
                uint8_t i;
                for(i=0;i<(chId+1);i++)
                {
                    g_pSenHndl[i] = pSensorHandle;
                }
                numCamerasEnabled = chId+1;
            }
            else if(0x08 > chId)
            {
                /*Broadcast mode - UB960_1*/
                uint8_t i;
                for(i=4;i<(chId+1);i++)
                {
                    g_pSenHndl[i] = pSensorHandle;
                }
                numCamerasEnabled = chId-3;
            }
            else if(0x0C /* 12 */ > chId)
            {
                /*Broadcast mode - UB9xx_2*/
                uint8_t i;
                for(i=8;i<ISS_SENSORS_MAX_SUPPORTED_SENSOR;i++)
                {
                    g_pSenHndl[i] = pSensorHandle;
                }
                numCamerasEnabled = chId-7;
            }
        }
        else
        {
            numCamerasEnabled = 1;
        }

        if(NULL != g_pSenHndl[chId])
        {
            issLogPrintf("Warning : channel ID %d already has a registered sensor handle \n", chId);
        }
        g_pSenHndl[chId] = pSensorHandle;

        if(NULL != pSensorHandle->sensorFxns->streamOn)
        {
            status = pSensorHandle->sensorFxns->streamOn(chId, handle);
        }

        if(status == 0)
        {
            pSensorHandle->sensorIntfPrms->numCamerasStreaming += numCamerasEnabled;
        }
    }

    return (status);
}

int32_t IssSensor_Stop(void* handle, uint32_t chId)
{
    int32_t status = -1;
    IssSensors_Handle *pSensorHandle;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;

        if (1) //(chId < pSensorHandle->createPrms->num_channels)
        {
            if(NULL != pSensorHandle->sensorFxns->streamOff)
            {
                status |= pSensorHandle->sensorFxns->streamOff(chId, handle);
            }
            pSensorHandle->sensorIntfPrms->numCamerasStreaming -= 1;
        }
        else
        {
            printf("IssSensor_Stop Error: Incorrect channel ID %d \n", chId);
            status = -1;
        }
    }

    g_pSenHndl[chId] = NULL;

    return (status);
}

int32_t IssSensor_SetAeParams(void *handle, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = -1;
    IssSensors_Handle * pSensorHandle = NULL;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;
        if(pSensorHandle != NULL)
        {
            if(NULL != pSensorHandle->sensorFxns->setAeParams)
            {
                status |= pSensorHandle->sensorFxns->setAeParams(handle, chId, pExpPrms);
            }
        }
        else
        {
            printf("IssSensor_SetAeParams Error: sensor handle is NULL for channel ID %d \n", chId);
            status = -1;
        }
    }

    return (status);
}

int32_t IssSensor_SetAwbParams(void *handle, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = -1;
    IssSensors_Handle * pSensorHandle = NULL;

    /* Check if the handle is valid or not */
    status = checkForHandle(handle);

    if (0 == status)
    {
        pSensorHandle = (IssSensors_Handle *)handle;
        //if(chId < pSensorHandle->createPrms->num_channels)
        if(pSensorHandle != NULL)
        {
            if(NULL != pSensorHandle->sensorFxns->setAwbParams)
            {
                status |= pSensorHandle->sensorFxns->setAwbParams(handle, chId, pWbPrms);
            }
            else
            {
                printf("IssSensor_SetAwbParams Error : Sensor setAwbParams callback is NULL \n");
                memset(pWbPrms, 0x0, sizeof(IssSensor_WhiteBalanceParams));
            }
        }
        else
        {
            printf("IssSensor_SetAwbParams Error : sensor handle is NULL for  channel ID %d \n", chId);
            status = -1;
        }
    }

    return (status);
}

int32_t IssSensor_Control(void* handle, uint32_t cmd, void* cmdArgs, void* cmdRetArgs)
{
    int32_t status = VX_FAILURE;
    uint32_t chId = 0xFF;
    IMAGE_SENSOR_CTRLCMD ctrlCmd;
    uint8_t * cmd_ptr = (uint8_t *)cmdArgs;

    if(NULL == cmd_ptr)
    {
        printf("Error : cmd_ptr is NULL \n");
        return VX_FAILURE;
    }

    cmd_ptr += ISS_SENSORS_MAX_NAME;

    memcpy(&chId, cmd_ptr, sizeof(uint32_t));
    cmd_ptr += sizeof(uint32_t);

    memcpy(&ctrlCmd, cmd_ptr, sizeof(IMAGE_SENSOR_CTRLCMD));
    cmd_ptr += sizeof(IMAGE_SENSOR_CTRLCMD);

    switch(ctrlCmd)
    {
        case IMAGE_SENSOR_CTRLCMD_GETEXPPRG:
            {
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                if(NULL != pSenHndl)
                {
                    status = pSenHndl->sensorFxns->getExpPrg(chId, (void*)pSenHndl, (IssAeDynamicParams *)cmd_ptr);
                }
                else
                {
                    printf("Error : sensor handle is NULL for channel %d \n", chId);
                    status = VX_FAILURE;
                }
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_GETWBCFG:
            {
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                if(NULL != pSenHndl)
                {
                    if(NULL != pSenHndl->sensorFxns)
                    {
                        if(NULL != pSenHndl->sensorFxns->getWbCfg)
                        {
                            status = pSenHndl->sensorFxns->getWbCfg(chId, (void*)pSenHndl, (IssAwbDynamicParams *)cmd_ptr);
                        }else
                        {
                            /*Sensor driver does not support WB update API*/
                            memset(cmd_ptr, 0, sizeof(IssAwbDynamicParams));
                        }
                    }
                }
                else
                {
                    memset(cmd_ptr, -1, sizeof(IssAwbDynamicParams));
                    status = VX_FAILURE;
                }
            }
            break;  
        case IMAGE_SENSOR_CTRLCMD_SETEXPGAIN:
            {
                IssSensor_ExposureParams aePrms;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                if(NULL != pSenHndl)
                {
                    memcpy(&aePrms, cmd_ptr, sizeof(IssSensor_ExposureParams));
                    status = IssSensor_SetAeParams(pSenHndl, chId, &aePrms);
                }
                else
                {
                    printf("Error : sensor handle is NULL for channel %d \n", chId);
                    status = VX_FAILURE;
                }                
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_GETEXPGAIN:
            /*Reserved for future use*/
            status = 0;
            break;
        case IMAGE_SENSOR_CTRLCMD_SETWBGAIN:
            {
                IssSensor_WhiteBalanceParams awbPrms;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                if(NULL != pSenHndl)
                {
                    memcpy(&awbPrms, cmd_ptr, sizeof(IssSensor_WhiteBalanceParams));
                    status = IssSensor_SetAwbParams(pSenHndl, chId, &awbPrms);
                }
                else
                {
                    printf("Error : sensor handle is NULL for channel %d \n", chId);
                    status = VX_FAILURE;
                }         
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_GETWBGAIN:
            /*Reserved for future use*/
            status = 0;
            break;
        case IMAGE_SENSOR_CTRLCMD_DEBUG:
            /*Reserved for future use*/
            status = 0;
            issLogPrintf("IMAGE_SENSOR_CTRLCMD_DEBUG \n");
            {
                uint32_t * ptr32 = (uint32_t * )cmd_ptr;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                uint32_t rw_flag = *ptr32++; /*ReadWrite Flag = 1 for Write*/
                uint32_t devType = *ptr32++; /*Device Type*/
                uint32_t regAddr32 = *ptr32++; /*Register Address*/
                uint32_t regVal32 = *ptr32; /*Register Value*/
                uint32_t slaveI2cAddr = 0x0;
                uint8_t regAddr = 0xBC;
                uint8_t regVal = 0xDE;

                if(0U == devType)
                {
                    slaveI2cAddr = 0x3D ;//pSenHndl->createPrms->i2cAddrDes[chId];
                }
                else if(1U == devType)
                {
                    slaveI2cAddr = pSenHndl->createPrms->i2cAddrSer[chId];
                }
                else if(2U == devType)
                {
                    slaveI2cAddr = pSenHndl->createPrms->i2cAddrSensor[chId];
                }
                else
                {
                    printf("IMAGE_SENSOR_CTRLCMD_DEBUG Error : Unsupported devType %d \n", devType);
                    return -1;
                }

                if(2U == devType)
                {
                    I2cParams sensorI2cParams;
                    sensorI2cParams.nDelay = 1U;
                    sensorI2cParams.nRegAddr = (uint16_t)(regAddr32);
                    sensorI2cParams.nRegValue = (uint16_t)(regVal32);
                    status = pSenHndl->sensorFxns->readWriteReg(chId, pSenHndl, rw_flag, &sensorI2cParams);
                    if(0 != status)
                    {
                        printf("IMAGE_SENSOR_CTRLCMD_DEBUG : Error reading from register 0x%x on the image sensor %s \n", regAddr, pSenHndl->createPrms->name);
                    }
                    else
                    {
                        ptr32 = (uint32_t * )cmdArgs;
                        *ptr32 = (uint32_t)sensorI2cParams.nRegValue;
                    }
                }
                else
                {
                    if(0U == rw_flag)
                    {
                        regAddr = (uint8_t)(regAddr32);
                        status = Board_i2c8BitRegRd(gISS_Sensor_I2cHandle, slaveI2cAddr, regAddr, &regVal, 1U, SENSOR_I2C_TIMEOUT);
                        if(0 != status)
                        {
                            printf("IMAGE_SENSOR_CTRLCMD_DEBUG : Error reading from register 0x%x on slave device 0x%x \n", regAddr, slaveI2cAddr);
                        }
                        issLogPrintf("IMAGE_SENSOR_CTRLCMD_DEBUG : Read 0x%x from register 0x%x on slave device 0x%x \n", regVal, regAddr, slaveI2cAddr);
                    }
                    else if (1U == rw_flag)
                    {
                        regAddr = (uint8_t)(regAddr32);
                        regVal =  (uint8_t)(regVal32);
                        status = Board_i2c8BitRegWr(gISS_Sensor_I2cHandle, slaveI2cAddr, regAddr, &regVal, 1U, SENSOR_I2C_TIMEOUT);
                        if(0 != status)
                        {
                            printf("IMAGE_SENSOR_CTRLCMD_DEBUG : Error writing 0x%x to register 0x%x on slave device 0x%x \n", regVal, regAddr, slaveI2cAddr);
                        }
                    }
                    else
                    {
                        status = -1;
                        printf("IMAGE_SENSOR_CTRLCMD_DEBUG Error : Invalid rwflag = %d \n", rw_flag);
                    }

                    if(0 == status)
                    {
                        ptr32 = (uint32_t * )cmdArgs;
                        *ptr32 = (uint32_t)regVal;
                    }
                }
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_READ_SENSOR_REG:
            {
                uint32_t * ptr32 = (uint32_t * )cmd_ptr;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                I2cParams reg_read;
                reg_read.nDelay = 0;
                reg_read.nRegAddr = (uint16_t)(*ptr32);
                ptr32++;
                reg_read.nRegValue = 0xFF;
                status = pSenHndl->sensorFxns->readWriteReg(chId, pSenHndl, 0, &reg_read);
                *ptr32 = reg_read.nRegValue;
            }
            break;
        case IMAGE_SENSOR_CTRLCMD_WRITE_SENSOR_REG:
            {
                uint32_t * ptr32 = (uint32_t * )cmd_ptr;
                IssSensors_Handle * pSenHndl = g_pSenHndl[chId];
                I2cParams reg_write;
                reg_write.nDelay = 0;
                reg_write.nRegAddr = (uint16_t)(*ptr32);
                ptr32++;
                reg_write.nRegValue = (uint16_t)(*ptr32);
                status = pSenHndl->sensorFxns->readWriteReg(chId, pSenHndl, 1, &reg_write);
                *ptr32 = reg_write.nRegValue;
            }
            break;
        default:
            status = -1;
            printf("IMAGE_SENSOR_CTRLCMD_DEBUG Error : Unknown control command %d \n", ctrlCmd);
            break;
    }

    return (status);
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t checkForHandle(void* handle)
{
    int32_t found = -1;
    uint32_t cnt;

    /* Find a free entry in the sensor table */
    for (cnt = 0U; cnt < ISS_SENSORS_MAX_SUPPORTED_SENSOR; cnt ++)
    {
        if (handle == (void*)gIssSensorTable[cnt])
        {
            found = 0;
            break;
        }
    }

    return (found);
}

void IssSensor_CloseHandle()
{
        I2C_close(gISS_Sensor_I2cHandle);
        gISS_Sensor_I2cHandle = NULL;
}

int32_t ImageSensor_RemoteServiceHandler(char *service_name, uint32_t cmd,
    void *prm, uint32_t prm_size, uint32_t flags)
{
    int32_t status = -1;
    uint16_t cmdMemoryNeeded = 0;
    uint8_t * cmd_param = (uint8_t * )prm;
    uint8_t chId;
    uint32_t sensor_features_requested = 0;
    uint32_t channel_mask = 0;
    //uint32_t channel_mask_supported = 0;
    char * sensor_name = NULL;
    IssSensors_Handle * pSenHndl = NULL;
    IssSensor_CreateParams * pSenParams = (IssSensor_CreateParams * )NULL;

    switch(cmd)
    {
        case IM_SENSOR_CMD_ENUMERATE:
            // issLogPrintf("-----0119----ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_ENUMERATE \n");
            cmdMemoryNeeded = ISS_SENSORS_MAX_NAME*ISS_SENSORS_MAX_SUPPORTED_SENSOR;
            if(prm_size < cmdMemoryNeeded)
            {
                printf("Error : Insufficient prm size %d, need at least %d \n", prm_size, cmdMemoryNeeded);
                return -1;
            }
            
            // IssSensor_DeserializerInit();
            status = setupI2CInst(6);
            if(status!=0)
            {
                printf(" I2C ERROR \n");
                return status;
            }

            {
                uint8_t count;
                for(count=0;count<ISS_SENSORS_MAX_SUPPORTED_SENSOR;count++)
                {
                    pSenHndl = gIssSensorTable[count];
                    if(NULL !=pSenHndl)
                    {
                        memcpy(cmd_param + (count*ISS_SENSORS_MAX_NAME), pSenHndl->createPrms->name, ISS_SENSORS_MAX_NAME);
                    }
                }
            }
            status = 0;

            break;
        case IM_SENSOR_CMD_QUERY:
            sensor_name = (char*)(cmd_param);
            issLogPrintf("Received Query for %s \n", sensor_name);

            /*Copy sensor properties at prm, after sensor name*/
            pSenParams = (IssSensor_CreateParams * )(cmd_param+ISS_SENSORS_MAX_NAME);
            status = IssSensor_GetSensorInfo(sensor_name, pSenParams);
            break;
        case IM_SENSOR_CMD_PWRON:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_PWRON \n");
            sensor_name = (char*)(cmd_param);
            memcpy(&channel_mask, (cmd_param+ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            issLogPrintf("IM_SENSOR_CMD_PWRON : channel_mask = 0x%x \n", channel_mask);
            pSenHndl = getSensorHandleFromName(sensor_name);
            if(NULL == pSenHndl)
            {
                status = -1;
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            else
            {
                status = IssSensor_PowerOn((void*)pSenHndl, channel_mask);
            }
            break;
        case IM_SENSOR_CMD_CONFIG:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_CONFIG \n");
            sensor_name = (char*)(cmd_param);
            memcpy(&sensor_features_requested, (cmd_param+ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            issLogPrintf("Application requested features = 0x%x \n ", sensor_features_requested);
            memcpy(&channel_mask, (cmd_param+ISS_SENSORS_MAX_NAME+sizeof(uint32_t)), sizeof(uint32_t));
            pSenHndl = getSensorHandleFromName(sensor_name);
            if(NULL == pSenHndl)
            {
                status = -1;
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            else
            {
                status = 0;
                int32_t probeStatus;
                //channel_mask_supported = (1<<pSenHndl->createPrms->num_channels) - 1;
                //channel_mask &= channel_mask_supported;
                issLogPrintf("IM_SENSOR_CMD_CONFIG channel_mask = %x\n", channel_mask);

                if(0 != channel_mask)
                {
                    chId = 0;
                    while( (channel_mask > 0) && (chId < ISS_SENSORS_MAX_CHANNEL) )
                    {
                        if((channel_mask & 0x1) == 0x1)
                        {
                            probeStatus = pSenHndl->sensorFxns->probe(chId, pSenHndl);
                            if(probeStatus < 0)
                            {
                                printf("Error : sensor probe failed for channel %d \n ", chId);
                                status |= probeStatus;
                             }
                             else
                             {
                                issLogPrintf("Configuring camera # %d \n", chId);
                                status |= IssSensor_Config((void*)pSenHndl, chId, sensor_features_requested);
                             }
                        }
                        chId++;
                        channel_mask = channel_mask >> 1;
                    }
                }
            }
            issLogPrintf("IM_SENSOR_CMD_CONFIG returning status = %d\n", status);

            break;
        case IM_SENSOR_CMD_STREAM_ON:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_STREAM_ON \n");
            sensor_name = (char*)(cmd_param);
            memcpy(&channel_mask, (cmd_param+ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            pSenHndl = getSensorHandleFromName(sensor_name);
            if(NULL == pSenHndl)
            {
                status = -1;
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            else
            {
                issLogPrintf("IM_SENSOR_CMD_STREAM_ON:  channel_mask = 0x%x\n", channel_mask);
                status = 0;
                chId = 0;
                while( (channel_mask > 0) && (chId < ISS_SENSORS_MAX_CHANNEL) )
                {
                    if(channel_mask & 0x1)
                    {
                        status |= IssSensor_Start((void*)pSenHndl, chId);
                        if(status < 0)
                        {
                            printf("Error : Failed to start sensor at channel Id %d \n", chId);
                        }
                        if( NULL ==g_pSenHndl[chId])
                        {
                            printf("Error : sensor handle at channel Id %d = NULL \n", chId);
                        }
                    }

                    chId++;
                    channel_mask = channel_mask >> 1U;
                }
            }

            break;
        case IM_SENSOR_CMD_STREAM_OFF:
            issLogPrintf("ImageSensor_RemoteServiceHandler: IM_SENSOR_CMD_STREAM_OFF \n");

            sensor_name = (char*)(cmd_param);
            memcpy(&channel_mask, (cmd_param+ISS_SENSORS_MAX_NAME), sizeof(uint32_t));
            pSenHndl = getSensorHandleFromName(sensor_name);
            if(NULL == pSenHndl)
            {
                status = -1;
                printf("ERROR : NULL handle returned for sensor %s \n", sensor_name);
            }
            else
            {
                issLogPrintf("IM_SENSOR_CMD_STREAM_ON:  channel_mask = 0x%x\n", channel_mask);
                status = 0;

                chId = 0;

                while( (channel_mask > 0) && (chId < ISS_SENSORS_MAX_CHANNEL) )
                {
                    if(channel_mask & 0x1)
                    {
                        status = IssSensor_Stop((void*)pSenHndl, chId);
                        if(status < 0)
                        {
                            printf("Warning : Failed to stop sensor at channel Id %d \n", chId);
                        }
                        if( NULL !=g_pSenHndl[chId])
                        {
                            printf("Warning : sensor handle at channel Id %d is not NULL \n", chId);
                        }
                    }

                    chId++;
                    channel_mask = channel_mask >> 1U;
                }
            }
            break;
        case IM_SENSOR_CMD_PWROFF:
            IssSensor_CloseHandle();
            status = 0;
            break;
        case IM_SENSOR_CMD_CTL:
            status = IssSensor_Control(NULL, 0, (void*)cmd_param, NULL);
            break;
        case IM_SENSOR_CMD_DETECT:
            {/*避免重复探测已知传感器,按位解析通道掩码,区分“无串行器” vs “有串行器但无传感器”*/
                uint8_t i = 0;
                uint8_t channel_id = 0;
                int32_t probeStatus;
                uint8_t serializers_detected = 0;
                memcpy(&channel_mask, cmd_param, sizeof(uint32_t));

                while(channel_mask > 0)
                {
                    if(channel_mask & 0x1)
                    {
                        int32_t detectedSensor;
                        int32_t tmp = (1<<channel_id);
                        if(tmp&serializers_detected)
                        {
                            if(g_detectedSensors[channel_id] == 0xFF)
                            {
                                /*Sensor has not been detected at this port yet. Try to detect */
                                detectedSensor = IssSensor_detect_sensor(channel_id);
                                if(detectedSensor < 0)
                                {
                                    issLogPrintf("IM_SENSOR_CMD_DETECT found serializer but no sensor at chId %d \n", chId);
                                    detectedSensor = 0xFF;
                                }
                                else
                                {
                                    issLogPrintf("IM_SENSOR_CMD_DETECT : Found sensor %d at port %d \n", detectedSensor, channel_id);
                                }
                                g_detectedSensors[channel_id] = (uint8_t)detectedSensor;
                                cmd_param[channel_id] = detectedSensor;
                            }
                            else
                            {
                                /*Sensor has previously been detected at this port. It might be getting used.*/
                                /*Trying to probe can overwrite the configuration*/
                                /*Return the previously detected ID instead of probing again*/
                                issLogPrintf("IM_SENSOR_CMD_DETECT : Previously found sensor %d at port %d \n", g_detectedSensors[chId], chId);
                                cmd_param[channel_id] = (uint8_t)g_detectedSensors[channel_id];
                            }
                        }else
                        {
                            issLogPrintf("IM_SENSOR_CMD_DETECT No serializer found at port %d \n", channel_id);
                            cmd_param[channel_id] = 0xFF;
                        }
                    }
                    else
                    {
                        cmd_param[channel_id] = 0xFF;
                    }
                    channel_mask = channel_mask >> 1;
                    channel_id++;
                }
                status = 0;
            }
            break;
        default:
            issLogPrintf("ImageSensor_RemoteServiceHandler : Unsupported command : %d\n", cmd);
            status = -1;
    }

    return status;
}

int32_t IssSensor_DeInit()
{
    int32_t status = -1;
    status = appRemoteServiceUnRegister(IMAGE_SENSOR_REMOTE_SERVICE_NAME);
    if(status!=0)
    {
        printf(" REMOTE_SERVICE_SENSOR: ERROR: Unable to unregister remote service sensor handler\n");
    }
    return status;
}

