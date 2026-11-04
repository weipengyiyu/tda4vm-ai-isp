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
#include "iss_sensor_imx623.h"

static IssSensor_CreateParams  IMX623CreatePrms = {
    SENSOR_IMX623,     /*sensor name*/
    0x6,                             /*i2cInstId*/
    {
        0x1a, 0x1a, 0x1a, 0x1a,
        0,0,0,0
    },/*i2cAddrSensor*/
    {
        0, 0, 0, 0,
        0, 0, 0, 0
    },/*i2cAddrSer*/
    /*IssSensor_Info*/
    {
        {
            1920,                               /*width*/
            1080,                               /*height*/
            1,                              /*num_exposures*/
            (vx_bool)vx_false_e,                     /*line_interleaved*/
            {
                {(uint32_t)TIVX_RAW_IMAGE_16_BIT, 11},    /*dataFormat and MSB [0] TIVX_RAW_IMAGE_P12_BIT TIVX_RAW_IMAGE_16_BIT*/
            },
            0,                                  /*meta_height_before*/
            0,                                  /*meta_height_after*/
        },
        ISS_SENSOR_IMX623_FEATURES,              /*features*/
        ALGORITHMS_ISS_AEWB_MODE_NONE,          /*aewbMode ALGORITHMS_ISS_AEWB_MODE_NONE ALGORITHMS_ISS_AEWB_MODE_AEWB*/
        60,                                     /*fps 60*/
        4,                                      /*numDataLanes*/
        {1, 2, 3, 4},                           /*dataLanesMap*/
        {0, 0, 0, 0},                           /*dataLanesPolarity*/
        CSIRX_LANE_BAND_SPEED_1200_TO_1350_MBPS, /* csi_laneBandSpeed 1920*1080*60*16/4=497664000 CSIRX_LANE_BAND_SPEED_480_TO_560_MBPS  1920*1080*60*16*1.5/4=746496000 1.5余量 CSIRX_LANE_BAND_SPEED_1200_TO_1350_MBPS  */
    },
    3,                                  /*numChan 这里跟i2cAddrSensor有关联*/
    623,                                /*dccId 623*/
};

static IssSensorFxns           IMX623SensorFxns = {
    IMX623_Probe,
    IMX623_Config,
    IMX623_StreamOn,
    IMX623_StreamOff,
    IMX623_PowerOn,
    IMX623_PowerOff,
    IMX623_GetExpParams,
    IMX623_SetAeParams,
    IMX623_GetDccParams,
    IMX623_InitAewbConfig,
    IMX623_GetIspConfig,
    IMX623_ReadWriteReg,
    IMX623_GetExpPrgFxn,
    IMX623_deinit,
    IMX623_GetWBPrgFxn,
    IMX623_SetAwbParams
};

static IssSensorIntfParams     IMX623SensorIntfPrms = {
    0,             /*sensorBroadcast*/
    0,             /*enableFsin*/
    0,			   /*numCamerasStreaming*/
};

IssSensors_Handle IMX623SensorHandle = {
    1,                                 /*isUsed*/
    &IMX623CreatePrms,                /*CreatePrms*/
    &IMX623SensorFxns,                /*SensorFxns*/
    &IMX623SensorIntfPrms,            /*SensorIntfPrms*/
};

/*
 * \brief DCC Parameters of IMX623
 */
//IssCapture_CmplxIoLaneCfg           IMX623Csi2CmplxIoLaneCfg;
extern IssSensors_Handle * gIssSensorTable[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
static uint16_t sp1hGainRegValueOld[ISS_SENSORS_MAX_CHANNEL];
static uint16_t redGain_prev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t greenGain_prev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t blueGain_prev[ISS_SENSORS_MAX_CHANNEL];

int32_t IssSensor_IMX623_Init()
{
    int32_t status;
    int32_t chId;
    status = IssSensor_Register(&IMX623SensorHandle);
    if(0 != status)
    {
        printf("IssSensor_IMX623_Init failed \n");
    }
    printf("IssSensor_IMX623_Init is ok \n");

    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t IMX623_RunSensorConfigScript(uint32_t i2cInstId, uint8_t sensorI2cAddr, const I2cParams *sensorCfg, uint16_t sensor_cfg_script_len)
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

        issLogPrintf(" Configuring IMX623 imager sensorI2cAddr = 0x%x.. Please wait till it finishes \n", sensorI2cAddr);
        while(regCnt<sensor_cfg_script_len)
        {
            status |= IMX623_WriteReg(i2cInstId, sensorI2cAddr, regAddr, regValue, 1u);

            if (0 != status)
            {
                errregCnt++;
                // issLogPrintf(" \n \n IMX623: Sensor Reg Write Failed for regAddr 0x%x \n \n", regAddr);
            }

            if(delayMilliSec > 0)
            {
               appLogWaitMsecs(delayMilliSec);
               issLogPrintf(" \n \n IMX623: regAddr 0x%x, delayMilliSec=%d ms \n \n", regAddr, delayMilliSec);
            }

            regCnt++;
            regAddr  = sensorCfg[regCnt].nRegAddr;
            regValue = sensorCfg[regCnt].nRegValue;
            delayMilliSec = sensorCfg[regCnt].nDelay;
            // appLogWaitMsecs(2);
        }
        /*Wait 100ms after the init is done*/
        appLogWaitMsecs(100);
        issLogPrintf(" IMX623 config done ,config total = %d, errregCnt = %d \n ", regCnt, errregCnt);
    }
    else
    {
        issLogPrintf(" IMX623 config script is NULL \n");
    }
    return status;
}

static int32_t IMX623_Probe(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;
    uint32_t serdesConfigSize = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];
    printf("IMX623_Probe sensorI2cAddr = 0x%x \n", sensorI2cAddr);


    return (status);
}
static int32_t IMX623_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams * pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);
    uint8_t  i2cAddrSensor = pCreatePrms->i2cAddrSensor[chId];
    uint32_t i2cInstId     = pCreatePrms->i2cInstId;
    uint8_t regValue = 0;
    uint32_t serdesConfigSize = 0;
    uint32_t sensorConfigSize = 0;
    
    appLogWaitMsecs(100);

    #if 1
    /*config serdes */
    serdesConfigSize = sizeof(max96712_portb_4lane_init_data_pre) / sizeof(I2cParams);
    status = IMX623_RunSensorConfigScript(i2cInstId, 0x29, max96712_portb_4lane_init_data_pre, serdesConfigSize);
    appLogWaitMsecs(25);

    /*config serdes 0x40 --> 0x44*/
    serdesConfigSize = sizeof(max96717_portb_4lane_init_data_pre_pre) / sizeof(I2cParams);
    status = IMX623_RunSensorConfigScript(i2cInstId, 0x40, max96717_portb_4lane_init_data_pre_pre, serdesConfigSize);
    appLogWaitMsecs(25);

    serdesConfigSize = sizeof(max96717_portb_4lane_init_data_pre) / sizeof(I2cParams);
    status = IMX623_RunSensorConfigScript(i2cInstId, 0x44, max96717_portb_4lane_init_data_pre, serdesConfigSize);

    serdesConfigSize = sizeof(max96712_portb_4lane_init_data_post) / sizeof(I2cParams);
    status = IMX623_RunSensorConfigScript(i2cInstId, 0x29, max96712_portb_4lane_init_data_post, serdesConfigSize);

    serdesConfigSize = sizeof(max96717_portb_4lane_init_data_post) / sizeof(I2cParams);
    status = IMX623_RunSensorConfigScript(i2cInstId, 0x44, max96717_portb_4lane_init_data_post, serdesConfigSize);
    #endif

    appLogWaitMsecs(100);
    if(sensor_features_requested & ISS_SENSOR_FEATURE_COMB_COMP_WDR_MODE)
    {
        sensorConfigSize = sizeof(imx623_raw12_sensor_cfg1) / sizeof(I2cParams);
        status = IMX623_RunSensorConfigScript(i2cInstId, i2cAddrSensor, imx623_raw12_sensor_cfg1, sensorConfigSize);
        sensorConfigSize = sizeof(imx623_raw12_sensor_cfg2) / sizeof(I2cParams);
        status = IMX623_RunSensorConfigScript(i2cInstId, i2cAddrSensor, imx623_raw12_sensor_cfg2, sensorConfigSize);
        sensorConfigSize = sizeof(imx623_raw12_sensor_cfg3) / sizeof(I2cParams);
        status = IMX623_RunSensorConfigScript(i2cInstId, i2cAddrSensor, imx623_raw12_sensor_cfg3, sensorConfigSize);
        sensorConfigSize = sizeof(imx623_raw12_sensor_cfg4) / sizeof(I2cParams);
        status = IMX623_RunSensorConfigScript(i2cInstId, i2cAddrSensor, imx623_raw12_sensor_cfg4, sensorConfigSize);
        sensorConfigSize = sizeof(imx623_raw12_sensor_cfg5) / sizeof(I2cParams);
        status = IMX623_RunSensorConfigScript(i2cInstId, i2cAddrSensor, imx623_raw12_sensor_cfg5, sensorConfigSize);

        if (status != 0)
        {
            issLogPrintf("Config error : IMX623_RunSensorConfigScript is error !!!\n");
            return status;
        }
    }

    issLogPrintf("Config : IMX623_RunSensorConfigScript is ok !!!\n");
    appLogWaitMsecs(100);
    
    return (status);
}

static int32_t IMX623_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    uint32_t sensorConfigSize = 0;
    issLogPrintf("IMX623_StreamOn  -------------------> start  chId:%d\n", chId);
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams * pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    uint8_t checkValue = 0;
    uint8_t  i2cAddrSensor = pCreatePrms->i2cAddrSensor[chId];
    uint32_t i2cInstId     = pCreatePrms->i2cInstId;
    /*CSI2 Expansion Interface Reset ('0' - device reset, '1' - normal*/
    sensorConfigSize = sizeof(imx623_raw12_sensor_cfg6) / sizeof(I2cParams);
    status = IMX623_RunSensorConfigScript(i2cInstId, i2cAddrSensor, imx623_raw12_sensor_cfg6, sensorConfigSize);
    if (status != 0)
    {
        issLogPrintf("IMX623_StreamOn: IMX623_RunSensorConfigScript error \n");
        return status;
    }

    issLogPrintf("----------->IMX623_StreamOn IMX623_WriteReg 0x0100:0x01 streaming is ok \n");
    return (status);
}

static int32_t IMX623_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    return status;
}

static int32_t IMX623_PowerOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    printf("IMX623_PowerOn : chId = 0x%x \n", chId);

    issLogPrintf("IMX623_PowerOn  & sensor  is reset -------------------> end\n");

    return status;
}

static int32_t IMX623_PowerOff(uint32_t chId, void *pSensorHdl)
{
    return (0);
}

static int32_t IMX623_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;

    printf("--------------> IMX623_SetAeParams \n");
    return (status);
}

static int32_t IMX623_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    printf("--------------> IMX623_GetDccParams \n");
    return (status);
}

static int32_t IMX623_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;
    printf("--------------> IMX623_GetExpParams \n");
    return (status);
}

static void IMX623_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    printf("--------------> IMX623_InitAewbConfig \n");
    return;
}

static void IMX623_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    printf("--------------> IMX623_GetIspConfig \n");
    return;
}

static void IMX623_deinit (uint32_t chId, void *pSensorHdl)
{
    printf("--------------> IMX623_deinit \n");
    return;
}

static int32_t IMX623_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg)
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
        status = IMX623_WriteReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, regValue, 1u);
    }
    else
    {
        /*read*/
        status = IMX623_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue, 1u);

        if (0 == status)
        {
            pReg->nRegValue = regValue;
        }
    }
    return (status);
}

static int32_t IMX623_ReadReg(uint8_t     i2cInstId,
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
        issLogPrintf("Error : I2C Timeout while reading from IMX623 register 0x%x \n", regAddr);
    }
    return (status);
}

static int32_t IMX623_WriteReg(uint8_t    i2cInstId,
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
        printf("Error : I2C Timeout while i2cAddr:%x writing 0x%x to IMX623 register 0x%x \n", i2cAddr, regVal, regAddr);
    }

    return (status);
}

static int32_t IMX623_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms)
{
    int32_t status = 0;
    printf("--------------> IMX623_GetExpPrgFxn \n");
    return (status);
}

static int32_t IMX623_GetWBPrgFxn(uint32_t chId, void *pSensorHdl, IssAwbDynamicParams *p_awb_dynPrms)
{
    int32_t  status = 0;
    printf("--------------> IMX623_GetWBPrgFxn \n");
    return (status);
}

static int32_t IMX623_SetAwbParams(void *pSensorHdl, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = 0;

    printf("--------------> IMX623_SetAwbParams \n");
    return (status);
}
