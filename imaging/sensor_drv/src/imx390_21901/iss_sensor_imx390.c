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
#include "iss_sensor_imx390.h"

static IssSensor_CreateParams  imx390CreatePrms = {
    SENSOR_SONY_IMX390_UB953_D3,     /*sensor name*/
    0x6,                             /*i2cInstId*/
    {
        0, 0, 0x1a, 0
    },/*i2cAddrSensor*/
    {
         0, 0, 0, 0
    },/*i2cAddrSer*/
    /*IssSensor_Info*/
    {
        {
            IMX390_OUT_WIDTH,               /*width*/
            IMX390_OUT_HEIGHT-IMX390_META_HEIGHT_AFTER,            /*height*/
            1,                              /*num_exposures*/
            vx_false_e,                     /*line_interleaved*/
            {
                {TIVX_RAW_IMAGE_16_BIT, 11},    /*dataFormat and MSB [0]*/
            },
            0,                              /*meta_height_before*/
            IMX390_META_HEIGHT_AFTER,      /*meta_height_after*/
        },
        ISS_SENSOR_IMX390_FEATURES,     /*features*/
        ALGORITHMS_ISS_AEWB_MODE_NONE,  /*aewbMode*/
        60,                             /*fps*/
        4,                              /*numDataLanes*/
        {1, 2, 3, 4},                   /*dataLanesMap*/
        {1, 1, 1, 1},                   /*dataLanesPolarity*/
        CSIRX_LANE_BAND_SPEED_880_TO_1040_MBPS, /*csi_laneBandSpeed*/
    },
    3,                                  /*numChan*/
    390,                                /*dccId*/
};

static IssSensorFxns           im390SensorFxns = {
    IMX390_Probe,
    IMX390_Config,
    IMX390_StreamOn,
    IMX390_StreamOff,
    IMX390_PowerOn,
    IMX390_PowerOff,
    IMX390_GetExpParams,
    IMX390_SetAeParams,
    IMX390_GetDccParams,
    IMX390_InitAewbConfig,
    IMX390_GetIspConfig,
    IMX390_ReadWriteReg,
    IMX390_GetExpPrgFxn,
    IMX390_deinit,
    IMX390_GetWBPrgFxn,
    IMX390_SetAwbParams
};

static IssSensorIntfParams     imx390SensorIntfPrms = {
    0,             /*sensorBroadcast*/
    0,             /*enableFsin*/
    0,			   /*numCamerasStreaming*/
};

IssSensors_Handle imx390SensorHandle = {
    1,                                 /*isUsed*/
    &imx390CreatePrms,                /*CreatePrms*/
    &im390SensorFxns,                /*SensorFxns*/
    &imx390SensorIntfPrms,            /*SensorIntfPrms*/
};

/*
 * \brief DCC Parameters of IMX390
 */
//IssCapture_CmplxIoLaneCfg           imx390Csi2CmplxIoLaneCfg;

int32_t IssSensor_IMX390_Init()
{
    int32_t status;
    int32_t chId;
    status = IssSensor_Register(&imx390SensorHandle);
    if(0 != status)
    {
        printf("IssSensor_IMX390_Init failed \n");
    }

    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t IMX390_Probe(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    return (status);
}

static int32_t IMX390_Sensor_RegConfig(uint32_t i2cInstId, uint8_t sensorI2cAddr, const I2cParams *sensorCfg, uint16_t sensor_cfg_script_len)
{
    int32_t status = 0;
    uint16_t regAddr;
    uint8_t regValue;
    uint16_t delayMilliSec;
    uint32_t regCnt;

    if(NULL != sensorCfg)
    {
        regCnt = 0;
        regAddr  = sensorCfg[regCnt].nRegAddr;
        regValue = sensorCfg[regCnt].nRegValue;
        delayMilliSec = sensorCfg[regCnt].nDelay;

        printf(" Configuring IMX390 imager 0x%x.. Please wait till it finishes \n", sensorI2cAddr);
        while(regCnt<sensor_cfg_script_len)
        {
            status |= IMX390_WriteReg(i2cInstId, sensorI2cAddr, regAddr, regValue, 1u);

            if (0 != status)
            {
                printf(" \n \n IMX390: Sensor Reg Write Failed for regAddr 0x%x \n \n", regAddr);
            }

            if(delayMilliSec > 0)
            {
               appLogWaitMsecs(delayMilliSec);
            }

            regCnt++;
            regAddr  = sensorCfg[regCnt].nRegAddr;
            regValue = sensorCfg[regCnt].nRegValue;
            delayMilliSec = sensorCfg[regCnt].nDelay;
        }
        /*Wait 100ms after the init is done*/
        appLogWaitMsecs(100);
        //printf(" IMX390 config done \n ");
    }
    else
    {
        printf(" IMX390 config script is NULL \n");
    }
    return status;
}

static int32_t IMX390_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested)
{
    int32_t status = 0;
    uint32_t i2cInstId;
    uint16_t sensor_cfg_script_len = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    uint8_t sensorI2cAddr;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    /*Deserializer config is done in IssSensor_PowerOn, Need to set sensor alias*/
    // 0x34 >> 1 = 0x1a 

    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];
    sensor_cfg_script_len = sizeof(imx390_linear_2lane_60fps_2m_12bit_init_all) / sizeof(I2cParams);
    status = IMX390_Sensor_RegConfig(i2cInstId, sensorI2cAddr, imx390_linear_2lane_60fps_2m_12bit_init_all, sensor_cfg_script_len);

    return (status);
}

static int32_t IMX390_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);


    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    printf(" IMX390 config sensorI2cAddr is = %d \n", sensorI2cAddr);
    status |= IMX390_WriteReg(i2cInstId, sensorI2cAddr, 0x0, 0x0, 1u);/*ACTIVE*/
    appLogWaitMsecs(10);
 
    return (status);
}

static int32_t IMX390_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];

    status |= IMX390_WriteReg(i2cInstId, sensorI2cAddr, 0x0, 0x1, 1u);/*STANDBY*/
    appLogWaitMsecs(10);

    return status;
}

static int32_t IMX390_PowerOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    printf("IMX390_PowerOn : chId = 0x%x \n", chId);

    return status;
}

static int32_t IMX390_PowerOff(uint32_t chId, void *pSensorHdl)
{
    return (0);
}

static int32_t IMX390_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    uint16_t regAddr;
    uint16_t cnt;
    uint8_t regValue;
    int32_t status = -1;
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    IssSensor_CreateParams * pCreatePrms;
    uint32_t i2cInstId;
    uint8_t sensorI2cAddr;
    uint32_t sp1h_again = 0U;

    assert(NULL != pSenHandle);
    pCreatePrms = pSenHandle->createPrms;
    assert(NULL != pCreatePrms);

    i2cInstId = pCreatePrms->i2cInstId;
    sensorI2cAddr = pCreatePrms->i2cAddrSensor[chId];


    return (status);
}

static int32_t IMX390_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    return (status);
}

static int32_t IMX390_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;

    assert(NULL != pExpPrms);

    return (status);
}

static void IMX390_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    return;
}

static void IMX390_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    return;
}

static void IMX390_deinit (uint32_t chId, void *pSensorHdl)
{
    return;
}

static int32_t IMX390_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg)
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
        status = IMX390_WriteReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, regValue, 1u);
    }
    else
    {
        /*read*/
        status = IMX390_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue, 1u);

        if (0 == status)
        {
            pReg->nRegValue = regValue;
        }
    }
    return (status);
}

static int32_t IMX390_ReadReg(uint8_t     i2cInstId,
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
        issLogPrintf("Error : I2C Timeout while reading from IMX390 register 0x%x \n", regAddr);
    }
    return (status);
}

static int32_t IMX390_WriteReg(uint8_t    i2cInstId,
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
        printf("Error : I2C Timeout while writing 0x%x to IMX390 register 0x%x \n", regVal, regAddr);
    }

    return (status);
}

static int32_t IMX390_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms)
{
    int32_t status = 0;
    // uint8_t count = 0;

    // p_ae_dynPrms->targetBrightnessRange.min = DEFAULT_TARGET_BRIGHTNESS_MIN;
    // p_ae_dynPrms->targetBrightnessRange.max = DEFAULT_TARGET_BRIGHTNESS_MAX;
    // p_ae_dynPrms->targetBrightness = DEFAULT_TARGET_BRIGHTNESS;
    // p_ae_dynPrms->threshold = DEFAULT_TARGET_THRESHOLD;
    // p_ae_dynPrms->exposureTimeStepSize = DEFAULT_EXPOSURE_TIME;
    // p_ae_dynPrms->enableBlc = DEFAULT_ENABLE_BLC;

    // p_ae_dynPrms->exposureTimeRange[count].min = 11000;
    // p_ae_dynPrms->exposureTimeRange[count].max = 11000;
    // p_ae_dynPrms->analogGainRange[count].min = 1024;
    // p_ae_dynPrms->analogGainRange[count].max = 8192;
    // p_ae_dynPrms->digitalGainRange[count].min = 256;
    // p_ae_dynPrms->digitalGainRange[count].max = 256;
    // count++;

    // p_ae_dynPrms->numAeDynParams = count;
    return (status);
}

static int32_t IMX390_GetWBPrgFxn(uint32_t chId, void *pSensorHdl, IssAwbDynamicParams *p_awb_dynPrms)
{
    int32_t  status = 0;

    p_awb_dynPrms->redGainRange.min = 512;
    p_awb_dynPrms->redGainRange.max = 2048;

    p_awb_dynPrms->greenGainRange.min = 512;
    p_awb_dynPrms->greenGainRange.max = 2048;

    p_awb_dynPrms->blueGainRange.min = 512;
    p_awb_dynPrms->blueGainRange.max = 2048;

    p_awb_dynPrms->sensor_pre_gain = 0;

    printf("IMX390_GetWBPrgFxn: sensor_pre_gain = %d \n", p_awb_dynPrms->sensor_pre_gain);
    return (status);
}

static int32_t IMX390_SetAwbParams(void *pSensorHdl, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = 0;

    return (status);
}
