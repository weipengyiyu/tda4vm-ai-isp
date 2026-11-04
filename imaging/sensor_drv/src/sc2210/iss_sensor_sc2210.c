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
#include "iss_sensor_SC2210.h"

/*;@@ X3C_1920x1080_60fps_HDR4_LFR_PWL12_mipi600 
; Sensor revision: OX03C10 R1D
; Input clock frequency: 24MHz
; Image output size：1920 x 1080
; Image crop size: 1920 x 1080
; Pixel data format:  PWL12 Raw
; Frame timing and frame rate: 60fps
; System clock frequency: 90 MHz
; Output interface and data rate: MIPI 4Lane, 600Mbps/lanes
; Backend processor: 
; Embedded line: 
; Others: HTS = 2548, VTS = 588，Time_one_double_line = 28.31us
; Core Setting version info: AE08

;;12202021, core =1.17, update OTP auto load and BLC zero line start/number, AE_03
;;20220124, update spd/vs, lcg/spd weighting table based on ae_03, ae_04*/
static IssSensor_CreateParams  SC2210CreatePrms = {
    SENSOR_SC2210,     /*sensor name*/
    0x6,                             /*i2cInstId*/
    {
        0x36, 0, 0, 0,
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
            1,                                  /*num_exposures*/
            vx_false_e,                         /*line_interleaved*/
            {
                {TIVX_RAW_IMAGE_16_BIT, 11},    /*dataFormat and MSB [0] TIVX_RAW_IMAGE_P12_BIT TIVX_RAW_IMAGE_16_BIT Unpacked 16-bit: 1920*1080*2 = 4,147,200 字节*/
            },
            0,                                  /*meta_height_before*/
            0,                                  /*meta_height_after*/
        },
        ISS_SENSOR_SC2210_FEATURES,              /*features*/
        ALGORITHMS_ISS_AEWB_MODE_NONE,          /*aewbMode ALGORITHMS_ISS_AEWB_MODE_NONE ALGORITHMS_ISS_AEWB_MODE_AEWB*/
        30,                                     /*fps*/
        4,                                      /*numDataLanes*/
        {1, 2, 3, 4},                           /*dataLanesMap*/
        {0, 0, 0, 0},                           /*dataLanesPolarity*/
        CSIRX_LANE_BAND_SPEED_720_TO_800_MBPS, /*csi_laneBandSpeed 1920*1080*60*16*1.5/4 = 746,496,000 bits/秒;1 Mbps = 1,000,000 bits/s 746.496 Mbps */
    },
    1,                                  /*numChan 这里跟i2cAddrSensor有关联*/
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
    uint8_t regValue = 0;
    uint32_t sensorConfigSize = 0;
    
    appLogWaitMsecs(100);

    sensorConfigSize = sizeof(ox03c10_linear_4lane_60fps_2m_12bit_init_all) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, i2cAddrSensor, ox03c10_linear_4lane_60fps_2m_12bit_init_all, sensorConfigSize);
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
    /*CSI2 Expansion Interface Reset ('0' - device reset, '1' - normal*/

    status = SC2210_WriteReg(i2cInstId, i2cAddrSensor, 0x0100, 0x01, 1u);
    if (status != 0)
    {
        issLogPrintf("SC2210_StreamOn: SC2210_WriteReg error \n");
        return status;
    }
    status = SC2210_ReadReg(i2cInstId, i2cAddrSensor, 0x0100, &checkValue, 1u);
    if (checkValue != 0x01)
    {
        issLogPrintf("Config : check write regAddr:0x0100 write_regValue:0x01 vs read_checkValue:0x%x, is error !!! status=%d \n", checkValue, status);
    }
    else
    {
        issLogPrintf("Config : check write regAddr:0x0100 write_regValue:0x01 vs read_checkValue:0x%x, is ok !!! status=%d \n", checkValue, status);
    }
    issLogPrintf("----------->SC2210_StreamOn SC2210_WriteReg 0x0100:0x01 streaming is ok \n");
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
    return (0);
}

static int32_t SC2210_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;


    return (status);
}

static int32_t SC2210_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    return (status);
}

static int32_t SC2210_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;
    return (status);
}

static void SC2210_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    return;
}

static void SC2210_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    return;
}

static void SC2210_deinit (uint32_t chId, void *pSensorHdl)
{
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

static int32_t SC2210_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms)
{
    int32_t status = 0;
    return (status);
}

static int32_t SC2210_GetWBPrgFxn(uint32_t chId, void *pSensorHdl, IssAwbDynamicParams *p_awb_dynPrms)
{
    int32_t  status = 0;
    return (status);
}

static int32_t SC2210_SetAwbParams(void *pSensorHdl, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = 0;

    return (status);
}
