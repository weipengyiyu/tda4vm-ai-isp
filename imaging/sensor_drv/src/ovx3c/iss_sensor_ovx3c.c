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
#include "iss_sensor_ovx3c.h"

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
static IssSensor_CreateParams  OVX3CCreatePrms = {
    SENSOR_OVX3C,     /*sensor name*/
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
        ISS_SENSOR_OVX3C_FEATURES,              /*features*/
        ALGORITHMS_ISS_AEWB_MODE_NONE,          /*aewbMode ALGORITHMS_ISS_AEWB_MODE_NONE ALGORITHMS_ISS_AEWB_MODE_AEWB*/
        30,                                     /*fps 60:ox03c10_linear_4lane_60fps_2m_12bit_init_all; 30:ox03c10_linear_4lane_30fps_2m_12bit_init_all*/
        4,                                      /*numDataLanes*/
        {1, 2, 3, 4},                           /*dataLanesMap*/
        {0, 0, 0, 0},                           /*dataLanesPolarity*/
        CSIRX_LANE_BAND_SPEED_320_TO_360_MBPS, /*CSIRX_LANE_BAND_SPEED_320_TO_360_MBPS;CSIRX_LANE_BAND_SPEED_720_TO_800_MBPS:csi_laneBandSpeed 1920*1080*60*16*1.5/4 = 746,496,000 bits/秒;1 Mbps = 1,000,000 bits/s 746.496 Mbps */
    },
    1,                                  /*numChan 这里跟i2cAddrSensor有关联*/
    333,                                /*dccId 333*/
};

static IssSensorFxns           Ovx3cSensorFxns = {
    OVX3C_Probe,
    OVX3C_Config,
    OVX3C_StreamOn,
    OVX3C_StreamOff,
    OVX3C_PowerOn,
    OVX3C_PowerOff,
    OVX3C_GetExpParams,
    OVX3C_SetAeParams,
    OVX3C_GetDccParams,
    OVX3C_InitAewbConfig,
    OVX3C_GetIspConfig,
    OVX3C_ReadWriteReg,
    OVX3C_GetExpPrgFxn,
    OVX3C_deinit,
    OVX3C_GetWBPrgFxn,
    OVX3C_SetAwbParams
};

static IssSensorIntfParams     OVX3CSensorIntfPrms = {
    0,             /*sensorBroadcast*/
    0,             /*enableFsin*/
    0,			   /*numCamerasStreaming*/
};

IssSensors_Handle OVX3CSensorHandle = {
    1,                                 /*isUsed*/
    &OVX3CCreatePrms,                /*CreatePrms*/
    &Ovx3cSensorFxns,                /*SensorFxns*/
    &OVX3CSensorIntfPrms,            /*SensorIntfPrms*/
};

/*
 * \brief DCC Parameters of OVX3C
 */
//IssCapture_CmplxIoLaneCfg           OVX3CCsi2CmplxIoLaneCfg;
extern IssSensors_Handle * gIssSensorTable[ISS_SENSORS_MAX_SUPPORTED_SENSOR];
static uint16_t sp1hGainRegValueOld[ISS_SENSORS_MAX_CHANNEL];
static uint16_t redGain_prev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t greenGain_prev[ISS_SENSORS_MAX_CHANNEL];
static uint16_t blueGain_prev[ISS_SENSORS_MAX_CHANNEL];

int32_t IssSensor_OVX3C_Init()
{
    int32_t status;
    int32_t chId;
    status = IssSensor_Register(&OVX3CSensorHandle);
    if(0 != status)
    {
        printf("IssSensor_OVX3C_Init failed \n");
    }
    printf("IssSensor_OVX3C_Init is ok \n");

    return status;
}

/*******************************************************************************
 *  Local Functions Definition
 *******************************************************************************
 */

static int32_t OVX3C_Probe(uint32_t chId, void *pSensorHdl)
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
            status |= OVX3C_WriteReg(i2cInstId, sensorI2cAddr, regAddr, regValue, 1u);

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
            // /*check write is ok*/
            // status = OVX3C_ReadReg(i2cInstId, sensorI2cAddr, regAddr, &checkValue, 1u);
            // if (checkValue != regValue)
            // {
            //     issLogPrintf("Config : check write regAddr:%x write_regValue:0x%x vs read_checkValue:0x%x, is error !!! status=%d \n",regAddr, regValue, checkValue, status);
            //     // return status;
            // }
            // else
            // {
            //     issLogPrintf("Config : check write regAddr:%x write_regValue:0x%x vs read_checkValue:0x%x, is ok !!! status=%d \n",regAddr, regValue, checkValue, status);
            // }
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

static uint32_t OVX3CFeaturesEnabled;
static int32_t OVX3C_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested)
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
#if 0
    uint32_t serdesConfigSize = 0;
    /*config serdes */
    serdesConfigSize = sizeof(max96712_portb_4lane_init_data_pre) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, 0x29, max96712_portb_4lane_init_data_pre, serdesConfigSize);

    serdesConfigSize = sizeof(max96717_portb_4lane_init_data_pre_pre) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, 0x42, max96717_portb_4lane_init_data_pre_pre, serdesConfigSize);

    serdesConfigSize = sizeof(max96717_portb_4lane_init_data_pre) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, 0x44, max96717_portb_4lane_init_data_pre, serdesConfigSize);

    serdesConfigSize = sizeof(max96712_portb_4lane_init_data_post) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, 0x29, max96712_portb_4lane_init_data_post, serdesConfigSize);

    serdesConfigSize = sizeof(max96717_portb_4lane_init_data_post) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, 0x44, max96717_portb_4lane_init_data_post, serdesConfigSize);

    appLogWaitMsecs(100);
#endif

    #if 1
    sensorConfigSize = sizeof(ox03c10_linear_4lane_60fps_2m_12bit_init_all) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, i2cAddrSensor, ox03c10_linear_4lane_60fps_2m_12bit_init_all, sensorConfigSize);
    #else
    sensorConfigSize = sizeof(ox03c10_linear_4lane_30fps_2m_12bit_init_all) / sizeof(I2cParams);
    status = OVC3C_RunSensorConfigScript(i2cInstId, i2cAddrSensor, ox03c10_linear_4lane_30fps_2m_12bit_init_all, sensorConfigSize);
    #endif

    if (status != 0)
    {
        issLogPrintf("Config error : OVC3C_RunSensorConfigScript is error !!!\n");
        return status;
    }
    issLogPrintf("Config : OVC3C_RunSensorConfigScript is ok !!!\n");
    appLogWaitMsecs(100);
    
    return (status);
}

static int32_t OVX3C_StreamOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    issLogPrintf("OVX3C_StreamOn  -------------------> start  chId:%d\n", chId);
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams * pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);

    uint8_t checkValue = 0;
    uint8_t  i2cAddrSensor = pCreatePrms->i2cAddrSensor[chId];
    uint32_t i2cInstId     = pCreatePrms->i2cInstId;
    /*CSI2 Expansion Interface Reset ('0' - device reset, '1' - normal*/

    status = OVX3C_WriteReg(i2cInstId, i2cAddrSensor, 0x0100, 0x01, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_StreamOn: OVX3C_WriteReg error \n");
        return status;
    }
    status = OVX3C_ReadReg(i2cInstId, i2cAddrSensor, 0x0100, &checkValue, 1u);
    if (checkValue != 0x01)
    {
        issLogPrintf("Config : check write regAddr:0x0100 write_regValue:0x01 vs read_checkValue:0x%x, is error !!! status=%d \n", checkValue, status);
    }
    else
    {
        issLogPrintf("Config : check write regAddr:0x0100 write_regValue:0x01 vs read_checkValue:0x%x, is ok !!! status=%d \n", checkValue, status);
    }
    issLogPrintf("----------->OVX3C_StreamOn OVX3C_WriteReg 0x0100:0x01 streaming is ok \n");
    return (status);
}

static int32_t OVX3C_StreamOff(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;

    return status;
}

static int32_t OVX3C_PowerOn(uint32_t chId, void *pSensorHdl)
{
    int32_t status = 0;
    printf("OVX3C_PowerOn : chId = 0x%x \n", chId);
#if 0
    issLogPrintf("OVX3C_PowerOn  -------------------> start  chId:%d\n", chId);
    IssSensors_Handle * pSenHandle = (IssSensors_Handle*)pSensorHdl;
    assert(pSenHandle != NULL);
    IssSensor_CreateParams * pCreatePrms = pSenHandle->createPrms;
    assert(pCreatePrms != NULL);
    uint8_t regVal = 0;
    // uint8_t  i2cAddrSensor = pCreatePrms->i2cAddrSensor[chId];
    uint32_t i2cInstId     = pCreatePrms->i2cInstId;
    /*CSI2 Expansion Interface Reset ('0' - device reset, '1' - normal*/
    status = OVX3C_ReadReg(i2cInstId, 0x20, 0x00, &regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: addr:%x OVX3C_WriteReg =%x error \n", 0x20, regVal);
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn: OVX3C_ReadReg 0x20: 0x00:%x \n", regVal);
    status = OVX3C_ReadReg(i2cInstId, 0x20, 0x01, &regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: addr:%x OVX3C_WriteReg =%x error \n", 0x20, regVal);
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn: OVX3C_ReadReg 0x20: 0x00:%x \n", regVal);
    status = OVX3C_ReadReg(i2cInstId, 0x20, 0x02, &regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: addr:%x OVX3C_WriteReg =%x error \n", 0x20, regVal);
        return status;
    }
    issLogPrintf("-----default=1------>OVX3C_PowerOn:addr:0x20 OVX3C_ReadReg : 0x02:%x \n", regVal);
    status = OVX3C_ReadReg(i2cInstId, 0x20, 0x03, &regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: addr:%x OVX3C_WriteReg =%x error \n", 0x20, regVal);
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn: OVX3C_ReadReg 0x20: 0x00:%x \n", regVal);

    /*// Step 1: 配置 P0 为输出 (C-0 = 0)
I2C_Write(0x20, 0x03, 0xFE); // 0b11111110 → C-0 = 0

// Step 2: 拉低 P0，触发复位
I2C_Write(0x20, 0x01, 0x00); // O-0 = 0 → RSTz = LOW

// Delay for reset duration (e.g., 1ms)
delay(1);

// Step 3: 拉高 P0，解除复位
I2C_Write(0x20, 0x01, 0x01); // O-0 = 1 → RSTz = HIGH*/
    regVal = 0xFE;
    status |= OVX3C_WriteReg(i2cInstId, 0x20, 0x03, regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: OVX3C_WriteReg error \n");
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn reset low: OVX3C_WriteReg 0x20: 0x03:%x \n", regVal);
    appLogWaitMsecs(20);
    regVal = 0x0;
    status |= OVX3C_WriteReg(i2cInstId, 0x20, 0x01, regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: OVX3C_WriteReg error \n");
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn reset high: OVX3C_WriteReg 0x20: 0x01:%x \n", regVal);
    regVal = 0x1;
    status |= OVX3C_WriteReg(i2cInstId, 0x20, 0x01, regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: OVX3C_WriteReg error \n");
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn reset high: OVX3C_WriteReg 0x20: 0x01:%x \n", regVal);
    appLogWaitMsecs(20);

    status = OVX3C_ReadReg(i2cInstId, 0x20, 0x00, &regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: addr:%x OVX3C_WriteReg =%x error \n", 0x20, regVal);
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn: OVX3C_ReadReg 0x20: 0x00:%x \n", regVal);
    status = OVX3C_ReadReg(i2cInstId, 0x20, 0x01, &regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: addr:%x OVX3C_WriteReg =%x error \n", 0x20, regVal);
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn: OVX3C_ReadReg 0x20: 0x01:%x \n", regVal);
    status = OVX3C_ReadReg(i2cInstId, 0x20, 0x02, &regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: addr:%x OVX3C_WriteReg =%x error \n", 0x20, regVal);
        return status;
    }
    issLogPrintf("-----default=1------>OVX3C_PowerOn:addr:0x20 OVX3C_ReadReg : 0x02:%x \n", regVal);
    status = OVX3C_ReadReg(i2cInstId, 0x20, 0x03, &regVal, 1u);
    if (status != 0)
    {
        issLogPrintf("OVX3C_PowerOn: addr:%x OVX3C_WriteReg =%x error \n", 0x20, regVal);
        return status;
    }
    issLogPrintf("----------->OVX3C_PowerOn: OVX3C_ReadReg 0x20: 0x03:%x \n", regVal);
#else


#endif

    issLogPrintf("OVX3C_PowerOn  & sensor  is reset -------------------> end\n");

    return status;
}

static int32_t OVX3C_PowerOff(uint32_t chId, void *pSensorHdl)
{
    return (0);
}

static int32_t OVX3C_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;

    printf("--------------> OVX3C_SetAeParams \n");
    return (status);
}

static int32_t OVX3C_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms)
{
    int32_t status = 0;
    printf("--------------> OVX3C_GetDccParams \n");
    return (status);
}

static int32_t OVX3C_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms)
{
    int32_t status = 0;
    printf("--------------> OVX3C_GetExpParams \n");
    return (status);
}

static void OVX3C_InitAewbConfig(uint32_t chId, void *pSensorHdl)
{
    printf("--------------> OVX3C_InitAewbConfig \n");
    return;
}

static void OVX3C_GetIspConfig (uint32_t chId, void *pSensorHdl)
{
    printf("--------------> OVX3C_GetIspConfig \n");
    return;
}

static void OVX3C_deinit (uint32_t chId, void *pSensorHdl)
{
    printf("--------------> OVX3C_deinit \n");
    return;
}

static int32_t OVX3C_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg)
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
        status = OVX3C_WriteReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, regValue, 1u);
    }
    else
    {
        /*read*/
        status = OVX3C_ReadReg(pCreatePrms->i2cInstId,
            pCreatePrms->i2cAddrSensor[chId], pReg->nRegAddr, &regValue, 1u);

        if (0 == status)
        {
            pReg->nRegValue = regValue;
        }
    }
    return (status);
}

static int32_t OVX3C_ReadReg(uint8_t     i2cInstId,
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
        issLogPrintf("Error : I2C Timeout while reading from OVX3C register 0x%x \n", regAddr);
    }
    return (status);
}

static int32_t OVX3C_WriteReg(uint8_t    i2cInstId,
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
        printf("Error : I2C Timeout while i2cAddr:%x writing 0x%x to OVX3C register 0x%x \n", i2cAddr, regVal, regAddr);
    }

    return (status);
}

static int32_t OVX3C_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms)
{
    int32_t status = 0;
    printf("--------------> OVX3C_GetExpPrgFxn \n");
    return (status);
}

static int32_t OVX3C_GetWBPrgFxn(uint32_t chId, void *pSensorHdl, IssAwbDynamicParams *p_awb_dynPrms)
{
    int32_t  status = 0;
    printf("--------------> OVX3C_GetWBPrgFxn \n");
    return (status);
}

static int32_t OVX3C_SetAwbParams(void *pSensorHdl, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms)
{
    int32_t status = 0;

    printf("--------------> OVX3C_SetAwbParams \n");
    return (status);
}
