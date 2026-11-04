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
#include <iss_sensor_priv.h>
#include <iss_sensor_if.h>
#include <iss_sensor_serdes.h>
/**< ISS AEWB plugin is included here to get the default AEWB configuration
     from each sensor */

#define OVX3C_OUT_WIDTH   (1936)
#define OVX3C_OUT_HEIGHT  (1100)
#define OVX3C_META_HEIGHT_AFTER (4)

#include "ox03c10_linear_4lane_60fps_2m_12bit_init_all.h"
#include "ox03c10_linear_4lane_30fps_2m_12bit_init_all.h"
#include "ox03c10_built_in_4lane_30fps_2m_12bit_init_all.h"

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */
#define OVX3C_CHIP_ID_REG_ADDR               (0x0330)
#define OVX3C_CHIP_ID_REG_VAL                (0x15)

#define ISS_SENSOR_OVX3C_DEFAULT_EXP_RATIO     (128U)
/*    (ISS_SENSOR_FEATURE_MANUAL_EXPOSURE|   \
                                         ISS_SENSOR_FEATURE_MANUAL_GAIN|       \
                                         ISS_SENSOR_FEATURE_LINEAR_MODE|       \
                                         ISS_SENSOR_FEATURE_COMB_COMP_WDR_MODE|       \
                                         ISS_SENSOR_FEATURE_DCC_SUPPORTED)*/
// #define ISS_SENSOR_OVX3C_FEATURES      (ISS_SENSOR_FEATURE_MANUAL_EXPOSURE |   \
//                                         ISS_SENSOR_FEATURE_MANUAL_GAIN |       \
//                                         ISS_SENSOR_FEATURE_LINEAR_MODE |       \
//                                         ISS_SENSOR_FEATURE_DCC_SUPPORTED)

#define ISS_SENSOR_OVX3C_FEATURES      (ISS_SENSOR_FEATURE_MANUAL_EXPOSURE |   \
                                        ISS_SENSOR_FEATURE_MANUAL_GAIN |       \
                                        ISS_SENSOR_FEATURE_DCC_SUPPORTED |       \
                                        ISS_SENSOR_FEATURE_COMB_COMP_WDR_MODE)

/*******************************************************************************
 *  AE Parameters
 *******************************************************************************
 */
/* OVX3C (OX03C10) timing parameters for 1920x1080@30fps */
#define OVX3C_LINE_LEN_TIME_NS        (106167U)    /* HTS(2548) / 24MHz = ~106.167us */
#define OVX3C_MAX_EXP_ROWS             (0x248)      /* Max exposure rows (585 = 0x249, use 584) */
#define OVX3C_MIN_EXP_ROWS            (2U)         /* Min exposure rows */

/* OVX3C analog gain registers */
#define OVX3C_EXP_COARSE_ADDR         (0x3501U)     /* Coarse integration time high byte */
#define OVX3C_EXP_FINE_ADDR           (0x3502U)     /* Fine integration time low byte */
#define OVX3C_ANALOG_GAIN_ADDR        (0x3508U)     /* Analog gain high byte */
#define OVX3C_ANALOG_GAIN_LOW_ADDR    (0x3509U)     /* Analog gain low byte */
#define OVX3C_GROUP_HOLD_ADDR         (0x3507U)     /* Group hold register */

/* OVX3C analog gain table size */
#define ISS_SENSORS_OVX3C_ANALOG_GAIN_TBL_SIZE    (15U)


/*******************************************************************************
 *  Local Functions Declarations
 *******************************************************************************
 */

static int32_t OVX3C_Probe(uint32_t chId, void *pSensorHdl);
static int32_t OVX3C_Config(uint32_t chId, void *pSensorHdl, uint32_t sensor_features_requested);
static int32_t OVX3C_StreamOn(uint32_t chId, void *pSensorHdl);
static int32_t OVX3C_StreamOff(uint32_t chId, void *pSensorHdl);
static int32_t OVX3C_PowerOn(uint32_t chMask, void *pSensorHdl);
static int32_t OVX3C_PowerOff(uint32_t chId, void *pSensorHdl);
static int32_t OVX3C_GetExpParams(uint32_t chId, void *pSensorHdl, IssSensor_ExposureParams *pExpPrms);
static int32_t OVX3C_SetAeParams(void *pSensorHdl, uint32_t chId, IssSensor_ExposureParams *pExpPrms);
static int32_t OVX3C_GetDccParams(uint32_t chId, void *pSensorHdl, IssSensor_DccParams *pDccPrms);
static void OVX3C_InitAewbConfig(uint32_t chId, void *pSensorHdl);
static void OVX3C_GetIspConfig (uint32_t chId, void *pSensorHdl);
static void OVX3C_deinit (uint32_t chId, void *pSensorHdl);
static int32_t OVX3C_ReadWriteReg (uint32_t chId, void *pSensorHdl, uint32_t readWriteFlag, I2cParams *pReg);
static int32_t OVX3C_GetExpPrgFxn(uint32_t chId, void *pSensorHdl, IssAeDynamicParams *p_ae_dynPrms);
static int32_t OVX3C_SetAwbParams(void *pSensorHdl, uint32_t chId, IssSensor_WhiteBalanceParams *pWbPrms);
static int32_t OVX3C_GetWBPrgFxn(uint32_t chId, void *pSensorHdl, IssAwbDynamicParams *p_awb_dynPrms);

static int32_t OVX3C_WriteReg(uint8_t    i2cInstId,
                             uint8_t       i2cAddr,
                             uint16_t         regAddr,
                             uint8_t          regValue,
                             uint32_t      numRegs);

static int32_t OVX3C_ReadReg(uint8_t      i2cInstId,
                            uint8_t         i2cAddr,
                            uint16_t        regAddr,
                            uint8_t         *regVal,
                            uint32_t        numRegs);
