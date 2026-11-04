/*
 * OVX3C (OX03C10) Analog Gain Table
 * 
 * Format: {target_gain, gain_coarse, gain_fine}
 * - target_gain: Q8 format (256 = 1x, 512 = 2x, etc.)
 * - gain_coarse: Coarse gain register value (bit[15:8])
 * - gain_fine: Fine gain register value (bit[7:0])
 * 
 * Gain formula: total_gain = (coarse << 8 | fine) / 0x400
 * Example: 0x3400 = 13312 / 1024 = 13x gain
 */

#ifndef OVX3C_GAIN_TABLE_H_
#define OVX3C_GAIN_TABLE_H_

/* OVX3C Analog Gain Table - maps target gain to register values */
static uint32_t ovx3cGainsTable[ISS_SENSORS_OVX3C_ANALOG_GAIN_TBL_SIZE][3] =
{
    /* {target_gain_Q8, coarse_gain, fine_gain} */
    {256,   0x04, 0x00},   /* 1.0x gain  */
    {362,   0x05, 0xA0},   /* 1.4x gain  */
    {512,   0x08, 0x00},   /* 2.0x gain  */
    {724,   0x0B, 0x40},   /* 2.8x gain  */
    {1024,  0x10, 0x00},   /* 4.0x gain  */
    {1448,  0x16, 0x80},   /* 5.6x gain  */
    {2048,  0x20, 0x00},   /* 8.0x gain  */
    {2896,  0x2D, 0x00},   /* 11.3x gain */
    {4096,  0x40, 0x00},   /* 16.0x gain */
    {5792,  0x5A, 0x00},   /* 22.6x gain */
    {8192,  0x80, 0x00},   /* 32.0x gain */
    {9216,  0x90, 0x00},   /* 36.0x gain */
    {10240, 0xA0, 0x00},  /* 40.0x gain */
    {11520, 0xB4, 0x00},  /* 45.0x gain */
    {32768, 0x200, 0x00}, /* 128x gain (max) */
};

#endif /* OVX3C_GAIN_TABLE_H_ */
