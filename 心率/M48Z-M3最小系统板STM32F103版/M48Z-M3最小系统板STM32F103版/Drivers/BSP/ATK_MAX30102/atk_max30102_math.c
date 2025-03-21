/**
 ****************************************************************************************************
 * @file        atk_max30102_math.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-10-17
 * @brief       FIR滤波器 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20241017
 * 第一次发布
 *
 ****************************************************************************************************
 */
#include "./BSP/ATK_MAX30102/atk_max30102.h"
#include "./BSP/ATK_MAX30102/atk_max30102_math.h"
#include "./SYSTEM/usart/usart.h"
#include "math.h"


#define BLOCK_SIZE           1                              /* 调用一次arm_fir_f32处理的采样点个数 */
#define NUM_TAPS             29                             /* 滤波器系数个数 */

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = BLOCK_SIZE;                            /* 需要调用arm_fir_f32的次数 */
arm_fir_instance_f32 S_ir, S_red;
static float firStateF32_ir[BLOCK_SIZE + NUM_TAPS - 1];     /* 状态缓存，大小numTaps + blockSize - 1*/
static float firStateF32_red[BLOCK_SIZE + NUM_TAPS - 1];    /* 状态缓存，大小numTaps + blockSize - 1*/

/* 低通滤波器系数 通过fadtool获取 */
const float firCoeffs32LP[NUM_TAPS] = 
{
    -0.001542701735, -0.002211477375, -0.003286228748, -0.00442651147, -0.004758632276,
    -0.003007677384, 0.002192312852, 0.01188309677, 0.02637642808, 0.04498152807,
    0.06596207619, 0.0867607221, 0.1044560149, 0.1163498312, 0.1205424443,
    0.1163498312, 0.1044560149, 0.0867607221, 0.06596207619, 0.04498152807,
    0.02637642808, 0.01188309677, 0.002192312852, -0.003007677384, -0.004758632276,
    -0.00442651147, -0.003286228748, -0.002211477375, -0.001542701735
};

/**
 * @brief   初始化FIR滤波器
 * @param   无   
 * @retval  无
 */
void max30102_fir_init(void)
{
    arm_fir_init_f32(&S_ir, NUM_TAPS, (float32_t *)&firCoeffs32LP[0], &firStateF32_ir[0], blockSize);
    arm_fir_init_f32(&S_red, NUM_TAPS, (float32_t *)&firCoeffs32LP[0], &firStateF32_red[0], blockSize);
}

/**
 * @brief   红外光FIR滤波
 * @param   input  ：指向输入信号的数据数组   
 * @param   output ：指向输出滤波结果的数据数组。
 * @retval  无
 */
void ir_max30102_fir(float *input, float *output)
{
    arm_fir_f32(&S_ir, input,output, blockSize);
}

/**
 * @brief   红光FIR滤波
 * @param   input  ：指向输入信号的数据数组   
 * @param   output ：指向输出滤波结果的数据数组。
 * @retval  无
 */
void red_max30102_fir(float *input, float *output)
{
    arm_fir_f32(&S_red,input, output, blockSize);
}

