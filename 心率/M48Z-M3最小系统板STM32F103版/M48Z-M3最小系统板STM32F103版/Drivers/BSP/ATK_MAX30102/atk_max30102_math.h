/**
 ****************************************************************************************************
 * @file        atk_max30102.c
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

#ifndef _ATK_MAX30102_MATH_H
#define _ATK_MAX30102_MATH_H
#include "./SYSTEM/sys/sys.h"
#include "arm_math.h"
#include "arm_const_structs.h"


/******************************************************************************************/
/* 外部接口函数*/

void max30102_fir_init(void);
void ir_max30102_fir(float *input,float *output);
void red_max30102_fir(float *input,float *output);

#endif

