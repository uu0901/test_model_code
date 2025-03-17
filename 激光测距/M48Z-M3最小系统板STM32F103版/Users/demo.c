/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_TOF模块 测试代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M48Z-M3最小系统板STM32F103版
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "demo.h"
#include "./BSP/ATK_TOF/atk_tof.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "VI530x_API.h"


/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    VI530x_Status ret = VI530x_OK;
    VI530x_MEASURE_TypeDef result;

    atk_tof_hw_init();      /* 初始化ATK_TOF模块 */
    
    while (1)
    {
        ret = VI530x_Get_Measure_Data(&result);     /* 读取距离数据 */
        
        if(ret == VI530x_OK)                        /* 读取成功 */
        {
            printf("距离 = %4d mm \r\n", result.correction_tof);
        }
    }
}


