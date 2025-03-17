/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       心率血氧传感器 测试代码
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
 ****************************************************************************************************
 */

#include "demo.h"
#include "./BSP/ATK_MAX30102/atk_max30102.h"
#include "./BSP/ATK_MAX30102/atk_max30102_math.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"


#define CACHE_NUMS          150             /* 缓存数&采集次数 */
#define PPG_DATA_THRESHOLD  100000          /* 检测阈值 */

float g_red_data[1];                        /* 红光数据缓冲区，用于计算spo2(血氧含量) */
float g_ir_data[1];                         /* 红外光数据缓冲区，用于计算heart(心率) */

float fir_output[2];                        /* 经过FIR滤波后的数据缓冲区 */
float ppg_data_cache_red[CACHE_NUMS] = {0}; /* 缓存区 */
float ppg_data_cache_ir[CACHE_NUMS] = {0};  /* 缓存区 */

/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{      
    uint16_t t = 0;
    uint16_t cnt = 0;               /* 缓存计数器 */
    uint16_t heart = 0;             /* 心率缓存区 */
    float spo2 = 0;                 /* 血氧缓存区 */
    
    atk_max30102_init();            /* 初始化max30102 */
    max30102_fir_init();            /* FIR滤波初始化 */
    printf("温度：%.2f℃\r\n", atk_max30102_get_temp());
    
    while(1)
    {
        if(g_max30102_int_flag)
        {
            g_max30102_int_flag = 0;
            atk_max30102_fifo_read(&g_red_data[0], &g_ir_data[0]);
            
            /* 使用DSP库实现FIR滤波 */
            red_max30102_fir(&g_red_data[0], &fir_output[0]);  
            ir_max30102_fir(&g_ir_data[0], &fir_output[1]);
            
            /* 检测是否超过阈值，超过代表有皮肤接触传感器 */
            if((g_red_data[0] > PPG_DATA_THRESHOLD) && (g_ir_data[0] > PPG_DATA_THRESHOLD))
            {   
                ppg_data_cache_red[cnt] = fir_output[0];
                ppg_data_cache_ir[cnt] = fir_output[1];
                cnt++;
            }
            else                        /* 小于阈值 */
            {
                cnt = 0;
            }
            if(cnt >= CACHE_NUMS)       /* 收集满了数据 */
            {
                heart = atk_max30102_get_heart(ppg_data_cache_ir, CACHE_NUMS);                  /* 获取心率值 */
                spo2 = atk_max30102_get_spo2(ppg_data_cache_ir, ppg_data_cache_red, CACHE_NUMS);/* 获取血氧值 */
                if(heart == 0)
                {
                    printf("心率测量失败，请将皮肤紧贴传感器，并保持静止\r\n");
                }
                else
                {
                    printf("心率：%d 次/min\r\n", heart);
                }
                printf("血氧：%.2f%%\r\n", spo2);                
                printf("\r\n");
                cnt = 0;
            }
            
            t++;
            if(t == 20)
            {
                t = 0;
                LED0_TOGGLE();
            }
        }
    }
}


