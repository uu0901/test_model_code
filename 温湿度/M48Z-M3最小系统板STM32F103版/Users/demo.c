/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       AHT20温湿度传感器 测试代码
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
#include "./BSP/ATK_AHT20/atk_aht20.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"


/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    float temperature;
    float humidity;
    
    while(atk_aht20_init())
    {
        printf("ATH20传感器初始化失败\r\n");
        delay_ms(1000);
    }
    
    while (1)
    {
        atk_aht20_read_data(&temperature, &humidity);       /* 读取ATH20传感器数据 */

        printf("温度: %.2f℃\r\n", temperature);            /* 计算得到湿度值 */
        printf("湿度: %.2f%%\r\n", humidity);               /* 计算得到温度值 */
        printf("\r\n");
        
        delay_ms(1000);
    }
}


