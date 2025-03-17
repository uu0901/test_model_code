/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       继电器模块 测试代码
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
#include "./BSP/ATK_RELAY/atk_relay.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include <stdio.h>


/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    uint8_t time = 0;
    uint8_t key = 0;
    uint8_t toggle_flag = 0;
    
    key_init();
    atk_relay_init();           /* 初始化继电器模块 */
    
    printf("继电器实验\r\n");
    printf("请通过KEY0控制继电器\r\n");
    
    while (1)
    {
        key = key_scan(0);      /* 扫描按键 */
        
        if (key == KEY0_PRES)
        {
            toggle_flag ^= 1;
            
            if (toggle_flag)
            {
                ATK_RELAY_IN(1);
                printf("继电器闭合\r\n");
            }
            else
            {
                ATK_RELAY_IN(0);
                printf("继电器断开\r\n");
            }
        }
        
        if (time++ >= 50)
        {
            LED0_TOGGLE(); /* 闪烁LED,提示系统正在运行 */
            time = 0;
        }
        
        delay_ms(10);
    }
}


