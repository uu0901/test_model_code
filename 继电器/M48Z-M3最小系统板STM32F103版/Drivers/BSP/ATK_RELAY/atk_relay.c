/**
 ****************************************************************************************************
 * @file        atk_relay.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       继电器模块 驱动代码
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

#include "./BSP/ATK_RELAY/atk_relay.h"


/**
 * @brief       继电器模块初始化函数
 * @param       无
 * @retval      无
 */
void atk_relay_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    ATK_RELAY_IN_GPIO_CLK_ENABLE();                                 /* 时钟使能 */

    gpio_init_struct.Pin = ATK_RELAY_IN_GPIO_PIN;                   /* 引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                    /* 推拉输出 */
    gpio_init_struct.Pull = GPIO_NOPULL;                            /* 无上下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;                   /* 低速 */
    HAL_GPIO_Init(ATK_RELAY_IN_GPIO_PORT, &gpio_init_struct);       /* 引脚初始化 */
}

