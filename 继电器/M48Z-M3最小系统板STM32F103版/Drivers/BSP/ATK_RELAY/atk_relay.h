/**
 ****************************************************************************************************
 * @file        atk_relay.h
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

#ifndef __ATK_RELAY_H
#define __ATK_RELAY_H

#include "./SYSTEM/sys/sys.h"


/**************************************************************************************************/
/* 引脚 定义 */

#define ATK_RELAY_IN_GPIO_PORT            GPIOA
#define ATK_RELAY_IN_GPIO_PIN             GPIO_PIN_3
#define ATK_RELAY_IN_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/**************************************************************************************************/
/* 写IN引脚 宏定义 */

#define ATK_RELAY_IN(x)  do{ x ? \
                            HAL_GPIO_WritePin(ATK_RELAY_IN_GPIO_PORT, ATK_RELAY_IN_GPIO_PIN, GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(ATK_RELAY_IN_GPIO_PORT, ATK_RELAY_IN_GPIO_PIN, GPIO_PIN_RESET); \
                           }while(0)

/**************************************************************************************************/

void atk_relay_init(void);

#endif

