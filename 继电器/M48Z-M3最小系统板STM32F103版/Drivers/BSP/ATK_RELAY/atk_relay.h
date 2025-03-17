/**
 ****************************************************************************************************
 * @file        atk_relay.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       �̵���ģ�� ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� M48Z-M3��Сϵͳ��STM32F103��
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATK_RELAY_H
#define __ATK_RELAY_H

#include "./SYSTEM/sys/sys.h"


/**************************************************************************************************/
/* ���� ���� */

#define ATK_RELAY_IN_GPIO_PORT            GPIOA
#define ATK_RELAY_IN_GPIO_PIN             GPIO_PIN_3
#define ATK_RELAY_IN_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/**************************************************************************************************/
/* дIN���� �궨�� */

#define ATK_RELAY_IN(x)  do{ x ? \
                            HAL_GPIO_WritePin(ATK_RELAY_IN_GPIO_PORT, ATK_RELAY_IN_GPIO_PIN, GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(ATK_RELAY_IN_GPIO_PORT, ATK_RELAY_IN_GPIO_PIN, GPIO_PIN_RESET); \
                           }while(0)

/**************************************************************************************************/

void atk_relay_init(void);

#endif

