/**
 ****************************************************************************************************
 * @file        atk_relay.c
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

#include "./BSP/ATK_RELAY/atk_relay.h"


/**
 * @brief       �̵���ģ���ʼ������
 * @param       ��
 * @retval      ��
 */
void atk_relay_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = {0};
    ATK_RELAY_IN_GPIO_CLK_ENABLE();                                 /* ʱ��ʹ�� */

    gpio_init_struct.Pin = ATK_RELAY_IN_GPIO_PIN;                   /* ���� */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                    /* ������� */
    gpio_init_struct.Pull = GPIO_NOPULL;                            /* �������� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;                   /* ���� */
    HAL_GPIO_Init(ATK_RELAY_IN_GPIO_PORT, &gpio_init_struct);       /* ���ų�ʼ�� */
}

