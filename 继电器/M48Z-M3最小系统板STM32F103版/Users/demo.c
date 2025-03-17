/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       �̵���ģ�� ���Դ���
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F103������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
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
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
 */
void demo_run(void)
{
    uint8_t time = 0;
    uint8_t key = 0;
    uint8_t toggle_flag = 0;
    
    key_init();
    atk_relay_init();           /* ��ʼ���̵���ģ�� */
    
    printf("�̵���ʵ��\r\n");
    printf("��ͨ��KEY0���Ƽ̵���\r\n");
    
    while (1)
    {
        key = key_scan(0);      /* ɨ�谴�� */
        
        if (key == KEY0_PRES)
        {
            toggle_flag ^= 1;
            
            if (toggle_flag)
            {
                ATK_RELAY_IN(1);
                printf("�̵����պ�\r\n");
            }
            else
            {
                ATK_RELAY_IN(0);
                printf("�̵����Ͽ�\r\n");
            }
        }
        
        if (time++ >= 50)
        {
            LED0_TOGGLE(); /* ��˸LED,��ʾϵͳ�������� */
            time = 0;
        }
        
        delay_ms(10);
    }
}


