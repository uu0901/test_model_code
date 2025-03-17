/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_RS485ģ�� ���Դ���
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

#include "demo.h"
#include "./BSP/ATK_RS485/atk_rs485.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/KEY/key.h"
#include "./BSP/LED/led.h"
#include <stdio.h>


/**
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
 */
void demo_run(void)
{
    uint8_t key;
    
    uint8_t i = 0, t = 0;
    uint8_t cnt = 0;
    uint8_t rs485buf[200];
    
    atk_rs485_init(256000);                     /* ��ʼ��RS485��������256000 */
    
    while (1)
    {
        key = key_scan(0);

        if (key == KEY0_PRES)                   /* KEY0����,����һ������ */
        {
            printf("RS485ģ�� ���͵�����Ϊ��");
            
            for (i = 0; i < 200; i++)
            {
                rs485buf[i] = cnt + i;          /* ��䷢�ͻ����� */
                printf("%02X", rs485buf[i]);    /* ��ӡ������� */
            }
            
            printf("\r\n");

            atk_rs485_send_data(rs485buf, 200); /* ����200���ֽ� */
        }

        atk_rs485_receive_data(rs485buf, &key);

        if (key)                                /* ���յ����� */
        {   
            if (key > 200)key = 200;            /* �����Ч�ֽ�����200�� */

            printf("RS485ģ�� ���յ�����Ϊ��");
            
            for (i = 0; i < key; i++)
            {
                printf("%02X", rs485buf[i]);    /* ��ӡ������� */
            }
            
            printf("\r\n");
        }

        t++;
        delay_ms(10);

        if (t == 20)
        {
            LED0_TOGGLE();                      /* LED0��˸, ��ʾϵͳ�������� */
            t = 0;
            cnt++;
        }
    }
}


