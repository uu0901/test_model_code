/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       AHT20��ʪ�ȴ����� ���Դ���
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
#include "./BSP/ATK_AHT20/atk_aht20.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"


/**
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
 */
void demo_run(void)
{
    float temperature;
    float humidity;
    
    while(atk_aht20_init())
    {
        printf("ATH20��������ʼ��ʧ��\r\n");
        delay_ms(1000);
    }
    
    while (1)
    {
        atk_aht20_read_data(&temperature, &humidity);       /* ��ȡATH20���������� */

        printf("�¶�: %.2f��\r\n", temperature);            /* ����õ�ʪ��ֵ */
        printf("ʪ��: %.2f%%\r\n", humidity);               /* ����õ��¶�ֵ */
        printf("\r\n");
        
        delay_ms(1000);
    }
}


