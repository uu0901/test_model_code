/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_TOFģ�� ���Դ���
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
#include "./BSP/ATK_TOF/atk_tof.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "VI530x_API.h"


/**
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
 */
void demo_run(void)
{
    VI530x_Status ret = VI530x_OK;
    VI530x_MEASURE_TypeDef result;

    atk_tof_hw_init();      /* ��ʼ��ATK_TOFģ�� */
    
    while (1)
    {
        ret = VI530x_Get_Measure_Data(&result);     /* ��ȡ�������� */
        
        if(ret == VI530x_OK)                        /* ��ȡ�ɹ� */
        {
            printf("���� = %4d mm \r\n", result.correction_tof);
        }
    }
}


