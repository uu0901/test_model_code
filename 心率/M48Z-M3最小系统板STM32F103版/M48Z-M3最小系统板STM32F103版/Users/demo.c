/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ����Ѫ�������� ���Դ���
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
#include "./BSP/ATK_MAX30102/atk_max30102.h"
#include "./BSP/ATK_MAX30102/atk_max30102_math.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"


#define CACHE_NUMS          150             /* ������&�ɼ����� */
#define PPG_DATA_THRESHOLD  100000          /* �����ֵ */

float g_red_data[1];                        /* ������ݻ����������ڼ���spo2(Ѫ������) */
float g_ir_data[1];                         /* ��������ݻ����������ڼ���heart(����) */

float fir_output[2];                        /* ����FIR�˲�������ݻ����� */
float ppg_data_cache_red[CACHE_NUMS] = {0}; /* ������ */
float ppg_data_cache_ir[CACHE_NUMS] = {0};  /* ������ */

/**
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
 */
void demo_run(void)
{      
    uint16_t t = 0;
    uint16_t cnt = 0;               /* ��������� */
    uint16_t heart = 0;             /* ���ʻ����� */
    float spo2 = 0;                 /* Ѫ�������� */
    
    atk_max30102_init();            /* ��ʼ��max30102 */
    max30102_fir_init();            /* FIR�˲���ʼ�� */
    printf("�¶ȣ�%.2f��\r\n", atk_max30102_get_temp());
    
    while(1)
    {
        if(g_max30102_int_flag)
        {
            g_max30102_int_flag = 0;
            atk_max30102_fifo_read(&g_red_data[0], &g_ir_data[0]);
            
            /* ʹ��DSP��ʵ��FIR�˲� */
            red_max30102_fir(&g_red_data[0], &fir_output[0]);  
            ir_max30102_fir(&g_ir_data[0], &fir_output[1]);
            
            /* ����Ƿ񳬹���ֵ������������Ƥ���Ӵ������� */
            if((g_red_data[0] > PPG_DATA_THRESHOLD) && (g_ir_data[0] > PPG_DATA_THRESHOLD))
            {   
                ppg_data_cache_red[cnt] = fir_output[0];
                ppg_data_cache_ir[cnt] = fir_output[1];
                cnt++;
            }
            else                        /* С����ֵ */
            {
                cnt = 0;
            }
            if(cnt >= CACHE_NUMS)       /* �ռ��������� */
            {
                heart = atk_max30102_get_heart(ppg_data_cache_ir, CACHE_NUMS);                  /* ��ȡ����ֵ */
                spo2 = atk_max30102_get_spo2(ppg_data_cache_ir, ppg_data_cache_red, CACHE_NUMS);/* ��ȡѪ��ֵ */
                if(heart == 0)
                {
                    printf("���ʲ���ʧ�ܣ��뽫Ƥ�������������������־�ֹ\r\n");
                }
                else
                {
                    printf("���ʣ�%d ��/min\r\n", heart);
                }
                printf("Ѫ����%.2f%%\r\n", spo2);                
                printf("\r\n");
                cnt = 0;
            }
            
            t++;
            if(t == 20)
            {
                t = 0;
                LED0_TOGGLE();
            }
        }
    }
}


