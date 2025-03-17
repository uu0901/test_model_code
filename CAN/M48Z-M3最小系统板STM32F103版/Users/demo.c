/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       CANģ�� ���Դ���
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
#include "./BSP/ATK_CAN/atk_can.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"

   
/**
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
 */
void demo_run(void)
{      
    uint8_t key;
    uint8_t i = 0, t = 0, res;
    uint8_t cnt = 0;
    uint8_t rxlen = 0;
    uint8_t canbuf[8];
    uint8_t mode = 1; /* CAN����ģʽ: 0,����ģʽ; 1,�ػ�ģʽ */
    
    key_init();                                                                 /* ��ʼ������ */
    atk_can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_LOOPBACK);  /* CAN��ʼ��, �ػ�ģʽ, ������500Kbps */
    
    while (1)
    {
        key = key_scan(0);
        if (key == KEY0_PRES)       /* KEY0����,����һ������ */
        {
            for (i = 0; i < 8; i++)
            {
                canbuf[i] = cnt + i; /* ��䷢�ͻ����� */
                printf("Send Data : %d \r\n", canbuf[i]);
            }
            res = atk_can_send_msg(0X12, canbuf, 8); /* ID = 0X12, ����8���ֽ� */
            if(res)
            {
                printf("Send Data Failed! \r\n");
            }
            else
            {
                printf("Send Data OK! \r\n");
            }
            printf("\r\n");
        }
        else if (key == WKUP_PRES)  /* WK_UP���£��ı�CAN�Ĺ���ģʽ */
        {
            mode = !mode;   
            if (mode == 0) /* ����ģʽ����Ҫ2�������� */
            {
                atk_can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_NORMAL);    /* CAN����ģʽ��ʼ��, ����ģʽ, ������500Kbps */
                printf("Normal Mode \r\n");
            }
            else /* �ػ�ģʽ,һ��������Ϳ��Բ�����. */
            {
                atk_can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_LOOPBACK);  /* CAN�ػ�ģʽ��ʼ��, �ػ�ģʽ, ������500Kbps */
                printf("LoopBack Mode \r\n");
            }
            printf("\r\n");
        }
        rxlen = atk_can_receive_msg(0X12, canbuf); /* CAN ID = 0X12, �������ݲ�ѯ */
        if (rxlen) /* ���յ������� */
        {
            for (i = 0; i < rxlen; i++)
            {
                printf("Receive Data : %d \r\n", canbuf[i]);
            }
            printf("Receive Data OK! \r\n");
            printf("\r\n");
        }
        
        t++;
        delay_ms(10);

        if (t == 20)
        {
            LED0_TOGGLE(); /* ��ʾϵͳ�������� */
            t = 0;
            cnt++;
        }
    }
}


