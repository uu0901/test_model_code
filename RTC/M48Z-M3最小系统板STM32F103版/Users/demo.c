/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       RTCģ�� ���Դ���
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
#include "./BSP/ATK_RTC/atk_rtc.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"


/* ʱ��ṹ�建���� */
sd3078_time_t sd3078_time_result;

/**
 * @brief       ���Ӳ��Ժ���
 * @param       ��
 * @retval      ��
 */
void atk_rtc_alarm_test(void)
{
    sd3078_time_alarm_t alarm;

    alarm.year_a = 24;
    alarm.mon_a = 9;
    alarm.day_a = 30;
    alarm.week_a = ALARM_WEEK_MON;  /* ����һ */
    alarm.hour_a = 16;
    alarm.min_a = 19;
    alarm.sec_a = 30;
    alarm.enable_a = ALARM_SEC;     /* �����ӣ�ÿ�μ�����������Ϊ30��ʱ���ᴥ�����ӣ�����INT���� */
    alarm.ie_a = 1;                 /* ʹ���ж� */
    alarm.int_period = 0;           /* ���¼����� */

    atk_rtc_set_alarm_interrupt(&alarm);
}
    

/**
 * @brief       ������ʾ��ں���
 * @param       ��
 * @retval      ��
 */
void demo_run(void)
{    
    while(atk_rtc_init())               /* ���RTCģ�� */
    {
        printf("RTC Check Failed!\r\n");
        delay_ms(500);
        LED0_TOGGLE();                  /* �����˸ */
    }
    printf("RTC Ready!\r\n");
    atk_rtc_alarm_test();               /* ���Ӳ��Ժ��� */
    while (1)
    {
        /* ��ȡRTC���� */
        if(atk_rtc_get_time(&sd3078_time_result) == 0)
        {
            printf("����%d \r\n", sd3078_time_result.week);
            printf("���ڣ�20%d-%d-%d\r\n", sd3078_time_result.year, sd3078_time_result.month, sd3078_time_result.day);
            printf("ʱ�䣺%d:%d:%d\r\n", sd3078_time_result.hour, sd3078_time_result.minute, sd3078_time_result.second);
            printf("\r\n");
        }

        /* ���Ӵ����ж� */
        if(!atk_rtc_flag_get(SD3078_ALARM_TIME_FLAG))
        {
            printf("���Ӵ�����\r\n");
            atk_rtc_flag_clear(SD3078_ALARM_TIME_FLAG); /* �����־λ��INT�Żָ��ߵ�ƽ */
        }
        delay_ms(1000);
        LED0_TOGGLE();
    }
}


