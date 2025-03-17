/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       RTC模块 测试代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "demo.h"
#include "./BSP/ATK_RTC/atk_rtc.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"


/* 时间结构体缓存区 */
sd3078_time_t sd3078_time_result;

/**
 * @brief       闹钟测试函数
 * @param       无
 * @retval      无
 */
void atk_rtc_alarm_test(void)
{
    sd3078_time_alarm_t alarm;

    alarm.year_a = 24;
    alarm.mon_a = 9;
    alarm.day_a = 30;
    alarm.week_a = ALARM_WEEK_MON;  /* 星期一 */
    alarm.hour_a = 16;
    alarm.min_a = 19;
    alarm.sec_a = 30;
    alarm.enable_a = ALARM_SEC;     /* 秒闹钟，每次计数的秒钟数为30秒时，会触发闹钟，拉低INT引脚 */
    alarm.ie_a = 1;                 /* 使能中断 */
    alarm.int_period = 0;           /* 单事件报警 */

    atk_rtc_set_alarm_interrupt(&alarm);
}
    

/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{    
    while(atk_rtc_init())               /* 检测RTC模块 */
    {
        printf("RTC Check Failed!\r\n");
        delay_ms(500);
        LED0_TOGGLE();                  /* 红灯闪烁 */
    }
    printf("RTC Ready!\r\n");
    atk_rtc_alarm_test();               /* 闹钟测试函数 */
    while (1)
    {
        /* 获取RTC数据 */
        if(atk_rtc_get_time(&sd3078_time_result) == 0)
        {
            printf("星期%d \r\n", sd3078_time_result.week);
            printf("日期：20%d-%d-%d\r\n", sd3078_time_result.year, sd3078_time_result.month, sd3078_time_result.day);
            printf("时间：%d:%d:%d\r\n", sd3078_time_result.hour, sd3078_time_result.minute, sd3078_time_result.second);
            printf("\r\n");
        }

        /* 闹钟触发判断 */
        if(!atk_rtc_flag_get(SD3078_ALARM_TIME_FLAG))
        {
            printf("闹钟触发！\r\n");
            atk_rtc_flag_clear(SD3078_ALARM_TIME_FLAG); /* 清除标志位，INT脚恢复高电平 */
        }
        delay_ms(1000);
        LED0_TOGGLE();
    }
}


