/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       CAN模块 测试代码
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
#include "./BSP/ATK_CAN/atk_can.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"

   
/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{      
    uint8_t key;
    uint8_t i = 0, t = 0, res;
    uint8_t cnt = 0;
    uint8_t rxlen = 0;
    uint8_t canbuf[8];
    uint8_t mode = 1; /* CAN工作模式: 0,正常模式; 1,回环模式 */
    
    key_init();                                                                 /* 初始化按键 */
    atk_can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_LOOPBACK);  /* CAN初始化, 回环模式, 波特率500Kbps */
    
    while (1)
    {
        key = key_scan(0);
        if (key == KEY0_PRES)       /* KEY0按下,发送一次数据 */
        {
            for (i = 0; i < 8; i++)
            {
                canbuf[i] = cnt + i; /* 填充发送缓冲区 */
                printf("Send Data : %d \r\n", canbuf[i]);
            }
            res = atk_can_send_msg(0X12, canbuf, 8); /* ID = 0X12, 发送8个字节 */
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
        else if (key == WKUP_PRES)  /* WK_UP按下，改变CAN的工作模式 */
        {
            mode = !mode;   
            if (mode == 0) /* 正常模式，需要2个开发板 */
            {
                atk_can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_NORMAL);    /* CAN正常模式初始化, 正常模式, 波特率500Kbps */
                printf("Normal Mode \r\n");
            }
            else /* 回环模式,一个开发板就可以测试了. */
            {
                atk_can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_LOOPBACK);  /* CAN回环模式初始化, 回环模式, 波特率500Kbps */
                printf("LoopBack Mode \r\n");
            }
            printf("\r\n");
        }
        rxlen = atk_can_receive_msg(0X12, canbuf); /* CAN ID = 0X12, 接收数据查询 */
        if (rxlen) /* 接收到有数据 */
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
            LED0_TOGGLE(); /* 提示系统正在运行 */
            t = 0;
            cnt++;
        }
    }
}


