/**
 ****************************************************************************************************
 * @file        atk_rs485.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_RS485ģ�� ��������
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

#ifndef __ATK_RS485_H
#define __ATK_RS485_H

#include "./SYSTEM/sys/sys.h"


/**************************************************************************************************/
/* ���� ���� */

#define ATK_RS485_TX_GPIO_PORT                  GPIOA
#define ATK_RS485_TX_GPIO_PIN                   GPIO_PIN_2
#define ATK_RS485_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA��ʱ��ʹ�� */

#define ATK_RS485_RX_GPIO_PORT                  GPIOA
#define ATK_RS485_RX_GPIO_PIN                   GPIO_PIN_3
#define ATK_RS485_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA��ʱ��ʹ�� */

#define ATK_RS485_UX                            USART2
#define ATK_RS485_UX_IRQn                       USART2_IRQn
#define ATK_RS485_UX_IRQHandler                 USART2_IRQHandler
#define ATK_RS485_UX_CLK_ENABLE()               do{ __HAL_RCC_USART2_CLK_ENABLE(); }while(0)  /* USART2 ʱ��ʹ�� */

#define RS485_REC_LEN       220                 /* �����������ֽ��� 64 */
#define RS485_EN_RX         1                   /* ʹ�ܣ�1��/��ֹ��0��RS485���� */

extern uint8_t g_RS485_rx_buf[RS485_REC_LEN];   /* ���ջ���,���RS485_REC_LEN���ֽ� */
extern uint8_t g_RS485_rx_cnt;                  /* �������ݳ��� */

/******************************************************************************************/

void atk_rs485_init( uint32_t baudrate);                 /* RS485��ʼ�� */
void atk_rs485_send_data(uint8_t *buf, uint8_t len);     /* RS485�������� */
void atk_rs485_receive_data(uint8_t *buf, uint8_t *len); /* RS485�������� */

#endif





