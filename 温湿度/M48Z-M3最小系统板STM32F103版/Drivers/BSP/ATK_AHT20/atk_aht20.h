/**
 ****************************************************************************************************
 * @file        atk_aht20.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       AHT20��ʪ�ȴ����� ��������
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

#ifndef __ATK_AHT20_H
#define __ATK_AHT20_H


#include "./SYSTEM/sys/sys.h"
#include "./BSP/IIC/myiic.h"
#include "./SYSTEM/delay/delay.h"


/**************************************************************************************************/

#define ATK_AHT20_IIC_ADDR          0x70    /* AHT20�ϵ�Ĭ��IICͨѶ��ַ */

/* AHT20ָ�� */
#define INIT                        0xBE    /* ��ʼ��ָ�� */
#define START_TEST                  0xAC    /* ��ʼ����ָ�� */

/**************************************************************************************************/

uint8_t atk_aht20_init(void);                                                   /* AHT20��������ʼ�� */
uint8_t atk_aht20_read_status(void);                                            /* ��ȡAHT20״̬ */
uint8_t atk_aht20_check(void);                                                  /* ���AHT20�Ƿ���� */
void atk_aht20_read_data(float *temp, float *humi) ;                            /* ��ȡAHT20���¶Ⱥ�ʪ������ */
uint8_t atk_aht20_write_nbytes(uint8_t reg_addr, uint8_t *data, uint8_t len);   /* дN���ֽ����� */
uint8_t atk_aht20_read_nbytes(uint8_t *data, uint8_t len);                      /* ��N���ֽ����� */

#endif





