/**
 ****************************************************************************************************
 * @file        atk_tof.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_TOFģ�� ��������
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

#ifndef __ATK_TOF_H
#define __ATK_TOF_H

#include "stdint.h"
#include "stdio.h"
#include "./SYSTEM/sys/sys.h"
#include "./BSP/IIC/myiic.h"
#include "./SYSTEM/delay/delay.h"


/**************************************************************************************************/
/* XS���Ŷ��� */

#define ATK_TOF_XSH_GPIO_PORT            GPIOB
#define ATK_TOF_XSH_GPIO_PIN             GPIO_PIN_6
#define ATK_TOF_XSH_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* IO���� */
#define ATK_TOF_XSH(x)                   do{ x ?                                                                                  \
                                                HAL_GPIO_WritePin(ATK_TOF_XSH_GPIO_PORT, ATK_TOF_XSH_GPIO_PIN, GPIO_PIN_SET) :    \
                                                HAL_GPIO_WritePin(ATK_TOF_XSH_GPIO_PORT, ATK_TOF_XSH_GPIO_PIN, GPIO_PIN_RESET);   \
                                         }while(0)

/* ATK-TOFģ���ϵ�Ĭ��IICͨѶ��ַ */
#define ATK_TOF_IIC_ADDR          0xD8

/* ATK-TOFģ��״ֵ̬ */
typedef enum 
{
    VI530x_OK                   = 0x00,
    VI530x_RANGING              = 0x01,
    VI530x_BUSY                 = 0x02,
    VI530x_SLEEP                = 0x04,
    VI530x_IIC_ERROR            = 0x08,
    VI530x_ERROR                = 0x10,
    VI530x_ERROR_XTALK_CALIB    = 0x20,
    VI5300_ERROR_OFFSET_CALIB   = 0x40,
    VI530x_ERROR_FW_FAILURE     = 0x80
} VI530x_Status;

/* ATK-TOFģ���ȡģʽ
 * 0�����ζ�ȡ
 * 1��������ȡ
 */
#define CONTINUOUS_READ           1

/**************************************************************************************************/

void atk_tof_hw_reset(void);                                                          /* ATK-TOFģ��Ӳ����λ */
void atk_tof_hw_init(void);                                                           /* ATK-TOFģ��Ӳ����ʼ�� */
VI530x_Status atk_tof_write_byte(uint8_t reg_addr, uint8_t data);                     /* дһ���ֽ����� */
VI530x_Status atk_tof_write_nbytes(uint8_t reg_addr, uint8_t *data, uint16_t len);    /* дN���ֽ����� */
VI530x_Status atk_tof_read_byte(uint8_t reg_addr, uint8_t *data);                     /* ��һ���ֽ����� */
VI530x_Status atk_tof_read_nbytes(uint8_t reg_addr, uint8_t *data, uint16_t len);     /* ��N���ֽ����� */

#endif





