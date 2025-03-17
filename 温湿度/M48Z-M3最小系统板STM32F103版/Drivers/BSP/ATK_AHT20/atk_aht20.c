/**
 ****************************************************************************************************
 * @file        atk_aht20.c
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

#include "./BSP/ATK_AHT20/atk_aht20.h"


/**
 * @brief       ��ʼ��AHT20������
 * @param       ��
 * @retval      ��ʼ�������0���ɹ���1��ʧ��
 */
uint8_t atk_aht20_init(void)
{
    uint8_t command[2] = {0x08, 0x00};

    iic_init();                                     /* ��ʼ��IIC�ӿ� */
    
    delay_ms(40);                                   /* ��ʱ40ms����Ӳ���ȶ� */

    atk_aht20_write_nbytes(INIT, command, 2);       /* ���ͳ�ʼ��ָ�� */

    delay_ms(500);                                  /* ��ʱ500ms����Ӳ���ȶ� */
    
    return atk_aht20_check();                       /* ���AHT20��״̬ */
}

/**
 * @brief       ��ȡAHT20��״̬�Ĵ���
 * @param       ��
 * @retval      ״ֵ̬
 */
uint8_t atk_aht20_read_status(void)
{
    uint8_t ret;
    
    atk_aht20_read_nbytes(&ret, 1);                 /* ��ȡ״̬�Ĵ��� */

    return ret;
}

/**
 * @brief       ���AHT20�������Ƿ����
 * @param       ��
 * @retval      0, ����
 *              1, �쳣/������
 */
uint8_t atk_aht20_check(void)
{
    if((atk_aht20_read_status() & 0x08) == 0x08)    /* У׼��������ã� */
    {
        return 0;       /* ����������0 */
    }
    else
    {
        return 1;       /* �쳣������1 */
    }
}

/**
 * @brief       ��ȡAHT20���¶Ⱥ�ʪ������
 * @param       *temp: �¶�ֵָ��
 * @param       *humi: ʪ��ֵָ��
 * @retval      ��
 */
void atk_aht20_read_data(float *temp, float *humi) 
{
    uint32_t humi_data = 0, temp_data = 0;
    uint8_t raw_data[10];
    uint8_t command[2] = {0x33, 0x00};

    atk_aht20_write_nbytes(START_TEST, command, 2);         /* ���Ͳ���ָ�� */
    delay_ms(80);                                           /* �ȴ�80ms */

    atk_aht20_read_nbytes(raw_data, 7);                     /* ��ȡԭʼ���� */

    if((raw_data[0] & 0x80) == 0x00)
    {
        humi_data = 0;                                      /* �������� */
        humi_data = (humi_data | raw_data[1]) << 8;         /* ȡ����һ���ֽ�ʪ�����ݣ���[19:12]λ */
        humi_data = (humi_data | raw_data[2]) << 8;         /* ȡ���ڶ����ֽ�ʪ�����ݣ���[11:4]λ */
        humi_data = (humi_data | raw_data[3]);              /* ȡ���������ֽ�ʪ�����ݣ���[3:0]λ */
        humi_data = humi_data >> 4;                         /* ʪ����Ч���ݹ���20λ���������ֽ�����ֻ�и�4λ��Ч�������Ҫ����4λ�����ܵõ���Ч���� */
        *humi = (float)humi_data * 100 /1024 /1024;         /* ����ʪ��ֵ */

        temp_data = 0;                                      /* �������� */
        temp_data = (temp_data | raw_data[3]) << 8;         /* ȡ����һ���ֽ��¶����ݣ���[19:16]λ */
        temp_data = (temp_data | raw_data[4]) << 8;         /* ȡ���ڶ����ֽ��¶����ݣ���[15:8]λ */
        temp_data = (temp_data | raw_data[5]);              /* ȡ���������ֽ��¶����ݣ���[7:0]λ */
        temp_data = temp_data & 0xfffff;                    /* �¶���Ч���ݹ���20λ����һ���ֽ�����ֻ�е�4λ��Ч�������Ҫ����4λ�����ܵõ���Ч���� */
        *temp = (float)temp_data * 200 / 1024 /1024 - 50;   /* �����¶�ֵ */
    }
}

/**
 * @brief   дN���ֽڵ�AHT20�������ļĴ���
 * @param   reg_addr:   �Ĵ�����ַ
 * @param   data:       д������
 * @param   len:        ���ݳ���
 * @retval  д����
 * @arg     0   : �ɹ�
 * @arg     ����: ʧ��
 */
uint8_t atk_aht20_write_nbytes(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    iic_start();                        /* ��ʼ�ź� */
    
    iic_send_byte(ATK_AHT20_IIC_ADDR | 0x00);    /* ����IICͨѶ��ַ */
    if (iic_wait_ack() == 1)            /* �ȴ�Ӧ�� */
    {
        iic_stop();
        return 1;
    }
    
    iic_send_byte(reg_addr);            /* ���ͼĴ�����ַ */
    if (iic_wait_ack() == 1)            /* �ȴ�Ӧ�� */
    {
        iic_stop();
        return 1;
    }
    
    for (uint16_t i = 0; i < len; i++)  /* ѭ���������� */
    {
        iic_send_byte(data[i]);
        if (iic_wait_ack() == 1)        /* �ȴ�Ӧ�� */
        {
            iic_stop();
            return 1;
        }
    }
    
    iic_stop();                         /* ֹͣ�ź� */
    
    return 0;
}

/**
 * @brief   ��AHT20��������ȡN���ֽ�����
 * @param   data:       ���ݴ洢buff
 * @param   len:        ���ݳ���
 * @retval  �������
 * @arg     0   : �ɹ�
 * @arg     ����: ʧ��
 */
uint8_t atk_aht20_read_nbytes(uint8_t *data, uint8_t len)
{    
    iic_start();                        /* ��ʼ�ź� */
    
    iic_send_byte(ATK_AHT20_IIC_ADDR | 0x01);
    if (iic_wait_ack() == 1)            /* �ȴ�Ӧ�� */
    {
        iic_stop();
        return 1;
    }
    
    while (len)                         /* ѭ���������� */
    {
        *data = iic_read_byte((len > 1) ? 1 : 0);
        len--;
        data++;
    }
    
    iic_stop();                         /* ֹͣ�ź� */
    
    return 0;
}




