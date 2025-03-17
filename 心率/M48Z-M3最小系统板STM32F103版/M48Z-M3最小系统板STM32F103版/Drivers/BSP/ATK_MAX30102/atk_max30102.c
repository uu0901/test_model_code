/**
 ****************************************************************************************************
 * @file        atk_max30102.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-10-17
 * @brief       ����Ѫ�������� ��������
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
 * �޸�˵��
 * V1.0 20241017
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "./BSP/ATK_MAX30102/atk_max30102.h"
#include "./BSP/IIC/myiic.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "math.h"
#include "./BSP/LED/led.h"

uint8_t g_max30102_int_flag = 0;            /* �жϱ�־ */

/**
 * @brief   дһ���ֽڵ�max30102�ļĴ���
 * @param   reg: �Ĵ�����ַ
 * @param   data: �Ĵ�������
 * @retval  д����
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_max30102_write_byte(uint8_t reg, uint8_t data)
{
    iic_start();
    iic_send_byte(MAX30102_I2C_ADDR | 0x00);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    iic_send_byte(reg);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    iic_send_byte(data);        /* ����һ�ֽ� */
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    iic_stop();                 /* ����һ��ֹͣ���� */
    
    return 0;
}

/**
 * @brief   ��max30102ָ���Ĵ�����ַ����һ������
 * @param   reg:        �Ĵ�����ַ
 * @retval  ����������
 */
uint8_t atk_max30102_read_byte(uint8_t reg)
{
    uint8_t temp = 0;
    
    iic_start();
    iic_send_byte(MAX30102_I2C_ADDR | 0x00);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    iic_send_byte(reg);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    
    iic_start();
    iic_send_byte(MAX30102_I2C_ADDR | 0x01);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    temp = iic_read_byte(0);
    iic_stop();
    
    return temp;
}

/**
 * @brief   ��max30102��ȡN�ֽ�����
 * @param   reg:        �Ĵ�����ַ
 * @param   date:       ���ݴ洢buf
 * @param   len:        ���ݳ���
 * @retval  �������
 * @retval  0, �����ɹ�
 *          ����, ����ʧ��
 */
int atk_max30102_read_nbytes(uint8_t reg, uint8_t *date, uint8_t len)
{
    uint8_t i;
    
    iic_start();
    iic_send_byte(MAX30102_I2C_ADDR | 0x00);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    iic_send_byte(reg);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    
    iic_start();
    iic_send_byte(MAX30102_I2C_ADDR | 0x01);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    for (i = 0; i < len; i++)
    {
        date[i] = iic_read_byte((i == (len - 1)) ? 0 : 1);
    }
    iic_stop();
    
    return 0;
}

/**
 * @brief   max30102д��N�ֽ�����
 * @param   reg:        �Ĵ�����ַ
 * @param   data:       д������
 * @param   len:        ���ݳ���
 * @retval  д����
 * @arg     0:          �ɹ�
 * @arg     1:          ʧ��
 */
uint8_t atk_max30102_write_nbytes(uint8_t reg, uint8_t* data, uint8_t len)
{
    uint8_t i;
    
    iic_start();
    iic_send_byte(MAX30102_I2C_ADDR | 0x00);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    iic_send_byte(reg);
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    for (i = 0; i < len; i++)
    {
        iic_send_byte(data[i]);
        if (iic_wait_ack() != 0)
        {
            iic_stop();
            return 1;
        }
    }
    iic_stop();
    
    return 0;
}

/**
 * @brief   ��λmax30102
 * @param   ��   
 * @retval  ��
 */
void atk_max30102_reset(void)
{
    atk_max30102_write_byte(MAX30102_MODE_CONFIG, 0x40);
    delay_ms(10);
}

/**
 * @brief   ��ȡFIFO����
 * @param   red_data :    �洢RED����   
 * @param   ir_data  :    �洢IR����    
 * @retval  ��
 */
void atk_max30102_fifo_read(float *red_data, float *ir_data)
{
    uint8_t receive_data[6];
    /* ͨ����ȡ���ж�״̬�Ĵ��������жϱ�־λ��� */
    atk_max30102_read_byte(MAX30102_INTR_STATUS_1);
    atk_max30102_read_byte(MAX30102_INTR_STATUS_2);
    
    atk_max30102_read_nbytes(MAX30102_FIFO_DATA, receive_data, 6);
    *red_data = ((receive_data[0] << 16 | receive_data[1] << 8 | receive_data[2]) & 0x03ffff);  /* RED���� */
    *ir_data = ((receive_data[3] << 16 | receive_data[4] << 8 | receive_data[5]) & 0x03ffff);   /* IR���� */
}

/**
 * @brief   ��ȡ�¶�ֵ���棩
 * @param   ��   
 * @retval  �����¶�ֵ
 */
float atk_max30102_get_temp(void)
{
    float integer = 0.0f, fraction = 0.0f;
    
    atk_max30102_write_byte(MAX30102_TEMP_CONFIG, 0x01);    /* ʹ���¶ȴ����� ��ȡ�¶�ֵ���Զ���0 */
    
    integer = atk_max30102_read_byte(MAX30102_TEMP_INTR);   /* ��ȡ�¶�ֵ�������� */
    fraction = atk_max30102_read_byte(MAX30102_TEMP_FRAC);  /* ��ȡ�¶�ֵС������ */

    if(integer < 0x80)                                      /* С��0x80Ϊ��ֵ */
    {
        return (integer + fraction * 0.0625f);
    }
    else                                                    /* ���ڵ���0x80Ϊ��ֵ */
    {
        integer -= 256;
        return (integer + fraction * 0.0625f);
    }
}


/**
 * @brief   ��ʼ��max30102
 * @param   ��   
 * @retval  ��
 */
void atk_max30102_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    INT_GPIO_CLK_ENABLE();
    /* INT���ų�ʼ��Ϊ�ⲿ�ж��� */
    gpio_init_struct.Pin = INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;           /* �����ش��� */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* ���� */
    HAL_GPIO_Init(INT_GPIO_PORT, &gpio_init_struct);        /* INT����Ϊ�½��ش����ж� */
    
    HAL_NVIC_SetPriority(INT_IRQn, 0, 2);                   /* ��ռ0�������ȼ�2 */
    HAL_NVIC_EnableIRQ(INT_IRQn);                           /* ʹ���ж���5 */
    
    iic_init();                                             /* ��ʼ��I2C�ӿ� */
    
    atk_max30102_reset();                                   /* ��λ�豸 */
    
    atk_max30102_write_byte(MAX30102_INTR_ENABLE_1, 0xC0);  /* �ж�ʹ�ܣ�FIFO���Լ���FIFO���ݾ��� */
    atk_max30102_write_byte(MAX30102_INTR_ENABLE_2, 0x00);
    
    atk_max30102_write_byte(MAX30102_FIFO_WR_PTR, 0x00);    /* ���ָ��FIFO_WR_PTR[4:0] */
    atk_max30102_write_byte(MAX30102_OVF_COUNTER, 0x00);    /* ���ָ��OVF_COUNTER[4:0] */
    atk_max30102_write_byte(MAX30102_FIFO_RD_PTR, 0x00);    /* ���ָ��FIFO_RD_PTR[4:0] */
    
    atk_max30102_write_byte(MAX30102_FIFO_CONFIG, 0x4F);    /* ����ƽ����4����FIFO������0����FIFO������ֵ�������ж�ʱΪ15�������������� */
    atk_max30102_write_byte(MAX30102_MODE_CONFIG, 0x03);    /* ����ΪSpO2��Ѫ�����ͣ�ģʽ�������RED �� IR */
    atk_max30102_write_byte(MAX30102_SPO2_CONFIG, 0x2A);    /* SpO2����: ADC��Χ: 4096nA���������ʿ���:200Hz����LED������:215us�� */
    
    atk_max30102_write_byte(MAX30102_LED1_PA, 0x2F);        /* IR LED ����ѡ��9.4mA */
    atk_max30102_write_byte(MAX30102_LED2_PA, 0x2F);        /* RED LED ����ѡ��9.4mA */
    atk_max30102_write_byte(MAX30102_TEMP_CONFIG, 0x01);    /* ʹ���¶ȴ����� ��ȡ�¶�ֵ���Զ���0 */
    delay_ms(100);                                          /* �ȴ��¶ȴ������ȶ� */
    
    /* �����ʼ��ʱ��ͨ����ȡ���ж�״̬�Ĵ��������жϱ�־λ��� */
    atk_max30102_read_byte(MAX30102_INTR_STATUS_1);
    atk_max30102_read_byte(MAX30102_INTR_STATUS_2);
}

/*******************************************************************************************/
#if 0
/**
 * @brief   ��ȡ����ֵ
 * @param   input_data ������FIR�˲���ĺ��������    
 * @param   cache_nums ����������    
 * @note    �����˲����źţ���������������Եķ�ֵ��ÿ����ֵ����һ��������
 *          �趨һ����ֵ��һ��Ϊ�ɼ�������ƽ��ֵ�������źų�������ֵʱ��ʶ��Ϊ��ֵ
 *          ��������������Ĳ�ֵ��Ϊ��׼ȷ��ͬ����βɼ�ȡƽ�� �õ��ȶ������β������ֵ�����������ļ��ʱ�� = �����������ֵ/����Ƶ�ʣ�
 *          �������ʹ�ʽ��bpm ��ÿ����������������Heart_Rate = 60 * ����Ƶ�� / �����������ֵ
 * @retval  ��������ֵ
 * @retval  ����0 ����ʶ��ʧ�ܣ�Ƥ��Ҫ���������ֲ���
 */
uint16_t atk_max30102_get_heart(float *input_data, uint16_t cache_nums)
{
    float input_data_sum_aver = 0;
    uint16_t i, temp;
    
    for(i = 0; i < cache_nums; i++)
    {
        input_data_sum_aver += *(input_data + i);
    }
    input_data_sum_aver = input_data_sum_aver / cache_nums;
    for(i = 0; i < cache_nums; i++)
    {
        /* ����ֵ */
        if((*(input_data + i) > input_data_sum_aver) && (*(input_data + i + 1) < input_data_sum_aver))
        {
            temp = i;       /* ��ǰ��ֵ��ʱ�䣨�����㣩 */
            break;
        }
    }
    i++;
    for(; i < cache_nums; i++)
    {
            if((*(input_data + i) > input_data_sum_aver) && (*(input_data + i + 1) < input_data_sum_aver))
            {
                temp = i - temp;
                break;
            }
    }
    /* ���� = 60 * sampling_rate / temp  ��λ��bpm(��/min) */

    return 3000 / temp;
}
#else
/**
 * @brief   ��ȡ����ֵ
 * @param   input_data ������FIR�˲���ĺ��������    
 * @param   cache_nums ����������    
 * @note    �����˲����źţ���������������Եķ�ֵ��ÿ����ֵ����һ��������
 *          �趨һ����ֵ��һ��Ϊ�ɼ�������ƽ��ֵ�������źų�������ֵʱ��ʶ��Ϊ��ֵ
 *          ��������������Ĳ�ֵ��Ϊ��׼ȷ��ͬ����βɼ�ȡƽ�� �õ��ȶ������β������ֵ�����������ļ��ʱ�� = �����������ֵ/����Ƶ�ʣ�
 *          �������ʹ�ʽ��bpm ��ÿ����������������Heart_Rate = 60 * ����Ƶ�� / �����������ֵ
 * @retval  ��������ֵ
 * @retval  ����0 ����ʶ��ʧ�ܣ�Ƥ��Ҫ���������ֲ���
 */
uint16_t atk_max30102_get_heart(float *input_data, uint16_t cache_nums)
{
    float input_data_sum_aver = 0;
    uint16_t i, temp;
    int num_peaks = 0;
    int current_peak_time = 0;
    int last_peak_time = 0;
    int total_intervals = 0;
    
    for(i = 0; i < cache_nums; i++)
    {
        input_data_sum_aver += *(input_data + i);
    }
    input_data_sum_aver = input_data_sum_aver / cache_nums; /* ƽ��ֵ���������ֵ */
    
    for(i = 0; i < cache_nums; i++)
    {
        /* ����ֵ */
        if((*(input_data + i) > input_data_sum_aver) && (*(input_data + i + 1) < input_data_sum_aver))
        {
            current_peak_time = i;                          /* ��¼��ǰ������ */
            if(last_peak_time != 0)
            {
                temp = current_peak_time - last_peak_time;  /* ���������������Ĳ� */
                total_intervals += temp;                    /* ���ۼ� */
                num_peaks++;                                /* �����ۼӣ�����ȡƽ������ */
            }
            last_peak_time = current_peak_time;             /* ��¼�ϴβ����㣬�Ա��´μ���������ֵ������Ĳ� */
        }
    }

    /* ��λ��bpm(��/min) ���� = 60 * sampling_rate / �����������Ĳ�ֵ   ����sampling_rate = ������/�������� 200/4 = 50HZ */
    if(num_peaks > 0)
    {
        int avg_interval = total_intervals / num_peaks;     /* �õ�������ֵ���ƽ���� */
        return (3000 / avg_interval);                       /* �������� */
    }
    return 0;
}
#endif

/**
 * @brief   ��ȡѪ��ֵ
 * @param   ir_input_data ������FIR�˲���ĺ��������    
 * @param   red_input_data ������FIR�˲���ĺ������    
 * @param   cache_nums ����������   
 * @note    ����ͨ���Ժ��ͺ�����źŵ������Сֵ��������AC��DC����
 *          Ȼ��ͨ����ʽ�����R = (ACred/DCred) / (ACir/DCir) ��
 *          ������Rֵʹ�ù�ʽSpO2 = aR^2 + bR + c  ����a b c�Ǹ��ݴ���������ϵó���ϵ����
 * @retval  Ѫ��ֵ
 */
float atk_max30102_get_spo2(float *ir_input_data, float *red_input_data, uint16_t cache_nums)
{
    float ir_max = *ir_input_data, ir_min = *ir_input_data;
    float red_max = *red_input_data, red_min = *red_input_data;
    float R;        /* ����R */
    uint16_t i;
    
    /* Ѱ�Һ��ͺ��������ֵ����Сֵ */
    for(i = 1; i < cache_nums; i++)
    {
        if(ir_max < *(ir_input_data + i))
        {
            ir_max = *(ir_input_data + i);
        }
        if(ir_min > *(ir_input_data + i))
        {
            ir_min = *(ir_input_data + i);
        }
        if(red_max < *(red_input_data + i))
        {
            red_max = *(red_input_data + i);
        }
        if(red_min > *(red_input_data + i))
        {
            red_min = *(red_input_data + i);
        }
    }

    /* (ir_max - ir_min) �� (red_max - red_min) ��ʾ������ͺ���źŵ�AC�����������ɷ֣��� */
    /* (red_max + red_min) �� (ir_max + ir_min)����ʾ�����ͺ����źŵ�DC��������̬�ɷ֣� */
    R = ((ir_max + ir_min) * (red_max - red_min)) / ((red_max + red_min) * (ir_max - ir_min));
    return ((-45.060f) * R * R + 30.354f * R + 94.845f);    /* SpO2 = aR^2 + bR + c */
}

/*******************************************************************************************/

/**
 * @brief       INT �ⲿ�жϷ������
 * @param       ��
 * @retval      ��
 */
void INT_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(INT_GPIO_PIN);         /* �����жϴ����ú��� ���INT�����ж��� ���жϱ�־λ */
    __HAL_GPIO_EXTI_CLEAR_IT(INT_GPIO_PIN);         /* HAL��Ĭ�������ж��ٴ���ص����˳�ʱ����һ���жϣ����ⰴ�������󴥷� */
}

/**
 * @brief       �жϷ����������Ҫ��������
                ��HAL�������е��ⲿ�жϷ�����������ô˺���
 * @param       GPIO_Pin:�ж����ź�
 * @retval      ��
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT_GPIO_PIN)
    {
        g_max30102_int_flag = 1;
    }
}



