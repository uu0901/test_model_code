/**
 ****************************************************************************************************
 * @file        atk_rtc.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2024-10-17
 * @brief       RTCģ�� ��������
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

#include "./BSP/ATK_RTC/atk_rtc.h"
#include "./BSP/IIC/myiic.h"
#include "./SYSTEM/usart/usart.h"


/**
 * @brief   дһ���ֽڵ�SD3078�ļĴ���
 * @param   reg: �Ĵ�����ַ
 * @param   data: �Ĵ�������
 * @retval  д����
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_write_byte(uint8_t reg, uint8_t data)
{
    iic_start();
    iic_send_byte(RTC_I2C_ADDR | 0x00);
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
 * @brief   ��SD3078ָ���Ĵ�����ַ����һ������
 * @param   reg:        �Ĵ�����ַ
 * @retval  ����������
 */
uint8_t atk_rtc_read_byte(uint8_t reg)
{
    uint8_t temp = 0;
    
    iic_start();
    iic_send_byte(RTC_I2C_ADDR | 0x00);
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
    iic_send_byte(RTC_I2C_ADDR | 0x01);
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
 * @brief   ��SD3078��ȡN�ֽ�����
 * @param   reg:        �Ĵ�����ַ
 * @param   date:       ���ݴ洢buf
 * @param   len:        ���ݳ���
 * @retval  �������
 * @retval  0, �����ɹ�
 *          ����, ����ʧ��
 */
int atk_rtc_read_nbytes(uint8_t reg, uint8_t *date, uint8_t len)
{
    uint8_t i;
    
    iic_start();
    iic_send_byte(RTC_I2C_ADDR | 0x00);
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
    iic_send_byte(RTC_I2C_ADDR | 0x01);
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
 * @brief   SD3078д��N�ֽ�����
 * @param   reg:        �Ĵ�����ַ
 * @param   data:       д������
 * @param   len:        ���ݳ���
 * @retval  д����
 * @arg     0:          �ɹ�
 * @arg     1:          ʧ��
 */
uint8_t atk_rtc_write_nbytes(uint8_t reg, uint8_t* data, uint8_t len)
{
    uint8_t i;
    
    iic_start();
    iic_send_byte(RTC_I2C_ADDR | 0x00);
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

/************************************************** SD3078�������� *********************************************************/
/**
 * @brief   ��BCD���ݸ�ʽת��16��������
 * @param   bcd��BCD��ʽ������
 * @retval  ת�����
 */
static uint8_t bcd_to_hex(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0f);
}

/**
 * @brief   ��16��������ת��BCD���ݸ�ʽ
 * @param   hex��16���Ƹ�ʽ������
 * @retval  ת�����
 */
static uint8_t hex_to_bcd(uint8_t hex)
{
    return ((hex / 10) << 4) | (hex % 10);
}


/**
 * @brief   RTCдʹ��
 * @param   ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_write_enable(void)
{
    if(atk_rtc_write_byte(SD3078_REG_CTR2, 0x80) != 0)
    {
        return 1;
    }
    if(atk_rtc_write_byte(SD3078_REG_CTR1, 0x84) != 0)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief   RTCд��ֹ
 * @param   ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_write_disable(void)
{
    uint8_t buf[2] = {0x00, 0x00};
    if(atk_rtc_write_nbytes(SD3078_REG_CTR1, buf, 2) != 0)  
    {
        return 1;
    }
    return 0;
}

/**
 * @brief   ����RTCʵʱʱ������
 * @param   *rtc_time �� ʱ��ṹ��ָ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_set_time(sd3078_time_t *rtc_time)
{
    /* ���ڴ���롢�֡�Сʱ�����ڡ��ա��¡��� */
    uint8_t data[7];    
    /* ������飬ȷ��ÿ���ֶζ�Ӧ��ȷ�ļĴ�����ַ */
    data[0] = hex_to_bcd(rtc_time->second);         /* second */
    data[1] = hex_to_bcd(rtc_time->minute);         /* minute */
    data[2] = hex_to_bcd(rtc_time->hour);           /* hour  */
    if (rtc_time->hour_type == SD3078_HOUR_24)  
    {
        data[2] |= SD3078_12_24_BIT;                /* ���Ϊ24Сʱ�ƣ�������Ϊ24Сʱ�� */
    }
    else if(rtc_time->am_pm == SD3078_PM)           /* ���Ϊ12���ƣ������ж����ó�AM����PM��ʽ */
    {
        data[2] |= SD3078_AM_PM_BIT;
    }    
    data[3] = hex_to_bcd(rtc_time->week);           /* week */
    data[4] = hex_to_bcd(rtc_time->day);            /* day */
    data[5] = hex_to_bcd(rtc_time->month);          /* month */
    data[6] = hex_to_bcd(rtc_time-> year);          /* year */
    
    atk_rtc_write_enable();                         /* дʹ�� */

    if (atk_rtc_write_nbytes(SD3078_REG_SEC, data, sizeof(data)) != 0)  /* ����ʱ������ */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();                        /* д��ֹ */

    return 0;
}

/**
 * @brief   ��ȡRTCʵʱʱ��
 * @param   *rtc_time �� �洢ʱ��ṹ��ָ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_get_time(sd3078_time_t *rtc_time)
{
    uint8_t data[7];   
    if (atk_rtc_read_nbytes(SD3078_REG_SEC, data, sizeof(data)) != 0)
    {
        return 1;   /* ��ȡʧ�� */
    }
    /* ����ȡ����������䵽 time_t �ṹ���� */
    rtc_time->second = bcd_to_hex(data[0] & 0x7f);      /* ��Ӧ�� */
    rtc_time->minute = bcd_to_hex(data[1] & 0x7f);      /* ��Ӧ�� */
    if ((data[2] & SD3078_12_24_BIT) != 0)     
    {
        rtc_time->am_pm = SD3078_AM_PM_NULL;
        rtc_time->hour_type = SD3078_HOUR_24;           
        rtc_time->hour = bcd_to_hex(data[2] & 0x3f);    /* ��ӦСʱ */ 
    }
    else                                       
    {
        if ((data[2] & SD3078_AM_PM_BIT) != 0)
        {
            rtc_time->am_pm = SD3078_PM;
        }
        else
        {
            rtc_time->am_pm = SD3078_AM;
        }
        rtc_time->hour_type = SD3078_HOUR_12;
        rtc_time->hour = bcd_to_hex(data[2] & 0x1f);
    }   
    rtc_time->week   = bcd_to_hex(data[3] & 0x07);      /* ��Ӧ���� */
    rtc_time->day    = bcd_to_hex(data[4] & 0x3f);      /* ��Ӧ�� */
    rtc_time->month  = bcd_to_hex(data[5] & 0x1f);      /* ��Ӧ�� */
    rtc_time->year   = bcd_to_hex(data[6]);             /* ��Ӧ�� */
    return 0;
}

/**
 * @brief   ���õ���ʱ�ж�
 * @param   *count_down �� ����ʱ�жϽṹ��ָ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_set_countdown_interrupt(count_down_t *count_down)
{
    uint8_t buf[3];
    /* ���������ֻ��24λ�Լ�����Ƶ�ʾ����ó���SD3078�ĺϷ�ֵ */
    if((count_down->count > 0xFFFFFF) || (count_down->source > SD3078_COUNT_DOWN_1_60))
    {
        return 1;
    }
    atk_rtc_write_enable();                                 /* дʹ�� */

    if (atk_rtc_read_nbytes(SD3078_REG_CTR2, buf, 2) != 0)  /* ��ȡCTR2��CTR3�Ĵ�����ԭʼֵ */
    {
        atk_rtc_write_disable();
        return 1;   
    } 
    
    /* �Ƚ�CTR2��INTDE��0 */
    buf[0] &= ~SD3078_INTDE;   
    if (atk_rtc_write_nbytes(SD3078_REG_CTR2, buf, 1) != 0)  /* ���INTDEλ �ȹر��ж� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    /* ����INTDE INTS1 INTS0 IMλ������������ʱ�ж� */
    if (count_down->ie == 1)    
    {
        buf[0] |= SD3078_INTDE;    
        buf[0] |= SD3078_INTS1;   
        buf[0] |= SD3078_INTS0; 
    }
    if (count_down->int_period == 1)
    {
        buf[0] |= SD3078_IM;    
    }
    else
    {
        buf[0] &= ~SD3078_IM;  
    }
    /* ����CTR3��TDS1��TDS0λ��ѡ�����Ƶ�� */
    buf[1] &=  ~(0x03 << 4);   
    buf[1] |=  (count_down->source  << 4);  
    if (atk_rtc_write_nbytes(SD3078_REG_CTR2, buf, 2) != 0)  /* ���úõ���ʱ�жϼ���Ƶ���Լ�ʹ���жϺ���� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    buf[0] = count_down->count & 0xff;
    buf[1] = (count_down->count >> 8 ) & 0xff;
    buf[2] = (count_down->count >> 16 ) & 0xff;
    if (atk_rtc_write_nbytes(SD3078_REG_TD0, buf, 3) != 0)  /* ���õ���ʱֵ */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}

/**
 * @brief   ���ñ����жϣ����ӹ��ܣ�
 * @param   *alarm �� �����жϽṹ��ָ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_set_alarm_interrupt(sd3078_time_alarm_t *alarm)
{
    uint8_t buf[8];
    
    buf[0] = hex_to_bcd(alarm->sec_a);
    buf[1] = hex_to_bcd(alarm->min_a);
    buf[2] = hex_to_bcd(alarm->hour_a);
    buf[3] = alarm->week_a & 0x7f;
    buf[4] = hex_to_bcd(alarm->day_a);
    buf[5] = hex_to_bcd(alarm->mon_a);
    buf[6] = hex_to_bcd(alarm->year_a);
    buf[7] = alarm->enable_a & 0x7f;                                /* �����ж����ͣ�������Ϳ��Ի����� */                   
    
    atk_rtc_write_enable();                                         /* дʹ�� */
    if (atk_rtc_write_nbytes(SD3078_REG_ALARM_SEC, buf, 8) != 0)    /* ������������ʱ�� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    if (atk_rtc_read_nbytes(SD3078_REG_CTR2, buf, 1) != 0)          /* ��ȡCTR2��ԭʼֵ */
    {
        atk_rtc_write_disable();
        return 1;   
    } 
    buf[0] = buf[0];    
    buf[0] &= ~SD3078_INTS1;    
    buf[0] |= SD3078_INTS0; 

    if(alarm->ie_a == 1)    
    {
        buf[0] |= SD3078_INTAE; 
    }
    else
    {
        buf[0] &= ~SD3078_INTAE;                         
    }
    if(alarm->int_period == 1)                         
    {
        buf[0] |= SD3078_IM;    
    }   
    else
    {
        buf[0] &= ~SD3078_IM;   
    }
    if (atk_rtc_write_nbytes(SD3078_REG_CTR2, buf, 1) != 0)         /* ����CTR2�Ĵ��� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;         
}

/**
 * @brief   ����Ƶ���жϣ�INT���Ƶ�ʷ�����
 * @param   *clk ��Ƶ���жϽṹ��ָ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_set_clk_interrupt(sd3078_clk_out_t *clk)
{
    uint8_t buf[2];
    if(clk->freq > SD3078_CLK_OUT_1_SEC)  
    {
        return 1;
    } 
    atk_rtc_write_enable();                                     /* дʹ�� */
    if (atk_rtc_read_nbytes(SD3078_REG_CTR2, buf, 2) != 0)      /* ��ȡCTR2��CTR3��ԭʼֵ */
    {
        atk_rtc_write_disable();
        return 1;   
    } 
    buf[1] &= 0xf0;
    buf[1] |= clk->freq;  
    if(clk->oe == 1)
    {
        buf[0] |= SD3078_INTFE;   
    }
    else
    {
        buf[0] &= ~SD3078_INTFE;
    }
    buf[0] |= SD3078_INTS1;     
    buf[0] &= ~SD3078_INTS0;
    if (atk_rtc_write_nbytes(SD3078_REG_CTR2, buf, 2) != 0)     /* ����CTR2�Ĵ��� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}

/****************************************************************************************/
/**
 * @brief   ��س�����
 * @param   *ctrl ������ؽṹ��ָ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_battery_charge_ctrl(sd3078_charge_t *ctrl)
{
    uint8_t buf;
    
    if (ctrl->chage_en > 1)   
    {
        return 1;
    }
    if (ctrl->resistance > 3)   
    {
        return 1;
    }    

    buf = (ctrl->chage_en << 7) | ctrl->resistance;
    atk_rtc_write_enable();                                     /* дʹ�� */
    if (atk_rtc_write_nbytes(SD3078_REG_CHARGE, &buf, 1) != 0)  /* ����CTR2�Ĵ��� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}

/**
 * @brief   ��ȡ��ص�ѹ
 * @param   *vol �� ��ص�ѹ������
 * @retval  ��ص�ѹ
 */
uint8_t atk_rtc_get_battery_voltage(uint16_t * vol)
{
    uint8_t buf[2];
    
    if (atk_rtc_read_nbytes(SD3078_REG_CTR5, buf, 2) != 0)      /* ��ȡCTR5��ԭʼֵ */
    {
        return 1;   
    } 
    *vol = (((buf[0] & 0x80 ) << 1) | buf[1]);
    return 0;
}

/**
 * @brief   ���õ�ع���״̬�£�IICͨ��ʹ�ܿ���
 * @param   en �� 1 ʹ�ܣ� 0 ʧ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_battery_i2c_ctrl(uint8_t en)
{
    uint8_t buf;

    if(en != 0 && en != 1) 
    {
        return 1;
    }        
    if(en)
    {
        buf = 0x80;
    } 
    else
    {
        buf =0x00;
    }
    atk_rtc_write_enable();                                         /* дʹ�� */
    if (atk_rtc_write_nbytes(SD3078_REG_I2C_CTRL, &buf, 1) != 0)    /* ���üĴ��� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}

/**
 * @brief   ��س��ʹ��
 * @param   ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_battery_charge_enable(void)
{
    sd3078_charge_t charge;
    charge.chage_en = 1;            /* �����繦�� */
    charge.resistance = 0x02;       /* ��������2k */
    if(atk_rtc_battery_charge_ctrl(&charge) != 0)
    {
        return 1;
    }
    return 0;
}

/*******************************************************************************************/
/**
 * @brief   ��ȡSD3078оƬ�¶�
 * @param   *temp �� оƬ�¶Ȼ�����
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_get_temperature(uint8_t *temp)
{
    uint8_t buf;

    if (atk_rtc_read_nbytes(SD3078_REG_TEMP, &buf, 1) != 0)     /* ��ȡ�¶����� */
    {
        return 1;   
    } 
    *temp = buf;
    return 0;
}

/**
 * @brief   ��ȡSD3078оƬ�豸ID
 * @param   *buf ��ID������
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_read_device_id(uint8_t *buf)
{
    if (atk_rtc_read_nbytes(SD3078_REG_DEVICE_ID, buf, 8) != 0)     /* ��ȡ�豸ID */
    {
        return 1;   
    } 
    return 0;
}

/**
 * @brief   ��ȡ�û�RAM��
 * @param   reg  ��RAM��ַ
 * @param   *buf �����ݻ�����
 * @param   len  ����ȡ�����ݳ���
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t sd3078_read_ram(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* �ж����ݺϷ��� */
    if (reg < SD3078_REG_USER_RAM_START || reg > SD3078_REG_USER_RAM_END || len > 70)
    {
        return 1;
    }
    if (atk_rtc_read_nbytes(reg, buf, len) != 0)     /* ��ȡRAM���� */
    {
        return 1;   
    } 
    return 0;
}

/**
 * @brief   д���û�RAM��
 * @param   reg  ��RAM��ַ
 * @param   *buf �����ݻ�����
 * @param   len  ����ȡ�����ݳ���
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t sd3078_write_ram(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* �ж����ݺϷ��� */
    if (reg < SD3078_REG_USER_RAM_START || reg > SD3078_REG_USER_RAM_END || len > 70)
    {
        return 1;
    }
    atk_rtc_write_enable();                             /* дʹ�� */
    
    if (atk_rtc_write_nbytes(reg, buf, len) != 0)       /* ���üĴ��� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}
/************************************** �жϱ�־��ȡ����� *******************************************/

/**
 * @brief   �����־λ
 * @param   flag ����־����
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_flag_clear(uint8_t flag)
{
    uint8_t buf;

    atk_rtc_write_enable();                                     /* дʹ�� */

    switch (flag)
    {
        case SD3078_OSF_FLAG:
            buf = ~0x40;
            break;
        case SD3078_ALARM_TIME_FLAG://alarm time flag
            buf = ~0x20;
            break;
        case SD3078_COUNT_DWON_FLAG: //count down flag
            buf = ~0x10;
            break;
        case SD3078_BLF_FLAG:
            buf = ~0x08;
            break;
        case SD3078_PMF_FLAG:
            buf = ~0x02;
            break;
        case SD3078_RTCF_FLAG:
            buf = ~0x01;
            break;
        default:
            return 1;
    }
    buf |= 0x84;
    if (atk_rtc_write_nbytes(SD3078_REG_CTR1, &buf, 1) != 0)    /* ���üĴ��� */
    {
        atk_rtc_write_disable();
        return 1;                           
    }

    atk_rtc_write_disable();
    return 0;
}

/**
 * @brief   ��ȡ��־λ
 * @param   flag ����־����
 * @retval  ���
 * @arg     0: ��ȡ��־�ɹ�
 * @arg     1: ��ȡ��־ʧ��
 */
uint8_t atk_rtc_flag_get(sd3078_flag_e flag)
{
    uint8_t sta = 0;

    atk_rtc_read_nbytes(SD3078_REG_CTR1, &sta, 1);      /* ��ȡCTR1���� */

    switch (flag)
    {
        case SD3078_OSF_FLAG:
            sta &= 0x40;
            break;
        case SD3078_ALARM_TIME_FLAG:    /* alarm time flag */
            sta &= 0x20;
            break;
        case SD3078_COUNT_DWON_FLAG:    /* count down flag */
            sta &= 0x10;
            break;
        case SD3078_BLF_FLAG:
            sta &= 0x08;
            break;
        case SD3078_PMF_FLAG:
            sta &= 0x02;
            break;
        case SD3078_RTCF_FLAG:
            sta &= 0x01;
            break;
        default:
            break;
    }
    if(sta == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/*********************************************************************************/
/**
 * @brief   ����RTCģ��ĳ�ʼʱ��
 * @param   ��
 * @retval  ���
 * @arg     0: �ɹ�
 * @arg     1: ʧ��
 */
uint8_t atk_rtc_init_time(void)
{
    uint8_t res = 0;
    uint8_t rtc_init_flag = 0;
    sd3078_time_t time_buf;

    /* ��ֹ��λ�ظ���ʼ��RTCʱ�� */
    res = sd3078_read_ram(SD3078_REG_USER_RAM_START, &rtc_init_flag, 1);    
    if(rtc_init_flag != 0xBB)
    {
        /* ���ó�ʼʱ��Ϊ��2024��10��11�ţ�16��49��50�� ������ */
        time_buf.second = 50;
        time_buf.minute = 49;
        time_buf.hour = 16;
        time_buf.week = 5;
        time_buf.day = 11;
        time_buf.month = 10;
        time_buf.year = 24;
        time_buf.hour_type = SD3078_HOUR_24;  
        time_buf.am_pm = SD3078_AM_PM_NULL;
        res = atk_rtc_set_time(&time_buf);
        if (res == 0)
        {
            rtc_init_flag = 0xBB;
            sd3078_write_ram(SD3078_REG_USER_RAM_START, &rtc_init_flag, 1);
            printf("sd3078 set time success!\n");
            return 0;
        } 
    }
    return res;
}



/**
 * @brief       ��ʼ��SD3078
 * @param       ��
 * @retval      �����
 *              0: ���ɹ�
 *              1: ���ʧ��
 */
uint8_t atk_rtc_init(void)
{
    uint8_t sd3078_id[8], res = 0;
    uint8_t temp = 0;
    uint16_t vol = 0;
    /* ��ʼ��IIC�ӿ� */
    iic_init();                               
    
    /* ��ȡ�豸ID */
    res = atk_rtc_read_device_id(sd3078_id);
    if((res == 0) && (sd3078_id[1] != 0))   /* ��ȡ�ɹ������豸ID�·ݲ�����0 */
    {
        printf ("device id = %x-%x-%x-%x-%x-%x-%x-%x\r\n", sd3078_id[0], sd3078_id[1], sd3078_id[2], \
                sd3078_id[3], sd3078_id[4], sd3078_id[5], sd3078_id[6], sd3078_id[7]);
    }
    else
    {
        return 1;
    }
    atk_rtc_battery_charge_enable();        /* ʹ�ܳ�� */
    atk_rtc_init_time();                    /* ���ó�ʼʱ�� */
    
    res = atk_rtc_get_temperature(&temp);   /* ��ȡRTCоƬ�¶�ֵ */
    if(res == 0)
    {
        printf("temperature :%d ��\r\n",temp);
    }
    res = atk_rtc_get_battery_voltage(&vol);/* ��ȡ���õ�ص�ѹֵ */    
    if(res == 0)
    {
        printf("battery voltage: %.2fv \r\n", (float)vol / 100.0f);
    }
    return res;
}

