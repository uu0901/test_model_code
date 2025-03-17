/**
 ****************************************************************************************************
 * @file        atk_rtc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-10-17
 * @brief       RTC模块 驱动代码
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
 * 修改说明
 * V1.0 20241017
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/ATK_RTC/atk_rtc.h"
#include "./BSP/IIC/myiic.h"
#include "./SYSTEM/usart/usart.h"


/**
 * @brief   写一个字节到SD3078的寄存器
 * @param   reg: 寄存器地址
 * @param   data: 寄存器数据
 * @retval  写入结果
 * @arg     0: 成功
 * @arg     1: 失败
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
    iic_send_byte(data);        /* 发送一字节 */
    if (iic_wait_ack() != 0)
    {
        iic_stop();
        return 1;
    }
    iic_stop();                 /* 产生一个停止条件 */
    
    return 0;
}

/**
 * @brief   在SD3078指定寄存器地址读出一个数据
 * @param   reg:        寄存器地址
 * @retval  读到的数据
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
 * @brief   从SD3078读取N字节数据
 * @param   reg:        寄存器地址
 * @param   date:       数据存储buf
 * @param   len:        数据长度
 * @retval  读出结果
 * @retval  0, 操作成功
 *          其他, 操作失败
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
 * @brief   SD3078写入N字节数据
 * @param   reg:        寄存器地址
 * @param   data:       写入数据
 * @param   len:        数据长度
 * @retval  写入结果
 * @arg     0:          成功
 * @arg     1:          失败
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

/************************************************** SD3078驱动部分 *********************************************************/
/**
 * @brief   将BCD数据格式转成16进制数据
 * @param   bcd：BCD格式的数据
 * @retval  转换结果
 */
static uint8_t bcd_to_hex(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0f);
}

/**
 * @brief   将16进制数据转成BCD数据格式
 * @param   hex：16进制格式的数据
 * @retval  转换结果
 */
static uint8_t hex_to_bcd(uint8_t hex)
{
    return ((hex / 10) << 4) | (hex % 10);
}


/**
 * @brief   RTC写使能
 * @param   无
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
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
 * @brief   RTC写禁止
 * @param   无
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
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
 * @brief   设置RTC实时时钟数据
 * @param   *rtc_time ： 时间结构体指针
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_set_time(sd3078_time_t *rtc_time)
{
    /* 用于存放秒、分、小时、星期、日、月、年 */
    uint8_t data[7];    
    /* 填充数组，确保每个字段对应正确的寄存器地址 */
    data[0] = hex_to_bcd(rtc_time->second);         /* second */
    data[1] = hex_to_bcd(rtc_time->minute);         /* minute */
    data[2] = hex_to_bcd(rtc_time->hour);           /* hour  */
    if (rtc_time->hour_type == SD3078_HOUR_24)  
    {
        data[2] |= SD3078_12_24_BIT;                /* 如果为24小时制，则设置为24小时制 */
    }
    else if(rtc_time->am_pm == SD3078_PM)           /* 如果为12进制，还需判断设置成AM还是PM格式 */
    {
        data[2] |= SD3078_AM_PM_BIT;
    }    
    data[3] = hex_to_bcd(rtc_time->week);           /* week */
    data[4] = hex_to_bcd(rtc_time->day);            /* day */
    data[5] = hex_to_bcd(rtc_time->month);          /* month */
    data[6] = hex_to_bcd(rtc_time-> year);          /* year */
    
    atk_rtc_write_enable();                         /* 写使能 */

    if (atk_rtc_write_nbytes(SD3078_REG_SEC, data, sizeof(data)) != 0)  /* 设置时间数据 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();                        /* 写禁止 */

    return 0;
}

/**
 * @brief   读取RTC实时时间
 * @param   *rtc_time ： 存储时间结构体指针
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_get_time(sd3078_time_t *rtc_time)
{
    uint8_t data[7];   
    if (atk_rtc_read_nbytes(SD3078_REG_SEC, data, sizeof(data)) != 0)
    {
        return 1;   /* 读取失败 */
    }
    /* 将读取到的数据填充到 time_t 结构体中 */
    rtc_time->second = bcd_to_hex(data[0] & 0x7f);      /* 对应秒 */
    rtc_time->minute = bcd_to_hex(data[1] & 0x7f);      /* 对应分 */
    if ((data[2] & SD3078_12_24_BIT) != 0)     
    {
        rtc_time->am_pm = SD3078_AM_PM_NULL;
        rtc_time->hour_type = SD3078_HOUR_24;           
        rtc_time->hour = bcd_to_hex(data[2] & 0x3f);    /* 对应小时 */ 
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
    rtc_time->week   = bcd_to_hex(data[3] & 0x07);      /* 对应星期 */
    rtc_time->day    = bcd_to_hex(data[4] & 0x3f);      /* 对应日 */
    rtc_time->month  = bcd_to_hex(data[5] & 0x1f);      /* 对应月 */
    rtc_time->year   = bcd_to_hex(data[6]);             /* 对应年 */
    return 0;
}

/**
 * @brief   设置倒计时中断
 * @param   *count_down ： 倒计时中断结构体指针
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_set_countdown_interrupt(count_down_t *count_down)
{
    uint8_t buf[3];
    /* 计数器最大只有24位以及计数频率均不得超过SD3078的合法值 */
    if((count_down->count > 0xFFFFFF) || (count_down->source > SD3078_COUNT_DOWN_1_60))
    {
        return 1;
    }
    atk_rtc_write_enable();                                 /* 写使能 */

    if (atk_rtc_read_nbytes(SD3078_REG_CTR2, buf, 2) != 0)  /* 读取CTR2和CTR3寄存器的原始值 */
    {
        atk_rtc_write_disable();
        return 1;   
    } 
    
    /* 先将CTR2的INTDE清0 */
    buf[0] &= ~SD3078_INTDE;   
    if (atk_rtc_write_nbytes(SD3078_REG_CTR2, buf, 1) != 0)  /* 清除INTDE位 先关闭中断 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    /* 设置INTDE INTS1 INTS0 IM位，即开启倒计时中断 */
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
    /* 配置CTR3的TDS1和TDS0位，选择计数频率 */
    buf[1] &=  ~(0x03 << 4);   
    buf[1] |=  (count_down->source  << 4);  
    if (atk_rtc_write_nbytes(SD3078_REG_CTR2, buf, 2) != 0)  /* 设置好倒计时中断计数频率以及使能中断和输出 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    buf[0] = count_down->count & 0xff;
    buf[1] = (count_down->count >> 8 ) & 0xff;
    buf[2] = (count_down->count >> 16 ) & 0xff;
    if (atk_rtc_write_nbytes(SD3078_REG_TD0, buf, 3) != 0)  /* 设置倒计时值 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}

/**
 * @brief   设置报警中断（闹钟功能）
 * @param   *alarm ： 报警中断结构体指针
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
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
    buf[7] = alarm->enable_a & 0x7f;                                /* 报警中断类型，多个类型可以或起来 */                   
    
    atk_rtc_write_enable();                                         /* 写使能 */
    if (atk_rtc_write_nbytes(SD3078_REG_ALARM_SEC, buf, 8) != 0)    /* 设置闹钟日期时间 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    if (atk_rtc_read_nbytes(SD3078_REG_CTR2, buf, 1) != 0)          /* 读取CTR2的原始值 */
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
    if (atk_rtc_write_nbytes(SD3078_REG_CTR2, buf, 1) != 0)         /* 设置CTR2寄存器 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;         
}

/**
 * @brief   设置频率中断（INT输出频率方波）
 * @param   *clk ：频率中断结构体指针
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_set_clk_interrupt(sd3078_clk_out_t *clk)
{
    uint8_t buf[2];
    if(clk->freq > SD3078_CLK_OUT_1_SEC)  
    {
        return 1;
    } 
    atk_rtc_write_enable();                                     /* 写使能 */
    if (atk_rtc_read_nbytes(SD3078_REG_CTR2, buf, 2) != 0)      /* 读取CTR2和CTR3的原始值 */
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
    if (atk_rtc_write_nbytes(SD3078_REG_CTR2, buf, 2) != 0)     /* 设置CTR2寄存器 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}

/****************************************************************************************/
/**
 * @brief   电池充电控制
 * @param   *ctrl ：充电电池结构体指针
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
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
    atk_rtc_write_enable();                                     /* 写使能 */
    if (atk_rtc_write_nbytes(SD3078_REG_CHARGE, &buf, 1) != 0)  /* 设置CTR2寄存器 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}

/**
 * @brief   读取电池电压
 * @param   *vol ： 电池电压缓冲区
 * @retval  电池电压
 */
uint8_t atk_rtc_get_battery_voltage(uint16_t * vol)
{
    uint8_t buf[2];
    
    if (atk_rtc_read_nbytes(SD3078_REG_CTR5, buf, 2) != 0)      /* 读取CTR5的原始值 */
    {
        return 1;   
    } 
    *vol = (((buf[0] & 0x80 ) << 1) | buf[1]);
    return 0;
}

/**
 * @brief   备用电池供电状态下，IIC通信使能控制
 * @param   en ： 1 使能， 0 失能
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
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
    atk_rtc_write_enable();                                         /* 写使能 */
    if (atk_rtc_write_nbytes(SD3078_REG_I2C_CTRL, &buf, 1) != 0)    /* 设置寄存器 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}

/**
 * @brief   电池充电使能
 * @param   无
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_battery_charge_enable(void)
{
    sd3078_charge_t charge;
    charge.chage_en = 1;            /* 允许充电功能 */
    charge.resistance = 0x02;       /* 限流电阻2k */
    if(atk_rtc_battery_charge_ctrl(&charge) != 0)
    {
        return 1;
    }
    return 0;
}

/*******************************************************************************************/
/**
 * @brief   获取SD3078芯片温度
 * @param   *temp ： 芯片温度缓冲区
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_get_temperature(uint8_t *temp)
{
    uint8_t buf;

    if (atk_rtc_read_nbytes(SD3078_REG_TEMP, &buf, 1) != 0)     /* 读取温度数据 */
    {
        return 1;   
    } 
    *temp = buf;
    return 0;
}

/**
 * @brief   获取SD3078芯片设备ID
 * @param   *buf ：ID缓冲区
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_read_device_id(uint8_t *buf)
{
    if (atk_rtc_read_nbytes(SD3078_REG_DEVICE_ID, buf, 8) != 0)     /* 读取设备ID */
    {
        return 1;   
    } 
    return 0;
}

/**
 * @brief   读取用户RAM区
 * @param   reg  ：RAM地址
 * @param   *buf ：数据缓冲区
 * @param   len  ：读取的数据长度
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t sd3078_read_ram(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* 判断数据合法性 */
    if (reg < SD3078_REG_USER_RAM_START || reg > SD3078_REG_USER_RAM_END || len > 70)
    {
        return 1;
    }
    if (atk_rtc_read_nbytes(reg, buf, len) != 0)     /* 读取RAM数据 */
    {
        return 1;   
    } 
    return 0;
}

/**
 * @brief   写入用户RAM区
 * @param   reg  ：RAM地址
 * @param   *buf ：数据缓冲区
 * @param   len  ：读取的数据长度
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t sd3078_write_ram(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* 判断数据合法性 */
    if (reg < SD3078_REG_USER_RAM_START || reg > SD3078_REG_USER_RAM_END || len > 70)
    {
        return 1;
    }
    atk_rtc_write_enable();                             /* 写使能 */
    
    if (atk_rtc_write_nbytes(reg, buf, len) != 0)       /* 设置寄存器 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }
    atk_rtc_write_disable();
    return 0;
}
/************************************** 中断标志获取与清除 *******************************************/

/**
 * @brief   清除标志位
 * @param   flag ：标志类型
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_flag_clear(uint8_t flag)
{
    uint8_t buf;

    atk_rtc_write_enable();                                     /* 写使能 */

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
    if (atk_rtc_write_nbytes(SD3078_REG_CTR1, &buf, 1) != 0)    /* 设置寄存器 */
    {
        atk_rtc_write_disable();
        return 1;                           
    }

    atk_rtc_write_disable();
    return 0;
}

/**
 * @brief   获取标志位
 * @param   flag ：标志类型
 * @retval  结果
 * @arg     0: 获取标志成功
 * @arg     1: 获取标志失败
 */
uint8_t atk_rtc_flag_get(sd3078_flag_e flag)
{
    uint8_t sta = 0;

    atk_rtc_read_nbytes(SD3078_REG_CTR1, &sta, 1);      /* 读取CTR1数据 */

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
 * @brief   设置RTC模块的初始时间
 * @param   无
 * @retval  结果
 * @arg     0: 成功
 * @arg     1: 失败
 */
uint8_t atk_rtc_init_time(void)
{
    uint8_t res = 0;
    uint8_t rtc_init_flag = 0;
    sd3078_time_t time_buf;

    /* 防止复位重复初始化RTC时间 */
    res = sd3078_read_ram(SD3078_REG_USER_RAM_START, &rtc_init_flag, 1);    
    if(rtc_init_flag != 0xBB)
    {
        /* 设置初始时间为：2024年10月11号，16点49分50秒 星期五 */
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
 * @brief       初始化SD3078
 * @param       无
 * @retval      检测结果
 *              0: 检测成功
 *              1: 检测失败
 */
uint8_t atk_rtc_init(void)
{
    uint8_t sd3078_id[8], res = 0;
    uint8_t temp = 0;
    uint16_t vol = 0;
    /* 初始化IIC接口 */
    iic_init();                               
    
    /* 读取设备ID */
    res = atk_rtc_read_device_id(sd3078_id);
    if((res == 0) && (sd3078_id[1] != 0))   /* 读取成功并且设备ID月份不等于0 */
    {
        printf ("device id = %x-%x-%x-%x-%x-%x-%x-%x\r\n", sd3078_id[0], sd3078_id[1], sd3078_id[2], \
                sd3078_id[3], sd3078_id[4], sd3078_id[5], sd3078_id[6], sd3078_id[7]);
    }
    else
    {
        return 1;
    }
    atk_rtc_battery_charge_enable();        /* 使能充电 */
    atk_rtc_init_time();                    /* 设置初始时间 */
    
    res = atk_rtc_get_temperature(&temp);   /* 获取RTC芯片温度值 */
    if(res == 0)
    {
        printf("temperature :%d ℃\r\n",temp);
    }
    res = atk_rtc_get_battery_voltage(&vol);/* 读取备用电池电压值 */    
    if(res == 0)
    {
        printf("battery voltage: %.2fv \r\n", (float)vol / 100.0f);
    }
    return res;
}

