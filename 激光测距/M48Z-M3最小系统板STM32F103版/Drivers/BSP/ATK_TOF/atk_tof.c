/**
 ****************************************************************************************************
 * @file        atk_tof.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2024-11-01
 * @brief       ATK_TOF模块 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 M48Z-M3最小系统板STM32F103版
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "./BSP/ATK_TOF/atk_tof.h"
#include "VI530x_API.h"
#include "VI530x_Firmware.h"
#include "VI530x_System_Data.h"


/**
 * @brief       ATK-TOF模块硬件复位
 * @param       无
 * @retval      无
 */
void atk_tof_hw_reset(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    ATK_TOF_XSH_GPIO_CLK_ENABLE();
    
    /* 初始化XS引脚 */
    gpio_init_struct.Pin    = ATK_TOF_XSH_GPIO_PIN;
    gpio_init_struct.Mode   = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull   = GPIO_PULLUP;
    gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ATK_TOF_XSH_GPIO_PORT, &gpio_init_struct);
        
    ATK_TOF_XSH(0);         /* 拉低XS引脚 */
    delay_ms(30);           /* 延时 */
    ATK_TOF_XSH(1);         /* 拉高XS引脚 */
    delay_ms(30);           /* 延时 */
}

/**
 * @brief       ATK-TOF模块硬件初始化
 * @param       无
 * @retval      无
 */
void atk_tof_hw_init(void)
{
    VI530x_Status ret = VI530x_OK;
    
    /* 初始化IIC接口 */
    iic_init();
    
    /* 软件中断，GPIO引脚直接上拉 */
    VI530x_Cali_Data.VI530x_Interrupt_Mode_Status = 0x00;

    /* 初始化VI5300 */
    ret |= VI530x_Chip_Init();

    /* 写入VI5300固件 */
    ret |= VI530x_Download_Firmware((uint8_t *)VI5300_M31_firmware_buff, FirmwareSize());

    /* 配置标定值 */
    ret |= VI530x_Set_Californiation_Data(VI530x_Cali_Data.VI530x_Calibration_Offset);

    /* 开启温度校准, 0x00:关，0x01:开 */
    ret |= VI530x_Set_Sys_Temperature_Enable(0x01);
    
    /* 配置帧率，积分次数 */
    ret |= VI530x_Set_Integralcounts_Frame(30, 131072);

    /* 配置数据读取模式 */
#if CONTINUOUS_READ == 1
    ret |= VI530x_Start_Continue_Ranging_Cmd(); /* 连续模式 */
#else
    ret = VI530x_Start_Single_Ranging_Cmd();    /* 单次模式 */
#endif

    if(ret)
    {
        printf("VI5300初始化失败!\r\n");
    }
    else
    {
        printf("VI5300初始化成功!\r\n");
    }
}

/**
 * @brief   写N个字节到ATK-TOF模块的寄存器
 * @param   reg_addr:   寄存器地址
 * @param   data:       写入数据
 * @param   len:        数据长度
 * @retval  写入结果
 * @arg     0   : 成功
 * @arg     其他: 失败
 */
VI530x_Status atk_tof_write_nbytes(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    iic_start();                        /* 起始信号 */
    
    iic_send_byte(ATK_TOF_IIC_ADDR);    /* 发送IIC通讯地址 */
    if (iic_wait_ack() == 1)            /* 等待应答 */
    {
        iic_stop();
        return VI530x_IIC_ERROR;
    }
    
    iic_send_byte(reg_addr);            /* 发送寄存器地址 */
    if (iic_wait_ack() == 1)            /* 等待应答 */
    {
        iic_stop();
        return VI530x_IIC_ERROR;
    }
    
    for (uint16_t i = 0; i < len; i++)  /* 循环发送数据 */
    {
        iic_send_byte(data[i]);
        if (iic_wait_ack() == 1)        /* 等待应答 */
        {
            iic_stop();
            return VI530x_IIC_ERROR;
        }
    }
    
    iic_stop();                         /* 停止信号 */
    
    return VI530x_OK;
}

/**
 * @brief   写一个字节到ATK-TOF模块的寄存器
 * @param   reg_addr:   寄存器地址
 * @param   data:       数据
 * @retval  写入结果
 * @arg     0   : 成功
 * @arg     其他: 失败
 */
VI530x_Status atk_tof_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t ret = 0;
    
    ret = atk_tof_write_nbytes(reg_addr, &data, 1);     /* 写入一个字节数据 */
    
    if(ret != 0)
    {
        return VI530x_IIC_ERROR;
    }
    else
    {
        return VI530x_OK;
    }
}

/**
 * @brief   从ATK-TOF模块读取N个字节数据
 * @param   reg_addr:   寄存器地址
 * @param   data:       数据存储buff
 * @param   len:        数据长度
 * @retval  读出结果
 * @arg     0   : 成功
 * @arg     其他: 失败
 */
VI530x_Status atk_tof_read_nbytes(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    iic_start();                        /* 起始信号 */
    
    iic_send_byte(ATK_TOF_IIC_ADDR);    /* 发送IIC通讯地址 */
    if (iic_wait_ack() == 1)            /* 等待应答 */
    {
        iic_stop();
        return VI530x_IIC_ERROR;
    }
    
    iic_send_byte(reg_addr);            /* 发送寄存器地址 */
    if (iic_wait_ack() == 1)            /* 等待应答 */
    {
        iic_stop();
        return VI530x_IIC_ERROR;
    }
    
    iic_start();                        /* 起始信号 */
    
    iic_send_byte(ATK_TOF_IIC_ADDR | 0x01);
    if (iic_wait_ack() == 1)            /* 等待应答 */
    {
        iic_stop();
        return VI530x_IIC_ERROR;
    }
    
    while (len)                         /* 循环接收数据 */
    {
        *data = iic_read_byte((len > 1) ? 1 : 0);
        len--;
        data++;
    }
    
    iic_stop();                         /* 停止信号 */
    
    return VI530x_OK;
}

/**
 * @brief   从ATK-TOF模块读取1个字节数据
 * @param   reg_addr: 寄存器地址
 * @param   data:     数据存储buff
 * @arg     0   : 成功
 * @arg     其他: 失败
 */
VI530x_Status atk_tof_read_byte(uint8_t reg_addr, uint8_t *data)
{
    uint8_t ret = 0;
    
    ret = atk_tof_read_nbytes(reg_addr, data, 1);       /* 读取一个字节数据 */
    
    if(ret != 0)
    {
        return VI530x_IIC_ERROR;
    }
    else
    {
        return VI530x_OK;
    }
}



