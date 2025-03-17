/**
 ****************************************************************************************************
 * @file        atk_rtc.h
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

#ifndef _ATK_RTC_H
#define _ATK_RTC_H
#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 器件地址 */
#define RTC_I2C_ADDR                  0x64  /* 设备地址  */

/* 寄存器地址 */
/*time reg*/
#define SD3078_REG_SEC                0x00
#define SD3078_REG_MIN                0x01
#define SD3078_REG_HOUR               0x02
#define SD3078_REG_WDAY               0x03
#define SD3078_REG_MDAY               0x04
#define SD3078_REG_MON                0x05
#define SD3078_REG_YEAR               0x06
/*alarm reg*/
#define SD3078_REG_ALARM_SEC          0x07
#define SD3078_REG_ALARM_MIN          0x08
#define SD3078_REG_ALARM_HOUR         0x09
#define SD3078_REG_ALARM_WEEK         0x0A
#define SD3078_REG_ALARM_DAY          0x0B
#define SD3078_REG_ALARM_MONTH        0x0C
#define SD3078_REG_ALARM_YEAR         0x0D
#define SD3078_REG_ALARM_OE           0x0E
/* control reg*/
#define SD3078_REG_CTR1               0x0F
#define SD3078_REG_CTR2               0x10
#define SD3078_REG_CTR3               0x11
#define SD3078_REG_TTF                0x12
#define SD3078_REG_TD0                0x13
#define SD3078_REG_TD1                0x14
#define SD3078_REG_TD2                0x15
/*temperature reg*/
#define SD3078_REG_TEMP               0x16
/*i2c control reg*/
#define SD3078_REG_I2C_CTRL           0x17
/*charge reg*/
#define SD3078_REG_CHARGE             0x18
/*extend control reg*/
#define SD3078_REG_CTR4               0x19
#define SD3078_REG_CTR5               0x1A
/*battery voltage reg*/
#define SD3078_REG_BAT_VAL            0x1B
/*temperature low/hign alarm reg*/    
#define SD3078_REG_TEMP_AL            0x1C
#define SD3078_REG_TEMP_AH            0x1D
/*history max/min temperature reg*/
#define SD3078_REG_HIS_L              0x1E
#define SD3078_REG_HIS_H              0x1F
/*history temperature lowest  time ram*/
#define SD3078_REG_HIS_L_MIN          0x20
#define SD3078_REG_HIS_L_HOUR         0x21
#define SD3078_REG_HIS_L_WEEK         0x22
#define SD3078_REG_HIS_L_DAY          0x23
#define SD3078_REG_HIS_L_MON          0x24
#define SD3078_REG_HIS_L_YEAR         0x25
/*history temperature highest  time ram*/
#define SD3078_REG_HIS_H_MIN          0x26
#define SD3078_REG_HIS_H_HOUR         0x27
#define SD3078_REG_HIS_H_WEEK         0x28
#define SD3078_REG_HIS_H_DAY          0x29
#define SD3078_REG_HIS_H_MON          0x2A
#define SD3078_REG_HIS_H_YEAR         0x2B
/*user ram reg 2cH -71H*/
#define SD3078_REG_USER_RAM_START     0x2C
#define SD3078_REG_USER_RAM_END       0x71
/*device id 72H-79H*/
#define SD3078_REG_DEVICE_ID          0x72


#define BIT(x)  (1<< x)
/*reg bit define*/
#define SD3078_12_24_BIT              BIT(7)
#define SD3078_AM_PM_BIT              BIT(5)
#define SD3078_WRTC1                  BIT(7)
#define SD3078_WRTC2                  BIT(2)
#define SD3078_WRTC3                  BIT(7)

#define SD3078_TEMP_AL_OE             BIT(2)  /*temperature lowest alarm enable*/
#define SD3078_TEMP_AH_OE             BIT(3)  /*temperature lowest alarm enable*/
#define SD3078_IM                     BIT(6)
#define SD3078_INTS1                  BIT(5)
#define SD3078_INTS0                  BIT(4)
#define SD3078_ALARM_EN               BIT(7)
#define SD3078_INT_AF                 BIT(5)
#define SD3078_INTFE                  BIT(0)
#define SD3078_INTAE                  BIT(1)
#define SD3078_INTDE                  BIT(2)
#define SD3078_TDS1                   BIT(5)
#define SD3078_TDS0                   BIT(4)

#define  DRIVER_VER                   " v1.0.2"

/******************************************************************************************/
/* 12 or 24 hour define */
typedef enum
{
    SD3078_HOUR_12 = 0,
    SD3078_HOUR_24 = 1,
}sd3078_hour_type_e;

/* am or pm define,only use for */
typedef enum
{
    SD3078_AM = 0,
    SD3078_PM,
    SD3078_AM_PM_NULL //if set 24 hour
}sd3078_am_pm_e;

/* 此结构体定义了时间信息包括年、月、日、星期、时、分、秒 */
typedef struct 
{
    uint8_t  second;
    uint8_t  minute;
    uint8_t  hour;
    uint8_t  week;
    uint8_t  day;
    uint8_t  month;
    uint8_t  year;
    sd3078_hour_type_e hour_type;  
    sd3078_am_pm_e am_pm;   /* if hour_type set as SD3078_HOUR_24, am_pm is invalid */      
} sd3078_time_t;

/******************************************************************************************/
/* 闹钟报警 枚举类型 */
typedef enum
{
    ALARM_SEC   = 1 << 0,     /* 允许秒报警中断 */
    ALARM_MIN   = 1 << 1,     /* 允许分报警中断 */
    ALARM_HOUR  = 1 << 2,     /* 允许时报警中断 */
    ALARM_WEEK  = 1 << 3,     /* 允许星期报警中断 */
    ALARM_DAY   = 1 << 4,     /* 允许日报警中断 */
    ALARM_MON   = 1 << 5,     /* 允许月报警中断 */
    ALARM_YEAR  = 1 << 6,     /* 允许年报警中断 */
}alarm_e;

typedef enum
{
    ALARM_WEEK_SUN   = 1 << 0,     /* 周日 */
    ALARM_WEEK_MON   = 1 << 1,     /* 周一 */
    ALARM_WEEK_TUE   = 1 << 2,     /* 周二 */
    ALARM_WEEK_WED   = 1 << 3,     /* 周三 */
    ALARM_WEEK_THU   = 1 << 4,     /* 周四 */
    ALARM_WEEK_FRI   = 1 << 5,     /* 周五 */
    ALARM_WEEK_SAT   = 1 << 6      /* 周六 */
} alarm_week_e;


/* time alarm extend struct */
typedef struct 
{
    uint8_t sec_a;
    uint8_t min_a;
    uint8_t hour_a;
    uint8_t week_a;     /* alarm_week_e枚举类型 */
    uint8_t day_a;
    uint8_t mon_a;
    uint8_t year_a;

    uint8_t ie_a;       /*interrupt enable ,0:disable,1:enable*/
    uint8_t int_period; /*0:int outpute 0,1:int output pulse*/
    uint8_t enable_a;   /*alarm enable;bit[6]:year,bit[5]:month,bit[4]:day,bit[3]:week,bit[2]:hour,bit[1]:minute,bit[0]:second*/
}sd3078_time_alarm_t;

/******************************************************************************************/
/* 频率中断相关定义 */
typedef enum
{
    SD3078_CLK_OUT_4096 = 0x02,  
    SD3078_CLK_OUT_1024 = 0x03,
    SD3078_CLK_OUT_64   = 0x04,
    SD3078_CLK_OUT_32   = 0x05,
    SD3078_CLK_OUT_16   = 0x06,
    SD3078_CLK_OUT_8    = 0x07,
    SD3078_CLK_OUT_4    = 0x08,
    SD3078_CLK_OUT_2    = 0x09,
    SD3078_CLK_OUT_1    = 0x0a,
    SD3078_CLK_OUT_1_2  = 0x0b,
    SD3078_CLK_OUT_1_4  = 0x0c,
    SD3078_CLK_OUT_1_8  = 0x0d,
    SD3078_CLK_OUT_1_16 = 0x0e,
    SD3078_CLK_OUT_1_SEC= 0x0f  
}sd3078_clk_out_e;

/*frequency output struct*/
typedef struct 
{
    sd3078_clk_out_e freq;
    uint8_t oe;            /*= clk out enable, set,0:disable,1:enable*/
}sd3078_clk_out_t;

/******************************************************************************************/
/* 倒计时中断频率 枚举类型 */
typedef enum
{
    SD3078_COUNT_DOWN_4096  = 0x00,     /* 4096HZ =  244us */
    SD3078_COUNT_DOWN_1024  = 0x01,     /* 1024HZ =  976us */
    SD3078_COUNT_DOWN_1     = 0x02,     /* 1HZ =  1s */
    SD3078_COUNT_DOWN_1_60  = 0x03      /* 1min =  60s */
}count_down_e;

/* 此结构体定义了倒计时中断相关结构体 */
typedef struct 
{
    uint32_t count;             /* SD3078 通过 24 位的倒计时寄存器来控制倒计时器 */
    count_down_e source;        /* 计时器计时频率，也就是即每隔多少时间倒计时器递减 1 */
    uint8_t ie;                 /* interrupt enable, 0:disable, 1:enable */
    uint8_t int_period;         /* 0:int outpute 0,  1:int output pulse */
}count_down_t;

/************************************************************************/
typedef enum
{
    IIC_DIS  = 0,       /* iic失能 */
    IIC_EN              /* iic使能能 */
}iic_e;
/* 充电控制结构体 */
typedef struct 
{
   uint8_t chage_en;    /*0:disable,1:enable*/
   uint8_t resistance;  /*0:10k; 1:5k; 2:2k; 3:open*/
}sd3078_charge_t;

/*rtc flag define*/
typedef enum
{
    SD3078_OSF_FLAG = 0,
    SD3078_ALARM_TIME_FLAG,     //alarm time flag
    SD3078_COUNT_DWON_FLAG,     //count down flag
    SD3078_BLF_FLAG,      
    SD3078_PMF_FLAG,
    SD3078_RTCF_FLAG
}sd3078_flag_e;

/******************************************************************************************/
/* 外部接口函数*/
uint8_t atk_rtc_init(void);                         /* 初始化SD3078 */
uint8_t atk_rtc_set_time(sd3078_time_t *rtc_time);
uint8_t atk_rtc_get_time(sd3078_time_t *rtc_time);
uint8_t atk_rtc_set_countdown_interrupt(count_down_t *count_down);
uint8_t atk_rtc_set_alarm_interrupt(sd3078_time_alarm_t *alarm);
uint8_t atk_rtc_set_clk_interrupt(sd3078_clk_out_t *clk);
uint8_t atk_rtc_get_battery_voltage(uint16_t * vol);
uint8_t atk_rtc_battery_i2c_ctrl(uint8_t en);
uint8_t atk_rtc_battery_charge_enable(void);
uint8_t atk_rtc_get_temperature(uint8_t *temp);
uint8_t sd3078_read_ram(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t sd3078_write_ram(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t atk_rtc_flag_get(sd3078_flag_e flag);
uint8_t atk_rtc_flag_clear(uint8_t flag);

#endif
