/**
 ****************************************************************************************************
 * @file        oled.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-21
 * @brief       OLED ��������
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
 * V1.0 20200421
 * ��һ�η���
 *
 ****************************************************************************************************
 */
 
#ifndef __OLED_H
#define __OLED_H

#include "stdlib.h" 
#include "./SYSTEM/sys/sys.h"


/* ����/���� ���� */
#define OLED_CMD        0       /* д���� */
#define OLED_DATA       1       /* д���� */

#define OLED_I2C_ADDR   (0x78)  /* OLED���豸��ַ */
/******************************************************************************************/
    
static void oled_wr_byte(uint8_t data, uint8_t cmd);    /* дһ���ֽڵ�OLED */
static uint32_t oled_pow(uint8_t m, uint8_t n);         /* OLED��ƽ������ */


void oled_init(void);           /* OLED��ʼ�� */
void oled_clear(void);          /* OLED���� */
void oled_display_on(void);     /* ����OLED��ʾ */
void oled_display_off(void);    /* �ر�OLED��ʾ */
void oled_refresh_gram(void);   /* �����Դ浽OLED */ 
void oled_draw_point(uint8_t x, uint8_t y, uint8_t dot);    /* OLED���� */
void oled_fill(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t dot);        /* OLED������� */
void oled_show_char(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode); /* OLED��ʾ�ַ� */
void oled_show_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);  /* OLED��ʾ���� */
void oled_show_string(uint8_t x, uint8_t y, const char *p, uint8_t size);           /* OLED��ʾ�ַ��� */
#endif




