/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2021,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����ϵ�Ա��ͷ�
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            bldc_config
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ3184284598)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        ADS v1.5.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2021-12-10
 ********************************************************************************************************************/

#ifndef _BLDC_CONFIG_H
#define _BLDC_CONFIG_H


//0��������ɲ������     1������
#define BLDC_BRAKE_ENABLE       1

//0���������ٶȱջ�     1������
#define BLDC_CLOSE_LOOP_ENABLE  1           //���ڵ���������ز�ͬ Ĭ�ϲ�����PID����


#if BLDC_CLOSE_LOOP_ENABLE==1
    //�������ת��
    #define BLDC_MAX_SPEED          32000

#endif





#endif
