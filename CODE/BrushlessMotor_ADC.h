/*
 * BrushlessMotor_ADC.h
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */

#ifndef CODE_BRUSHLESSMOTOR_ADC_H_
#define CODE_BRUSHLESSMOTOR_ADC_H_

#include "common.h"

#define ADC_NUMBER              ADC_2           //����ADC���

#define BOARD_POTENTIOMET_PORT  ADC2_CH11_A45     //���ص�λ����ȡͨ��
#define A_PHASE_PORT            ADC2_CH3_A35     //A�������ȡͨ��
#define B_PHASE_PORT            ADC2_CH4_A36     //B�������ȡͨ��
#define C_PHASE_PORT            ADC2_CH5_A37     //C�������ȡͨ��
#define CENTER_PHASE_PORT       ADC2_CH6_A38     //ĸ�ߵ�����ȡͨ��

typedef struct
{
    uint16 voltage_bus;         //ĸ�ߵ���
    uint16 current_a;           //A�����
    uint16 current_b;           //B�����
    uint16 current_c;           //C�����
    uint16 current_board;       //���ص�λ����ѹ
}adc_struct;



#endif /* CODE_BRUSHLESSMOTOR_ADC_H_ */
