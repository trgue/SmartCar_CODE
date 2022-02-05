/*
 * BrushlessMotor_ADC.c
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */


#include "zf_vadc.h"
#include "BrushlessMotor_ADC.h"


adc_struct adc_information;

void adc_collection_init(void)
{
    adc_init(ADC_NUMBER, BOARD_POTENTIOMET_PORT);   //���ص�λ��
    adc_init(ADC_NUMBER, A_PHASE_PORT);             //A�����
    adc_init(ADC_NUMBER, B_PHASE_PORT);             //B�����
    adc_init(ADC_NUMBER, C_PHASE_PORT);             //C�����
    adc_init(ADC_NUMBER, CENTER_PHASE_PORT);        //ĸ�ߵ���
}

void adc_read(void)
{
    adc_information.current_board   = adc_convert(ADC_NUMBER, BOARD_POTENTIOMET_PORT, ADC_12BIT);   //��ȡ���ص�λ����ѹ
    adc_information.current_a       = adc_convert(ADC_NUMBER, A_PHASE_PORT, ADC_12BIT);             //��ȡA�����
    adc_information.current_b       = adc_convert(ADC_NUMBER, B_PHASE_PORT, ADC_12BIT);             //��ȡB�����
    adc_information.current_c       = adc_convert(ADC_NUMBER, C_PHASE_PORT, ADC_12BIT);             //��ȡC�����
    adc_information.voltage_bus     = adc_convert(ADC_NUMBER, CENTER_PHASE_PORT, ADC_12BIT);        //��ȡĸ�ߵ���
}







