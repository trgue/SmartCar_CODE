/*
 * BrushlessMotor_ADC.c
 *
 *  Created on: 2022年1月29日
 *      Author: guoguo
 */


#include "zf_vadc.h"
#include "BrushlessMotor_ADC.h"


adc_struct adc_information;

void adc_collection_init(void)
{
    adc_init(ADC_NUMBER, BOARD_POTENTIOMET_PORT);   //板载电位器
    adc_init(ADC_NUMBER, A_PHASE_PORT);             //A相电流
    adc_init(ADC_NUMBER, B_PHASE_PORT);             //B相电流
    adc_init(ADC_NUMBER, C_PHASE_PORT);             //C相电流
    adc_init(ADC_NUMBER, CENTER_PHASE_PORT);        //母线电流
}

void adc_read(void)
{
    adc_information.current_board   = adc_convert(ADC_NUMBER, BOARD_POTENTIOMET_PORT, ADC_12BIT);   //获取板载电位器电压
    adc_information.current_a       = adc_convert(ADC_NUMBER, A_PHASE_PORT, ADC_12BIT);             //获取A相电流
    adc_information.current_b       = adc_convert(ADC_NUMBER, B_PHASE_PORT, ADC_12BIT);             //获取B相电流
    adc_information.current_c       = adc_convert(ADC_NUMBER, C_PHASE_PORT, ADC_12BIT);             //获取C相电流
    adc_information.voltage_bus     = adc_convert(ADC_NUMBER, CENTER_PHASE_PORT, ADC_12BIT);        //获取母线电流
}







