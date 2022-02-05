/*
 * BrushlessMotor_PWM_Input.c
 *
 *  Created on: 2022年1月29日
 *      Author: guoguo
 */


#include "headfile.h"
#include "CCU6_PWM.H"
#include "BrushlessMotor_PWM_Input.h"

uint16 pwm_in_duty;

IfxGtm_Tim_In driver;

//输入捕获初始化
void pwm_input_init(void)
{
    IfxGtm_enable(&MODULE_GTM);
    if(!(MODULE_GTM.CMU.CLK_EN.U & 0x2))
    {
        IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, 100000000);
        IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0);
    }

    IfxGtm_Tim_In_Config config;
    IfxGtm_Tim_In_initConfig(&config, &MODULE_GTM);
    config.timIndex                     = IfxGtm_Tim_1;
    config.channelIndex                 = IfxGtm_Tim_Ch_1;
    config.isrPriority                  = GTM_PWM_IN_PRIORITY;
    config.capture.irqOnNewVal          = TRUE;
    config.capture.irqOnCntOverflow     = TRUE;
    config.timeout.clock                = IfxGtm_Cmu_Clk_0;
    config.filter.inputPin              = &IfxGtm_TIM1_1_TIN95_P11_2_IN;
    config.filter.inputPinMode          = IfxPort_InputMode_pullDown;


    IfxGtm_Tim_In_init(&driver, &config);
    driver.periodTick = FPWM;


    gpio_init(MOTOR_DIR_IN_PIN, GPI, 0, PULLDOWN);      //初始化方向设置引脚
}


