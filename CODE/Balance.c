/*
 * Balance.c
 *
 *  Created on: 2022��1��29��
 *      Author: guoguo
 */


#include "headfile.h"
#include "Balance.h"
#include "tuoluoyi.h"
#include "ICM20602_Angle_Get.h"
/*
#include "BrushlessMotor_ADC.h"
#include "BrushlessMotor_HALL.h"
#include "BrushlessMotor.h"
#include "Filter.h"
*/

//��������
#define MOTOR_DEADNUM 500
#define MOTOR3_A   ATOM0_CH4_P02_4  //����3�����תPWM����
#define MOTOR3_B   ATOM0_CH5_P02_5  //����3�����תPWM����
int16 encoder_flywheel = 0;
uint8 Flag = 0;
float flywheel_balance_P = 1000;
float flywheel_balance_I = 4;
float flywheel_balance_D = 0;
float flywheel_speed_P = 90;
float flywheel_speed_I = 90;
//float flywheel_speed_D = 0;
int16 flywheel_duty = 0;
float Angle = 0;
float Angle_Zero = 0;
extern float Angle_X_Final;




//������ֵ��ȡ
void encoder_Flywheel_get()
{
    encoder_flywheel = gpt12_get(GPT12_T4);
    gpt12_clear(GPT12_T4);
}






//������PID����ƽ��
float flywheel_balance(float angle , float gyro)
{
    float PWM = 0;
    float error = 0;
    static float errors;
    error = angle - Angle_Zero;
    errors += error;
    if(errors > 30)
    {
        errors = 30;
    }
    if(errors < -30)
    {
        errors = -30;
    }
    PWM = error * flywheel_balance_P + errors * flywheel_balance_I + gyro * flywheel_balance_D;
    return PWM;
}


//�������ٶȱջ�
float flywheel_speed(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = encoder;                                  //�ٶ��˲�
    Encoder *= 0.7;                                           //һ�׵�ͨ�˲���
    Encoder += Encoder_Least*0.3;                             //һ�׵�ͨ�˲���
    Encoder_Integral += Encoder;                              //���ֳ�λ��
    if(Encoder_Integral > +2000) Encoder_Integral = +2000;    //�����޷�
    if(Encoder_Integral < -2000) Encoder_Integral = -2000;    //�����޷�
    Velocity = Encoder * flywheel_speed_P + Encoder_Integral * flywheel_speed_I/100;
                                                              //��ȡ������ֵ
    if(Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;//ֹͣʱ��������
    return Velocity;
}


//�����ֿ���(��ˢ���)
void flywheel_control_Brush()
{
    int16 PWM , PWM_error;
    encoder_Flywheel_get();
    Angle = Angle_X_Final;
    PWM = flywheel_balance(Angle , icm_acc_x);
    PWM_error = flywheel_speed(encoder_flywheel);
    flywheel_duty = -(PWM - PWM_error);
    //�޷�
    if(flywheel_duty < -8000)
    {
        flywheel_duty = -8000;
    }
    else if(flywheel_duty > 8000)
    {
        flywheel_duty = 8000;
    }
    //����У��
    else if(flywheel_duty > 0)
    {
        flywheel_duty += MOTOR_DEADNUM;
    }
    else if(flywheel_duty < 0)
    {
        flywheel_duty -= MOTOR_DEADNUM;
    }
    if((flywheel_duty < 1000)&&(flywheel_duty > 1000))
    {
        flywheel_duty = 0;
    }
    if(0<=flywheel_duty) //���1   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/GTM_ATOM0_PWM_DUTY_MAX*100)
    {
        pwm_duty(MOTOR3_A, flywheel_duty);
        pwm_duty(MOTOR3_B, 0);
    }
    else                //���1   ��ת
    {
        pwm_duty(MOTOR3_A, 0);
        pwm_duty(MOTOR3_B, -flywheel_duty);
    }



}

/*
//�����ֿ���(��ˢ���)
//ʹ����ˢʱ�轫PWM�жϿ���
void flywheel_control_Brushless()
{
    //�ǶȻ�ȡ
    Angle = Angle_Get();
    motor_set_dir();
    //ͨ���ٶ�������������ǰ�ٶ�
    motor_speed_out();
    motor_control.set_speed = flywheel_balance(Angle , icm_acc_x);
    //����PI�ջ�����
    if(motor_control.dir == FORWARD)
        duty = (int16)closed_loop_pi_calc((float)(motor_control.set_speed - speed_filter.data_average));
    else
        duty = (int16)closed_loop_pi_calc((float)(motor_control.set_speed + speed_filter.data_average));
}

*/









