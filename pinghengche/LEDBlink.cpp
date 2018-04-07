#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include "bsp_SysTick.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "mpu6050.h"
#include "bsp_i2c.h"
#include "bsp_AdvanceTim.h"
#include "math.h"



#define TASK_ENABLE 0
#define NumOfTask 3
unsigned int Task_Delay[NumOfTask];
float Gyro_y;          //Y�������������ݴ�
float Accel_x;        //X����ٶ�ֵ�ݴ�
float Angle_gy;       //�ɽ��ٶȼ������б�Ƕ�
float Angle_ax;      //�ɼ��ٶȼ������б�Ƕ�

float Angle;         //С��������б�Ƕ�
float Gyro_yf;       //С���˲�����ٶ�
float s=0;           //·�� PWM����     t
float vy = 1000 ;      //�ٶ�Ԥ��       t
float v = 0;          //ʵʱ�ٶ�
float vb = 0.05;      //10������ٱ�
float p = 0;          //ƽ��λ��        t

int64_t PWM;             //��ֵΪ�� ��ֵΪ��
int Gyro_yp = 0;      //Y����ٶ����ƫ��            ��
int Accel_xp = 50;     //X����ٶ����ƫ��            ��
float Gyro_yb = 16;     //�����ǲ������˲�����ռ����   ��

static const float Kp = 600;    //PID����           ��
static const float Kd = 28;     //PID����            ��
static const float Ksp =1;      //PID����         ��
uint16_t ch;                       //���ڼĴ�������

	//******����������************
    
const float Q_angle = 0.001;    
const float Q_gyro = 0.003;
const float R_angle = 0.45;
const float dt = 0.01;        //dtΪkalman�˲�������ʱ��;
const char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] = { 0, 0, 0, 0 };
float PP[2][2] = { { 1, 0 }, { 0, 1 } };



/*****************�������˲�**************************************************/
void Kalman_Filter(float Accel, float Gyro) 
{
	Angle += (Gyro - Q_bias) * dt; //�������
 
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
 
	PP[0][0] += Pdot[0] * dt;// Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;// =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - Angle; //zk-�������
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;//����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	Angle += K_0 * Angle_err;//�������
	Q_bias += K_1 * Angle_err;//�������
	Gyro_yf = Gyro - Q_bias;//���ֵ(�������)��΢��=���ٶ�

}
	

void Angle_Calculate(void)
{

/****************************���ٶ�****************************************/


	//Angle_ax = (Accel_x - Accel_xp) / 16384;//ȥ�����ƫ��,����õ��Ƕȣ����ȣ�
	Angle_ax = ((Accel_x - Accel_xp) / 16384) *1.2 * 180 / 3.14;//����ת��Ϊ��,

	/****************************���ٶ�****************************************/

 
	Gyro_y = (Gyro_y - Gyro_yp) / Gyro_yb;//ȥ�����ƫ�ƣ�������ٶ�ֵ 
	//Angle_gy = Angle_gy + Gyro_y*dt;//���ٶȻ��ֵõ���б�Ƕ�,��Ϊ�������������ʱ��dt�����Դ˴����û���

	/***************************�������ں�*************************************/
	Kalman_Filter(Angle_ax, -Gyro_y); //�������˲��������

	/****************************�����˲�****************************************/

	 /***����ԭ����ȡ��ǰ��Ǻͼ��ٶȻ����ǲ�ֵ���зŴ�Ȼ����
	 ****�����ǽ��ٶȵ��Ӻ��ٻ��֣��Ӷ�ʹ��������Ϊ���ٶȻ�õ�
	 ****�Ƕ� 0.5Ϊ�Ŵ������ɵ��ڲ����ȣ�0.01Ϊϵͳ����10ms 
	 *************************************************************/

	  //Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01); //��Ϊ�ӻ򲻼Ӵ���䴦��Ч��һ������ʡ�ԣ�ԭ��㲻���������

}

extern "C" 
void SysTick_Handler(void)
{
	unsigned char i;
	for (i = 0; i < NumOfTask; i++)
	{
		if (Task_Delay[i])
		{
			Task_Delay[i]--;
		}
	}
}

void Delay()
{
	int i;
	for (i = 0; i < 1000000; i++)
		asm("nop");
}

void PWM_Calculate(void)
{
	int64_t n = 0;
	int PWML;
	int PWMR;
	
	
	if (Angle<-60 || Angle>60)  //��ǹ���͹رյ����������ѭ����ֱ����λ 
	{
		Mstop();
		while (1)
			;
	} 
	else if (Angle >= 20)
	{
		PWM = 9999;
	}
	else if (Angle <= -20)
	{
		PWM = -9999;
	}
	else
	{
		//PWM = (short)(Kp*(Angle-p) + Kd*Gyro_y; //PID�����ٶȺͽǶȵ���  - (sd*(1 - p / 10))*PWM
		n = Kp*(Angle - p)*sqrt((Angle - p)*(Angle - p) / 100) + Kd*Gyro_yf;
		//n = (short)(Kp*(Angle - p) + Kd*Gyro_yf);
		PWM = v + Ksp*n; //PID�����ٶȵ���
	}
	if (PWM >= 9999)
		PWM = 9999;
	else if (PWM <= -9999)
		PWM = -9999;
	
	v = v + vb*(PWM - v);
	
//	if (PWM >= 0)
//	{
//		GO(PWM-300, PWM+1000);	
//	}
//	else
//	{
//		GO(PWM-1000, PWM+300);	
//	}
	
	
	if (ch == 3)
	{
		if (PWM >= 0)
		 {
			GO(PWM - 300, PWM + 1000);	
		}
		else
		{
			GO(PWM - 1000, PWM + 300);	
		}
	}
	else if (ch == 4)
	{
		if (PWM <= 0)
		{
			GO(PWM + 300, PWM - 1000);	
		}
		else
		{
			GO(PWM +1000, PWM - 300);	
		}
		
	}
	else
	GO(PWM, PWM);
	
}



void P_Calculate(void)
{
	
	if (s >= 2)
	{
		p = -2;
	}
	else if (s <= -2)
	{
		p = 2 ;
	}
	else
	{p = -s; }
	
//	if (v >= vy)
//	{
//		p -= 0.0008*(v - vy);
//	}
//	else if (v <= -vy)
//	{
//		p -= 0.0008*(v + vy);
//	}
	
	
	if (ch == 1)
	{
		
		s = -2;
	}
	else if (ch == 2)
	{
		
		s = 2;
	}
	else
		s = 0;
	p -= 0.0011*v;
}
	
	


int main()

		
{
	short Acel[3];
	short Gyro[3];
	float Temp;
	int i=0;

	SysTick_Init();
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	
	LED_GPIO_Config();

	USART_Config();

	i2c_GPIO_Config();

	MPU6050_Init();
	
	/* �߼���ʱ����ʼ�� */
	ADVANCE_TIM_Init();   //T=20ms;ccr=(0,10000-1)
	
		//���MPU6050
	if (MPU6050ReadID() == 1)
	{	
		
		while (1)
		{
			if (Task_Delay[0] == TASK_ENABLE)
			{
				LED1_TOGGLE;
				
//				printf("���ٶȣ�%8d%8d%8d", Acel[0], Acel[1], Acel[2]);
//	           (uint16_t)(USARTx->DR & (uint16_t)0x01FF)
//				printf("    ������%8d%8d%8d", Gyro[0], Gyro[1], Gyro[2]);
//			     MPU6050_ReturnTemp(&Temp);
//				printf("    �¶�%8.2f", Temp);
//				printf("    �˲���Ƕ�%8.8f", Angle);//Angle_gy
//				printf("    �Ƕ�%8.5f\r\n", Angle_ax);
//				printf("    �¶�%8d", PWM); 
//				printf("    �˲�����ٶ�%8.8f\r\n", Gyro_y);
				Task_Delay[0] = 500;
			}
			
 			if (Task_Delay[1] == 0)
			{
				MPU6050ReadAcc(Acel);
				Accel_x = Acel[0];
				MPU6050ReadGyro(Gyro);
				Gyro_y = Gyro[1];
				ch=(uint16_t)(USART1->DR & (uint16_t)0x01FF);
				Angle_Calculate();
				s += 0.001*v*dt;
				P_Calculate();
//				if (ch == 1)
//				{
//					p = -2; 
//					s = 0;
//				}
//				else if (ch == 2)
//				{
//					p = 2;
//					s = 0;
//				}
//				else p = 0;
				 PWM_Calculate();
				//GO(PWM, PWM);
				Task_Delay[1] = 1000*dt;
				
			}
	
		
		
		
		
		
		}
	}
	else
	{
		printf("\r\nû�м�⵽MPU6050��������\r\n");
		while (1)
			;
	}
}
