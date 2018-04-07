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
float Gyro_y;          //Y轴陀螺仪数据暂存
float Accel_x;        //X轴加速度值暂存
float Angle_gy;       //由角速度计算的倾斜角度
float Angle_ax;      //由加速度计算的倾斜角度

float Angle;         //小车最终倾斜角度
float Gyro_yf;       //小车滤波后角速度
float s=0;           //路程 PWM积分     t
float vy = 1000 ;      //速度预计       t
float v = 0;          //实时速度
float vb = 0.05;      //10毫秒加速比
float p = 0;          //平衡位置        t

int64_t PWM;             //正值为进 负值为退
int Gyro_yp = 0;      //Y轴角速度零点偏移            调
int Accel_xp = 50;     //X轴加速度零点偏移            调
float Gyro_yb = 16;     //陀螺仪参数在滤波中所占比重   调

static const float Kp = 600;    //PID参数           调
static const float Kd = 28;     //PID参数            调
static const float Ksp =1;      //PID参数         调
uint16_t ch;                       //串口寄存器命令

	//******卡尔曼参数************
    
const float Q_angle = 0.001;    
const float Q_gyro = 0.003;
const float R_angle = 0.45;
const float dt = 0.01;        //dt为kalman滤波器采样时间;
const char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] = { 0, 0, 0, 0 };
float PP[2][2] = { { 1, 0 }, { 0, 1 } };



/*****************卡尔曼滤波**************************************************/
void Kalman_Filter(float Accel, float Gyro) 
{
	Angle += (Gyro - Q_bias) * dt; //先验估计
 
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
 
	PP[0][0] += Pdot[0] * dt;// Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;// =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - Angle; //zk-先验估计
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;//后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	Angle += K_0 * Angle_err;//后验估计
	Q_bias += K_1 * Angle_err;//后验估计
	Gyro_yf = Gyro - Q_bias;//输出值(后验估计)的微分=角速度

}
	

void Angle_Calculate(void)
{

/****************************加速度****************************************/


	//Angle_ax = (Accel_x - Accel_xp) / 16384;//去除零点偏移,计算得到角度（弧度）
	Angle_ax = ((Accel_x - Accel_xp) / 16384) *1.2 * 180 / 3.14;//弧度转换为度,

	/****************************角速度****************************************/

 
	Gyro_y = (Gyro_y - Gyro_yp) / Gyro_yb;//去除零点偏移，计算角速度值 
	//Angle_gy = Angle_gy + Gyro_y*dt;//角速度积分得到倾斜角度,因为卡尔曼计算带有时间dt，所以此处不用积分

	/***************************卡尔曼融合*************************************/
	Kalman_Filter(Angle_ax, -Gyro_y); //卡尔曼滤波计算倾角

	/****************************互补滤波****************************************/

	 /***补偿原理是取当前倾角和加速度获得倾角差值进行放大，然后与
	 ****陀螺仪角速度叠加后再积分，从而使倾角最跟踪为加速度获得的
	 ****角度 0.5为放大倍数，可调节补偿度；0.01为系统周期10ms 
	 *************************************************************/

	  //Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01); //因为加或不加此语句处理效果一样，故省略，原因搞不清楚。。。

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
	
	
	if (Angle<-60 || Angle>60)  //倾角过大就关闭电机，进入死循环，直到复位 
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
		//PWM = (short)(Kp*(Angle-p) + Kd*Gyro_y; //PID：角速度和角度调节  - (sd*(1 - p / 10))*PWM
		n = Kp*(Angle - p)*sqrt((Angle - p)*(Angle - p) / 100) + Kd*Gyro_yf;
		//n = (short)(Kp*(Angle - p) + Kd*Gyro_yf);
		PWM = v + Ksp*n; //PID：车速度调节
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
	
	/* 高级定时器初始化 */
	ADVANCE_TIM_Init();   //T=20ms;ccr=(0,10000-1)
	
		//检测MPU6050
	if (MPU6050ReadID() == 1)
	{	
		
		while (1)
		{
			if (Task_Delay[0] == TASK_ENABLE)
			{
				LED1_TOGGLE;
				
//				printf("加速度：%8d%8d%8d", Acel[0], Acel[1], Acel[2]);
//	           (uint16_t)(USARTx->DR & (uint16_t)0x01FF)
//				printf("    陀螺仪%8d%8d%8d", Gyro[0], Gyro[1], Gyro[2]);
//			     MPU6050_ReturnTemp(&Temp);
//				printf("    温度%8.2f", Temp);
//				printf("    滤波后角度%8.8f", Angle);//Angle_gy
//				printf("    角度%8.5f\r\n", Angle_ax);
//				printf("    温度%8d", PWM); 
//				printf("    滤波后角速度%8.8f\r\n", Gyro_y);
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
		printf("\r\n没有检测到MPU6050传感器！\r\n");
		while (1)
			;
	}
}
