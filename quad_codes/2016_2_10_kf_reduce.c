#include "stm32f30x.h"
#include "stm32f30x_it.h"
#include<stdio.h>
#include<math.h>
#include "stm32f3_discovery_lsm303dlhc.h"
#include "stm32f3_discovery_l3gd20.h"
#include <string.h>

#ifndef _ACC_C_
#define _ACC_C_
 

#define LSM_Acc_Sensitivity_2g     (float)     1.0f            /*!< accelerometer sensitivity with 2 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_4g     (float)     0.5f            /*!< accelerometer sensitivity with 4 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_8g     (float)     0.25f           /*!< accelerometer sensitivity with 8 g full scale [LSB/mg] */
#define LSM_Acc_Sensitivity_16g    (float)     0.0834f         /*!< accelerometer sensitivity with 12 g full scale [LSB/mg] */

#define PI                         (float)     3.14159265f

#define L3G_Sensitivity_250dps     (float)   114.285f         /*!< gyroscope sensitivity with 250 dps full scale [LSB/dps] */
#define L3G_Sensitivity_500dps     (float)    57.1429f        /*!< gyroscope sensitivity with 500 dps full scale [LSB/dps] */
#define L3G_Sensitivity_2000dps    (float)    14.285f	      /*!< gyroscope sensitivity with 2000 dps full scale [LSB/dps] */

LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;
LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStructure;
L3GD20_InitTypeDef L3GD20_InitStructure;
L3GD20_FilterConfigTypeDef L3GD20_FilterStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
GPIO_InitTypeDef GPIO_InitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

void Acc_Config(void);
void Acc_ReadData(float* pfData);
void TIM_Config(void);
void gyroConfig(void);
void gyroReadAngRate (float* pfData);
void GPIO_Config();
void USART3_Config();
void compassConfig(void);
void compassReadMag (float* pfData);

uint16_t PrescalerValue = 0,read=0;
float avg=0.0f,ratio=0.0f;
float pitch=0.0f,pitch_acc=0.0f,set_pitch=0.0f,error_pitch[2]={0.0f},sum_pitch=0.0f;
float roll=0.0f,roll_acc=0.0f,set_roll=0.0f,error_roll[2]={0.0f},sum_roll=0.0f;
float yaw=0.0f,yaw_acc=0.0f,set_yaw=0.0f,error_yaw[2]={0.0f},sum_yaw=0.0f;
float u=0.0f,u_roll=0.0f,u_pitch=0.0f,u_yaw=0.0f,kp=0.0f,ki=0.0f,kd=0.0f,ccr;
//float k_p[3]={0.0,0.0,0.87},k_i[3]={0.0,0.0044,0.09},k_d[3]={0.0,0.0,0.28};
//float k_p[3]={0.72,0.56,1.07},k_i[3]={0.0629,0.09,0.6},k_d[3]={0.24,0.29,0.28};        //stable values
//float k_p[3]={0.56,0.37,2.7},k_i[3]={0.0629,0.0044,0},k_d[3]={0.122,0.109,0.0};
float k_p[3]={0.0,0.0,0.0},k_i[3]={0.0,0.0,0.0},k_d[3]={0.0,0.0,0.0};
float ccr_roll=0.0f,ccr_pitch=0.0f,ccr_yaw=0.0f,meas_roll=0.0f,meas_pitch=0.0f,meas_yaw=0.0f;
float ccr_front_motor=0.0f,ccr_back_motor=0.0f,ccr_left_motor=0.0f,ccr_right_motor=0.0f,CCR_prev=500.0f;
float meas[3]={0.0f};
float mpitch=0,mroll=0,myaw=0;
int times=0;
float arr_gyro[5][3]={0.0f};
float arr_acc[5][3]={0.0f};
float arr_mag[5][3]={0.0f};
float ccr_throttle=0.0f;
float AccBuffer[3] = {0.0f};
float W=0.90;
float x=0.0f,y=0.0f,z=0.0f;
float sum_counts=0.0f;
int z1=0;

float a=0.0f,b=0.0f;
int counts=0;

float Axz_0=0.0f,Ayz_0=0.0f,Axz,Ayz,Rxest,Rxest_0=0.0f,Ryest,Ryest_0=0.0f,Rzest,Rzest_0=0.0f,Rate_Axz,Rate_Axz_0=0.0f;
float Rate_Ayz,Rate_Ayz_0=0.0f,Rate_Axz_avg,Rate_Ayz_avg;
float Rx_gyro,Ry_gyro,Rz_gyro,Rx_acc,Ry_acc,Rz_acc,Roll_gyro,Pitch_gyro,kangle=0.0f;

float GyroBuffer[3] = {0.0f},Gyrooffset[3]={0.0f},Gyrooffset_prev[3]={0.0f},Gyroavg[3],Gyroavg_1[3]={0.0f},Gyroavg_2[3]={0.0f},Gyroangle[3] = {0.0f},Accavg[3]={0.0f},Accavg_1[3]={0.0f},t_kal=1.0f/150.0f,t=1.0f/30.0f,Final_gyro[3],observe[100];
float Mag_abs=0.0f,Magavg[3]={0.0f},Magavg_1[3]={0.0f};
int n=0,k=0,count=0,read5=0,check=0,counter=0,readsensor=0,counter2=0,retard=0,delayy=2,pass=0;
char m=1;
float MagBuffer[3] = {0.0f},mag=0.0f,dir=0.0f,initdir=0.0f;
float Mag_avg=0.0f,mag_X=0.0f,mag_Y=0.0f,mag_Z=0.0f,Yh=0.0f,Xh=0.0f;
float gap=0.0;
float temp,temp1;   //for temporary use

int flag=0, pidupdate=0;
	
//variables for kalman filter pitch
	
float Q_angle_pitch = 0.0001f;    //0.0001
//Q_gyroBias should be low, since gyro bias change extremely slow
float Q_ang_vel_pitch = 0.001f;    //100
float Q_gyroBias_pitch = 0.01f;     //0.003
//R_measure should be greater than Q_angle to prevent acceleration from affecting the angle
float R_angle_pitch = 100.0f;       //1000//100000
float R_ang_vel_pitch = 1.0f;     //0.001
float pitch_est=0.0f,pitch_inn=0.0f,pitch_rate=0.0f,pitch_ang_vel=0.0f,pitch_ang_vel_inn=0.0f,pitch_den=0.0f;
float pitch_P[3][3]={1000.0f,0.0f,0.0f,0.0f,1000.0f,0.0f,0.0f,0.0f,1000.0f};
float pitch_K[3][2]={0.0f};
float pitch_S[2][2]={0.0f};

//variables for kalman filter of roll
	
float Q_angle_roll = 0.0001f;    //0.001
//Q_gyroBias should be low, since gyro bias change extremely slow
float Q_ang_vel_roll = 0.001f;
float Q_gyroBias_roll = 0.01f;     //0.003
//R_measure should be greater than Q_angle to prevent acceleration from affecting the angle
float R_angle_roll = 100.0f;       //0.03//100000
float R_ang_vel_roll = 1.0f;
float roll_est=0.0f,roll_inn=0.0f,roll_rate=0.0f,roll_ang_vel=0.0f,roll_ang_vel_inn=0.0f,roll_den=0.0f;
float roll_P[3][3]={1000.0f,0.0f,0.0f,0.0f,1000.0f,0.0f,0.0f,0.0f,1000.0f};
float roll_K[3][2]={0.0f};
float roll_S[2][2]={0.0f};


int fputc1(int ch, FILE *f)
{
    return(ITM_SendChar(ch));
}




uint16_t CCR1,ii,go,start;
uint32_t iii;

uint8_t update_flag;
int takeaction=0;
int dutta=0;
long a_uart=0;
uint16_t CCR1_Val = 0;
uint16_t CCR2_Val = 0;
uint16_t CCR3_Val = 0;
uint16_t CCR4_Val = 0;

unsigned int edge1=0,edge2=0,edge3=0,edge4=0,edge5=0,edge6=0,frequency=0,CCR=0,init=54,final=35949,CCRshow=0,flag1=0;
float Capture[2]={0.0f},CCR_3=0.0f,CCR_2=0.0f,gap1=0.0f;
int counts1=0,start1=0,check1=0,counter1=0;
int counts2=0,check2=0;


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

unsigned long	gyrodata=0;
unsigned int gyrosend;	
	
	


int main(void)

{
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOE, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM7, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
  gyroConfig();
	Acc_Config();
  TIM_Config();
  GPIO_Config();  
	compassConfig();
	USART3_Config();
	
	/* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
	TIM_Cmd(TIM7, ENABLE);
	
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	
	
	kangle=180.0f/3.14;
	
	
  /* Infinite loop */
   while (1)
  {
		
				if(takeaction==1)
		{
			takeaction=0; 
			ccr_throttle=CCR;
      		

		  TIM3->CCR1=CCR1_Val;     //front
			TIM3->CCR4=CCR2_Val;     //back
 			TIM3->CCR2=CCR3_Val;     //left
			TIM3->CCR3=CCR4_Val;     //right
			
    }			
 			   			
	
	if(readsensor==1)		
	{
		readsensor=0;	
		
		gyroReadAngRate(GyroBuffer);
		Acc_ReadData(AccBuffer);
		compassReadMag(MagBuffer);
		
				
				
						gyrodata = (roll_est+1000)*1000;
							//gyrodata = (1000+1000)*1;
							times=0;
							//USART_SendData(USART3, 48);
							while(times<8)							
							{ 
								a_uart= pow(10,7-times);
								gyrosend = gyrodata/a_uart;
								USART_SendData(USART3, gyrosend+48);
								gyrodata = gyrodata%a_uart;
								times++;
								int i2=90;
								while(i2--);
							
						}//}
						USART_SendData(USART3, 10);
						
							
							
							
						
						
		
		if(n<500)			
				{
					for(k=0; k<3; k++)
					{
 					 	Gyroavg_2[k]+=GyroBuffer[k];	
 				  }
				}
			n++;
			
		if(n==500)
			{
				
				for(k=0; k<3; k++)
					{
 					 	Gyrooffset[k]=Gyroavg_2[k]/500.0f;
						if(-2.0f>Gyrooffset[k]||Gyrooffset[k]>2.0f)
							Gyrooffset[k]=Gyrooffset_prev[k];
						Gyrooffset_prev[k]=Gyrooffset[k];
						Gyroavg_2[k]=0.0f;	
 				  }
					
			}	
				
		if(n>=500)
			{
				n=521;
				read5++;
				
				if(read5==6)
				{read5=1;}
				
				Gyroavg[2]+=read5*GyroBuffer[2];
				
 				for(k=0; k<3; k++)
 				{
					
 						Magavg[k]+=read5*MagBuffer[k];
 				}
				
				
				if(read5==5)
				{
					
					 Gyroavg_1[2]=Gyroavg[2]/15.0f;
					temp=(float)((Gyroavg_1[2]-Gyrooffset[2])*t);
							
							Gyroangle[2]=-1.0f*meas[2]+temp;
					Gyroavg[2]=0.0;
					
						for(k=0; k<3; k++)
						{
							
							Magavg_1[k]=Magavg[k]/15.0f;
						Magavg[k]=0;	
							
						}
						
						Mag_abs=sqrt((Magavg_1[0]*Magavg_1[0])+(Magavg_1[1]*Magavg_1[1])+(Magavg_1[2]*Magavg_1[2]));
						mag_X=MagBuffer[0]/Mag_abs;
						mag_Y=MagBuffer[1]/Mag_abs;
						mag_Z=MagBuffer[2]/Mag_abs;
						
						a=(mag_Y*cos(roll_est/kangle));
						b=(mag_Z*sin(roll_est/kangle));
						Yh=a-b;
						Xh=mag_X*cos(pitch_est/kangle)+mag_Y*sin(roll_est/kangle)*sin(pitch_est/kangle)+mag_Z*cos(roll_est/kangle)*sin(pitch_est/kangle);
						dir=atan(Yh/Xh)*(kangle);
						if(Xh>0&&Yh>0)
							dir=-180+dir;
						if(Xh>0&&Yh<0)
							dir=180+dir;
						
 						if(m==1)
 							initdir=dir;
 						
 						m=2;
						temp1=dir-initdir;
						meas[2]=-1.0*Gyroangle[2]*0.98+(dir-initdir)*0.02;
					}
				
						if(pidupdate==1)
		{
			pidupdate=0;
			//***********************your space*******************************
			
		
						
						if(ccr_throttle>20)
				{
				
				error_roll[0]=error_roll[1];
				error_roll[1]=roll_est-set_roll;
				sum_roll=sum_roll+(error_roll[1]*t);
				
				//for CCR=550, Kp=0.56,Kd=0.3	
					
				//if(error_roll[1]>1||error_roll[1]<-1)
				//kp=0.01795,ki=0,kd=0.0055
				ccr_roll=k_p[0]*error_roll[1] + k_i[0]*sum_roll + k_d[0]*(error_roll[1]-error_roll[0])/t;
				//ccr_roll=0*error_roll[1] + 0*sum_roll + 0.3*GyroBuffer[1];
				//else
				//ccr_roll=0;
				
				//ccr_roll=(250*u_roll)/7.7;
				
				error_pitch[0]=error_pitch[1];
				error_pitch[1]=pitch_est-set_pitch;
				sum_pitch=sum_pitch+(error_pitch[1]*t);

				//if(error_pitch[1]>1||error_pitch[1]<-1)
				//kp=0.039,ki=0,kd=0.009, tested for CCR 580
				ccr_pitch=k_p[1]*error_pitch[1] + k_i[1]*sum_pitch + k_d[1]*(error_pitch[1]-error_pitch[0])/t;
				//else
				//ccr_pitch=0;
				
				//ccr_pitch=(250*u_pitch)/7.7;
				
				error_yaw[0]=error_yaw[1];
				error_yaw[1]=meas[2]-set_yaw;
				sum_yaw=sum_yaw+(error_yaw[1]*t);
				
				//if(error_yaw[1]>0.5||error_yaw[1]<-0.5)
				//kp=0.02000,ki=0.00199,kd=0.0100	
				ccr_yaw=k_p[2]*error_yaw[1] + k_i[2]*sum_yaw + k_d[2]*(error_yaw[1]-error_yaw[0])/t;
				//u_yaw=0;
				//else
				//u_yaw=0;
				
				//ccr_yaw=(250*u_yaw)/7.7;
				static int i1=1;
				if(i1==1)
				{
						GPIOA->BSRR = 0x0002;
						i1=0;
				}
				else
				{
						GPIOA->BRR = 0x0002 ;
						i1=1;			
				}		
			
				mpitch=meas[0];
				mroll=meas[1];
				myaw=meas[2];
				
				
				ccr_front_motor=ccr_throttle+ccr_pitch-ccr_yaw;
					if(ccr_front_motor>850)
						ccr_front_motor=850;
					if(ccr_front_motor<0)
						ccr_front_motor=20;
				
				ccr_back_motor=ccr_throttle-ccr_pitch-ccr_yaw;
					if(ccr_back_motor>850)
						ccr_back_motor=850;
					if(ccr_back_motor<0)
						ccr_back_motor=20;
					
				ccr_left_motor=ccr_throttle+ccr_roll+ccr_yaw;
					if(ccr_left_motor>850)
						ccr_left_motor=850;
					if(ccr_left_motor<0)
						ccr_left_motor=20;
					
				ccr_right_motor=ccr_throttle-ccr_roll+ccr_yaw;
					if(ccr_right_motor>850)
						ccr_right_motor=850;
					if(ccr_right_motor<0)
						ccr_right_motor=20;
				}
				else
				{
					ccr_front_motor=ccr_throttle;
					ccr_back_motor=ccr_throttle;
					ccr_left_motor=ccr_throttle;
					ccr_right_motor=ccr_throttle;
				}
			CCR1_Val = ccr_front_motor;
			CCR2_Val = ccr_back_motor;
			CCR3_Val = ccr_left_motor;
			CCR4_Val = ccr_right_motor;
			}				
				

			
						
						//LKF for pitch measurements

//state prediction
pitch_rate=pitch_ang_vel-Gyrooffset[0];
pitch_est=pitch_est+(t_kal*pitch_rate);        

//Covariance prediction

pitch_P[0][0] = pitch_P[0][0] + t_kal*(pitch_P[1][0]-pitch_P[2][0]) + t_kal*(pitch_P[0][1] + t_kal*(pitch_P[1][1]-pitch_P[2][1])) - t_kal*(pitch_P[0][2] + t_kal*(pitch_P[1][2]-pitch_P[2][2])) + Q_angle_pitch;
pitch_P[0][1] = pitch_P[0][1] + t_kal*(pitch_P[1][1]-pitch_P[2][1]);
pitch_P[0][2] = pitch_P[0][2] + t_kal*(pitch_P[1][2]-pitch_P[2][2]);
pitch_P[1][0] = pitch_P[1][0] + t_kal*pitch_P[1][1] - t_kal*pitch_P[1][2];
pitch_P[1][1] = pitch_P[1][1] + Q_ang_vel_pitch;
pitch_P[1][2] = pitch_P[1][2];
pitch_P[2][0] = pitch_P[2][0] + t_kal*pitch_P[2][1] - t_kal*pitch_P[2][2];
pitch_P[2][1] = pitch_P[2][1];
pitch_P[2][2] = pitch_P[2][2] + Q_gyroBias_pitch;

//State innovation
pitch_inn=pitch_acc-pitch_est;
pitch_ang_vel_inn=-GyroBuffer[0]-pitch_ang_vel;         //significance of -ve sign before Gyroavg_1[0]

//covariance innovation
pitch_S[0][0] = pitch_P[0][0] + R_angle_pitch;
pitch_S[0][1] = pitch_P[0][1];
pitch_S[1][0] = pitch_P[1][0];
pitch_S[1][1] = pitch_P[1][1] + R_ang_vel_pitch;

//Kalman gain
pitch_den=(pitch_S[0][0]*pitch_S[1][1])-(pitch_S[0][1]*pitch_S[1][0]);
pitch_K[0][0] = ((pitch_P[0][0]*pitch_S[1][1])-(pitch_P[0][1]*pitch_S[1][0]))/pitch_den;
pitch_K[0][1] = (-(pitch_P[0][0]*pitch_S[0][1])+(pitch_P[0][1]*pitch_S[0][0]))/pitch_den;
pitch_K[1][0] = ((pitch_P[1][0]*pitch_S[1][1])-(pitch_P[1][1]*pitch_S[1][0]))/pitch_den;
pitch_K[1][1] = (-(pitch_P[1][0]*pitch_S[0][1])+(pitch_P[1][1]*pitch_S[0][0]))/pitch_den;
pitch_K[2][0] = ((pitch_P[2][0]*pitch_S[1][1])-(pitch_P[2][1]*pitch_S[1][0]))/pitch_den;
pitch_K[2][1] = (-(pitch_P[2][0]*pitch_S[0][1])+(pitch_P[2][1]*pitch_S[0][0]))/pitch_den;

//state update
pitch_est += (pitch_K[0][0] * pitch_inn) + (pitch_K[0][1] * pitch_ang_vel_inn);
pitch_ang_vel += (pitch_K[1][0] * pitch_inn) + (pitch_K[1][1] * pitch_ang_vel_inn);
Gyrooffset[0] += (pitch_K[2][0] * pitch_inn) + (pitch_K[2][1] * pitch_ang_vel_inn);

//covariance update
float pitch_P00_temp = pitch_P[0][0];
float pitch_P01_temp = pitch_P[0][1];
float pitch_P02_temp = pitch_P[0][2];
float pitch_P10_temp = pitch_P[1][0];
float pitch_P11_temp = pitch_P[1][1];
float pitch_P12_temp = pitch_P[1][2];

pitch_P[0][0]=(1-pitch_K[0][0])*pitch_P00_temp - pitch_K[0][1]*pitch_P10_temp;
pitch_P[0][1]=(1-pitch_K[0][0])*pitch_P01_temp - pitch_K[0][1]*pitch_P11_temp;
pitch_P[0][2]=(1-pitch_K[0][0])*pitch_P02_temp - pitch_K[0][1]*pitch_P11_temp;
pitch_P[1][0]= - pitch_K[1][0]*pitch_P00_temp + (1-pitch_K[1][1])*pitch_P10_temp;
pitch_P[1][1]= - pitch_K[1][0]*pitch_P01_temp + (1-pitch_K[1][1])*pitch_P11_temp;
pitch_P[1][2]= - pitch_K[1][0]*pitch_P02_temp + (1-pitch_K[1][1])*pitch_P11_temp;
pitch_P[2][0]= - pitch_K[2][0]*pitch_P00_temp - pitch_K[2][1]*pitch_P10_temp + pitch_P[2][0];
pitch_P[2][1]= - pitch_K[2][0]*pitch_P01_temp - pitch_K[2][1]*pitch_P11_temp + pitch_P[2][1];
pitch_P[2][2]= - pitch_K[2][0]*pitch_P02_temp - pitch_K[2][1]*pitch_P12_temp + pitch_P[2][2];

Gyroavg[0]=0;				


//LKF for roll measurements

//state prediction
roll_rate=roll_ang_vel-Gyrooffset[1];
roll_est=roll_est+(t_kal*roll_rate);
//roll_ang_vel=-1.67*(CCR3_Val-CCR4_Val);          //significance of negative sign

//Covariance prediction

roll_P[0][0] = roll_P[0][0] + t_kal*(roll_P[1][0]-roll_P[2][0]) + t_kal*(roll_P[0][1] + t_kal*(roll_P[1][1]-roll_P[2][1])) - t_kal*(roll_P[0][2] + t_kal*(roll_P[1][2]-roll_P[2][2])) + Q_angle_roll;
roll_P[0][1] = roll_P[0][1] + t_kal*(roll_P[1][1]-roll_P[2][1]);
roll_P[0][2] = roll_P[0][2] + t_kal*(roll_P[1][2]-roll_P[2][2]);
roll_P[1][0] = roll_P[1][0] + t_kal*roll_P[1][1] - t_kal*roll_P[1][2];
roll_P[1][1] = roll_P[1][1] + Q_ang_vel_roll;
roll_P[1][2] = roll_P[1][2];
roll_P[2][0] = roll_P[2][0] + t_kal*roll_P[2][1] - t_kal*roll_P[2][2];
roll_P[2][1] = roll_P[2][1];
roll_P[2][2] = roll_P[2][2] + Q_gyroBias_roll;

//State innovation
roll_inn=roll_acc-roll_est;
roll_ang_vel_inn=-GyroBuffer[1]-roll_ang_vel;         //significance of -ve sign before Gyroavg_1[0]

//covariance innovation
roll_S[0][0] = roll_P[0][0] + R_angle_roll;
roll_S[0][1] = roll_P[0][1];
roll_S[1][0] = roll_P[1][0];
roll_S[1][1] = roll_P[1][1] + R_ang_vel_roll;

//Kalman gain
roll_den=(roll_S[0][0]*roll_S[1][1])-(roll_S[0][1]*roll_S[1][0]);
roll_K[0][0] = ((roll_P[0][0]*roll_S[1][1])-(roll_P[0][1]*roll_S[1][0]))/roll_den;
roll_K[0][1] = (-(roll_P[0][0]*roll_S[0][1])+(roll_P[0][1]*roll_S[0][0]))/roll_den;
roll_K[1][0] = ((roll_P[1][0]*roll_S[1][1])-(roll_P[1][1]*roll_S[1][0]))/roll_den;
roll_K[1][1] = (-(roll_P[1][0]*roll_S[0][1])+(roll_P[1][1]*roll_S[0][0]))/roll_den;
roll_K[2][0] = ((roll_P[2][0]*roll_S[1][1])-(roll_P[2][1]*roll_S[1][0]))/roll_den;
roll_K[2][1] = (-(roll_P[2][0]*roll_S[0][1])+(roll_P[2][1]*roll_S[0][0]))/roll_den;

//state update
roll_est += (roll_K[0][0] * roll_inn) + (roll_K[0][1] * roll_ang_vel_inn);
roll_ang_vel += (roll_K[1][0] * roll_inn) + (roll_K[1][1] * roll_ang_vel_inn);
Gyrooffset[1] += (roll_K[2][0] * roll_inn) + (roll_K[2][1] * roll_ang_vel_inn);

//covariance update
float roll_P00_temp = roll_P[0][0];
float roll_P01_temp = roll_P[0][1];
float roll_P02_temp = roll_P[0][2];
float roll_P10_temp = roll_P[1][0];
float roll_P11_temp = roll_P[1][1];
float roll_P12_temp = roll_P[1][2];

roll_P[0][0]=(1-roll_K[0][0])*roll_P00_temp - roll_K[0][1]*roll_P10_temp;
roll_P[0][1]=(1-roll_K[0][0])*roll_P01_temp - roll_K[0][1]*roll_P11_temp;
roll_P[0][2]=(1-roll_K[0][0])*roll_P02_temp - roll_K[0][1]*roll_P11_temp;
roll_P[1][0]= - roll_K[1][0]*roll_P00_temp + (1-roll_K[1][1])*roll_P10_temp;
roll_P[1][1]= - roll_K[1][0]*roll_P01_temp + (1-roll_K[1][1])*roll_P11_temp;
roll_P[1][2]= - roll_K[1][0]*roll_P02_temp + (1-roll_K[1][1])*roll_P11_temp;
roll_P[2][0]= - roll_K[2][0]*roll_P00_temp - roll_K[2][1]*roll_P10_temp + roll_P[2][0];
roll_P[2][1]= - roll_K[2][0]*roll_P01_temp - roll_K[2][1]*roll_P11_temp + roll_P[2][1];
roll_P[2][2]= - roll_K[2][0]*roll_P02_temp - roll_K[2][1]*roll_P12_temp + roll_P[2][2];

				

			
		//	printf("\n%f",meas[0]);
				}//bracket for read5
			}
    	}
			}
    	
		
		





void TIM_Config(void)
{  
 //timer 2 input capture
  
	//PrescalerValue=(uint16_t)(SystemCoreClock/500000)-1;
  TIM_TimeBaseStructure.TIM_Period = 35999;
  TIM_TimeBaseStructure.TIM_Prescaler = 79;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
 
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
	
//timer 3 pwm outputs
	PrescalerValue=(uint16_t)(SystemCoreClock/500000)-1;
  TIM_TimeBaseStructure.TIM_Period = 9999;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Init TIM_OCInitStructure */
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  /* Output Compare Toggle Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel2 */
 
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel3 */
 
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

  /* Output Compare Toggle Mode configuration: Channel4 */
 
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);


  //***************************timer 6 loop control*************************************888
	
	PrescalerValue=(uint16_t)(SystemCoreClock/500000)-1;
	TIM_TimeBaseStructure.TIM_Period = 35999;
  TIM_TimeBaseStructure.TIM_Prescaler = 9;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	
	//**************************timer 7 pid control********************************

	PrescalerValue=(uint16_t)(SystemCoreClock/3000)-1;
	TIM_TimeBaseStructure.TIM_Period = 99;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}


void GPIO_Config()
{
/* TIM2 channel 2 pin (Pd.04) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_2);
//   
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  

	/* GPIOc Configuration: TIM3 CH1 (Pc6) and TIM3 CH2 (Pc7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  /* GPIOB Configuration: TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    
  /* Connect TIM Channels to AF */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_2); 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);
	
	///////////USART
	/* Configure USART Tx & Rx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;//tx rx
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
		/* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_7);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_7);
	

}

void USART3_Config()
{
	  USART_InitTypeDef USART_InitStructure;
 
  /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
  /* USART configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 921600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
  /* USART configuration */
  USART_Init(USART3, &USART_InitStructure);
 
  /* Enable USART */
  USART_Cmd(USART3, ENABLE);
}	

void send_data(uint8_t data)
{
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){}; // Wait for Empty
    USART_SendData(USART3, data); // Send 'I'
}	

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART3, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {}

  return ch;
}



void Acc_Config(void)
{
//   LSM303DLHCAcc_InitTypeDef LSM303DLHCAcc_InitStructure;
//   LSM303DLHCAcc_FilterConfigTypeDef LSM303DLHCFilter_InitStructure;
     
   /* Fill the accelerometer structure */
  LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
  LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_200_HZ;
  LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
  LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
  LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Single;
  LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
  LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
  /* Configure the accelerometer main parameters */
  LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);
   
  /* Fill the accelerometer LPF structure */
  LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
  LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;
 
  /* Configure the accelerometer LPF main parameters */
  LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}

void Acc_ReadData(float* pfData)
{
  int16_t pnRawData[3];
  uint8_t ctrlx[2];
  float LSM_Acc_Sensitivity = LSM_Acc_Sensitivity_2g;
  uint8_t buffer[6], cDivider;
  uint8_t i = 0;
   
  /* Read the register content */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);   // this function is used to configure 2 control registers
	
	
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, buffer, 6);
  
   
  if(ctrlx[1]&0x40)
    cDivider=64;
  else
    cDivider=16;
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
   /* check in the control register4 the data alignment*/
	if(!(ctrlx[0] & 0x40) || (ctrlx[1] & 0x40)) /* Little Endian Mode or FIFO mode */
		{
			for(i=0; i<3; i++)
				{
					pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i])/cDivider;
				}
		}
	else /* Big Endian Mode */
		{
			for(i=0; i<3; i++)
			pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])/cDivider;
		}
/* Read the register content */
LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, ctrlx,2);
  

	
  for(i=0; i<3; i++)
  {
		if(i==0)
    pfData[i]=(float)((pnRawData[i]/(LSM_Acc_Sensitivity))+20);
		if(i==1)
		pfData[i]=(float)((pnRawData[i]/(LSM_Acc_Sensitivity))-5);
		else
		pfData[i]=(float)((pnRawData[i]/(LSM_Acc_Sensitivity))+0);	
  }
}

void gyroConfig(void)
{
  
  
  /* Configure Mems L3GD20 */
  L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
  L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_2;//1-95 ,,, 4-760
  L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
  L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_3;
  L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
  L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
  L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; // 250 or 500 or 2000
  L3GD20_Init(&L3GD20_InitStructure);
   
  L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
  L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_8; // 0 to 9
  L3GD20_FilterConfig(&L3GD20_FilterStructure) ;
  
  L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_DISABLE);// DISABLED
}


void gyroReadAngRate (float* pfData)          //probably this function actually accepts address of variables
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;

  L3GD20_Read(&tmpreg,L3GD20_CTRL_REG4_ADDR,1);         // probably used to configure CTRLREG4
  
  L3GD20_Read(tmpbuffer,L3GD20_OUT_X_L_ADDR,6);         // used to get gyro readings
  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & 0x40))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & 0x30)
  {
  case 0x00:
    sensitivity=L3G_Sensitivity_250dps;
    break;
    
  case 0x10:
    sensitivity=L3G_Sensitivity_500dps;
    break;
    
  case 0x20:
    sensitivity=L3G_Sensitivity_2000dps;
    break;
  }
  /* divide by sensitivity */
	
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)RawData[i]/sensitivity;
  }
}

void compassConfig(void)
{
  
  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_InitStructure.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
  LSM303DLHC_InitStructure.MagOutput_DataRate =LSM303DLHC_ODR_75_HZ ;
  LSM303DLHC_InitStructure.MagFull_Scale = LSM303DLHC_FS_8_1_GA;
  LSM303DLHC_InitStructure.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_InitStructure);
  
//    /* Fill the accelerometer structure */
//   LSM303DLHCAcc_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
//   LSM303DLHCAcc_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
//   LSM303DLHCAcc_InitStructure.Axes_Enable= LSM303DLHC_AXES_ENABLE;
//   LSM303DLHCAcc_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
//   LSM303DLHCAcc_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
//   LSM303DLHCAcc_InitStructure.Endianness=LSM303DLHC_BLE_LSB;
//   LSM303DLHCAcc_InitStructure.High_Resolution=LSM303DLHC_HR_ENABLE;
//   /* Configure the accelerometer main parameters */
//   LSM303DLHC_AccInit(&LSM303DLHCAcc_InitStructure);
//   
//   /* Fill the accelerometer LPF structure */
//   LSM303DLHCFilter_InitStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
//   LSM303DLHCFilter_InitStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
//   LSM303DLHCFilter_InitStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
//   LSM303DLHCFilter_InitStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;

//   /* Configure the accelerometer LPF main parameters */
//   LSM303DLHC_AccFilterConfig(&LSM303DLHCFilter_InitStructure);
}


/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void compassReadMag (float* pfData)
{
  static uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  uint16_t Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB, 1);
  
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, buffer, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, buffer+1, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, buffer+2, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, buffer+3, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, buffer+4, 1);
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, buffer+5, 1);
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }
  
  for(i=0; i<2; i++)
  {
    pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i] << 8) + buffer[2*i+1])*1000)/Magn_Sensitivity_XY;
  }
  pfData[2]=(float)((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5])*1000)/Magn_Sensitivity_Z;
}


void TIM2_IRQHandler(void)
{ 
	
   if(!(TIM_GetITStatus(TIM2, TIM_IT_CC4)) == RESET) 
   {
 		    
     TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
 		    
 		if(check==0)
			{
			  edge1 = TIM2->CCR4;
				check = 1;
			}
    else if(check==1) 
			{
				edge2 = TIM2->CCR4;
				check=0;
				counts=36000*counter + edge2-edge1;	
				CCR=counts*500/900;
				counter=0;
	     }
    }
	
	
	if(!(TIM_GetITStatus(TIM2, TIM_IT_CC3)) == RESET) 
  {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
		
		if(check1==0)
			{
				edge3 = TIM2->CCR3;
				check1 = 1;
			}
    else if(check1==1) 
			{
				edge4 = TIM2->CCR3;
				check1=0;
				counts1=36000*counter1 + edge4-edge3;	
				if (counts1>36000)
						counts1=18000;
				CCR_3=(float)(counts1-1350)*200.0/2700.0;//5 degrees till 1600
				//k_i[0]=0.5+CCR_3;
				set_roll=-1*CCR_3;
				//set_pitch=CCR_3;
				counter1=0;
			}
	}
	
	if(!(TIM_GetITStatus(TIM2, TIM_IT_CC2)) == RESET) 
  {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		
		if(check2==0)
			{
				edge5 = TIM2->CCR2;
				check2 = 1;
			}
    else if(check2==1) 
			{
				edge6 = TIM2->CCR2;
				check2=0;
				counts2=36000*counter2 + edge6-edge5;	
				if (counts2>36000)
						counts2=18000;
				CCR_2=(float)(counts2-1350)*200.0/2700.0;//5 degrees till 1600
				set_pitch=CCR_2;
				counter2=0;
			}
	}
	
	
	if(!(TIM_GetITStatus(TIM2, TIM_IT_Update)) == RESET)
	{
		flag=1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if (check==1)
				counter++;
		
		if (check1==1)
				counter1++;
		
		if (check2==1)
				counter2++;
	}
}


void TIM6_DAC1_IRQHandler(void)
{
	if(!(TIM_GetITStatus(TIM6, TIM_IT_Update)) == RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		takeaction=1;
		readsensor=1;
	}
}


void TIM7_DAC2_IRQHandler(void)
{
	if(!(TIM_GetITStatus(TIM7, TIM_IT_Update)) == RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		pidupdate=1;
	}
}

#endif

