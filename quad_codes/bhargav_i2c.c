#include "stm32f30x.h"
#define SDA GPIO_Pin_7
#define SCL GPIO_Pin_6
int receive_data=1;
uint8_t scldel;
uint8_t sdadel;

uint32_t uncomp_temp=0;
I2C_InitTypeDef I2C_InitTypeDefStructure;
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
void I2C_Config(void);
void GPIO_Config(void);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
uint8_t received_data[2],Data=0,data1=0,check=0,receive_flag=0;
/*for 6 ms delay*/
void delay(void)
{ 
		 unsigned long i=432;
	for(i=432;i<=0;i--)
	{
	}
	
}
void Start_tmeas(void)
{
	I2C_GenerateSTART(I2C1,ENABLE);
	I2C_SendData(I2C1,0xEE);//write address
	receive_data=I2C_ReceiveData(I2C1);
	if (receive_data==0)
	{ receive_data=1;
		I2C_SendData(I2C1,0xF4);//control register address
	}
		receive_data=I2C_ReceiveData(I2C1);
		if(receive_data==0)
		{ receive_data=1;
			I2C_SendData(I2C1,0x2E);//word for pressure measurement
		}
			receive_data=I2C_ReceiveData(I2C1);
			if(receive_data==0)
			{receive_data=1;
				I2C_GenerateSTOP(I2C1,ENABLE);
			}
		}

	uint16_t Read_UT(void) 
		{					
							uint16_t UT;

							Start_tmeas();
							delay(); // Wait for 4.5ms by datasheet

							I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge
							I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
							receive_data=I2C_ReceiveData(I2C1);
									if (receive_data==0)
									{		
											receive_data=1;
											I2C_SendData(I2C1,0xEE); // Send slave address
									}
							receive_data=I2C_ReceiveData(I2C1);
									if (receive_data==0)
									{	
										receive_data=1;
										I2C_SendData(I2C1,0xF6); // Send ADC MSB register address
									}
							receive_data=I2C_ReceiveData(I2C1);
									if (receive_data==0)
									{  
										receive_data=1;
										I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
										I2C_SendData(I2C1,0xEF); // Send slave address for READ
									}
												receive_data=I2C_ReceiveData(I2C1);
									if (receive_data==0)
									{	
											receive_data=1;
											UT = (uint16_t)I2C_ReceiveData(I2C1) << 8; // Receive MSB
											I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgment
											UT |= I2C_ReceiveData(I2C1); // Receive LSB
											I2C_AcknowledgeConfig(I2C1,ENABLE);
											I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition
									}
							return UT;
}






int main()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
	I2C_Config();
	GPIO_Config();
  I2C_Cmd(I2C1, ENABLE);	
	while(1)
	{		
		
		uncomp_temp=Read_UT();

	}
	
}

void GPIO_Config(void)
{
/*-------- Configuring  --------*/
 
	GPIO_InitStructure.GPIO_Pin = SDA | SCL ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 //Add an external pull up if necessary (typically 4.7 KOhm).  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);

}
 
 void I2C_Config(void)
{
	 I2C_InitTypeDefStructure.I2C_DigitalFilter = 0;
	 I2C_InitTypeDefStructure.I2C_AnalogFilter = ENABLE;
	 I2C_InitTypeDefStructure.I2C_Timing = 0x10C08DCF; //This parameter must be set to a value lower than 100kHz */
	 I2C_InitTypeDefStructure.I2C_Mode = I2C_Mode_I2C  ;
	 //I2C_InitTypeDefStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	 I2C_InitTypeDefStructure.I2C_OwnAddress1 = 0;
	 I2C_InitTypeDefStructure.I2C_Ack = I2C_Ack_Enable ;
	 I2C_InitTypeDefStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	 I2C1->TIMINGR = 0x10420F13| ((scldel & 0x0F) << 20) | ((sdadel & 0x0F) << 16);

   I2C_Init(I2C1, &I2C_InitTypeDefStructure);
}
 /*for initializing temperature and pressure measurement*/
void Start_pmeas(void)
{
	I2C_GenerateSTART(I2C1,ENABLE);
	I2C_SendData(I2C1,0xEE);//write address
	receive_data=I2C_ReceiveData(I2C1);
	if (receive_data==0)
	{ receive_data=1;
		I2C_SendData(I2C1,0xF4);//control register address
	}
		receive_data=I2C_ReceiveData(I2C1);
		if(receive_data==0)
		{ receive_data=1;
			I2C_SendData(I2C1,0x34);//word for pressure measurement
		}
			receive_data=I2C_ReceiveData(I2C1);
			if(receive_data==0)
			{receive_data=1;
				I2C_GenerateSTOP(I2C1,ENABLE);
			}
		}
