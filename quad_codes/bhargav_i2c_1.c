#include "stm32f30x.h"
#define SDA GPIO_Pin_7
#define SCL GPIO_Pin_6
#define I2C1_CR2  (*((volatile unsigned long *)0x00004))
int receive_data=1,TXIS_flag=0,RXNE_flag=0,NACKF_flag=0;
uint16_t uncomp_temp=0;
uint8_t scldel;
uint8_t sdadel;
I2C_TypeDef I2C_TypeDefStructure;
I2C_InitTypeDef I2C_InitTypeDefStructure;
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
void I2C_Config(void);
void GPIO_Config(void);
void delay(void)		// 6 ms delay
{	int i=0;
	for(i=0;i<=72000;i++)
	{}
}

void Start_tmeas(void)
{					
					I2C_10BitAddressingModeCmd (I2C1,DISABLE);
	        I2C_AutoEndCmd(I2C1,DISABLE);
					I2C_NumberOfBytesConfig (I2C1,2);
					I2C_SlaveAddressConfig(I2C1,0xEE);//write address
					I2C_MasterRequestConfig(I2C1,I2C_Direction_Transmitter);
				  I2C_StretchClockCmd (I2C1,ENABLE);
					I2C_GenerateSTART(I2C1,ENABLE);
					//TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TXIS);
					while (TXIS_flag==0)
					{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TXIS);}				
										TXIS_flag=0;
										I2C_SendData(I2C1,0xF4);//control register address
				
				  while (TXIS_flag==0)
					{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE);}
										TXIS_flag=0;
										I2C_SendData(I2C1,0x2E);//word for temperature measurement
				
					while (TXIS_flag==0)
					{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TC);}						
										I2C_TransferHandling (I2C1,0xEE,0,I2C_SoftEnd_Mode,I2C_Generate_Stop);
					//		I2C_GenerateSTOP(I2C1,ENABLE);
}

	uint16_t Read_UT(void) 
		{					
							uint16_t UT;

							Start_tmeas();
							delay(); // Wait for max 4.5ms by datasheet
			
							I2C_10BitAddressingModeCmd (I2C1,DISABLE);
							I2C_NumberOfBytesConfig (I2C1,1);
							I2C_SlaveAddressConfig(I2C1,0xEE);//write address
							I2C_AutoEndCmd(I2C1,DISABLE);
							I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge
							I2C_MasterRequestConfig(I2C1,I2C_Direction_Transmitter);
							I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
          
			        while (TXIS_flag==0)
					    {TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TXIS);	}					
											TXIS_flag=0;
											I2C_SendData(I2C1,0xF6); // Send ADC MSB register address											
						  while (TXIS_flag==0)
					    {TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TC);}
							TXIS_flag=0;
							I2C_GenerateSTOP(I2C1,ENABLE);
							I2C_GenerateSTART(I2C1,DISABLE);
							//I2C1->CR2&= 0xFFFFEFFF;
						//	I2C1->CR2|= 0x00000400;
//							I2C_GenerateSTOP(I2C1,ENABLE);
							I2C_MasterRequestConfig(I2C1,I2C_Direction_Receiver);
							I2C_10BitAddressingModeCmd (I2C1,DISABLE);
							I2C_AutoEndCmd(I2C1,DISABLE);
							I2C_StretchClockCmd (I2C1,ENABLE);
							  I2C_SlaveAddressConfig(I2C1,0xEF);//write address
							  I2C_NumberOfBytesConfig (I2C1,2);
							I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
                //I2C_TransferHandling (I2C1,0xEF,1,I2C_SoftEnd_Mode,I2C_Generate_Start_Read);										
//								I2C_SendData(I2C1,0xEF); // Send slave address for READ				
						  while (TXIS_flag==0)
					   {TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE);}
							TXIS_flag=0;
						  while (RXNE_flag==0)
					    {RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);}
							RXNE_flag=0;							
							I2C_AcknowledgeConfig(I2C1,ENABLE);
						//	UT = (uint16_t)I2C_ReceiveData(I2C1)<< 8; // Receive MSB
//											UT|= I2C_ReceiveData(I2C1); // Receive LSB
							 UT=I2C_ReadRegister (I2C1,I2C_Register_RXDR);
							   UT=UT<<8;
////					RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);
							while (RXNE_flag==0)
							{RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);}			 	
							RXNE_flag=0;
							UT|=I2C_ReadRegister(I2C1,I2C_Register_RXDR);
							while (TXIS_flag==0)
							{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TC);}	
							I2C_AcknowledgeConfig(I2C1,DISABLE);
							I2C_GenerateSTOP(I2C1,ENABLE);
							//I2C_TransferHandling (I2C1,0xEE,0,I2C_SoftEnd_Mode,I2C_Generate_Stop);
										return UT;
}
		
uint16_t  read_tmeas(void)
{
					uint32_t ucomp;
					uint16_t ucomp1;
					I2C_10BitAddressingModeCmd (I2C1,DISABLE);
	        I2C_AutoEndCmd(I2C1,DISABLE);
					I2C_NumberOfBytesConfig (I2C1,2);
					I2C_SlaveAddressConfig(I2C1,0xEF);//write address
					I2C_MasterRequestConfig(I2C1,I2C_Direction_Receiver);
				  I2C_StretchClockCmd (I2C1,ENABLE);
					I2C_AcknowledgeConfig(I2C1,DISABLE);
					I2C_GenerateSTART(I2C1,ENABLE);
					//TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TXIS);
					while (TXIS_flag==0)
					{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE);}				
					TXIS_flag=0;
					while (RXNE_flag==0)
							{RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);}			 	
							RXNE_flag=0;
					//I2C_ReadRegister(I2C1,I2C_Register_RXDR);//control register address
							RXNE_flag=0;
						//	UT = (uint16_t)I2C_ReceiveData(I2C1)<< 8; // Receive MSB
//											UT|= I2C_ReceiveData(I2C1); // Receive LSB
							ucomp1=(uint16_t)I2C_ReceiveData(I2C1)<<8;
								ucomp=(uint16_t)I2C_ReadRegister (I2C1,I2C_Register_RXDR)<<8;
							  // ucomp=ucomp<<8;
								 I2C_AcknowledgeConfig(I2C1,ENABLE);

////					RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);
							ucomp1|=I2C_ReceiveData(I2C1);
							while (RXNE_flag==0)
							{RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);}			 	
							RXNE_flag=0;
							ucomp|=I2C_ReadRegister (I2C1,I2C_Register_RXDR);
							TXIS_flag=0;
							
							while (TXIS_flag==0)
							{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TC);}	
							I2C_AcknowledgeConfig(I2C1,DISABLE);
							I2C_GenerateSTOP(I2C1,ENABLE);
							return ucomp1;
}
int main()
{			    	
						I2C_StretchClockCmd (I2C1,ENABLE);
						RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
						RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
						uint32_t bhargav;
						I2C_Config();
						GPIO_Config();
						I2C_Cmd(I2C1, ENABLE);
						while(1)
						{//bhargav=read_tmeas();
							//delay();
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
	 //I2C_InitTypeDefStructure.I2C_Timing = 0x10C08DCF; //This parameter must be set to a value lower than 100kHz */
	 I2C_InitTypeDefStructure.I2C_Mode = I2C_Mode_I2C  ;
	 //I2C_InitTypeDefStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	 I2C_InitTypeDefStructure.I2C_OwnAddress1 = 0;
	 I2C_InitTypeDefStructure.I2C_Ack = I2C_Ack_Enable ;
	 I2C_InitTypeDefStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	 I2C1->TIMINGR = 0x10420F13| ((scldel & 0x0F) << 20) | ((sdadel & 0x0F) << 16);
   I2C_Init(I2C1, &I2C_InitTypeDefStructure);
}
