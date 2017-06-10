#include "stm32f30x.h"
#define SDA GPIO_Pin_7
#define SCL GPIO_Pin_6
#define I2C1_CR2  (*((volatile unsigned long *)0x00004))
int receive_data=1,TXIS_flag=0,RXNE_flag=0,NACKF_flag=0;
uint16_t uncomp_temp=0,ucomp_pres=0;
int16_t true_temp=0,true_pres=0;
uint8_t scldel;
uint8_t sdadel;
I2C_TypeDef I2C_TypeDefStructure;
I2C_InitTypeDef I2C_InitTypeDefStructure;
GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
void I2C_Config(void);
void GPIO_Config(void);
typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	int32_t B5;
} BMP180_Calibration_TypeDef;
/* Calibration parameters from E2PROM of BMP180 */
BMP180_Calibration_TypeDef BMP180_Calibration;

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


void Start_pmeas(void)
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
										I2C_SendData(I2C1,0x74);//word for temperature measurement
				
					while (TXIS_flag==0)
					{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TC);}						
										I2C_TransferHandling (I2C1,0xEE,0,I2C_SoftEnd_Mode,I2C_Generate_Stop);
					//		I2C_GenerateSTOP(I2C1,ENABLE);
}

uint16_t Read_UP(void)
{						
							uint16_t UP;

							Start_pmeas();
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
							  I2C_NumberOfBytesConfig (I2C1,3);
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
							 UP=I2C_ReadRegister (I2C1,I2C_Register_RXDR);
							 UP=UP<<16;
////					RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);
							while (RXNE_flag==0)
							{RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);}			 	
							RXNE_flag=0;
							UP|=I2C_ReadRegister(I2C1,I2C_Register_RXDR);
							UP=UP<<8;
							while (RXNE_flag==0)
							{RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);}			 	
							RXNE_flag=0;
							UP|=I2C_ReadRegister(I2C1,I2C_Register_RXDR);
							while (TXIS_flag==0)
							{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TC);}	
							I2C_AcknowledgeConfig(I2C1,DISABLE);
							I2C_GenerateSTOP(I2C1,ENABLE);
							//I2C_TransferHandling (I2C1,0xEE,0,I2C_SoftEnd_Mode,I2C_Generate_Stop);
								
							return UP>>7;
}

void BMP180_ReadCalibration(void)
{	uint8_t i;
	uint8_t buffer[22];

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
											I2C_SendData(I2C1,0xAA); // Send ADC MSB register address	
							while (TXIS_flag==0)
					    {TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TC);}
							TXIS_flag=0;
											I2C_GenerateSTOP(I2C1,ENABLE);
											I2C_GenerateSTART(I2C1,DISABLE);
							
							I2C_MasterRequestConfig(I2C1,I2C_Direction_Receiver);
							I2C_10BitAddressingModeCmd (I2C1,DISABLE);
							I2C_AutoEndCmd(I2C1,DISABLE);
							I2C_StretchClockCmd (I2C1,ENABLE);
							I2C_SlaveAddressConfig(I2C1,0xEF);//write address
						  I2C_NumberOfBytesConfig (I2C1,22);
							I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
							
							while (TXIS_flag==0)
					   {TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE);}
							TXIS_flag=0;
							for (i=0;i<21;i++)
	{
							while (RXNE_flag==0)
					    {RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);}
							RXNE_flag=0;							
							I2C_AcknowledgeConfig(I2C1,ENABLE);
		
							buffer[i] =I2C_ReadRegister(I2C1,I2C_Register_RXDR); //Receive byte
	}
							while (RXNE_flag==0)
							{RXNE_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_RXNE);}			 	
											RXNE_flag=0;
							buffer[i] = I2C_ReadRegister(I2C1,I2C_Register_RXDR); // Receive last byte
							while (TXIS_flag==0)
							{TXIS_flag=I2C_GetFlagStatus(I2C1,I2C_FLAG_TC);}	
							I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgment
							I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition

	BMP180_Calibration.AC1 = (buffer[0]  << 8) | buffer[1];
	BMP180_Calibration.AC2 = (buffer[2]  << 8) | buffer[3];
	BMP180_Calibration.AC3 = (buffer[4]  << 8) | buffer[5];
	BMP180_Calibration.AC4 = (buffer[6]  << 8) | buffer[7];
	BMP180_Calibration.AC5 = (buffer[8]  << 8) | buffer[9];
	BMP180_Calibration.AC6 = (buffer[10] << 8) | buffer[11];
	BMP180_Calibration.B1  = (buffer[12] << 8) | buffer[13];
	BMP180_Calibration.B2  = (buffer[14] << 8) | buffer[15];
	BMP180_Calibration.MB  = (buffer[16] << 8) | buffer[17];
	BMP180_Calibration.MC  = (buffer[18] << 8) | buffer[19];
	BMP180_Calibration.MD  = (buffer[20] << 8) | buffer[21];
}
int16_t BMP180_Calc_RT(uint16_t UT) 
{
	BMP180_Calibration.B5  = (((int32_t)UT - (int32_t)BMP180_Calibration.AC6) * (int32_t)BMP180_Calibration.AC5) >> 15;
	BMP180_Calibration.B5 += ((int32_t)BMP180_Calibration.MC << 11) / (BMP180_Calibration.B5 + BMP180_Calibration.MD);

	return (BMP180_Calibration.B5 + 8) >> 4;
}

int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss) 
{
	int32_t B3,B6,X3,p;
	uint32_t B4,B7;

	B6 = BMP180_Calibration.B5 - 4000;
	X3 = ((BMP180_Calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_Calibration.AC2 * B6) >> 11);
	B3 = (((((int32_t)BMP180_Calibration.AC1) * 4 + X3) << oss) + 2) >> 2;
	X3 = (((BMP180_Calibration.AC3 * B6) >> 13) + ((BMP180_Calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (BMP180_Calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000) 
	{p = (B7 << 1) / B4;} 
	else 
	{p = (B7 / B4) << 1;}
	p += ((((p >> 8) * (p >> 8)*3038)>> 16) + ((-7357* p) >> 16)+3791) >> 4;

	return p;
}
int main()
{			    	
						I2C_StretchClockCmd (I2C1,ENABLE);
						RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
						RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
					//	uint32_t bhargav;
						I2C_Config();
						GPIO_Config();
						I2C_Cmd(I2C1, ENABLE);
						BMP180_ReadCalibration();
						while(1)
						{  
								//BMP180_ReadCalibration();
							//bhargav=read_tmeas();
							   //delay();
								uncomp_temp=Read_UT();
								ucomp_pres=Read_UP();
								true_temp= BMP180_Calc_RT(uncomp_temp);
								//true_pres=BMP180_Calc_RP(ucomp_pres,1);
								
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
