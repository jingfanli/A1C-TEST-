/**
  ******************************************************************************
  * @file    programm.c
  * @author  jingfan.li
  * @version V1.0.0
  * @date    08/2017
  * @brief   delay functions
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/

#include "stm8l15x_clk.h"
#include "Programm.h"
/**
  * @brief  
  * @caller auto_test
  * @param  
  * @retval None
  */

float  vdd_cal;
float a;
bool usart_sdflag;
bool usart_reflag;
uint16_t usart_buffer[64];
uint8_t usart_recI;
uint16_t num1;
uint16_t pulsenum;
uint16 m_u16_TickCycle = {0};
static uint16_t tick_num=0;
static uint16_t  pwm_num=0;

static uint8_t  send_num=0;
static uint8_t pwm_flag=0;
static uint8_t speed=0;
static uint8_t send_flag=0;
static uint8_t tick_num_send=0;


void gpio_init(void)
{
	GPIOA->DDR = 0xC7;
	GPIOB->DDR = 0xF7;
	GPIOC->DDR = 0xFC;
	GPIOD->DDR = 0xFF;
	GPIOE->DDR = 0xFF;
	GPIOF->DDR = 0xF3;
	GPIOG->DDR = 0xFF;
	GPIOH->DDR = 0xFF;
	GPIOI->DDR = 0xFF;

	GPIOA->CR1 = 0xC7;
	GPIOB->CR1 = 0xF7;
	GPIOC->CR1 = 0xFC;
	GPIOD->CR1 = 0x3F;
	GPIOE->CR1 = 0xFF;
	GPIOF->CR1 = 0xF2;
	GPIOG->CR1 = 0xFF;
	GPIOH->CR1 = 0xFF;
	GPIOI->CR1 = 0xFF;
}


void adc_calc()
{
	uint16_t adcvals[10];
	uint16_t averad=0;
	float addis;
	uint8_t i,n;
	for(i=0;i<10;i++)
		{
			adcvals[i]= adc_getval(ADC1,ADC_Channel_Vrefint);
		}

	

	i=0;
	sort(&adcvals[0],10);

	for(i=1;i<9;i++)
		{
			averad=averad+adcvals[i];
			
		}
    averad=averad/8;
	vdd_cal=(1.224L)*4096/averad;
}
uint16_t adc_getval(ADC_TypeDef* ADCx,ADC_Channel_TypeDef ADC_Channels)
{
	uint16_t adval;
	ADC_DeInit(ADC1);
	CLK_PeripheralClockConfig(CLK_Peripheral_ADC1,ENABLE);
	ADC_Init (ADC1,ADC_ConversionMode_Single,ADC_Resolution_12Bit,ADC_Prescaler_1);

	ADC_Cmd(ADC1,ENABLE);
	ADC_VrefintCmd(ENABLE);
	ADC_SamplingTimeConfig(ADC1,ADC_Group_SlowChannels, ADC_SamplingTime_9Cycles);
	ADC_ChannelCmd(ADC1,ADC_Channels,ENABLE);
	ADC_SoftwareStartConv(ADC1);

	while((ADC1->SR&0x01)==0);

	adval=(ADC1->DRL);
	adval=adval|(ADC1->DRH)<<8;
	
        
        ADC_ChannelCmd(ADC1,ADC_Channels,DISABLE);
	ADC_Cmd(ADC1,DISABLE);
        
	ADC_VrefintCmd(ENABLE);
	CLK_PeripheralClockConfig(CLK_Peripheral_ADC1,DISABLE);
	return adval;
	
		
	
}


void lcd_config(void)
{
	CLK_PeripheralClockConfig(CLK_Peripheral_RTC,ENABLE);
	CLK_PeripheralClockConfig(CLK_Peripheral_LCD,ENABLE);
	
	CLK_RTCClockConfig(CLK_RTCCLKSource_LSI,CLK_RTCCLKDiv_1);

	LCD_Init(LCD_Prescaler_1,LCD_Divider_31,LCD_Duty_1_4,LCD_Bias_1_3,LCD_VoltageSource_Internal);

	LCD_PortMaskConfig(LCD_PortMaskRegister_0, 0xFF);
  	LCD_PortMaskConfig(LCD_PortMaskRegister_1, 0xFF);
  	LCD_PortMaskConfig(LCD_PortMaskRegister_2, 0xff);
  	LCD_ContrastConfig(LCD_Contrast_3V0);
	LCD_DeadTimeConfig(LCD_PulseOnDuration_0);
	LCD_PulseOnDurationConfig(LCD_PulseOnDuration_1);
	
	
	LCD_Cmd(ENABLE);
	
}

void usart_config()
{
	uint32_t BaudRate_Mantissa;
	
              GPIO_ExternalPullUpConfig(GPIOH,GPIO_Pin_4 | GPIO_Pin_5,ENABLE);
              CLK_PeripheralClockConfig(CLK_Peripheral_USART2,ENABLE);
		//USART2->CR1 |=  ; //odd even cheak,enable the function set the 8bit data transit
		USART2->CR2 |= (0x04| 0x08 | 0X20) ;//enable the tx and rx interrupt
		
		BaudRate_Mantissa  = (uint32_t)(16000000/115200);
		USART2->BRR2 =(uint8_t)((BaudRate_Mantissa)&0x0f);
		USART2->BRR2 |=(uint8_t)((BaudRate_Mantissa)>>(uint8_t)8)&0xf0;
	
		USART2->BRR1 |=(BaudRate_Mantissa>>(uint8_t)4);
	
		USART2->CR4 =adress; 
		USART2->CR1 &=~0X20;

		
		//SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA,ENABLE);
	
	
}
void uart1_recive(uint8_t *rebuf,uint8_t *renum)
{
	uint8_t recive_num;
	uint8_t i;
	recive_num=usart_recI;
	*renum=0;
	//delay_ms(10);
	if(recive_num==usart_recI&&recive_num)
		{
			for(i=0;i<recive_num;i++)
				{
					*(rebuf+i)=usart_buffer[i];
					i++;
				}
			*renum=recive_num;
			usart_recI=0;
			
		}
	
}
void uart1_senddata(uint8_t *x,uint8_t n)
{
	uint8_t i;
	for(i=0;i<n;i++)
		{
			uart1_sendbyte(*(x+i));
		}
	
	
}

void uart1_sendbyte(uint8_t data)
{
	USART1->DR=data;
	while((USART1->SR&0x80)==0);
	usart_recI=0;
}



void uart2_sendbyte(uint8_t data)
{
	USART2->DR=data;
	while((USART2->SR&0x80)==0);
	usart_recI=0;
}

void TIM2_ETR_config()
{
	CLK_PeripheralClockConfig(CLK_Peripheral_TIM2 ,ENABLE);
	TIM2->ETR |= 0xc0;
	TIM2->CR1 |=0x01;
	
}
void TIM2_PULSE_CLAC(uint16_t * n)
{	
	
	*n=TIM2->CNTRL;
	*n |=(TIM2->CNTRH)>>8;
}
void TIM3_PWMCONFIG(void)
{
	
}
int mot_drive(motor_flag state)
{
	switch(state)
		{
			case 0:                
                    GPIO_SetBits(GPIOF,GPIO_Pin_5);
					GPIO_ResetBits(GPIOF,GPIO_Pin_6);
					break;
			case 1:	GPIO_SetBits(GPIOF,GPIO_Pin_6);
				GPIO_ResetBits(GPIOF,GPIO_Pin_5);
					break;
			case 2:   GPIO_SetBits(GPIOF,GPIO_Pin_6);
                      GPIO_SetBits(GPIOF,GPIO_Pin_5);
					break;
			default :break;
		}
}

void Button_init(void)
{
	uint8_t u8_Value;
	
	u8_Value=DRV_GPIO_MODE_INPUT;
	DrvGPIO_SetConfig(Button_start,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	u8_Value=1;
	DrvGPIO_SetConfig(Button_start,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);
	
}

void ready_init(void)
{
	uint8_t u8_Value;
	u8_Value=DRV_GPIO_MODE_INPUT;
	DrvGPIO_SetConfig(testcard_present,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	u8_Value=1;
	DrvGPIO_SetConfig(testcard_present,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);
}


char Ready_flag(void)
{
	char flag;
	
	flag=0;
	
	flag=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3);

	if(flag==0)
		flag=FUNCTION_SUCCESS;
	else 
		flag=FUNCTION_FAIL;
	return flag;
}

void open_led1(void)
{
	GPIO_SetBits(GPIOF,GPIO_Pin_0);
}
void open_led2(void)
{
	GPIO_SetBits(GPIOF,GPIO_Pin_1);
}

BitStatus clear_etr(void)
{
	uint16_t num;	
	TIM2->CNTRL=0;
	TIM2->CNTRL=0;
	TIM2_PULSE_CLAC(&num);
	if(num==0)
		return FUNCTION_SUCCESS;
	else 
		return FUNCTION_FAIL;
}

BitStatus BUTTON_start_READ()
{
	BitStatus button_staus;
	button_staus=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7);
	
	return button_staus;
}
int TestHA1c_process(void)
{
	char ready_flag=0;
	BitStatus locacteA=0;
	BitStatus locacteB=0;
	uint16_t clear_etrcnt;
	uint16_t tempnum;
	uint16_t ledsensorval;

	pulsenum=0;
	tempnum=0;
	
	clear_etrcnt=0;
	ready_flag=DrvGPIO_Read(Button_start);
	if(ready_flag==0)
	{
		ready_flag=Ready_flag();
		if(ready_flag==1)
			{
				if(Ready_flag())
					{
						//open_led1();
						//open_led2();
						//delay_ms(2);
						locacteA=DrvGPIO_Read(locateA);

						while(locacteA==0)
							{
								mot_drive(Reverse);
								locacteA=DrvGPIO_Read(locateA);
							}
					while(!clear_etr()==FUNCTION_SUCCESS)
						{
							clear_etr();
							clear_etrcnt++;
								if(clear_etrcnt>1000)
									{
										break;
									}
						}
					if(clear_etr()==FUNCTION_SUCCESS)
						{
						locacteB = DrvGPIO_Read(locateB);
							while(locacteB==0)
								{
								
								PWM_MOT(4);

								TIM2_PULSE_CLAC(&pulsenum);

						if(send_flag)
							{
							ledvalsend();
							send_flag=0;
							tick_num_send=0;
							}
				

									/*if((pulsenum-tempnum)>=10)
										{	pwm_flag=0;
											mot_drive(stop);
											TIM2_PULSE_CLAC(&pulsenum);
											tempnum=pulsenum;
											ledvalsend();
											pwm_flag=1;
										}*/
								/*if((pulsenum-tempnum)>=5)
									{
										mot_drive(stop);
										delay_ms(10);
										tempnum = pulsenum;
										
										//ledvalsend();
										
										
									}*/
								//mot_drive(Clockwise);
								locacteB = DrvGPIO_Read(locateB);
								}
							//DrvGPIO_Clear(LED1);
							//DrvGPIO_Clear(LED2);
							locacteA = DrvGPIO_Read(locateA);
							while(locacteA==0)
								{
								pwm_flag=0;
								mot_drive(Reverse);
								locacteA=DrvGPIO_Read(locateA);

								}
							
							mot_drive(stop);
							clear_etr();
							return FUNCTION_SUCCESS;
						}
				}
			else
				{
					return FUNCTION_FAIL;
				}
		}
	else
		return FUNCTION_FAIL;
	
}
}


void ledvalsend(void)
{
	uint16_t led1tabl1[10];
	uint16_t led1tabl2[10];
	uint16_t ledsensor1=0;
	uint16_t ledsensor2=0;
	uint8_t  temp;
	uint8_t ledtabl[10]={0};
	uint8_t i;


	for(i=0;i<10;i++)
		{
			led1tabl1[i]= adc_getval(ADC1,ADC_Channel_2);
		}
	sort(&led1tabl1[0],10);

	for(i=1;i<9;i++)
		{
			ledsensor1=ledsensor1+led1tabl1[i];
			
		}
    ledsensor1=ledsensor1/8;

		for(i=0;i<10;i++)
		{
			led1tabl2[i]= adc_getval(ADC1,ADC_Channel_1);
		}
	sort(&led1tabl2[0],10);

	for(i=1;i<9;i++)
		{
			ledsensor2=ledsensor2+led1tabl2[i];
			
		}
    ledsensor2=ledsensor2/8;
	
	ledsensor1=HtoD_4(ledsensor1);
	ledsensor2=HtoD_4(ledsensor2);

	temp=(ledsensor1)>>12;
	temp=temp&0x0f;
	temp=chartohex(temp);
	ledtabl[0]=temp;

	temp=(ledsensor1)>>8;
	temp=temp&0x0f;
	temp=chartohex(temp);
	ledtabl[1]=temp;

	temp=(ledsensor1)>>4;
	temp=temp&0x0f;
	temp=chartohex(temp);
	ledtabl[2]=temp;
	
	temp=(ledsensor1)>>0;
	temp=temp&0x0f;
	temp=chartohex(temp);
	ledtabl[3]=temp;

	ledtabl[4]=0x09;
	
	temp=(ledsensor2)>>12;
	temp=temp&0x0f;
	temp=chartohex(temp);
	ledtabl[5]=temp;

	temp=(ledsensor2)>>8;
	temp=temp&0x0f;
	temp=chartohex(temp);
	ledtabl[6]=temp;

	temp=(ledsensor2)>>4;
	temp=temp&0x0f;
	temp=chartohex(temp);
	ledtabl[7]=temp;
	
	temp=(ledsensor2)>>0;
	temp=temp&0x0f;
	temp=chartohex(temp);
	ledtabl[8]=temp;

	ledtabl[9]=0x0a;

	for(temp=0;temp<10;temp++)
		{
			uart2_sendbyte(ledtabl[temp]);
		}

	
	

}

unsigned char chartohex(unsigned char hexdata)
{
	if(hexdata<=9&&hexdata>=0)
		{
			hexdata+=0x30;
		}
	else if(hexdata<=15&&hexdata>=10)
		{
			hexdata+=0x37;
		}
	else
		{
			hexdata=0xff;
		}
	return hexdata;

}

unsigned int HtoD_4(unsigned int adata)
{
    unsigned char  i,xbuf[4]; 
    unsigned int  dd,div;
    dd=1000;
    for (i=0;i<4;i++)                 
    {                                
        xbuf[i]=adata/dd;
        adata%=dd;
        dd/=10;
     }
    div=xbuf[0];
    div<<=4;
    div|=xbuf[1];
    div<<=4;
    div|=xbuf[2];
    div<<=4;
    div|=xbuf[3];
    return div;
}
void locate_init(void)
{
	uint8_t u8_Value;
	u8_Value=DRV_GPIO_MODE_INPUT;
	DrvGPIO_SetConfig(locateA,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(locateB,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	u8_Value=1;
	DrvGPIO_SetConfig(locateB,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(locateA,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);
}

void Distance_Sensor(void)
{
	uint8_t u8_Value;
	u8_Value=DRV_GPIO_MODE_OUTPUT;
	DrvGPIO_SetConfig(Distance_sensor,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	
	u8_Value=0;
	DrvGPIO_SetConfig(Distance_sensor,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);
	
	DrvGPIO_Set(Distance_sensor);
}

void adgpio_init(void)
{
	uint8_t u8_Value;
	u8_Value=DRV_GPIO_MODE_INPUT;
	DrvGPIO_SetConfig(LED_SENSOR1,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(LED_SENSOR2,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(PD4,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(PD5,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	u8_Value=0;
	DrvGPIO_SetConfig(LED_SENSOR1,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(LED_SENSOR2,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);

	DrvGPIO_SetConfig(PD4,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(PD5,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);

	//DrvGPIO_Set(LED_SENSOR1);
	//DrvGPIO_Set(LED_SENSOR2);
}


uint DrvRTC_Initialize(void)
{
	uint ui_Value;


	m_u16_TickCycle = 10;

	CLK->PCKENR2 |= CLK_PCKENR2_RTC;

	/* Disable the write protection for RTC registers */
	RTC->WPR = DRV_RTC_UNLOCK_KEY1;
	RTC->WPR = DRV_RTC_UNLOCK_KEY2;
	
	RTC->CR1 = 0x03;
	RTC->CR2 = 0;
	RTC->ISR1= 0x01;
	
	/* Enable alarm for hour and minute */
	RTC->ALRMAR1 = 0;
	RTC->ALRMAR2 &= ~RTC_ALRMAR2_MSK2;
	RTC->ALRMAR3 &= ~RTC_ALRMAR3_MSK3;
	RTC->ALRMAR4 |= RTC_ALRMAR4_MSK4;

	/* Enable wake up and alarm unit Interrupt */
	RTC->CR2 |= (uint8)((RTC_IT_WUT ) & 0x00F0);
	RTC->TCR1 |= (uint8)((RTC_IT_WUT) & RTC_TCR1_TAMPIE);

	/* Wait until WUTWF flag is set */
	while (((RTC->ISR1 & RTC_ISR1_WUTWF) == RESET) ||
		((RTC->ISR1 & RTC_ISR1_ALRAWF) == RESET))	
	{
		;
	}

	/* Configure the Wakeup Timer counter */
	RTC->WUTRH = (uint8)(GET_WAKEUP_COUNTER(20) >> 8);
	RTC->WUTRL = (uint8)(GET_WAKEUP_COUNTER(20));
    RTC->CR2 |= 0x04;
          
	/* Enable the write protection for RTC registers */
	RTC->WPR = 0xff; 



	/* Enable the write protection for RTC registers */
	RTC->WPR = DRV_RTC_LOCK_KEY; 


	return 1;
}


uint DrvRTC_SetConfig(const uint8 *u8p_Value)
{
			m_u16_TickCycle = *((uint16 *)u8p_Value);
			RTC->WPR = DRV_RTC_UNLOCK_KEY1;
			RTC->WPR = DRV_RTC_UNLOCK_KEY2;


				RTC->CR2 &= ~RTC_CR2_WUTE;

				while ((RTC->ISR1 & RTC_ISR1_WUTWF) == RESET)
				{
					;
				}

				RTC->WUTRH = (uint8)(GET_WAKEUP_COUNTER(m_u16_TickCycle) >> 8);
				RTC->WUTRL = (uint8)(GET_WAKEUP_COUNTER(m_u16_TickCycle));
				RTC->CR2 |= RTC_CR2_WUTE;
		
	

			RTC->WPR = DRV_RTC_LOCK_KEY; 
}

void DrvRTC_Interrupt(void)
{
	uint8 u8_ISR2;


	u8_ISR2 = RTC->ISR2;

	if ((u8_ISR2 & (uint8)(RTC_IT_WUT >> 4)) != RESET)
	{
		RTC->ISR2 = ~((uint8)(RTC_IT_WUT >> 4));

			if(tick_num>=1)
				{
				
				if(send_flag==0)
					{
					send_flag=1;
					
					}
				tick_num=0;
				}
			tick_num++;
	}
	
	if ((u8_ISR2 & (uint8)(RTC_IT_ALRA >> 4)) != RESET)
	{
		RTC->ISR2 = ~((uint8)(RTC_IT_ALRA >> 4));


	}
}

void ex(uint16_t *x,uint16_t *y)
{
	uint16_t temp;
	temp=*x;
	*x=*y;
	*y=temp;
}
void sort(uint16_t * x,uint8_t N)
{
	uint8_t i,n;
		for(i=0;i<N;i++)
		{

           
			for(n=0;n<(N-i);n++)
				{
					if(*(x+n)<*(x+n+1));
						else 
							ex(&*(x+n),&*(x+n+1));
						
				}

          
		}
}

void moto_init(void)
{
	uint8_t u8_Value;
	u8_Value=DRV_GPIO_MODE_OUTPUT;
	DrvGPIO_SetConfig(INA1,DRV_GPIO_PARAM_MODE,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(INA2,DRV_GPIO_PARAM_MODE,(const uint8*) &u8_Value);
	u8_Value=1;
	DrvGPIO_SetConfig(INA1,DRV_GPIO_PARAM_PULLUP,(const uint8 *)&u8_Value);
	DrvGPIO_SetConfig(INA2,DRV_GPIO_PARAM_PULLUP,(const uint8*) &u8_Value);
}


void PWM_MOT(uint8_t sp)
{
	pwm_flag=1;
	speed=sp;
}

void Drv_InitializeClock(void)
{
	uint ui_Value;


	/* High speed internal clock prescaler: 1*/
	CLK->CKDIVR = 0x00;

	/* Enable RTC clock */
	CLK->CRTCR = 0x10;

	/* Wait for LSE clock to be ready */
	while ((CLK->ECKCR & CLK_ECKCR_LSERDY) == 0)
	{
		;
	}

	CLK->ICKCR |= CLK_ICKCR_LSION;

	/* Wait for LSI clock to be ready */
	while ((CLK->ICKCR & CLK_ICKCR_LSIRDY) == 0)
	{
		;
	}


}

void Pwm_interupt()
{

		
			if(pwm_flag)
			{
			pwm_num++;
			if(pwm_num<speed)
				{
					mot_drive(Clockwise);
					
				}
			else
				{
					mot_drive(stop);
				}
			}

			if(pwm_num>=10)
				{
					pwm_num=0;
				}
}
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
