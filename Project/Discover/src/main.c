/**
  ******************************************************************************
  * @file    main.c
  * @author jingfanli 
  * @version V0.9
  * @date    07/2017
  * @brief   Main program body
  ******************************************************************************
  * @copy

 
/* Includes ------------------------------------------------------------------*/

#include "Programm.h"
void main(void)
{
    uint8_t i;
 
	
	uint16_t num1=0;
	i=0;
        uint16 a=0;
        uint16 b=0;
	
	
	Drv_InitializeClock();
	gpio_init();
	usart_config();
	
	//TIM2_ETR_config();
	//GPIO_SetBits(GPIOG,GPIO_Pin_6);
	Button_init();
	ready_init();
	moto_init();
	Distance_Sensor();
	adgpio_init();
	DrvRTC_Initialize();
	CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
	TIM4_TimeBaseInit(TIM4_Prescaler_16, 10);
   /* Clear TIM4 update flag */
   	TIM4_ClearFlag(TIM4_FLAG_Update);
   /* Enable update interrupt */
   	TIM4_ITConfig(TIM4_IT_Update, ENABLE);
    

	enableInterrupts();
	TIM4_Cmd(ENABLE);

	//PWM_MOT(4);
	while(1)
		{


				
		//ledvalsend();	
                //  mot_drive(Clockwise);
		TestHA1c_process();
              //ledvalsend();
			

		}
}





/******************* *********/
