/**
  ******************************************************************************
  * @file    Programm.h
  * @author  jingfan.li
  * @version V1.0.0
  * @date    08/2017
  * @brief   delay functions header
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

#ifndef __PROGRAMM_H
#define __PROGRAMM_H

#include "stm8l15x.h"
#include "stm8l_discovery_lcd.h"
#include "discover_board.h"
#include "icc_measure.h"
#include "discover_functions.h"
#include "stm8l15x_usart.h"
#include "stm8l15x_syscfg.h"
#include "drv_gpio.h"
#include "global.h"

#define baurte 9600;
#define adress 1;


#define DRV_RTC_UNLOCK_KEY1			0xCA
#define DRV_RTC_UNLOCK_KEY2			0x53
#define DRV_RTC_LOCK_KEY			0xFF
#define GET_WAKEUP_COUNTER(cycle)	(((uint32)((cycle) * (32768 / 16) / 1000)) - 1)
#define DRV_RTC_TICK_CYCLE_DEFAULT	10

#define testcard_present                    (DRV_GPIO_PORTA| DRV_GPIO_PIN3)
#define locateA                             (DRV_GPIO_PORTF | DRV_GPIO_PIN3)
#define locateB								(DRV_GPIO_PORTF | DRV_GPIO_PIN2)
#define INA1                                (DRV_GPIO_PORTF | DRV_GPIO_PIN5)
#define INA2								(DRV_GPIO_PORTF | DRV_GPIO_PIN6)
#define LED1								(DRV_GPIO_PORTF | DRV_GPIO_PIN0)
#define LED2								(DRV_GPIO_PORTF | DRV_GPIO_PIN1)
#define Button_start						(DRV_GPIO_PORTE | DRV_GPIO_PIN6)
#define Distance_sensor						(DRV_GPIO_PORTB| DRV_GPIO_PIN3)	
#define LED_SENSOR1							(DRV_GPIO_PORTA| DRV_GPIO_PIN4)	
#define LED_SENSOR2							(DRV_GPIO_PORTA| DRV_GPIO_PIN5)
#define PD5                                  (DRV_GPIO_PORTD| DRV_GPIO_PIN5)
#define PD4									(DRV_GPIO_PORTD| DRV_GPIO_PIN4)


typedef enum
{
	DRV_RTC_FLAG_WAKEUP_ENABLE = 0,
	DRV_RTC_FLAG_ALARM_ENABLE,
	DRV_RTC_FLAG_TIME_FORMAT,
	DRV_RTC_COUNT_FLAG
} drv_rtc_flag;

typedef enum
{
	Clockwise,
	Reverse,
	stop
	
}motor_flag;

typedef enum
{
  FUNCTION_FAIL=0,
  FUNCTION_SUCCESS
}FUNCTION_flag;




void lcd_config();
void adc_calc();
void usart_config();
void uart1_sendbyte(uint8_t data);
void uart2_sendbyte(uint8_t data);
void uart1_senddata(uint8_t *x,uint8_t n);
void TIM2_ETR_config();
void TIM2_PULSE_CLAC(uint16_t *n);
void TIM3_PWMCONFIG(void);
void uart1_recive(uint8_t *rebuf,uint8_t *renum);
int mot_drive(motor_flag state);
void ledvalsend(void);
unsigned int HtoD_4(unsigned int adata);
unsigned char chartohex(unsigned char hexdata);
void Button_init(void);
void ready_init(void);
void locate_init(void);
void Distance_Sensor(void);
void adgpio_init(void);
void open_led1(void);
void open_led2(void);
uint DrvRTC_Initialize(void);
uint16_t adc_getval(ADC_TypeDef* ADCx,ADC_Channel_TypeDef ADC_Channels);

void gpio_init(void);

void sort(uint16_t * x,uint8_t N);
uint DrvRTC_SetConfig(const uint8 *u8p_Value);
void DrvRTC_Interrupt(void);
uint DrvRTC_Initialize(void);
void PWM_MOT(uint8_t sp);
void Drv_InitializeClock(void);
void Pwm_interupt(void);










#endif


/**


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
