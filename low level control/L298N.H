#ifndef __L298N_H
#define __L298N_H
 
 
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#define High    1  
#define Low     0
 
#define IN1(a) if (a)  \
	GPIO_SetBits(GPIOD,GPIO_Pin_9);\
else  \
	GPIO_ResetBits(GPIOD,GPIO_Pin_9)
 
#define IN2(a) if (a)  \
	GPIO_SetBits(GPIOD,GPIO_Pin_10);\
else  \
	GPIO_ResetBits(GPIOD,GPIO_Pin_10)
 
#define IN3(a) if (a)  \
	GPIO_SetBits(GPIOD,GPIO_Pin_11);\
else  \
	GPIO_ResetBits(GPIOD,GPIO_Pin_11)
 
#define IN4(a) if (a)  \
	GPIO_SetBits(GPIOD,GPIO_Pin_12);\
else  \
	GPIO_ResetBits(GPIOD,GPIO_Pin_12)
 
void Motor_12_Config(void);  
 
void Motor_1_STOP(void);
void Motor_1_CW(void);
void Motor_1_CCW(void);
 
void Motor_2_STOP(void);
void Motor_2_CW(void);
void Motor_2_CCW(void);

void stop(void);
void forward(void);
void back(void);
void turn_left(void);
void turn_right(void);
 
#endif