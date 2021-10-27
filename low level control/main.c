#include "stm32f4xx_exti.h"
#include "stm32f4xx.h"     
#include "stm32f4xx_usart.h"  
#include "L298N.H"        
#include "stm32f4_discovery.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include <stdio.h>
#include "tm_stm32f4_i2c.h"


void Configure_PE4(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    // EXTI_InitTypeDef EXTI_InitStruct;
    // NVIC_InitTypeDef NVIC_InitStruct;
    
    /* Enable clock for GPIOB*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void Configure_PE5(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    // EXTI_InitTypeDef EXTI_InitStruct;
    // NVIC_InitTypeDef NVIC_InitStruct;
    
    /* Enable clock for GPIOB*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStruct);
}


void Configure_PE6(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    // EXTI_InitTypeDef EXTI_InitStruct;
    // NVIC_InitTypeDef NVIC_InitStruct;
    
    /* Enable clock for GPIOE*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStruct);
}




void Configure_PD15 (void) {
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);  
  
  // GPIOD Configuration  
  GPIO_InitTypeDef GPIO_InitStruct;  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_Init(GPIOD, &GPIO_InitStruct);
}


void Configure_PA1(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
  // GPIOD Configuration  
  GPIO_InitTypeDef GPIO_InitStruct;  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;  
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}



void Configure_PA2(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
  // GPIOD Configuration  
  GPIO_InitTypeDef GPIO_InitStruct;  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;  
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;  
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}



void Configure_PA3(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
  // GPIOD Configuration  
  GPIO_InitTypeDef GPIO_InitStruct;  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;  
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;  
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void delay_us(u32 time) {
  u32 i=0;
  while(time--) {
    i=9;  
    while(i--) ;
  }
}

void basic_control(void) {
  GPIO_ResetBits(GPIOD, GPIO_Pin_15);
  // main control logic using polling
  if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)) {
    delay_us(10000);
    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)) {
      forward();
      
      GPIO_ResetBits(GPIOA, GPIO_Pin_1);
      GPIO_ResetBits(GPIOA, GPIO_Pin_2);
      GPIO_SetBits(GPIOA, GPIO_Pin_3);
      //printf("forward: %d\n", GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4));
    }      
  } 
  else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) {
    delay_us(10000);
    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) {
      turn_left();
      
      GPIO_SetBits(GPIOA, GPIO_Pin_1);
      GPIO_ResetBits(GPIOA, GPIO_Pin_2);
      GPIO_ResetBits(GPIOA, GPIO_Pin_3);
      //printf("left: %d\n", GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5));
    }      
  } 
  
  else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) {
    delay_us(10000);
    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) {
      turn_right();
      
      GPIO_SetBits(GPIOA, GPIO_Pin_1);
      GPIO_ResetBits(GPIOA, GPIO_Pin_2);
      GPIO_ResetBits(GPIOA, GPIO_Pin_3);
      //printf("right: %d\n", GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6));
    }      
  }
  
  else {
    stop();
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    GPIO_SetBits(GPIOA, GPIO_Pin_2);
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    //printf("stop\n");
  }GPIO_ResetBits(GPIOD, GPIO_Pin_15);
  // main control logic using polling
  if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)) {
    delay_us(10000);
    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)) {
      forward();
      
      GPIO_ResetBits(GPIOA, GPIO_Pin_1);
      GPIO_ResetBits(GPIOA, GPIO_Pin_2);
      GPIO_SetBits(GPIOA, GPIO_Pin_3);
      //printf("forward: %d\n", GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4));
    }      
  } 
  else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) {
    delay_us(10000);
    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) {
      turn_left();
      GPIO_SetBits(GPIOA, GPIO_Pin_1);
      GPIO_ResetBits(GPIOA, GPIO_Pin_2);
      GPIO_ResetBits(GPIOA, GPIO_Pin_3);
      //printf("left: %d\n", GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5));
    }      
  } 
  
  else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) {
    delay_us(10000);
    if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) {
      turn_right();
      GPIO_SetBits(GPIOA, GPIO_Pin_1);
      GPIO_ResetBits(GPIOA, GPIO_Pin_2);
      GPIO_ResetBits(GPIOA, GPIO_Pin_3);
      //printf("right: %d\n", GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6));
    }      
  }
  
  else {
    stop();
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    GPIO_SetBits(GPIOA, GPIO_Pin_2);
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
    //printf("stop\n");
  }
}


void init(void) {
  Motor_12_Config(); 
  Configure_PE4();
  Configure_PE5();
  Configure_PE6();
  Configure_PD15();
  Configure_PA1();
  Configure_PA2();
  Configure_PA3();
}





int main(void)    
{
  init();
  while(1) { 
    basic_control();
  }
}




























/* void PIGSys_Init(void)  //系统基本外设的初始化函数
{
  delay_init();         //调用延时函数
  //USART1_Config();      //初始化串口
  Motor_12_Config();    //电机驱动初始化
  key_init();           //按键驱动初始化
} */


/*
void key_pros()          //按键控制电机实现正转；反转；停止
{
  if(k_up==1)	         //判断按键上k_up是否按下
  {
    delay_ms(10);        //消抖处理
    if(k_up==1)	         //再次判断按键k_up是否按下
    {
      Motor_1_PRun();	 	 //执行电机1顺时针
    }
    while(k_up);         //等待按键松开
  }
  if(k_down==0)          //判断按键k_down是否按下
  {
    delay_ms(10);        //消抖
    if(k_down==0)        //再次判断是否按下
    {
      Motor_1_NRun();		 //执行电机1逆时针
    }
    while(!k_down); 	   //等待松开
  }
  if(k_left==0)          //判断按键k_left是否按下
  {
    delay_ms(10);        //消抖
    if(k_left==0)        //再次判断是否按下
    {
      Motor_1_STOP();		 //执行电机1停止
    }
    while(!k_left);      //等待松开
  }
}*/

