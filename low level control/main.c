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




























/* void PIGSys_Init(void)  //ϵͳ��������ĳ�ʼ������
{
  delay_init();         //������ʱ����
  //USART1_Config();      //��ʼ������
  Motor_12_Config();    //���������ʼ��
  key_init();           //����������ʼ��
} */


/*
void key_pros()          //�������Ƶ��ʵ����ת����ת��ֹͣ
{
  if(k_up==1)	         //�жϰ�����k_up�Ƿ���
  {
    delay_ms(10);        //��������
    if(k_up==1)	         //�ٴ��жϰ���k_up�Ƿ���
    {
      Motor_1_PRun();	 	 //ִ�е��1˳ʱ��
    }
    while(k_up);         //�ȴ������ɿ�
  }
  if(k_down==0)          //�жϰ���k_down�Ƿ���
  {
    delay_ms(10);        //����
    if(k_down==0)        //�ٴ��ж��Ƿ���
    {
      Motor_1_NRun();		 //ִ�е��1��ʱ��
    }
    while(!k_down); 	   //�ȴ��ɿ�
  }
  if(k_left==0)          //�жϰ���k_left�Ƿ���
  {
    delay_ms(10);        //����
    if(k_left==0)        //�ٴ��ж��Ƿ���
    {
      Motor_1_STOP();		 //ִ�е��1ֹͣ
    }
    while(!k_left);      //�ȴ��ɿ�
  }
}*/

