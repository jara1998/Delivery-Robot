#include "L298N.H" 
#include "stm32f4xx.h"  
#include "stm32f4xx_rcc.h"  
#include "stm32f4xx_gpio.h"

void Motor_12_Config(void)  //����LED�����ŵĳ�ʼ������
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12; //����LED������
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);  // initialize GPIOD pin
  
  GPIO_ResetBits(GPIOD,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12); //������������
}

void Motor_1_STOP(void) //�����һ�������ֹͣ����
{
  IN1(High);  //����ͷ�ļ��еĺ궨�壬��������������������ʾֹͣ
  IN2(High);
}

void Motor_1_CW(void)  //����˳ʱ��ת��
{
  IN1(Low);
  IN2(High);
}

void Motor_1_CCW(void)  //������ʱ��ת��
{
  IN1(High);
  IN2(Low);
}

void Motor_2_STOP(void)  //�������Ƶڶ�̨���
{
  IN3(High);
  IN4(High);
}

void Motor_2_CW(void)
{
  IN3(Low);
  IN4(High);
}

void Motor_2_CCW(void)
{
  IN3(High);
  IN4(Low);
}


void forward() {
  Motor_1_CW();
  Motor_2_CW();
}

void back() {
  Motor_1_CCW();
  Motor_2_CCW();
}

void turn_left() {
  Motor_1_STOP();
  Motor_2_CW();
}

void turn_right() {
  Motor_1_CW();
  Motor_2_STOP();
}

void stop() {
  Motor_1_STOP();
  Motor_2_STOP();
}
