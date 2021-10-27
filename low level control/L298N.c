#include "L298N.H" 
#include "stm32f4xx.h"  
#include "stm32f4xx_rcc.h"  
#include "stm32f4xx_gpio.h"

void Motor_12_Config(void)  //定义LED的引脚的初始化函数
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12; //定义LED的引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);  // initialize GPIOD pin
  
  GPIO_ResetBits(GPIOD,GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12); //所有引脚拉低
}

void Motor_1_STOP(void) //定义第一个电机的停止函数
{
  IN1(High);  //调用头文件中的宏定义，两个控制引脚输出高则表示停止
  IN2(High);
}

void Motor_1_CW(void)  //控制顺时针转动
{
  IN1(Low);
  IN2(High);
}

void Motor_1_CCW(void)  //控制逆时针转动
{
  IN1(High);
  IN2(Low);
}

void Motor_2_STOP(void)  //再来控制第二台电机
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
