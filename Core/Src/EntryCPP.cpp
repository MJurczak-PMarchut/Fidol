/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "CommManager.hpp"
#include "ServoControl.hpp"
#include "RobotSpecificDefines.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;

CommManager CommunicationManager;
ServoControl Servos(&htim2);

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;

void main_cpp(void)
{
	Servos.AttachServo(SERVO00_GPIO_Port, SERVO00_Pin);
	Servos.AttachServo(SERVO01_GPIO_Port, SERVO01_Pin);
	Servos.AttachServo(SERVO02_GPIO_Port, SERVO02_Pin);
	Servos.AttachServo(SERVO03_GPIO_Port, SERVO03_Pin);
	Servos.AttachServo(SERVO04_GPIO_Port, SERVO04_Pin);
	Servos.AttachServo(SERVO05_GPIO_Port, SERVO05_Pin);
	Servos.AttachServo(SERVO06_GPIO_Port, SERVO06_Pin);
	Servos.AttachServo(SERVO07_GPIO_Port, SERVO07_Pin);
	Servos.AttachServo(SERVO08_GPIO_Port, SERVO08_Pin);
	Servos.AttachServo(SERVO09_GPIO_Port, SERVO09_Pin);
	Servos.AttachServo(SERVO10_GPIO_Port, SERVO10_Pin);
	Servos.AttachServo(SERVO11_GPIO_Port, SERVO11_Pin);
	Servos.AttachServo(SERVO12_GPIO_Port, SERVO12_Pin);
	Servos.AttachServo(SERVO13_GPIO_Port, SERVO13_Pin);
	Servos.AttachServo(SERVO14_GPIO_Port, SERVO14_Pin);//
	Servos.AttachServo(SERVO15_GPIO_Port, SERVO15_Pin);
	Servos.AttachServo(SERVO16_GPIO_Port, SERVO16_Pin);
	Servos.AttachServo(SERVO17_GPIO_Port, SERVO17_Pin);
	Servos.StartServos();
	while(1)
	{
		for(uint16_t u16Angle = 500; u16Angle < 1500; u16Angle += 40)
		{
			for(uint8_t u8Servo_No = 0; u8Servo_No < Servos.GetNoOfAttchedServos(); u8Servo_No++)
			{
				Servos.SetServoValue(u8Servo_No, u16Angle);
			}
			HAL_Delay(20);
		}
		for(uint16_t u16Angle = 1500; u16Angle > 500; u16Angle -= 100)
		{
			for(uint8_t u8Servo_No = 0; u8Servo_No < Servos.GetNoOfAttchedServos(); u8Servo_No++)
			{
				Servos.SetServoValue(u8Servo_No, u16Angle);
			}
			HAL_Delay(20);
		}
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		Servos.ServoControlCBHalfPulse();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		Servos.ServoControlCBUpdate();
	}
}

#ifdef __cplusplus
}
#endif
