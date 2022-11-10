/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LatchPower_Pin GPIO_PIN_6
#define LatchPower_GPIO_Port GPIOA

#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB



/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

	//L/R buttons
#define RightSteeringButton_Pin GPIO_PIN_14
#define RightSteeringButton_GPIO_Port GPIOB
#define LeftSteeringButton_Pin GPIO_PIN_15
#define LeftSteeringButton_GPIO_Port GPIOB

	//US
#define US_Front_Left_Trig_Pin GPIO_PIN_1
#define US_Front_Center_Trig_Pin GPIO_PIN_2
#define US_Front_Right_Trig_Pin GPIO_PIN_3

#define US_Rear_Left_Trig_Pin GPIO_PIN_4
#define US_Rear_Center_Trig_Pin GPIO_PIN_5
#define US_Rear_Right_Trig_Pin GPIO_PIN_7

#define US_Right_Echo_Pin GPIO_PIN_6
#define US_Left_Echo_Pin GPIO_PIN_8
#define US_Center_Echo_Pin GPIO_PIN_9

#define US_GPIO_Port GPIOC

	//Hook
#define GPIO_PORT_HOOK GPIOB
#define GPIO_PIN_HOOK GPIO_PIN_5



#define PERIOD_UPDATE_US 5 //Period in ms to update us data
#define TIMEOUT_US 50 //Timeout for US measurements [ms]

	//CAN
#define PERIOD_SEND_MOTORS 5 // Period in ms to send motors data
#define PERIOD_SEND_BATT 2000 // Period in ms to send battery level
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
