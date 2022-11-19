/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
#include "hook.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#define CAN_ID_MOTORS_DATA	0x200	// Odometry, Motors speed, Steering angle
#define CAN_ID_MOTORS_CMD 	0x100	// Speed and Steering Commands
#define CAN_ID_HOOK	0x111
#define CAN_ID_US1	0x211			//Front US [cm]
#define CAN_ID_US2	0x221			//Rear US [cm]
#define CAN_ID_BATT_LEVEL	0x273	//Battery Level
#define CAN_ID_CALIBRATION_MODE	0x400
#define CAN_ID_COMM_CHECKING 0x410


#define CALIBRATION_REQUEST	0x1		//frame[0]
#define CALIBRATION_IN_PROGRESS 0x2	//frame[0]
#define CALIBRATION_SUCCESS 0x3		//frame[0]
#define CALIBRATION_FAIL 0x4		//frame[0]

#define CALIBRATION_USER_NEED 0x1	//frame[1]

#define COMM_CHECKING_REQUEST 0x1 //frame[0]
#define COMM_CHECKING_ACK 0x1	  //frame[1]

#define HOOK_UNLOCK 0x0
#define HOOK_LOCK 0x1
#define HOOK_FDC 0x2

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_FilterConfig(void);
void CAN_Send(uint8_t* data, uint32_t id);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
