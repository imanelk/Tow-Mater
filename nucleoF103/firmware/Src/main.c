
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "power.h"
#include "control.h"
#include "calibrate.h"
#include "timer.h"
#include <FLASH_PAGE_F1.h>

/* USER CODE BEGIN Includes */

/* Modes
 * 0- Calibration
 * 1- Control
 */
int mode = 1;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int UPDATE_CMD_FLAG = 1;
int US_FLAG = 1;

int CAN_SEND_MOTORS = 1;
int CAN_SEND_US = 0;
int CAN_SEND_BATT = 1;

/* Tous ADC sur 12 bits pleine echelle 3.3V
 ADCBUF[0] mesure batterie
 ADCBUF[1] angle direction
 ADCBUF[2] I moteur arriere gauche
 ADCBUF[3] I moteur arriere droit
 ADCBUF[4] I moteur avant
 */
uint32_t ADCBUF[5];

uint32_t VMG_mes = 0, VMD_mes = 0, per_vitesseG = 0, per_vitesseD = 0;
int nbImpulsionG = 0;
int nbImpulsionD = 0;

int sendTrig = 1;
int usEchoStart = 1;
int usEchoReceived = 0;
uint64_t usEchoRisingTime = 0;
uint64_t usEchoDuration=0;
int currentUs = 0;
int cnt = 0; 	//Count the US measurement time (timeout)

uint16_t usTriggerPin[6] = {US_Front_Left_Trig_Pin,US_Front_Center_Trig_Pin,US_Front_Right_Trig_Pin,US_Rear_Left_Trig_Pin,US_Rear_Center_Trig_Pin,US_Rear_Right_Trig_Pin};

/*usDistance[] : Ultrasonic measurements [cm]
 * usDistance[0] front left
 * usDistance[1] front center
 * usDistance[2] front right
 * usDistance[3] rear left
 * usDistance[4] rear center
 * usDistance[5] rear right
 */
uint16_t usDistance[6] = {0,0,0,0,0,0};

CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

//Speed cmd
int leftRearSpeed = -1;
int rightRearSpeed = -1;
int steeringSpeed = -1;

//Communication checking request
int commCheckingRequest = 0;

extern CAN_HandleTypeDef hcan;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance==TIM2)
    {
        VMG_mes = 0;

    } else if (htim->Instance==TIM4){
        VMD_mes = 0;

    }else if (htim->Instance==TIM3){

    	microSecondTimerOverflow();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

    /***               Mesures des vitesses moteurs                      ***
     * F�quences entr�es micro, sorties capteurs, entre environ 2hz � 80 hz *
     * Timer 2,4 sur 16 bits (65535)cp ->compte p�riode 9999 pour 1s (1Hz)   *
     *                               ->compte p�riode 1000 pour 0.1s (10Hz)  *
     * Rapport r�duction 2279/64 ~ 36 impulsions/tour de roue                *
     * unite de 0.01*tr/mn = 168495/ cp                                     *
     */
    if (htim->Instance==TIM2)
    {
        per_vitesseG =	HAL_TIM_ReadCapturedValue (&htim2,TIM_CHANNEL_3);//PB10
        VMG_mes = 1684949/per_vitesseG ;// X 0.01 tr/mn
        
        __HAL_TIM_SET_COUNTER(&htim2,0);// mise a zero compteur apres capture
        nbImpulsionG +=1;
    }
    if (htim->Instance==TIM4)
    {
        per_vitesseD =	HAL_TIM_ReadCapturedValue (&htim4,TIM_CHANNEL_3);//PB8
        VMD_mes = 1684949/per_vitesseD ;// X 0.01 tr/mn
        
        __HAL_TIM_SET_COUNTER(&htim4,0);// mise a zero compteur apres capture
        nbImpulsionD +=1;
    }
}

// EXTI External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(usEchoStart){ //check pin state : rising
		usEchoRisingTime = microSecond();
		usEchoStart = 0;

	}else{
		uint64_t echoDuration = microSecond() - usEchoRisingTime;

		if (echoDuration >= 0)
			usEchoDuration = echoDuration;
		else
			usEchoDuration=-1;

		usEchoStart = 1;
		usEchoReceived = 1;

	}

}

void SYS_MicroDelay(uint32_t delay)
{
    volatile uint32_t cnt=(delay*6)+5;

    while (cnt >0) {
        cnt--;
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    hcan.pTxMsg = &TxMessage;
    hcan.pRxMsg = &RxMessage;
    /* USER CODE END 1 */
    
    /* MCU Configuration----------------------------------------------------------*/
    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    
    /* USER CODE BEGIN Init */
    
    /* USER CODE END Init */
    
    /* Configure the system clock */
    SystemClock_Config();
    
    /* USER CODE BEGIN SysInit */
    
    /* USER CODE END SysInit */
    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    power_boostrap();
    
    MX_DMA_Init();	// Set a breakpoint here for DEBUG : turn ON the car at this point before continuing !!
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_ADC1_Init();
    MX_CAN_Init();
    /* USER CODE BEGIN 2 */
    
    /* Initialisations */
    
    /* PWM MOTEURS */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    //Sorties complementaires
    HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Start(&htim1,TIM_CHANNEL_3);
    /*Vitesse*/
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

    HAL_TIM_IC_Start_IT (&htim2,TIM_CHANNEL_3);//autorisation IT capture CH3
    HAL_TIM_IC_Start_IT (&htim4,TIM_CHANNEL_3);//autorisation IT capture CH3
    
    /* ADC1 */
    HAL_ADC_Start_DMA (&hadc1,ADCBUF,5);
    
    //US timer
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    startMicroSecondCounter();



    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    
    /* Steering Initialization*/

    //Write default calibration values in flash memory (first use only)
    if ((int)Flash_Read_NUM(STEERING_CALIBRATION_A_DEFAULT_ADDR)!=(int)STEERING_CALIBRATION_A_DEFAULT
    	|| (int)Flash_Read_NUM(STEERING_CALIBRATION_B_DEFAULT_ADDR)!=(int)STEERING_CALIBRATION_B_DEFAULT)
		{

    	Flash_Write_NUM(STEERING_CALIBRATION_A_DEFAULT_ADDR, STEERING_CALIBRATION_A_DEFAULT);
    	Flash_Write_NUM(STEERING_CALIBRATION_B_DEFAULT_ADDR, STEERING_CALIBRATION_B_DEFAULT);

    	Flash_Write_NUM(STEERING_CALIBRATION_A_ADDR, STEERING_CALIBRATION_A_DEFAULT);
    	Flash_Write_NUM(STEERING_CALIBRATION_B_ADDR, STEERING_CALIBRATION_B_DEFAULT);
    }
    

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    	if (US_FLAG==1)
		{
			US_FLAG=0;

			if (currentUs >= 6){
				currentUs = 0;
			}

			if (sendTrig){
				usEchoStart = 1;

				HAL_GPIO_WritePin( US_GPIO_Port, usTriggerPin[currentUs], GPIO_PIN_SET); //Trigger ON
				SYS_MicroDelay(10);
				HAL_GPIO_WritePin( US_GPIO_Port, usTriggerPin[currentUs], GPIO_PIN_RESET); //Trigger OFF
				sendTrig = 0;

			} else {
				cnt += PERIOD_UPDATE_US;

				if (usEchoReceived){	//If we received the echo

					usDistance[currentUs] = usEchoDuration/58;

					usEchoReceived = 0;
					cnt = 0;
					currentUs+=1;
					sendTrig = 1;

				}else if (cnt >= TIMEOUT_US){ //If the echo is not received (i.e. sensor failure)

					usDistance[currentUs] = 1000;	//Set distance value out of range

					usEchoReceived = 0;
					cnt = 0;
					currentUs+=1;
					sendTrig = 1;
				}
			}


			if (currentUs == 6){	//When all the us sensors have been performed, restart with first us sensor (0) and send data to can
				currentUs = 0;
				CAN_SEND_US = 1;
			}
		}

        /* Update motors command*/
        if (UPDATE_CMD_FLAG){
            UPDATE_CMD_FLAG = 0;
            
			if (mode == 0){	//Calibration Mode
            	calibrate();
            	mode = 1;
			}
			else{	//Control Mode
            	car_control(leftRearSpeed,rightRearSpeed, steeringSpeed);
			}
        }
        
        /* CAN : Sending data*/
        if (CAN_SEND_MOTORS){

        	//Motors data
        	CAN_SEND_MOTORS = 0;

        	//Number of sensor pulses since last message (left rear wheel and right rear wheel)
        	data[0] = nbImpulsionG;
        	data[1] = nbImpulsionD;
        	nbImpulsionG = 0;
        	nbImpulsionD = 0;

            data[2] = (VMG_mes >> 8) & 0xFF; // Left Rear Speed MSB
            data[3] = VMG_mes & 0xFF; 	//LSB
            
            data[4] = (VMD_mes >> 8) & 0xFF; // Right Rear Speed MSB
            data[5] = VMD_mes & 0xFF; // LSB
            
            data[6] = steering_get_angle() & 0xFF;	//Steering Angle MSB
            
            CAN_Send(data, CAN_ID_MOTORS_DATA);

         }
         if (CAN_SEND_BATT){
        	//Battery Level
			CAN_SEND_BATT= 0;

            data[0] = (ADCBUF[0] >> 8) & 0xFF; // Vbatt MSB
            data[1] = ADCBUF[0] & 0xFF; 	//LSB

			CAN_Send(data, CAN_ID_BATT_LEVEL);

		 }

         if (CAN_SEND_US){
        	//Sending US1 data (front)
			data[0] = (usDistance[0] >> 8) & 0xFF;	//US Front Left
			data[1] = usDistance[0] & 0xFF;

			data[2] = (usDistance[1] >> 8) & 0xFF;	//US Front Center
			data[3] = usDistance[1] & 0xFF;

			data[4] = (usDistance[2] >> 8) & 0xFF;	//US Front Right
			data[5] = usDistance[2] & 0xFF;

			CAN_Send(data, CAN_ID_US1);

			//Sending US2 data (rear)
			data[0] = (usDistance[3] >> 8) & 0xFF;	//US Rear Left
			data[1] = usDistance[3] & 0xFF;

			data[2] = (usDistance[4] >> 8) & 0xFF;	//US Rear Center
			data[3] = usDistance[4] & 0xFF;

			data[4] = (usDistance[5] >> 8) & 0xFF;	//US Rear Right
			data[5] = usDistance[5] & 0xFF;

			CAN_Send(data, CAN_ID_US2);

			CAN_SEND_US = 0;
        }


        if (commCheckingRequest){

			data[1] = COMM_CHECKING_ACK;

			CAN_Send(data,CAN_ID_COMM_CHECKING); //Send ack

			commCheckingRequest = 0;
        }
        
    }
    /* USER CODE END 3 */
    
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;
    
    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
    
    /**Configure the Systick interrupt time
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    
    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 15U);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    /*while(1)
     {
     }*/
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
