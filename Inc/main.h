/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Temp_Pin GPIO_PIN_1
#define Temp_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_2
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_3
#define A2_GPIO_Port GPIOC
#define PA1_and_PA2_for_Tacho_Pin GPIO_PIN_1
#define PA1_and_PA2_for_Tacho_GPIO_Port GPIOA
#define PA1_and_PA2_for_Tacho_EXTI_IRQn EXTI1_IRQn
#define PA1_and_PA2_for_TachoA2_Pin GPIO_PIN_2
#define PA1_and_PA2_for_TachoA2_GPIO_Port GPIOA
#define PA1_and_PA2_for_TachoA2_EXTI_IRQn EXTI2_IRQn
#define SENSE_CENTER_Pin GPIO_PIN_4
#define SENSE_CENTER_GPIO_Port GPIOA
#define CURRENT_SENSE_Pin GPIO_PIN_5
#define CURRENT_SENSE_GPIO_Port GPIOA
#define U_BAT_SENSE_Pin GPIO_PIN_6
#define U_BAT_SENSE_GPIO_Port GPIOA
#define StarterPWM_Pin GPIO_PIN_7
#define StarterPWM_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_4
#define A3_GPIO_Port GPIOC
#define A4_Pin GPIO_PIN_5
#define A4_GPIO_Port GPIOC
#define InjectorPWM_Pin GPIO_PIN_0
#define InjectorPWM_GPIO_Port GPIOB
#define CoolerPWM_Pin GPIO_PIN_1
#define CoolerPWM_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOC
#define AH_Pin GPIO_PIN_8
#define AH_GPIO_Port GPIOA
#define BH_Pin GPIO_PIN_9
#define BH_GPIO_Port GPIOA
#define CH_Pin GPIO_PIN_10
#define CH_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
