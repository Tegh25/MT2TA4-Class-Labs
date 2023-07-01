/**
  ******************************************************************************
  * @file    TIM/TIM_TimeBase/Src/stm32f4xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   HAL MSP module.    
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */

/*
void HAL_TIM_Base_MspInit (TIM_HandleTypeDef *htim)
{
  //Enable peripherals and GPIO Clocks 
 
__HAL_RCC_TIM3_CLK_ENABLE(); //this is defined in stm32f4xx_hal_rcc.h
	
	
  //Configure the NVIC for TIMx 
	HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  
  // Enable the TIM global Interrupt 
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

*/

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{ 
  //Enable peripherals and GPIO Clocks
 __HAL_RCC_TIM4_CLK_ENABLE();
    
  //Configure the NVIC for TIMx 
	HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
  
  // Enable the TIM global Interrupt 
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}





void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	//		as for which GPIO pin has to be used for TIM3 alternative functions, refer to the use manualf for Discovery board---UM1472 User Manual , stm32f4discovery
	//		Major pins for TIM3 AF: PA6_TIM3_CH1, PA7--TIM3_CH2(also as TIM14_CH1),  PB0--TIM3_CH3, PB1--TIM3_CH4, PB4--TIM3_CH1,PB5--TIM3_CH2.  PC6--TIM3_CH1,
	//		PC7--TIM3_CH2, PC8--TIM3_CH3, PC9--TIM3_CH4, PD2--TIM3_ETR
		
	GPIO_InitTypeDef   GPIO_InitStruct;
  
  //- Enable peripherals and GPIO Clocks 
  __HAL_RCC_TIM3_CLK_ENABLE();
    
	
	
  // Enable GPIO Port Clocks 
  __HAL_RCC_GPIOA_CLK_ENABLE(); //for PinA7  and PinA6for Channel 2
  
    
  // Common configuration for all channels 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH; //???? 50MHz, 25Mhz...don't know what is the critics to select pin speed.
  
	
		//for Channel 1
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;//as for which AF, see table "alternate function mapping" in datasheet
																					//for stm32f429i-disco, see table 12 on page 73 (for pin PA7) in document
																					//DocID024030 Rev 7--datasheet for stm32f427xx--stm32f429xx
	
	//all the four channels of TIM3's alternate function are AF2.  
	
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
		 
	
	
	

	//the AF2 of TIM3-CH3 and TIM3-CH4 are mapped to PB0 and PB1
	
	 // Enable GPIO Port Clocks 
/*  __HAL_RCC_GPIOB_CLK_ENABLE(); //for PinB0 and PB1  and PinA6for Channel 2
	
	GPIO_InitStruct.Pin = GPIO_PIN_0;    //TIM3 AF2 is for TIM3's CH1, CH2 and CH3 and CH4
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_1;    //TIM3 AF2 is for TIM3's CH1, CH2 and CH3 and CH4
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
*/


	
	//the AF2 of TIM3-CH3 and TIM3-CH4 are also mapped to PC9 and PC9
	
	 // Enable GPIO Port Clocks 
  __HAL_RCC_GPIOC_CLK_ENABLE(); //for PinB0 and PB1  and PinA6for Channel 2
	
	GPIO_InitStruct.Pin = GPIO_PIN_8;    //TIM3 AF2 is for TIM3's CH1, CH2 and CH3 and CH4
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_9;    //TIM3 AF2 is for TIM3's CH1, CH2 and CH3 and CH4
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	

}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clock */
  ADCx_CHANNEL_GPIO_CLK_ENABLE();
  /* ADC3 Periph clock enable */
  ADCx_CLK_ENABLE();
  /* Enable DMA2 clock */
  //DMAx_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/ 
  /* ADC3 Channel8 GPIO pin configuration */
  GPIO_InitStruct.Pin = ADCx_CHANNEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);   
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}
  
/**
  * @brief ADC MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  static DMA_HandleTypeDef  hdma_adc;
  
  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC3 Channel8 GPIO pin */
  HAL_GPIO_DeInit(ADCx_CHANNEL_GPIO_PORT, ADCx_CHANNEL_PIN);
  
  /*##-3- Disable the DMA Streams ############################################*/
  /* De-Initialize the DMA Stream associate to transmission process */
  HAL_DMA_DeInit(&hdma_adc); 
    
  /*##-4- Disable the NVIC for interrupt ###########################################*/
  HAL_NVIC_DisableIRQ(ADC_IRQn);
  
}

 
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/