/****** 

1. For GPIO pins, Both OD mode and PP mode can drive the motor! However, some pins cannot output  high in OD mode!!! 
   
2. The signals do not need to be inverted before being fed in H-bridge.   
*/


#include "main.h"


#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);

static void ExtBtn_Config();
static void SystemClock_Config(void);
static void Error_Handler(void);

/*******************************************Start of New Code*************************/

TIM_HandleTypeDef    Tim3_Handle;
uint16_t Tim3_PrescalerValue;
TIM_OC_InitTypeDef Tim3_OCInitStructure;

__IO uint32_t Tim3_CCR = 4083; // the pulse of the TIM3
//Tim3_CCR=36750;       //  with clock counter freq as 500,000, this will make OC Freq as 1ms.

int FullS=0; //For full step state
int HalfS=0; //For half step state
int mode = 0;//0 means full step, 1 is half step
int CW_Or_CCW = 0;//0=>CCW and 1=> CW

void TIM3_Config();


void O1Config(void){ //configuring output pin 1 -> Pin C13
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void O2Config(void){ //configuring output pin 2 -> Pin C14
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void O3Config(void){ //configuring output pin 3 -> Pin C15
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void O4Config(void){ //configuring output pin 4 -> Pin C4
	GPIO_InitTypeDef   GPIO_InitStructure;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void ExtBtn1_Config(void)     //**********PC1.***********
// can only use PA0, PB0... to PA4, PB4 .... because only  only  EXTI0, ...EXTI4,on which the 
	//mentioned pins are mapped to, are connected INDIVIDUALLY to NVIC. the others are grouped! 
		//see stm32f4xx.h, there is EXTI0_IRQn...EXTI4_IRQn, EXTI15_10_IRQn defined
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	//__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void ExtBtn2_Config(void){  //**********PD2.***********
	  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	
}
void ExtBtn3_Config(void){  //**********PC3.***********
	  GPIO_InitTypeDef   GPIO_InitStructure;
  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_3);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);	
}

void  TIM3_Config(void)
{
  Tim3_PrescalerValue = (uint32_t) (SystemCoreClock /4000); // setting prescaler to 18000
  
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
   
  Tim3_Handle.Init.Period = 65535; // max period
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
}

void  TIM3_OC_Config(void)
{
		Tim3_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		//4083 initially, this means every 1.02 second generate an inerrupt -> will change as program changes this value
		Tim3_OCInitStructure.Pulse=Tim3_CCR;    
		Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim3_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim3_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
}

int main(void){
	
		/* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
		HAL_Init();
		
	
		 /* Configure the system clock to 72 MHz */
		SystemClock_Config();
		
		HAL_InitTick(0x0000); // set systick's priority to the highest.
	
	
		BSP_LCD_Init();
		//BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address);
		BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
															// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.
		//BSP_LCD_SelectLayer(uint32_t LayerIndex);
		BSP_LCD_SelectLayer(0);
		BSP_LCD_SetLayerVisible(0, ENABLE); 
		
		BSP_LCD_Clear(LCD_COLOR_WHITE);  //need this line, otherwise, the screen is dark	
		BSP_LCD_DisplayOn();
	 
		BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	
//		ExtBtn_Config();

		LCD_DisplayString(0, 6, (uint8_t *)"Lab 5");
		LCD_DisplayString(1, 2, (uint8_t *)"");
		LCD_DisplayString(4, 1, (uint8_t *)"Mode: Full Step ");
		LCD_DisplayString(5, 1, (uint8_t *)"Direction: CCW  ");

		//calling all functions to initialize
		O1Config();
		O2Config();
		O3Config();
		O4Config();
		TIM3_Config();
		TIM3_OC_Config();
		ExtBtn1_Config();
		ExtBtn2_Config();
		ExtBtn3_Config();
		BSP_LED_Init(LED4);
		BSP_LED_Init(LED3);
		BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI); 
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
		
		
		while(1) {	
			float temp = Tim3_CCR;
			if (mode == 0){
				temp = temp*48/4000;
			}else{
				temp = temp*96/4000;
			}
				
			
			LCD_DisplayString(3, 1, (uint8_t *)"Period: ");
			LCD_DisplayFloat(3,10, temp, 2);

			
			
		} // end of while loop
	
}  //end of main


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 

* 					Oscillator											=HSE
	*    				HSE frequencey 										=8,000,000   (8MHz)
	*      ----However, if the project is created by uVision, the default HSE_VALUE is 25MHz. thereore, need to define HSE_VALUE
	*						PLL Source											=HSE
  *            PLL_M                          = 4
  *            PLL_N                          = 72
  *            PLL_P                          = 2
  *            PLL_Q                          = 3
  *        --->therefore, PLLCLK =8MHz X N/M/P=72MHz   
	*            System Clock source            = PLL (HSE)
  *        --> SYSCLK(Hz)                     = 72,000,000
  *            AHB Prescaler                  = 1
	*        --> HCLK(Hz)                       = 72 MHz
  *            APB1 Prescaler                 = 2
	*        --> PCLK1=36MHz,  -->since TIM2, TIM3, TIM4 TIM5...are on APB1, thiese TIMs CLK is 36X2=72MHz
							 	
  *            APB2 Prescaler                 = 1
	*        --> PCLK1=72MHz 

  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

static void ExtBtn_Config() {
	GPIO_InitTypeDef GPIO_InitStructure;

	// Clock Enable
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// PC1
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	// PD2
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	// PC3
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_3);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
		if(GPIO_Pin == KEY_BUTTON_PIN)  //User button
		{
			if(mode==1){ //if previously in half step mode
					mode = 0; //setting to full step mode
					Tim3_CCR = 4083; //setting CCR to be full step speed
					TIM3_Config();
					TIM3_OC_Config();
					FullS=0;//setting initial state to 0
					//initializing first step of motor
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
				
				LCD_DisplayString(4, 1, (uint8_t *)"Mode: Full Step ");
				}
				else if(mode==0){ //if previoulsy in full step mode
					mode = 1; //setting to half step mode
					Tim3_CCR = 2042; //setting CCR to appropriate speed for half step mode
					TIM3_Config();
					TIM3_OC_Config();
					HalfS=0; //half step state initialized to 0
					//initializing first step of motor
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
					LCD_DisplayString(4, 1, (uint8_t *)"Mode: Half Step ");
				}
		}
		
		if(GPIO_Pin == GPIO_PIN_1) //if external button 1 is pushed
		{
				if(CW_Or_CCW==1){ //if previously CCW
					LCD_DisplayString(5, 1, (uint8_t *)"Direction: CCW ");
					CW_Or_CCW = 0; //setting to CCW
					FullS=0;HalfS=0;//states for full and half set to 0
					//iniotializing first step of motor
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
				}
				else if(CW_Or_CCW==0){ //if previously CW
					LCD_DisplayString(5, 1, (uint8_t *)"Direction: CW  ");
					CW_Or_CCW = 1; //setting to CCW
					HalfS=0;FullS=0; //states for full and half set to 0
					//iniotializing first step of motor
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
				}
		}  //end of PIN_1

		if(GPIO_Pin == GPIO_PIN_2){ // if external button 2 is pushed
				BSP_LED_Toggle(LED3);
				Tim3_CCR+=300; //increasing CCR -> decreasing speed
				TIM3_Config();
				TIM3_OC_Config();
				 
		} //end of if PIN_2	
		
		if(GPIO_Pin == GPIO_PIN_3){ //if external button 3 is pushed 
				BSP_LED_Toggle(LED3);
				Tim3_CCR-=300; //decreasing CCR -> increasing speed
				TIM3_Config();
				TIM3_OC_Config();
		} //end of if PIN_3
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
	BSP_LED_Toggle(LED4);
		if(mode==0&&CW_Or_CCW==0){ // if in FUll step AND moving CW
			
			//below is the pins we must write to in order to make the motor rotate
			//we increment the state each timne so on the next interrupt it turns on/off the next needed pins
			if(FullS==0){ // 14 1
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
				FullS = 1;
			}
			else if(FullS==1){ // 4 1
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,1);
				FullS=2;
			}
			else if(FullS==2){ // 15 1
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
				FullS =3;
			}
			else if(FullS==3){ // 13 1
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
				FullS=0;
			}
	}
	else if(mode==1&&CW_Or_CCW==0){ // if in Half step mode AND moving CW
		//below is the pins we must write to in order to make the motor rotate
		//we increment the state each timne so on the next interrupt it turns on/off the next needed pins
		if(HalfS==0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
			HalfS = 1;
		}
		else if(HalfS==1){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
			HalfS=2;
		}
		else if(HalfS==2){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,1);
			HalfS =3;
		}
		else if(HalfS==3){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
			HalfS=4;
		}
		else if(HalfS==4){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
			HalfS=5;
		}
		else if(HalfS==5){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
			HalfS=6;
		}
		else if(HalfS==6){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
			HalfS=7;
		}
		else if(HalfS==7){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
			HalfS=0;
		}
	}
	else if(mode==0&&CW_Or_CCW==1){ // if in Full step mode AND moving CCW
		//below is the pins we must write to in order to make the motor rotate
		//we increment the state each timne so on the next interrupt it turns on/off the next needed pins
		if(FullS==0){ // 14 1
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
			FullS = 3;
		}
		else if(FullS==1){ // 4 1
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,1);
			FullS=0;
		}
		else if(FullS==2){ // 15 1
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
			FullS =1;
		}
		else if(FullS==3){ // 13 1
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
			FullS=2;
		}
	}
	else if(mode==1&&CW_Or_CCW==1){ //if in Half step mode and moving CCW
		//below is the pins we must write to in order to make the motor rotate
		//we increment the state each timne so on the next interrupt it turns on/off the next needed pins
		if(HalfS==0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,1);
			HalfS = 7;
		}
		else if(HalfS==1){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
			HalfS=0;
		}
		else if(HalfS==2){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,1);
			HalfS =1;
		}
		else if(HalfS==3){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
			HalfS=2;
		}
		else if(HalfS==4){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,1);
			HalfS=3;
		}
		else if(HalfS==5){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);
			HalfS=4;
		}
		else if(HalfS==6){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);
			HalfS=5;
		}
		else if(HalfS==7){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,0);
			HalfS=6;
		}
	}
	//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
				__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
	
}
 
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */

/**
  * @}
  */