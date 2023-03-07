
#include "GPIO.h"
#include "main.h"
#include "common.h"

//------------------------------------------------------------------------
//---------------------------   Motor_Init   ------------------------------
//------------------------------------------------------------------------
void Motor_Init(void){
	m[0].EN = 1;	//M1 1- ON/0 - OFF
	m[1].EN = 1;	//M2 1- ON/0 - OFF
	m[2].EN = 1;	//M3 1- ON/0 - OFF
	m[3].EN = 1;	//M4 1- ON/0 - OFF
	m[4].EN = 1;	//M5 1- ON/0 - OFF
	m[5].EN = 1;	//M6 1- ON/0 - OFF
	m[6].EN = 1;	//M7 1- ON/0 - OFF
	m[7].EN = 1;	//M8 1- ON/0 - OFF

	m[0].perMaxPWM = 500;		//Maximum drive speed in % x10
	m[0].perStartPWM = 100;		//The speed in % x10 at which the acceleration of the drive begins
	m[0].tAcceleration = 200;	//Acceleration duration (in steps)

	m[1].perMaxPWM = 1000;
	m[1].perStartPWM = 100;
	m[1].tAcceleration = 200;

	m[2].perMaxPWM = 500;
	m[2].perStartPWM = 100;
	m[2].tAcceleration = 200;

	m[3].perMaxPWM = 1000;
	m[3].perStartPWM = 100;
	m[3].tAcceleration = 200;

	m[4].perMaxPWM = 1000;
	m[4].perStartPWM = 100;
	m[4].tAcceleration = 100;

	m[5].perMaxPWM = 1000;
	m[5].perStartPWM = 100;
	m[5].tAcceleration = 200;

	m[6].perMaxPWM = 1000;
	m[6].perStartPWM = 100;
	m[6].tAcceleration = 100;

	m[7].perMaxPWM = 1000;
	m[7].perStartPWM = 100;
	m[7].tAcceleration = 200;

	for (uint8_t i = 0; i<8; i++){
		m[i].StepSizeF = (((m[i].perMaxPWM - m[i].perStartPWM) * 100)/m[i].tAcceleration);
	}


}
//------------------------------------------------------------------------
//---------------------------   RCC_Init   ------------------------------
//------------------------------------------------------------------------
void RCC_Init(void)
{
		RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
		RCC->CR |= ((uint32_t)RCC_CR_HSEON);					// Enable HSE
		while (!(RCC->CR & RCC_CR_HSERDY));						// Ready start HSE

		FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;		// Cloclk Flash memory

		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;						// AHB = SYSCLK/1
		RCC->CFGR |= RCC_CFGR_PPRE_DIV1;						// APB = HCLK/1

		RCC->CFGR &= ~RCC_CFGR_PLLMUL;							// clear PLLMULL bits
		RCC->CFGR &= ~RCC_CFGR_PLLSRC;							// clear PLLSRC bits
		RCC->CFGR &= ~RCC_CFGR_PLLXTPRE;						// clear PLLXTPRE bits


		RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV ;				/*!< PLL entry clock source */
		RCC->CFGR |= RCC_CFGR_PLLXTPRE_HSE_PREDIV_DIV2;			/*!< HSE/PREDIV clock divided by 2 for PLL entry */

		RCC->CFGR |= RCC_CFGR_PLLMUL12;							// PLL x4: clock = 4 MHz * 12 = 48 MHz

		RCC->CR |= RCC_CR_PLLON;								// enable PLL
		while((RCC->CR & RCC_CR_PLLRDY) == 0) {} 				// wait till PLL is ready

		RCC->CFGR &= ~RCC_CFGR_SW;								// clear SW bits
		RCC->CFGR |= RCC_CFGR_SW_PLL;							// select source SYSCLK = PLL
		while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1) {}	// wait till PLL is used
}

//------------------------------------------------------------------------
//---------------------------   LEDtest   ------------------------------
//------------------------------------------------------------------------
void LEDtst(void){

	if (m[0].EN == 1){
		TIM1->CCR1 = 1000;
	}
	if (m[1].EN == 1){
		TIM1->CCR2 = 1000;
	}
	if (m[2].EN == 1){
		TIM1->CCR3 = 1000;
	}
	if (m[3].EN == 1){
		TIM1->CCR4 = 1000;
	}

	if (m[4].EN == 1){
		TIM3->CCR1 = 1000;
	}
	if (m[5].EN == 1){
		TIM3->CCR2 = 1000;
	}
	if (m[6].EN == 1){
		TIM3->CCR3 = 1000;
	}
	if (m[7].EN == 1){
		TIM3->CCR4 = 1000;
	}
	delay(1000);

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	if (m[0].EN == 1){
		GPIOA->BSRR |= GPIO_BSRR_BS_15;
	}
	if (m[1].EN == 1){
		GPIOB->BSRR |= GPIO_BSRR_BS_3;
	}
	if (m[2].EN == 1){
		GPIOB->BSRR |= GPIO_BSRR_BS_4;
	}
	if (m[3].EN == 1){
		GPIOB->BSRR |= GPIO_BSRR_BS_5;
	}
	if (m[4].EN == 1){
		GPIOB->BSRR |= GPIO_BSRR_BS_12;
	}
	if (m[5].EN == 1){
		GPIOB->BSRR |= GPIO_BSRR_BS_13;
	}
	if (m[6].EN == 1){
		GPIOB->BSRR |= GPIO_BSRR_BS_14;
	}
	if (m[7].EN == 1){
		GPIOB->BSRR |= GPIO_BSRR_BS_15;
	}

	delay(1000);
	GPIOA->BSRR |= GPIO_BSRR_BR_15;
	GPIOB->BSRR |= GPIO_BSRR_BR_3;
	GPIOB->BSRR |= GPIO_BSRR_BR_4;
	GPIOB->BSRR |= GPIO_BSRR_BR_5;

	GPIOB->BSRR |= GPIO_BSRR_BR_12;
	GPIOB->BSRR |= GPIO_BSRR_BR_13;
	GPIOB->BSRR |= GPIO_BSRR_BR_14;
	GPIOB->BSRR |= GPIO_BSRR_BR_15;


}
//------------------------------------------------------------------------
//---------------------------   GPIO_Init   ------------------------------
//------------------------------------------------------------------------
void GPIO_Init(void){
	//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  //Enable clock PORT A
	//GPIOA->MODER |= GPIO_MODER_MODER6_1; //OutPut mode
	//GPIOA->OTYPER &= ~GPIO_OTYPER_OT_6;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  //Enable clock PORT A
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  //Enable clock PORT B
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  //Enable clock PORT C

	//-----------------   PWM_En   ----------------------------------
	//M1_En
	GPIOA->MODER |= GPIO_MODER_MODER15_0;	//General purpose OutPut mode
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_15;	//PUSH PULL
	//M2_En
	GPIOB->MODER |= GPIO_MODER_MODER3_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_3;		//PUSH PULL
	//M3_En
	GPIOB->MODER |= GPIO_MODER_MODER4_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_3;		//PUSH PULL
	//M4_En
	GPIOB->MODER |= GPIO_MODER_MODER5_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_3;		//PUSH PULL
	//M5_En
	GPIOB->MODER |= GPIO_MODER_MODER12_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_12;	//PUSH PULL
	//M6_En
	GPIOB->MODER |= GPIO_MODER_MODER13_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_13;	//PUSH PULL
	//M7_En
	GPIOB->MODER |= GPIO_MODER_MODER14_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_14;	//PUSH PULL
	//M8_En
	GPIOB->MODER |= GPIO_MODER_MODER15_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_15;	//PUSH PULL
	//-------------------   LEDs   -------------------------------------
	//LED1 PB10 (GREEN)
	GPIOB->MODER |= GPIO_MODER_MODER10_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_10;	//PUSH PULL
	//LED2 PB11 (BLUE)
	GPIOB->MODER |= GPIO_MODER_MODER11_0;	//General purpose OutPut mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_11;	//PUSH PULL
	//----------   Motion Sensor (PB9)   ----------------------------
	GPIOB->MODER &= ~GPIO_MODER_MODER9_Msk; //Input mode

	//-------------   FOR TEST !!!   -------------------------------------------
	//M1_PWM
	//GPIOA->MODER |= GPIO_MODER_MODER8_0;	//General purpose OutPut mode
	//GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;		//PUSH PULL
	//M2_PWM
	//GPIOA->MODER |= GPIO_MODER_MODER9_0;	//General purpose OutPut mode
	//GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;		//PUSH PULL


	//----------   ������   ----------------------------
	//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  //Enable clock PORT A
	//GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk; //Input mode
	//GPIOA->OTYPER
}


//------------------------------------------------------------------------
//---------------------------   PWM_Init   ------------------------------
//------------------------------------------------------------------------
void PWM_Init(void){

	//----------------------   TIM3   ------------------------------------
	RCC->AHBENR|= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR|= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR|= RCC_APB1ENR_TIM3EN;	//Enable Clock TIM3

	//TIM3_CH1
	GPIOA->MODER &= ~GPIO_MODER_MODER6;			//RESET MODE
	GPIOA->MODER|= GPIO_MODER_MODER6_1; 		//10 ALTERNATIVE FUNCTION
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_6; 		//PUSH PULL
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR6; 	// MAX SPEED
	//TIM3_CH2
	GPIOA->MODER &= ~GPIO_MODER_MODER7;			//RESET MODE
	GPIOA->MODER|= GPIO_MODER_MODER7_1; 		//10 ALTERNATIVE FUNCTION
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_7; 		//PUSH PULL
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR7; 	// MAX SPEED
	//TIM3_CH3
	GPIOB->MODER &= ~GPIO_MODER_MODER0;			//RESET MODE
	GPIOB->MODER|= GPIO_MODER_MODER0_1; 		//10 ALTERNATIVE FUNCTION
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_0; 		//PUSH PULL
	GPIOB->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR0; 	// MAX SPEED
	//TIM3_CH4
	GPIOB->MODER &= ~GPIO_MODER_MODER1;			//RESET MODE
	GPIOB->MODER|= GPIO_MODER_MODER1_1; 		//10 ALTERNATIVE FUNCTION
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_1; 		//PUSH PULL
	GPIOB->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR1; 	// MAX SPEED


	GPIOA->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL6, 1); //GPIOA->AFR[0] |= (0x01<< 6*4);				// PA6 ALTERNATIVE FUNCTION PWM TIM3 CH1
	GPIOA->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL7, 1); //GPIOA->AFR[0] |= (0x01<< 7*4);				// PA7 ALTERNATIVE FUNCTION PWM TIM3 CH2

	GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL0, 1); //GPIOB->AFR[0] |= (0x01<< 0*4);				// PB0 ALTERNATIVE FUNCTION PWM TIM3 CH3
	GPIOB->AFR[0] |= _VAL2FLD(GPIO_AFRL_AFSEL1, 1); //GPIOB->AFR[0] |= (0x01 << 1*4);				// PB1 ALTERNATIVE FUNCTION PWM TIM3 CH4


	TIM3->ARR = 1000;							//Count to 1000
	TIM3->PSC = 48 - 1;							//65535max	Prescaler

	TIM3->CCR1 = 0;								//CH1 filling
	TIM3->CCR2 = 0;								//CH2 filling
	TIM3->CCR3 = 0;								//CH3 filling
	TIM3->CCR4 = 0;								//CH4 filling

	TIM3->CCMR1|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 			//TIM3 CH1 polarity (110 MODE)
	TIM3->CCMR1|= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; 			//TIM3 CH2 polarity (110 MODE)

	TIM3->CCMR2|= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; 			//TIM3 CH4 polarity (110 MODE)
	TIM3->CCMR2|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; 			//TIM3 CH3 polarity (110 MODE)

	TIM3->CCER |= TIM_CCER_CC1E ; 				//Capture/Compare 1 output enable
	TIM3->CCER |= TIM_CCER_CC2E ; 				//Capture/Compare 2 output enable
	TIM3->CCER |= TIM_CCER_CC3E;				//Capture/Compare 3 output enable
	TIM3->CCER |= TIM_CCER_CC4E;				//Capture/Compare 4 output enable

	TIM3->CCER &= ~TIM_CCER_CC1P; 				//Active polarity
	TIM3->CCER &= ~TIM_CCER_CC2P; 				//Active polarity
	TIM3->CCER &= ~TIM_CCER_CC3P; 				//Active polarity
	TIM3->CCER &= ~TIM_CCER_CC4P; 				//Active polarity

	TIM3->CR1 &= ~TIM_CR1_DIR;		 			//Count direction.
	TIM3->CR1 |= TIM_CR1_CEN;

	//----------------------   TIM1   ------------------------------------
	RCC->AHBENR|= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR|= RCC_APB2ENR_TIM1EN;				//Enable Clock TIM1

	GPIOA->MODER &= ~GPIO_MODER_MODER8;				//RESET MODE
	GPIOA->MODER|= GPIO_MODER_MODER8_1; 			//10 ALTERNATIVE FUNCTION
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8; 			//PUSH PULL
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR8; 		// MAX SPEED

	GPIOA->MODER &= ~GPIO_MODER_MODER9;				//RESET MODE
	GPIOA->MODER|= GPIO_MODER_MODER9_1; 			//10 ALTERNATIVE FUNCTION
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9; 			//PUSH PULL
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR9; 		// MAX SPEED

	GPIOA->MODER &= ~GPIO_MODER_MODER10;			//RESET MODE
	GPIOA->MODER|= GPIO_MODER_MODER10_1; 			//10 ALTERNATIVE FUNCTION
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_10; 			//PUSH PULL
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR10; 		// MAX SPEED

	GPIOA->MODER &= ~GPIO_MODER_MODER11;			//RESET MODE
	GPIOA->MODER|= GPIO_MODER_MODER11_1; 			//10 ALTERNATIVE FUNCTION
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_11; 			//PUSH PULL
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR11; 		// MAX SPEED


	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8;
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9;
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10;
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL11;

	GPIOA->AFR[1] |= _VAL2FLD(GPIO_AFRH_AFSEL8, 2);		//GPIOA->AFR[1] |= (0b10 << 0*4);	// PA8 ALTERNATIVE FUNCTION PWM TIM1 CH1
	GPIOA->AFR[1] |= _VAL2FLD(GPIO_AFRH_AFSEL9, 2);		//GPIOA->AFR[1] |= (0b10 << 1*4); 	// PA9 ALTERNATIVE FUNCTION PWM TIM1 CH2
	GPIOA->AFR[1] |= _VAL2FLD(GPIO_AFRH_AFSEL10, 2);	//GPIOA->AFR[1] |= (0b10 << 2*4);	// PA10 ALTERNATIVE FUNCTION PWM TIM1 CH3
	GPIOA->AFR[1] |= _VAL2FLD(GPIO_AFRH_AFSEL11, 2);	//GPIOA->AFR[1] |= (0b10 << 3*4);	// PA11 ALTERNATIVE FUNCTION PWM TIM1 CH4

	TIM1->ARR = 1000;													//Count to 1000
	TIM1->PSC = 48 - 1;												//65535max	Prescaler
	TIM1->CCR1 = 0;	//CH1 filling
	TIM1->CCR2 = 0;	//CH2 filling
	TIM1->CCR3 = 0;	//CH1 filling
	TIM1->CCR4 = 0;	//CH2 filling


	TIM1->CCMR1|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; 									//TIM1 CH1 polarity (110 MODE)
	TIM1->CCMR1|= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; 									//TIM1 CH2 polarity (110 MODE)

	TIM1->CCMR2|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; 									//TIM1 CH3 polarity (110 MODE)
	TIM1->CCMR2|= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; 									//TIM1 CH4 polarity (110 MODE)


	//TIM1->CCMR1|= TIM_CCMR1_OC1PE;			//Output Compare preload enable
	TIM1->BDTR |= TIM_BDTR_MOE;					//Main output enable

	TIM1->CCER |= TIM_CCER_CC1E;				//Capture/Compare 4 output enable
	TIM1->CCER |= TIM_CCER_CC2E;				//Capture/Compare 4 output enable
	TIM1->CCER |= TIM_CCER_CC3E;				//Capture/Compare 4 output enable
	TIM1->CCER |= TIM_CCER_CC4E;				//Capture/Compare 4 output enable

	TIM1->CCER &= ~TIM_CCER_CC1P; 				//Active polarity
	TIM1->CCER &= ~TIM_CCER_CC2P; 				//Active polarity
	TIM1->CCER &= ~TIM_CCER_CC3P; 				//Active polarity
	TIM1->CCER &= ~TIM_CCER_CC4P; 				//Active polarity

	TIM1->CR1 &= ~TIM_CR1_DIR;		 						//Count direction.
	TIM1->CR1 |= TIM_CR1_CEN;


}
//------------------------------------------------------------------------
//------------------------------------------------------------------------
//------------------------------------------------------------------------

