#include "stm32l476xx.h"
#include "I2C.h"
#include "ssd1306.h"


#include <string.h>
#include <stdio.h>

void System_Clock_Init(void);
void RTC_Clock_Init(void);
void RTC_Init(void);
void LCD_Initialization(void);
void LCD_Clear(void);
void I2C_GPIO_init(void);
void DisplayString(char* messge);
char date[] = "123456";



void DisplayString(char* message){

	ssd1306_Fill(White);
	ssd1306_SetCursor(2,0);
	ssd1306_WriteString(message, Font_11x18, Black);
	ssd1306_UpdateScreen();	

	
}



void SysTick_Handler(void){

////////////////////////////////////////////////
//USER CODE GOES HERE///////////////////////////		
//USER CODE GOES HERE///////////////////////////
//USER CODE GOES HERE///////////////////////////
//USER CODE GOES HERE///////////////////////////
////////////////////////////////////////////////
	
}
	

	

int main(void){
	
	// Enable High Speed Internal Clock (HSI = 16 MHz)
  RCC->CR |= ((uint32_t)RCC_CR_HSION);
	
  // wait until HSI is ready
  while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}
	
  // Select HSI as system clock source 
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;  //01: HSI16 oscillator used as system clock

  // Wait till HSI is used as system clock source 
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) == 0 ) {;}

	NVIC_SetPriority(SysTick_IRQn, 1);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);					// Enable EXTI0_1 interrupt in NVIC
  
		
  ////////////////////////////////////////////////
  //USER CODE GOES HERE///////////////////////////		
  //USER CODE GOES HERE///////////////////////////
  //USER CODE GOES HERE///////////////////////////
  //USER CODE GOES HERE///////////////////////////
  ////////////////////////////////////////////////

  // Dead loop & program hangs here
	while(1){	}
}




	
