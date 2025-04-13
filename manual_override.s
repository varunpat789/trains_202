;******************** (C) Yifeng ZHU *******************************************
; @file    main.s
; @author  Yifeng Zhu
; @date    May-17-2015
; @note
;           This code is for the book "Embedded Systems with ARM Cortex-M 
;           Microcontrollers in Assembly Language and C, Yifeng Zhu, 
;           ISBN-13: 978-0982692639, ISBN-10: 0982692633
; @attension
;           This code is provided for education purpose. The author shall not be 
;           held liable for any direct, indirect or consequential damages, for any 
;           reason whatever. More information can be found from book website: 
;           http:;www.eece.maine.edu/~zhu/book
;*******************************************************************************


	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	
	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY			
				
__main	PROC
	
	BL System_Clock_Init
	BL UART2_Init



;;;;;;;;;;;; YOUR CODE GOES HERE	;;;;;;;;;;;;;;;;;;;
	LDR r0, =RCC_BASE; // load RCC module to r0

		LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
		ORR r1,r1, #0x4; // enable GPIOC
		STR r1, [r0,#RCC_AHB2ENR];

		LDR r0, =RCC_BASE
		LDR r1, [r0, #RCC_AHB2ENR]
		ORR r1, r1, #RCC_APB2ENR_SYSCFGEN; 0x1
		STR r1, [r0, #RCC_AHB2ENR]

		LDR r0, =0x40010014
		LDR r1, [r0]
		BIC r1,#0xF0
		ORR r1,#0x20
		STR r1,[r0]

	; Connect PC4, PA10
	LDR r0, =SYSCFG_BASE

		LDR r1, [r0, #SYSCFG_EXTICR3]
		BIC r1, r1, #SYSCFG_EXTICR3_EXTI13; 0x7
		ORR r1, r1, #SYSCFG_EXTICR3_EXTI4_PC; 0x2
		STR r1, [r0, #SYSCFG_EXTICR3]
		
	LDR r0, =EXTI_BASE
		LDR r1, [r0, #EXTI_PR1]
		ORR r1, #0x2000  ; Clear pending bits for EXTI4 and EXTI10
		STR r1, [r0, #EXTI_PR1]

		LDR r1, [r0, #EXTI_RTSR1]; rising edge
		ORR r1, #0x2000; // EXTI4, EXTI10
		STR r1, [r0, #EXTI_RTSR1]
		
		LDR r1, [r0, #EXTI_IMR1]
		ORR r1, #0x2000; // EXTI4, EXTI10
		STR r1, [r0, #EXTI_IMR1]

	LDR r0, =NVIC_BASE           
		LDR r1, [r0, #NVIC_ISER]
		ORR r1, r1, #(1 << 4)     ; Set bit 10 in ISER0 for EXTI4
		STR r1, [r0, #NVIC_ISER]

		LDR r1, [r0, #NVIC_ISER]
		ORR r1, r1, #(1 << 10)  ; Set bit 10 in ISER1 for EXTI10 (42-32=10)
		STR r1, [r0, #NVIC_ISER]
	
	LDR r0,=GPIOC_BASE;//GPIOC
	
		LDR r1,[r0, #GPIO_MODER];
		BIC r1, r1, #0xC000000;//pin 13
		ORR r1, r1, #0x0;//input
		STR r1, [r0,#GPIO_MODER];
		
		LDR r1,[r0,#GPIO_OTYPER];
		BIC	r1,r1, #0x2000;//pin 4
		ORR r1,r1, #0x0;//set to push-pull
		STR r1, [r0,#GPIO_OTYPER];
		
		LDR r1, [r0, #GPIO_PUPDR];
		BIC r1,r1, #0xC000000;// pin 4
		ORR r1,r1, #0; // set to no pull-up pull-down
		STR r1, [r0, #GPIO_PUPDR];
	
main_loop
	b main_loop
	ENDP
		
EXTI13_IRQHandler PROC		
	LDR r0, =EXTI_BASE
	MOV r1, #EXTI_PR1_PIF4
	STR r1, [r0, #EXTI_PR1]

	mov r5, #1
	pop{r4-r11}
	bx lr
	ENDP	
					
	ALIGN			

	AREA myData, DATA, READWRITE
	ALIGN
	
char1 DCD 43
	END