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
	EXPORT EXTI15_10_IRQHandler ; very important to keep
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
		LDR r1, [r0, #RCC_APB2ENR]
		ORR r1, r1, #RCC_APB2ENR_SYSCFGEN; 0x1
		STR r1, [r0, #RCC_APB2ENR]

		LDR r0, =0x40010014
		LDR r1, [r0]
		BIC r1,#0xF0
		ORR r1,#0x20
		STR r1,[r0]

	; Connect PC13
	LDR r0, =0xE000E104
	mov r1, #0x100
	str r1, [r0]
	
	LDR r0, =SYSCFG_BASE
		
		LDR r1, [r0, #0x14]
		BIC r1, r1, #SYSCFG_EXTICR4_EXTI13; 
		ORR r1, r1, #SYSCFG_EXTICR4_EXTI13_PC; 
		STR r1, [r0, #0x14]
		
	LDR r0, =EXTI_BASE
	
		LDR r1, [r0, #EXTI_PR1]
		ORR r1, #0x2000  ; Clear pending bits for EXTI13
		STR r1, [r0, #EXTI_PR1]

		LDR r1, [r0, #EXTI_FTSR1]; rising edge
		ORR r1, #0x2000;
		STR r1, [r0, #EXTI_FTSR1]
		
		LDR r1, [r0, #EXTI_IMR1]
		ORR r1, #0x2000; 
		STR r1, [r0, #EXTI_IMR1]
	
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
		
	LDR r0, =RCC_BASE
	LDR r1, [r0, #RCC_AHB2ENR]
	ORR r1, r1, #0x1         ; Enable GPIOA clock
	STR r1, [r0, #RCC_AHB2ENR]

	; Set PA5 to output mode
	LDR r0, =GPIOA_BASE
	LDR r1, [r0, #GPIO_MODER]
	BIC r1, r1, #(0x3 << (5*2))     ; Clear MODER5 bits
	ORR r1, r1, #(0x1 << (5*2))     ; Set MODER5 to 01 (output)
	STR r1, [r0, #GPIO_MODER]
	
main_loop
    CMP r5, #1
    BNE main_loop
    MOV r5, #0
    B main_loop
		
EXTI15_10_IRQHandler PROC
    PUSH {r4-r11}

    ; Turn on PA5 (set bit 5)
    LDR r3, =GPIOA_BASE             
    LDR r4, [r3, #GPIO_ODR]         ; Load current output state
    ORR r4, r4, #(1 << 5)           ; Set PA5 high
    STR r4, [r3, #GPIO_ODR]         ; Write back to ODR

    ; Clear EXTI13 interrupt pending bit
    LDR r2, =EXTI_BASE
    MOV r3, #(1 << 13)
    STR r3, [r2, #EXTI_PR1]

    POP {r4-r11}
    BX lr
    ENDP

					
	ALIGN			

	AREA myData, DATA, READWRITE
	ALIGN
	
char1 DCD 43
	END