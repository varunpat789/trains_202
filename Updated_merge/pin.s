	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	
	AREA    main, CODE, READONLY
	EXPORT	pin_init
	ENTRY			
				
pin_init	PROC
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PIN_INIT START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 

    push{r0,r1,r2}                  ;preserve runtime in case
    
    ; Enable Port C clocks
	LDR r0, =RCC_BASE      			;load in base module
	LDR r1, [r0, #RCC_AHB2ENR]     	;load in clock module
	ORR r1, r1, #0x00000007         ;set clocks for Port A B C high
	STR r1, [r0, #RCC_AHB2ENR]     	;store result back to clock reg

	; TRAIN MOTOR
	LDR r0, =GPIOC_BASE        		;load in base module
	LDR r1, [r0, #GPIO_MODER]      	;load moad register
	BIC r1, r1, #0x000C0000         ;clear bits for PC9, 18-19
	ORR r1, r1, #0x00080000    		;set 01 for pin 9 for output
	
	BIC r1, r1, #0x00030000         ;clear bits for PC8, 16-17
	ORR r1, r1, #0x00010000    		;set 01 for pin 8 for output
	
	BIC r1, r1, #0x00003000         ;clear bits for PC6, 12-13
	ORR r1, r1, #0x00001000    		;set 01 for pin 6 for output
	
	BIC r1, r1, #0x00000C00         ;clear bits for PC5, 10-11
	ORR r1, r1, #0x00000800    		;set 01 for pin 13 for output
	STR r1, [r0,#GPIO_MODER]       	;store result back to moder
	
	;set the pupdr of PC9, PC8, PC6, PC5 
	LDR r0, =GPIOC_BASE        		;load in base module
	LDR r1, [r0, #GPIO_PUPDR]      	;load moad register
	BIC r1, r1, #0x000C0000         ;clear bits for PC9, 18-19
	ORR r1, r1, #0x00080000    		;set 01 for pin 9 for PU/PD
	
	BIC r1, r1, #0x00030000         ;clear bits for PC8, 16-17
	ORR r1, r1, #0x00010000    		;set 01 for pin 8 for PU/PD
	
	BIC r1, r1, #0x00003000         ;clear bits for PC6, 12-13
	ORR r1, r1, #0x00001000    		;set 01 for pin 6 for PU/PD
	
	BIC r1, r1, #0x00000C00         ;clear bits for PC5, 10-11
	ORR r1, r1, #0x00000800    		;set 01 for pin 13 for PU/PD
	STR r1, [r0,#GPIO_PUPDR]       	;store result back to pupdr


    ;DOOR MOTOR
	LDR r0, =GPIOC_BASE        		;load in base module
	LDR r1, [r0, #GPIO_MODER]      	;load moad register
	BIC r1, r1, #0x00300000         ;clear bits for PC10, 20-21
	ORR r1, r1, #0x00100000    		;set 01 for pin 10 for output
	
	BIC r1, r1, #0x00C00000         ;clear bits for PC11, 22-23
	ORR r1, r1, #0x00800000    		;set 01 for pin 11 for output
	
	BIC r1, r1, #0x03000000         ;clear bits for PC12, 24-25
	ORR r1, r1, #0x01000000    		;set 01 for pin 12 for output
	
	BIC r1, r1, #0x30000000         ;clear bits for PC14, 28-29
	ORR r1, r1, #0x10000000    		;set 01 for pin 14 for output
	STR r1, [r0,#GPIO_MODER]       	;store result back to moder
	
	;set the pupdr of PC9, PC8, PC6, PC5 
	LDR r0, =GPIOC_BASE        		;load in base module
	LDR r1, [r0, #GPIO_PUPDR]      	;load moad register
	BIC r1, r1, #0x00300000         ;clear bits for PC9, 18-19
	ORR r1, r1, #0x00100000    		;set 01 for pin 9 for PU/PD
	
	BIC r1, r1, #0x00C00000         ;clear bits for PC8, 16-17
	ORR r1, r1, #0x00800000    		;set 01 for pin 8 for PU/PD
	
	BIC r1, r1, #0x03000000         ;clear bits for PC6, 12-13
	ORR r1, r1, #0x01000000    		;set 01 for pin 6 for PU/PD
	
	BIC r1, r1, #0x30000000         ;clear bits for PC5, 10-11
	ORR r1, r1, #0x10000000    		;set 01 for pin 13 for PU/PD
	STR r1, [r0,#GPIO_PUPDR]       	;store result back to pupdr

    
    ; GREEN LED
	LDR r1, =GPIOC_BASE        		;load in base module
	LDR r2, [r1, #GPIO_MODER]      	;load moad register
	BIC r2, r2, #0x0C000000         ;clear bits for PC13, 26-27
	ORR r2, r2, #0x08000000    		;set 01 for pin 13 for output
	STR r1, [r0,#GPIO_MODER]       	;store result back to moder
	
	;set the pupdr of PC13
	LDR r1, =GPIOC_BASE        		;load in base module
	LDR r2, [r1, #GPIO_PUPDR]      	;load moad register
	BIC r2, r2, #0x0C000000         ;clear bits for PC13
	ORR r2, r2, #0x08000000    		;set 01 for pin 13 for PU/PD
	STR r2, [r1,#GPIO_PUPDR]       	;store result back to pupdr
	
	;enable system interrupt clk	
	LDR r0, =RCC_BASE
	LDR r1, [r0, #RCC_APB2ENR]
	ORR r1, r1, #RCC_APB2ENR_SYSCFGEN; 0x1
	STR r1, [r0, #RCC_APB2ENR]

	LDR r0, =0x40010014; SYSCFG_EXTICR4 
	LDR r1, [r0]
	BIC r1,#0xF0
	ORR r1,#0x20
	STR r1,[r0]

	; Connect PC13
	LDR r0, =0xE000E104; NVIC ISER1
	MOV r1, #0x100; Enable IRQ40 (EXITI15_10)
	STR r1, [r0]

	LDR r0, =SYSCFG_BASE
		
		LDR r1, [r0, #0x14]; offset for SYSCFG_EXTICR4
		BIC r1, r1, #SYSCFG_EXTICR4_EXTI13; 
		ORR r1, r1, #SYSCFG_EXTICR4_EXTI13_PC; 
		STR r1, [r0, #0x14]
		
	LDR r0, =EXTI_BASE
	
		LDR r1, [r0, #EXTI_PR1]; enable pending register  
		MOV r1, #(1 << 13)
		STR r1, [r0, #EXTI_PR1]

		LDR r1, [r0, #EXTI_FTSR1]; falling edge trigger
		ORR r1, #0x2000;
		STR r1, [r0, #EXTI_FTSR1]
		
		LDR r1, [r0, #EXTI_IMR1]; unmask to trigger IRQ handler
		ORR r1, #0x2000; 
		STR r1, [r0, #EXTI_IMR1]
	
	LDR r0,=GPIOC_BASE;//GPIOC
	
		LDR r1,[r0, #GPIO_MODER];
		BIC r1, r1, #0xC000000;//pin 13
		ORR r1, r1, #0x0;//input
		STR r1, [r0,#GPIO_MODER];
		
		LDR r1,[r0,#GPIO_OTYPER];
		BIC	r1,r1, #0x2000;//pin 13
		ORR r1,r1, #0x0;//set to push-pull
		STR r1, [r0,#GPIO_OTYPER];
		
		LDR r1, [r0, #GPIO_PUPDR];
		BIC r1,r1, #0xC000000;// pin 13
		ORR r1,r1, #0; no pull-up pull-down
		STR r1, [r0, #GPIO_PUPDR];
	
	CPSIE i; enable global interrupts
		

    pop{r0,r1,r2}                   ;load back in runtime

    BX LR                           ;branch back to main
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PIN_INIT END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	ENDP	
	END