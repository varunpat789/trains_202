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
	; Set the modes of C4 (A), C6 (A'), C8 (B), C9 (B') for output
	LDR r0, =GPIOC_BASE        		;load in base module
	LDR r1, [r0, #GPIO_MODER]      	;load in moder                
	BIC r1, r1, #0x00006300         ;clear all bits of pins 4,6	 
	BIC r1, r1, #0x000F0000         ;clear all bits of pins 8,9
	ORR r1, r1, #0x00001100     	;set pins 4,6 to 01 for output
	ORR r1, r1, #0x00050000     	;set pins 8,9 to 01 for output
	STR r1, [r0, #GPIO_MODER]      	;store result back to moder

    
    ; GREEN LED
	; Enable Port A clocks
	LDR r0, =RCC_BASE      			;load in base module
	LDR r1, [r0, #RCC_AHB2ENR]     	;load in clock module
	ORR r1, r1, #0x00000001         ;set clock for Port A high
	STR r1, [r0, #RCC_AHB2ENR]     	;store result back to clock reg

	;set the mode of PA5 for output
	LDR r0, =GPIOA_BASE        		;load in base module
	LDR r1, [r0, #GPIO_MODER]      	;load moad register
	BIC r1, r1, #0x00000C00         ;clear bits for PA5, 10-11
	ORR r1, r1, #0x00000400    		;set 01 for pin 5 for output
	STR r1, [r0,#GPIO_MODER]       	;store result back to moder
	
	;otyper
	LDR r0, =GPIOA_BASE        		;load in base module
	LDR r1, [r0, #GPIO_OTYPER]      	;load moad register
	BIC r1, r0, #0x00000020         ;clear bits for PA5, 10-11
	ORR r1, r0, #0x00000000    		;set 00 for pin 5 for output
	STR r1, [r0,#GPIO_OTYPER]       	;store result back to moder
	
	;set the pupdr of PA5
	LDR r0, =GPIOA_BASE        		;load in base module
	LDR r1, [r0, #GPIO_PUPDR]      	;load moad register
	BIC r1, r1, #0x00000C00         ;clear bits for PA5
	ORR r1, r1, #0x00000000    		;set 01 for pin 5 for PU/PD
	STR r1, [r0,#GPIO_PUPDR]       	;store result back to pupdr
	
	

	;interrupt

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



	;door motor pins
	;	Enable clocks for GPIOC
	LDR r0, =RCC_BASE; // load RCC module to r0
		
		LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
		ORR r1,r1, #0x4; // enable GPIOC
		STR r1, [r0,#RCC_AHB2ENR];
	
	; Set GPIOC pins 10,11,12,14 as output pins
	LDR r0,=GPIOC_BASE;//GPIOB

	LDR r1,[r0, #GPIO_MODER];
	BIC r1, #0x00F00000;pins 10,11
	BIC r1, #0x0000C000; pin 7
	BIC r1, #0x03000000; pin 12
	ORR r1, #0x00500000;//set all pins to output
	ORR r1, #0x00004000
	ORR r1, #0x01000000
	STR r1, [r0,#GPIO_MODER];

	;seven segment
	LDR r0, =RCC_BASE; // load RCC module to r0

	LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
	ORR r1,r1, #0x2; // enable GPIOB
	STR r1, [r0,#RCC_AHB2ENR];

	LDR r0,=GPIOB_BASE;//GPIOB=output

	LDR r1,[r0, #GPIO_MODER];
	BIC r1, r1, #0x00000F00;// clear pins 4,5
 	BIC r1, r1, #0x3C000000;// clear pins 13,14
 	ORR r1, r1, #0x00000500;//set all pins to output
 	ORR r1, r1, #0x14000000;//set all pins to output
 	STR r1, [r0,#GPIO_MODER];
	
	LDR r1, [r0, #GPIO_PUPDR];
	BIC r1, r1, #0x00000F00;// clear pins 4,5
	BIC r1, r1, #0x3C000000;// clear pins 13,14
	ORR r1,r1, #0x0000000; // set to no pull-up pull-down
	STR r1, [r0, #GPIO_PUPDR];
	
	CPSIE i; enable global interrupts
		

    pop{r0,r1,r2}                   ;load back in runtime

    BX LR                           ;branch back to main
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PIN_INIT END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	ENDP	
	END