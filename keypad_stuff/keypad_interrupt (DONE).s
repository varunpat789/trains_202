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
	EXPORT keypad_Init
	EXPORT keypad
	ENTRY			
				
keypad_Init	PROC



;;;;;;;;;;;; YOUR CODE GOES HERE	;;;;;;;;;;;;;;;;;;;
	LDR r0, =RCC_BASE; // load RCC module to r0
		
		LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
		ORR r1,r1, #0x2; // enable GPIOB
		STR r1, [r0,#RCC_AHB2ENR];
		
		LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
		ORR r1,r1, #0x4; // enable GPIOC
		STR r1, [r0,#RCC_AHB2ENR];

	LDR r0,=GPIOC_BASE;//GPIOC=output=rows
	
		LDR r1,[r0, #GPIO_MODER];
		BIC r1, r1, #0xFF;//pins 0,1,2,3
		ORR r1, r1, #0x55;//output
		STR r1, [r0,#GPIO_MODER];
		
		LDR r1,[r0,#GPIO_OTYPER];
		BIC	r1,r1, #0xF;//pins 0,1,2,3
		ORR r1,r1, #0xF;//set to open drain
		STR r1, [r0,#GPIO_OTYPER];
		
		LDR r1, [r0, #GPIO_PUPDR];
		BIC r1,r1, #0xFF;// pins 0,1,2,3
		ORR r1,r1, #0xAA; // set to pull down
		STR r1, [r0, #GPIO_PUPDR];

		LDR r1, [r0, #GPIO_ODR]
		BIC r1,#0xF
		STR r1, [r0, #GPIO_ODR]
		
	LDR r0,=GPIOB_BASE;//GPIOB=input=cols
	
		LDR r1,[r0, #GPIO_MODER];
		BIC r1, r1, #0xFC;//pins 1,2,3
		ORR r1, r1, #0x0;//set all pins to input
		STR r1, [r0,#GPIO_MODER];
	
		LDR r1,[r0,#GPIO_OTYPER];
		BIC	r1,r1, #0xE;//pins 1,2,3
		ORR r1,r1, #0x0;//set to push-pull
		STR r1, [r0,#GPIO_OTYPER];
		
		LDR r1, [r0, #GPIO_PUPDR];
		BIC r1, r1, #0xFC;//pins 1,2,3
		ORR r1,r1, #0x0; // set to no pull-up pull-down
		STR r1, [r0, #GPIO_PUPDR];

		LDR r1, [r0, #GPIO_IDR]
		ORR r1,#0xE
		STR r1, [r0, #GPIO_IDR]
		
	bx lr
	endp
        
keypad proc
	LDR r4,=GPIOB_BASE; //Loading values from GPIOB to r4
	LDR r5, [r4, #GPIO_IDR]; //Loading to r5 the IDR values (from physical keypad) from r4
	push {r2, lr}; //Pushing r2 and lr to the stack
	bl delay; //Branch linking to delay function
	pop {r2,lr}; //Popping r2 and lr out of the stack
	cmp r5,#0x1E; //comparing all cols are 1s
	beq keypad; //Branching to keypad whenever the value from r5 is all ones
	bne check_col_1; //Branching to check_col_1 as an else case
	
check_col_1
	mov r10, #0; //Moving 0 to r10
	cmp r5, #0x1C; //Comparing the value from r5 to the value for col 1
	addeq r10,#1; //Adding 1 to r10 if equal
	cmp r5, #0x1A; //Comparing the value from r5 to the value for col 2
	addeq r10,#2; //Adding 2 to r10 if equal
	cmp r5, #0x16; //Comparing the value from r5 to the value for col 1
	addeq r10,#3; //Adding 3 to r10 if equal
	mov r6,r5; //Copying the value of r5 to r6
	bl press_check; //Branch linking to press_check
	b row; //Branching to row after branch linking
	
row
	LDR r5, [r4, #GPIO_IDR]; //Loading the IDR value from r4 to r5
	mov r11, #0; //resetting r11 - is the row counter
	mov r9, #0; //resetting r9 - is a column checker
	ORR r5,#0xE; all cols are 1s; //Orring r5 with E to get all cols to 1
	bl o_row; //scan first row
	bl s_row; //scan second row
	bl t_row; //scan third row
	bl f_row; //scan fourth row
	b ascii_decoder; //branching to ascii_decoder

press_check
	LDR r4,=GPIOB_BASE; //Loading the GPIOB values to r4
	LDR r5, [r4, #GPIO_IDR]; //Loading the IDR values from r4 to r5
	push{lr}; //Pushing lr to stack
	bl delay; //Branch linking to delay
	pop{lr}; //Popping lr from stack
	cmp r5,r6; //checking the first iteration value and second iteration values are the same
	bne press_check; //When not equal, we go back to press_check
	bx lr; //If equal, break and store link register value
	
o_row
	push{lr}; //pushing lr to stack
	LDR r12,=GPIOC_BASE; //loading values of GPIOC to r12
	LDR r4,[r12,#GPIO_ODR]; //loading values of ODR form r12 to r4
	BIC r4,#0xF; //clear 0,1,2,3;
	ORR r4,#0xE; //row 0 low
	STR r4,[r12,#GPIO_ODR]
	bl delay
	bl scan_check; //Goes to scan_check to check for any shorts in the column in each row
	cmp r9, #0; //checking for which column we are in
	addne r11, #1; //if not equal, we know which column we are in
	pop{lr}
	bx lr

; //Does the same as o_row
s_row
	push{lr}
	LDR r12,=GPIOC_BASE;
	LDR r4,[r12,#GPIO_ODR]
	BIC r4,#0xF; clear 0,1,2,3
	ORR r4,#0xD; row 1 low
	STR r4,[r12,#GPIO_ODR]
	bl delay
	bl scan_check
	cmp r9, #0
	addne r11, #2
	pop{lr}
	bx lr

; //does the same as o_row
t_row
	push{lr}
	LDR r12,=GPIOC_BASE;
	LDR r4,[r12,#GPIO_ODR]
	BIC r4,#0xF; clear 0,1,2,3
	ORR r4,#0xB; row 2 low
	STR r4,[r12,#GPIO_ODR]
	bl delay
	bl scan_check
	cmp r9, #0
	addne r11, #3
	pop{lr}
	bx lr

; //Does the same as o_row
f_row
	push{lr}
	LDR r12,=GPIOC_BASE;
	LDR r4,[r12,#GPIO_ODR]
	BIC r4,#0xF; clear 0,1,2,3
	ORR r4,#0x7; row 3 low
	STR r4,[r12,#GPIO_ODR]
	bl delay
	bl scan_check
	cmp r9, #0
	addne r11, #4
	pop{lr}
	bx lr

scan_check
	LDR r12,=GPIOB_BASE;
	LDR r5,[r12,#GPIO_IDR]
	mov r9, #0
	cmp r5, #0x1C; //checking if col 1
	addeq r9,#1
	cmp r5, #0x1A; //checking if col 2
	addeq r9,#2
	cmp r5, #0x16; //checking if col 3
	addeq r9,#3
	bx lr
	
ascii_decoder
	cmp r11,#1; if row 1
	beq drow0
	cmp r11,#2; if row 2
	beq drow1
	cmp r11,#3; if row 3
	beq drow2
	cmp r11,#4; if row 4
	beq drow3
	
drow0; decoder for row 1
	cmp r10,#1; col 1
	moveq r5,#49; mov 1
	cmp r10,#2; col 2
	moveq r5,#50; mov 2
	cmp r10,#3; col 3
	moveq r5,#51; mov 3
	LDR r8,=char1 
	b displaykey

drow1; decoder for row 2
	cmp r10,#1; col 1
	moveq r5,#52; mov 4
	cmp r10,#2; col 2
	moveq r5,#53; mov 5
	cmp r10,#3; col 3
	moveq r5,#54; mov 6
	LDR r8,=char1 
	b displaykey
	
drow2; decoder for row 3
	cmp r10,#1; col 1
	moveq r5,#55; mov 7
	cmp r10,#2; col 2
	moveq r5,#56; mov 8
	cmp r10,#3; col 3
	moveq r5,#57; mov 9
	LDR r8,=char1 
	b displaykey
	
drow3; decoder for row 4
	cmp r10,#1; col 1
	moveq r5,#42; mov *
	cmp r10,#2; col 2
	moveq r5,#48; mov 0
	cmp r10,#3; col 3
	moveq r5,#35; mov #
	LDR r8,=char1 
	b displaykey

displaykey
	STR	r5, [r8]; r5  ascii
	LDR	r0, =char1
	;LDR r0, =str   ; First argument
	MOV r1, #1    ; Second argument
	BL USART2_Write
	bl delay
	b reset
	
reset
	LDR r0,=GPIOC_BASE;
		LDR r1, [r0, #GPIO_ODR]
		BIC r1,#0xF
		STR r1, [r0, #GPIO_ODR]
	LDR r0,=GPIOB_BASE;
		LDR r1, [r0, #GPIO_IDR]
		bl delay
		cmp r1,#0x1E; make sure that button is relased before checking for another value
		bne reset
		ORR r1,#0xE
		STR r1, [r0, #GPIO_IDR]
	
	bx lr

	ENDP		

delay	PROC
	; Delay for software debouncing
	LDR	r2, =0x9999
delayloop
	SUBS	r2, #1
	BNE	delayloop
	BX LR
	
	ENDP
		
					
	ALIGN			

	AREA myData, DATA, READWRITE
	ALIGN
	
char1 DCD 43
	END