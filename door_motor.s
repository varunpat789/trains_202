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

; PINS USED: PC10, PC11, PC12, PC14
; PC10, PC12
; PC11, PC14 don't work

	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	
	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY			
				
__main	PROC
	
	;	Enable clocks for GPIOC
	LDR r0, =RCC_BASE; // load RCC module to r0
		
		LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
		ORR r1,r1, #0x4; // enable GPIOC
		STR r1, [r0,#RCC_AHB2ENR];
	; Set GPIOC pin 13 (blue button) as an input pin//;
	LDR r0,=GPIOC_BASE;//GPIOC
	
		LDR r1,[r0, #GPIO_MODER];
		BIC r1, r1, #0xC000000;//pin 13
		ORR r1, r1, #0x0;//input
		STR r1, [r0,#GPIO_MODER];
		
		LDR r1, [r0, #GPIO_PUPDR];
		BIC r1,r1, #0xC000000;// pin 13
		ORR r1,r1, #0x0; // set to no pullup, pulldown
		STR r1, [r0, #GPIO_PUPDR];
	
	; Set GPIOC pins 10,11,12,14 as output pins
	LDR r0,=GPIOC_BASE;//GPIOB
	
		LDR r1,[r0, #GPIO_MODER];
		BIC r1, #0xF00000;pins 10,11
		BIC r1, #0x30000000; pin 14
		BIC r1, #0x3000000; pin 12
		ORR r1, #0x500000;//set all pins to output
		ORR r1, #0x10000000
		ORR r1, #0x1000000
		STR r1, [r0,#GPIO_MODER];
	
		LDR r1,[r0,#GPIO_OTYPER];
		BIC	r1,r1, #0x5C00;//pins 10,11,12,14 push_pull
		STR r1, [r0,#GPIO_OTYPER];
		
		LDR r1, [r0, #GPIO_PUPDR];
		BIC r1, #0xF00000;pins 10,11
		BIC r1, #0x30000000; pin 14
		BIC r1, #0x3000000; pin 12
		ORR r1,r1, #0x0; // set to no pull-up pull-down
		STR r1, [r0, #GPIO_PUPDR];
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;;;;;;;;; YOUR CODE GOES HERE ;;;;;;;;;;;;;;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;		
open
	mov r8,#0; outer motor spin 
	ldr r7, =half_step
	bl button; only happens at the open and at the open of ever new window shield cycle
	bic r4, #0xFF; reset r4 to be used for loading
	b loop_clockwise
close
	;delay function goes here
	bl button
	b loop_counter_clockwise

loop_counter_clockwise
	mov r5, #0; i
	bl delay
	sub r8, #1
	cmp r8, #0
	beq open
	push {r8}; so we can reuse r8
	b spin_cww
	
spin_cww
	bl delay
	mov r6, #0
	ldrb r4, [r7,r5]; A - pc10
	and r4, #0x8
	lsl r4,#7
	orr r6,r4
	
	ldrb r4, [r7,r5]; ~A - pc11
	and r4, #0x4
	lsl r4,#9
	orr r6,r4
	
	ldrb r4, [r7,r5]; B - pc12
	and r4, #0x2
	lsl r4,#11
	orr r6,r4
	
	ldrb r4, [r7,r5]; ~B - pc14
	and r4, #0x1
	lsl r4,#14
	orr r6,r4
	
	bl delay
	LDR r8,=GPIOC_BASE;
	LDR r1,[r8, #GPIO_ODR];
	BIC r1, #0x5C00
	mov r1,r6
	STR r1,[r8,#GPIO_ODR]; update the inner motor
	
	add r5, #1; increment the spin sequence
	cmp r5, #8; check if we finished the sequence
	
	blt spin_cww; if not loop 
	bl delay
	pop {r8}
	b loop_counter_clockwise; if we did go back to bigger loop
	
loop_clockwise; loop counter_clockwise but in reverse instead of increment we decrement now to end up in the same spot
	mov r5, #7; i
	bl delay
	add r8,#1
	cmp r8,#270; 145 degree counter clockwise spin
	beq close; branch back to open
	push {r8}
	
spin_cw
	bl delay
	mov r6, #0
	ldrb r4, [r7,r5]; A - pc10
	and r4, #0x8
	lsl r4,#7
	orr r6,r4
	
	ldrb r4, [r7,r5]; ~A - pc11
	and r4, #0x4
	lsl r4,#9
	orr r6,r4
	
	ldrb r4, [r7,r5]; B - pc12
	and r4, #0x2
	lsl r4,#11
	orr r6,r4
	
	ldrb r4, [r7,r5]; ~B - pc14
	and r4, #0x1
	lsl r4,#14
	orr r6,r4
	
	bl delay
	LDR r8,=GPIOC_BASE;
	LDR r1,[r8, #GPIO_ODR];
	BIC r1, #0x5C00
	mov r1,r6
	STR r1,[r8,#GPIO_ODR]
	
	sub r5, #1
	cmp r5, #0
	bgt spin_cw
	bl delay
	pop {r8}
	b loop_clockwise
	
button
	push {r4,r5,lr}
press
	LDR r4,=GPIOC_BASE;//GPIOC
	LDR r5, [r4, #GPIO_IDR]
	bl delay
	cmp r5,#0x2000; if button is pressed
	beq press; loop until button is pressed
	pop {r4,r5,lr}
	bx lr
	
	
	ENDP		
	
delay proc
	push {r2}
	mov r2, #0x700
dl
	SUBS	r2, #1
	BNE	dl
	pop {r2}
	BX LR
	
	ENDP


	
	ALIGN			

	AREA myData, DATA, READWRITE
	ALIGN
; Replace ECE1770 with your last name
half_step DCB 0x9, 0x8, 0xa, 0x2, 0x6, 0x4, 0x5, 0x1
str DCB "Zheng, James",0
	END