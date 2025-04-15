	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	
	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY			
				
__main	PROC
	
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;README;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; MOTOR 2(DOOR MOVEMENT)
	; Utilizing PORT C, PINS: PC10, PC11, PC12, PC14
	; Completes full 180 degree rotations clockwise and counterclockwise
    ; Clockwise = Opening
    ; CounterClockwise = Closing
	; Closing and opening sequence happen in tandem, not seperately
	; Alter speed of rotation by changing delay

	;IN ARGS: Nothing
	;OUT ARGS: Nothing

	;Registers used: r0, r1, r2, r7, r8, r9, r10, r11
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	PUSH{r7,r8,r9,r10,r11,r12}

	BL main_loop

	;cleanup, set everything to 0 for safety
	MOV r11, #0
	MOV r10, #0
	MOV r0,  #0
	MOV r1,  #0
	MOV r2,  #0
	MOV r9,  #0
    MOV r7,  #0
    MOV r8,  #0

	POP{r7,r8,r9,r10,r11,r12}

	BX LR
	
	
main_loop

	MOV r10, #215                   ;set max rotation counter before we enter subroutine. change as neccessary
	
	;error check: if reg 11 is still 215 from last time, then we need to reset
	CMP r11, r10					; Check if the max number of rotations has occured
	MOVEQ r11, #0 					; Reset the rotation counter if we have reached 215 sequences
	
	B start                ; Initiate open and close of dorr

    BX LR                            ;branch back to main once done sequence


; main
	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	
	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY			
				
__main	PROC
	
	;	Enable clocks for GPIOC, GPIOB//;	Enable clocks for GPIOA, GPIOB
	LDR r0, =RCC_BASE; // load RCC module to r0
		
		LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
		ORR r1,r1, #0x2; // enable GPIOB
		STR r1, [r0,#RCC_AHB2ENR];
		
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
	
	; Set GPIOB pins 2, 3, 6, 7 as output pins
	LDR r0,=GPIOB_BASE;//GPIOB
	
		LDR r1,[r0, #GPIO_MODER];
		BIC r1, r1, #0xF0;//pins 2,3
		BIC r1, r1, #0xF000;//pins 6,7
		ORR r1, r1, #0x50;//set all pins to output
		ORR r1, r1, #0x5000;//set all pins to output
		STR r1, [r0,#GPIO_MODER];
	
		LDR r1,[r0,#GPIO_OTYPER];
		BIC	r1,r1, #0xCC;//pins 2,3,6,7 push_pull
		STR r1, [r0,#GPIO_OTYPER];
		
		LDR r1, [r0, #GPIO_PUPDR];
		BIC r1, r1, #0xF0;//pins 2,3,6,7
		BIC r1, r1, #0xF000;//pins 2,3,6,7
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
	b loop_counter_clockwise

loop_counter_clockwise
	mov r5, #0; i
	bl delay
	sub r8, #1
	cmp r8, #0
	beq open
	push {r8}; so we can reuse r8
	b spin_ccw
	
spin_ccw
	bl delay
	mov r6, #0
	ldrb r4, [r7,r5]; A - pb2
	and r4, #0x8
	lsr r4,#1
	orr r6,r4
	
	ldrb r4, [r7,r5]; ~A - pb3
	and r4, #0x4
	lsl r4,#1
	orr r6,r4
	
	ldrb r4, [r7,r5]; B - pb6
	and r4, #0x2
	lsl r4,#5
	orr r6,r4
	
	ldrb r4, [r7,r5]; ~B - pb7
	and r4, #0x1
	lsl r4,#7
	orr r6,r4
	
	bl delay
	LDR r8,=GPIOB_BASE;//GPIOB
	LDR r1,[r8, #GPIO_ODR];
	BIC r1, #0xCC
	mov r1,r6
	STR r1,[r8,#GPIO_ODR]; update the inner motor
	
	add r5, #1; increment the spin sequence
	cmp r5, #8; check if we finished the sequence
	
	blt spin_ccw; if not loop 
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
	
spin_cc
	bl delay
	mov r6, #0
	ldrb r4, [r7,r5]; A - pb2
	and r4, #0x8
	lsr r4,#1
	orr r6,r4
	
	ldrb r4, [r7,r5]; ~A - pb3
	and r4, #0x4
	lsl r4,#1
	orr r6,r4
	
	ldrb r4, [r7,r5]; B - pb6
	and r4, #0x2
	lsl r4,#5
	orr r6,r4
	
	ldrb r4, [r7,r5]; ~B - pb7
	and r4, #0x1
	lsl r4,#7
	orr r6,r4
	
	bl delay
	LDR r8,=GPIOB_BASE;//GPIOB
	LDR r1,[r8, #GPIO_ODR];
	BIC r1, #0xCC
	mov r1,r6
	STR r1,[r8,#GPIO_ODR]
	
	sub r5, #1
	cmp r5, #0
	bgt spin_cc
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
	mov r2, #0x200
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