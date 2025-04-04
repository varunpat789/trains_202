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
	; MOTOR 1(TRAIN MOVEMENT)
	; Utilizing PORT C, PINS: PC9, PC8, PC6, PC5
	; Completes full 360 degree rotations clockwise and counterclockwise
    ; Clockwise = Forwards
    ; Counterclockwise = Reverse
    ; Do not need to check for input, if this function is been carried out then we are assuming it was done intentionly within main
	; Alter speed of rotation by changing delay

	;IN ARGS: R0 = DIRECTION FLAG(1 = clockwise[forward], 0 = counterclockwise[reverse])
	;OUT ARGS: Nothing, possible r0 = num rotations

	;Registers used: r0, r1, r2, r7, r8, r9, r10, r11
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	; Enable Port C clocks
	LDR r0, =RCC_BASE      			;load in base module
	LDR r1, [r0, #RCC_AHB2ENR]     	;load in clock module
	ORR r1, r1, #0x00000006         ;set clocks for Port B C high
	STR r1, [r0, #RCC_AHB2ENR]     	;store result back to clock reg

	;set the mode of PC for input
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
	
	
	; Initialize all of the outputs to zero
	LDR r0, =GPIOC_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x00000360   		;clear bits 9,8,6,5
	ORR r1, r1, #0x00000000
	STR r1, [r0, #GPIO_ODR]

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

	BX LR
	
	
main_loop
	
	MOV r10, #215                   ;set max rotation counter before we enter subroutine. change as neccessary
	
	
	;need this for when we call again, if reg 11 is still 215 from last time, then we need to reset
	CMP r11, r10					; Check if the max number of rotations has occured
	MOVEQ r11, #0 					; Reset the rotation counter if we have reached 215 sequences
	
	;check to see what direction we have specified in main
	LDR r0, =DIRECTION      		;placeholder for now, but flag for what direction to go
	LDR r0, [=DIRECTION]    		
	CMP r0, #1              		;compare flag with 1
	BEQ full_step_cycle_forwards    ;if we have set flag to 1, then we want to initaite forward movement
	CMP r0, #0						;compare flag with 0
	BEQ full_step_cycle_forwards    ;if we have set flag to 0, initiate reverse movement
	
	;cleanup, set everything to 0 for safety
	MOV r11, #0
	MOV r10, #0
	MOV r0,  #0
	MOV r1,  #0
	MOV r2,  #0
	MOV r9,  #0
	
	BX LR                            ;exit once rotation is done
	

full_step_cycle_forwards
	push{LR}					; Push the link register to the stack
	
	MOV r11, #0					; Initialize current rotation counter
	
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x000000CC
	ORR r1, r1, #0x00000084
	STR r1, [r0, #GPIO_ODR]
	
	BL delay

	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x000000CC
	ORR r1, r1, #0x00000044	
	STR r1, [r0, #GPIO_ODR]
	
	BL delay

	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x000000CC
	ORR r1, r1, #0x00000048
	STR r1, [r0, #GPIO_ODR]
	
	BL delay

	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x000000CC
	ORR r1, r1, #0x00000088
	STR r1, [r0, #GPIO_ODR]
	
	BL delay
	
	POP{LR}						; Pop the link register from the stack

	ADD r11, r11, #1			; Increase the rotation counter by 1
	CMP r11, r10				; Check if the max number of rotations has occured
	BXGT LR						; If so, return to the main function
	
	B full_step_cycle_forwards	; Else, repeat
	
full_step_cycle_reverse
	push{LR}					; Push the link register to the stack
	
	MOV r11, #0					; Initialize current rotation counter
	
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x000000CC
	ORR r1, r1, #0x00000088
	STR r1, [r0, #GPIO_ODR]
	
	BL delay
	
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x000000CC
	ORR r1, r1, #0x00000048
	STR r1, [r0, #GPIO_ODR]
	
	BL delay
	
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x000000CC
	ORR r1, r1, #0x00000044	
	STR r1, [r0, #GPIO_ODR]
	
	BL delay
	
	LDR r0, =GPIOB_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x000000CC
	ORR r1, r1, #0x00000084
	STR r1, [r0, #GPIO_ODR]
	
	BL delay
	
	POP{LR}						; Pop the link register from the stack
	
	ADD r11, r11, #1			; Increase the rotation counter by 1
	CMP r11, r10				; Check if the max number of rotations has occured
	BXGT LR						; If so, return to the main function
	
	B full_step_cycle_reverse	; Else, repeat

delay	PROC
	; Delay for software debouncing
	LDR	r2, =0xE10
delayloop
	SUBS	r2, #1
	BNE	delayloop
	BX LR

	ENDP
	
	ALIGN			

	AREA myData, DATA, READWRITE
	ALIGN
; Replace ECE1770 with your last name
str DCB "ECE1770",0
char1	DCD	43

	END