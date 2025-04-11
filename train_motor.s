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
	
	PUSH{r7,r8,r10,r11,r12} //want to listen on r9(direction)

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

	POP{r7,r8,r10,r11,r12}

	BX LR
	
	
main_loop
	
	MOV r10, #215                   ;set max rotation counter before we enter subroutine. change as neccessary
	
	
	;need this for when we call again, if reg 11 is still 215 from last time, then we need to reset
	CMP r11, r10					; Check if the max number of rotations has occured
	MOVEQ r11, #0 					; Reset the rotation counter if we have reached 215 sequences
	
	;check to see what direction we have specified in main
	MOV r0, r9     					;load in flag for what direction to go    		
	CMP r0, #1              		;compare flag with 1
	BEQ full_step_cycle_forwards    ;if we have set flag to 1, then we want to initaite forward movement
	CMP r0, #0						;compare flag with 0
	BEQ full_step_cycle_forwards    ;if we have set flag to 0, initiate reverse movement
	
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