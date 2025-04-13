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
	
	B full_step_init                ; Initiate open and close of dorr

    BX LR                            ;branch back to main once done sequence


full_step_init
	MOV r11, #0					; Initialize current rotation counter
	BL full_step_cycle_forwards ; Enter forwards loop, opening

    BL long_delay               ; allow time for doors to remain open

	MOV r11, #0					; Reset current rotation counter
	BL full_step_cycle_reverse  ; Enter reverse loop, closing

	B main_loop

full_step_cycle_forwards
	push{LR}					; Push the link register to the stack
		
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
	
	MOV r9, #0
	
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

long_delay
    push{LR}

    ;just call delay 200 times to act as longer delay
    MOV r8, #200
    MOV r7, #0
    BL delay
    ADD r7, #1
    CMP r7, r8

    pop{LR}
    BXEQ LR
    B long_delay   ;loop until r7 = 200

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