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
    ; Do not need to check for input, if this function is been carried out then we are assuming it was done intentionly within main
	; Alter speed of rotation by changing delay
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	; Enable Port C clocks
	LDR r0, =RCC_BASE      			;load in base module
	LDR r1, [r0, #RCC_AHB2ENR]     	;load in clock module
	ORR r1, r1, #0x00000006         ;set clocks for Port C high
	STR r1, [r0, #RCC_AHB2ENR]     	;store result back to clock reg

	;set the mode of PC13 for input
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
	
	
	; Initialize all of the outputs to zero
	LDR r0, =GPIOC_BASE
	LDR r1, [r0, #GPIO_ODR]
	BIC r1, r1, #0x00005C00   		;clear bits 14,12,11,10
	ORR r1, r1, #0x00000000
	STR r1, [r0, #GPIO_ODR]
	
	
main_loop

	MOV r10, #215                   ;set max rotation counter before we enter subroutine. change as neccessary
	
	;error check: if reg 11 is still 215 from last time, then we need to reset
	CMP r11, r10					; Check if the max number of rotations has occured
	MOVEQ r11, #0 					; Reset the rotation counter if we have reached 215 sequences
	
	B full_step_init                ; Initiate open and close of dorr
	
	;cleanup, set everything to 0 for safety
	MOV r11, #0
	MOV r10, #0
	MOV r0,  #0
	MOV r1,  #0
	MOV r2,  #0
	MOV r9,  #0
    MOV r7,  #0
    MOV r8,  #0

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