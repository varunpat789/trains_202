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
	; LED(MOVEMENT SIGNAL)
	; Utilizing PORT A, PINS: PA5
	; 
	;IN ARGS: status(moving = 1, stopped = 0)
	;OUT ARGS: Nothing

    ;REGISTERS USED: r1,r2,r3,r4,r5
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	; Enable Port A clocks
	LDR r1, =RCC_BASE      			;load in base module
	LDR r2, [r1, #RCC_AHB2ENR]     	;load in clock module
	ORR r2, r2, #0x00000001         ;set clock for Port A high
	STR r2, [r1, #RCC_AHB2ENR]     	;store result back to clock reg

	;set the mode of PA5 for output
	LDR r1, =GPIOA_BASE        		;load in base module
	LDR r2, [r1, #GPIO_MODER]      	;load moad register
	BIC r2, r2, #0x00000C00         ;clear bits for PA5, 10-11
	ORR r2, r2, #0x08000C00    		;set 01 for pin 5 for output
	STR r1, [r0,#GPIO_MODER]       	;store result back to moder
	
	;set the pupdr of PA5
	LDR r1, =GPIOA_BASE        		;load in base module
	LDR r2, [r1, #GPIO_PUPDR]      	;load moad register
	BIC r2, r2, #0x00000C00         ;clear bits for PA5
	ORR r2, r2, #0x08000C00    		;set 01 for pin 5 for PU/PD
	STR r2, [r1,#GPIO_PUPDR]       	;store result back to pupdr
	
	
	; Initialize all of the outputs to zero
	LDR r1, =GPIOA_BASE
	LDR r2, [r1, #GPIO_ODR]
	BIC r2, r2, #0x00000020   		`;clear bits 5
	ORR r2, r2, #0x00000000
	STR r2, [r1, #GPIO_ODR]
	
    BL main_loop                     ;branch to main functionality

    ;cleanup
	MOV r1,  #0
	MOV r2,  #0
	MOV r3,  #0
    MOV r4,  #0
    MOV r5,  #0

    BX LR
	

main_loop 

    LDR r3, =GPIOA_BASE             ;load in port A
    LDR r4, [r3, #GPIO_ODR]

    LDR r5, =STATUS                 ;placeholder, status of train
    LDR r5, [=STATUS]
    CMP r5, #1                      ;check to see what our status is
    MOVEQ r4, 0x00000020            ;if our status is moving set ODR high(green light)
	MOVNE r4, 0x00000000			;if our status is not moving, set ODR low
    STR r4, [r3, #GPIO_ODR]         ;store back result regardless

    BXLR

@ long_delay
@     push{LR}

@     ;just call delay 200 times to act as longer delay
@     MOV r8, #200
@     MOV r7, #0
@     BL delay
@     ADD r7, #1
@     CMP r7, r8

@     pop{LR}
@     BXEQ LR
@     B long_delay   ;loop until r7 = 200

@ delay	PROC
@ 	; Delay for software debouncing
@ 	LDR	r2, =0xE10
@ delayloop
@ 	SUBS	r2, #1
@ 	BNE	delayloop
@ 	BX LR
	
 	ENDP
	
 	ALIGN			

 	AREA myData, DATA, READWRITE
 	ALIGN
; Replace ECE1770 with your last name
str DCB "ECE1770",0
char1	DCD	43

	END