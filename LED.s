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
	; Utilizing PORT C, PINS: PC13
	; 
	;IN ARGS: status(moving = 1, stopped = 0)
	;OUT ARGS: Nothing

    ;REGISTERS USED: r1,r2,r3,r4,r5
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	; Enable Port C clocks
	LDR r1, =RCC_BASE      			;load in base module
	LDR r2, [r1, #RCC_AHB2ENR]     	;load in clock module
	ORR r2, r2, #0x00000006         ;set clocks for Port C high
	STR r2, [r1, #RCC_AHB2ENR]     	;store result back to clock reg

	;set the mode of PC13 for input
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
	
	
	; Initialize all of the outputs to zero
	LDR r1, =GPIOC_BASE
	LDR r2, [r1, #GPIO_ODR]
	BIC r2, r2, #0x00002000   		;clear bits 13
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

    LDR r3, =GPIOC_BASE             ;load in port C
    LDR r4, [r3, #GPIO_ODR]

    LDR r5, =STATUS                 ;placeholder, status of train
    LDR r5, [=STATUS]
    CMP r5, #1                      ;check to see what our status is
    MOVEQ r4, 0x00002000            ;if our status is moving set ODR high(green light)
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