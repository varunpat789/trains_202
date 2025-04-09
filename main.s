@ Base Function(Automatic):
@ Display stop to seven segment
@ Open door/close door
@ Turn off seven segment
@ Start moving motor full rotation
@ Display LED on
@ Stop motor rotation
@ Turn off LED
@ Turn on seven segment
@ Open door/close door
@ Manual Override:
@ Keypad pressed -> interrupt service routine. Keypad 1 = Stop A; Keypad 2 = Stop B; Keypad 3 = Stop C
@ ISR = Same as regular function, just moved to the specified stop. 
@ Move back to automatic functioning





	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	
	AREA    main, CODE, READONLY
	EXPORT	__main				; make __main visible to linker
	ENTRY			
				
__main	PROC

	;test commit

    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;README;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; MAIN 
	; Utilizing PORT A, B, C
    ; PORT B: Input for Keypad, Ouput for Seven Segment
    ; PORT C: Output for Keypad, Output for Motor1, Output for Motor2
	; GLOBALS & FLAGS: Stops(A-C) defined as data(DCD), current stop stored in r12/r11/r10
    ;                  r10 stores the current stop, r11 and r12 are used for proper train movement
    ;                  Direction stored in r9(1 = forward, 0 reverse)
    ;                  Status stored in r8(1 = moving, 0 = stopped)
    ;                  Count stored in r7(increments by 4 to access stop addresses)
    ; 
    ; Subroutine Doc:
    ;     seven_Segment: Read current stop(r10) and displays it to seven segment display
    ;     If status flag(r8) is 1, turns off display(or switched it to "moving",etc.)
    ;     doors_motor: Turns motor clockwise/counterclk to indicate opening/closing
    ;     green_led: Turns on green led based off the status(r8) of the train, only turns on when moving
    ;     train_motor: Reads direction(r9) and turns the motor a full rotation

	;Registers used: r12, r11, r10
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    ; PIN Intialization
    BL pin_init

    ; Define intial conditions
    
    ; Start from stop 1(A)
    LDR r12, =stops             ;load in address of data
    LDR r11, [r12]              ;load in value, stop 1

    ;Set Direction as forward
    MOV r9, #1                 ; 1 == forward, 0 == reverse

    ;Set Status as stopped
    MOV r8, #0                 ; 1 = moving, 0 = stopped

    ; Set count variable to ensure proper automatic movement
    MOV r7, #0  

    BL automatic                ; subroutine to carry out automatic funcitonality








automatic

    ; automatic functionining of train


    BL seven_segment                ; branch to seven_segment sub to display current stop

    BL doors_motor                  ; branch to open/close doors

    MOV r8, #1                      ; set status to 1 to indicate we're about to start moving

    BL seven_segment                ; call seven_segment again to change display

    BL green_led                    ; turn on green led to indicate movement

    BL train_motor                  ; move the train either forward or backward

    MOV r8, #0                      ; set status to 0 to indicate we've stopped moving

    BL green_led                    ; once train is done moving, turn off green led

    ; maybe change seven_segment here but shouldn't have to since we do at beginning of loop

    ADD r7, #4                      ; increment our count

    ; if we have reached count of 16(4), reset count to 0 and point current stop to start of array(stop 1)
    CMP r7, #16                     ; compare count with 16(4)
    MOVEQ r7, #0                    ; if equal reset count


    CMP r7, #12                     ; compare count with 12(3) to set r11 appropriately
    LDR r10, [r11, r7]!             ; load in data and iterate, have to use r11 so r12 is not modified
    LDREQ r11, [r12]                ; if equal point r11 back to original starting position so we "restart", else r11 will point to bad data

    ;psuedo for above
    @ for(i=0;i<=length(stops);i++){
    @     current_stop = stop
    @     stop = stops[i+1]
    @     if(i == length(stops)){
    @         stop = stops[0]
    @     }
    @ }

    B automatic                      ; continue this loop indefinitely, manual override will be interrupt

 
    
pin_init
    
    push{r0,r1,r2}                  ;preserve runtime in case
    
    ; Enable Port C clocks
	LDR r0, =RCC_BASE      			;load in base module
	LDR r1, [r0, #RCC_AHB2ENR]     	;load in clock module
	ORR r1, r1, #0x00000006         ;set clocks for Port B C high
	STR r1, [r0, #RCC_AHB2ENR]     	;store result back to clock reg

	; TRAIN MOTOR
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


    ;DOOR MOTOR
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

    
    ; GREEN LED
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

    pop{ro,r1,r2}                   ;load back in runtime

    BX LR                           ;branch back to main


; Replace ECE1770 with your last name

; Define stops as globals, 1 = A, 2 = B, 3 = C
stops DCD 1, 2, 3, 2                  ;defines movement stop 1, then 2, then 3, then 2, then restart at 1

str DCB "ECE1770",0
char1	DCD	43

	END

