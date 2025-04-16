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
	;				   Manual override flag stored in r6
	;				   Doors flag stored in r5
    ; 
    ; Subroutine Doc:
    ;     seven_Segment: Read current stop(r10) and displays it to seven segment display
    ;     If status flag(r8) is 1, turns off display(or switched it to "moving",etc.)
    ;     doors_motor: Turns motor clockwise/counterclk to indicate opening/closing
    ;     green_led: Turns on green led based off the status(r8) of the train, only turns on when moving
    ;     train_motor: Reads direction(r9) and turns the motor a full rotation

	;Registers used: r12, r11, r10,r9,r8,r7,r6, CANT USE r12
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	

	BL System_Clock_Init
	BL UART2_Init
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

	;considerations: push and pop CMP flag 
	CMP r6, #1                      ;if IGNORE flag(manual) is high, don't change status,movement, or seven-seg
									; only check at the beginning, we can't 

    BLNE seven_segment              ; branch to seven_segment sub to display current stop

    BLNE doors_motor                ; branch to open doors, only if stoppingb

	MOVNE r5, #1                    ; set flag for close doors

	BLNE long_delay                 ; branch to long delay

	BLNE doors_motor                ; branch to close doors,

	MOVNE r5, #0					; set flag for open doors

    MOVNE r8, #1                    ; set status to 1 to indicate we're about to start moving, only update if we are moving stop to stop

    BLNE seven_segment              ; call seven_segment again to change display

    BL green_led                    ; turn on green led to indicate movement, keep on

    BL train_motor                  ; move the train either forward or backward

    MOVNE r8, #0                    ; set status to 0 to indicate we've stopped moving

    BLNE green_led                  ; once train is done moving, turn off green led

    B automatic                      ; continue this loop indefinitely, manual override will be interrupt

 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PIN_INIT START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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

    pop{r0,r1,r2}                   ;load back in runtime

    BX LR                           ;branch back to main
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PIN_INIT END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;SEVEN_SEGMENT START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
seven_segment
	push {lr, r0, r1, r2, r3}
	cmp r10, #1                  	; see what station we are at
	beq stationA					; if r10 is 1, we are at station A	and branch accordinl
	cmp r10, #0x2                   
	beq stationB					; if r10 is 1 or 21(station B on return direction), we are station B
	cmp r10, #0x21
	beq stationB
	cmp r10, #0x3
	beq stationC					; if r10 is 3, we are station C
	b exit							; if r10 does not hold any of these values, exit 
	

stationA; when we get to station A, we will be moving the correct value to output A
	LDR r0,=GPIOB_BASE		   ; load GPIOB=output
	LDR r1, [r0, #GPIO_ODR]
	ORR r1, r1, #0x0010        ; set Pins 13(D) and Pin 4(B) high for 1010 to DCBA
	ORR r1, r1, #0x2000        ; set Pins 13(D) and Pin 4(B) high for 1010 to DCBA
	STR r1, [r0, #GPIO_ODR]	   ; store r1, which is our ASCII value into output
	b exit					   ; return 
            
stationB; when we get to station B, we will be moving the correct value to output B
	LDR r0,=GPIOB_BASE		   ; load GPIOB=output
	LDR r1, [r0, #GPIO_ODR]
	ORR r1, r1, #0x0010        ; set Pins 13(D), Pin 14(C) and Pin 4(B) high for 1011 to DCBA
	ORR r1, r1, #0x6000        ; set Pins 13(D), Pin 14(C) and Pin 4(B) high for 1011 to DCBA
	STR r1, [r0, #GPIO_ODR]	   ; store r1, which is our ASCII value into output
	b exit; return

stationC; when we get to station C, we will be moving the correct value to output C
	LDR r0,=GPIOB_BASE		   ; load GPIOB=output
	LDR r1, [r0, #GPIO_ODR]
	ORR r1, r1, #0x0020        ; set Pins 13(D) and Pin 5() high for 1100 to DCBA
	ORR r1, r1, #0x2000        ; set Pins 13(D) and Pin 5() high for 1100 to DCBA
	STR r1, [r0, #GPIO_ODR]	   ; store r1, which is our ASCII value into output
	b exit
	
exit 
	pop {lr, r0, r1, r2, r3}	; pop from seven_segment main
	BX lr
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;SEVEN_SEGMENT END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;TRAIN_MOTOR START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;README;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; MOTOR 1(TRAIN MOVEMENT)
	; Utilizing PORT C, PINS: PC9, PC8, PC6, PC5
	; Completes full 360 degree rotations clockwise and counterclockwise
    ; Clockwise = Forwards
    ; Counterclockwise = Reverse
    ; Do not need to check for input, if this function is been carried out then we are assuming it was done intentionly within main
	; Alter speed of rotation by changing delay
	; Must update station

	;IN ARGS: R9 = DIRECTION FLAG(1 = clockwise[forward], 0 = counterclockwise[reverse])
	;OUT ARGS: Nothing

	;Registers used: r0, r1, r2, r7, r8, r9, r10, r11
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	
train_motor
	
	PUSH{r5,r6,r7,r8,r11} 			;want to listen on r9(direction) and r10(stop), need r12 as it holds base address
	
	MOV r10, #215                   ;set max rotation counter before we enter subroutine. change as neccessary
	
	
	;need this for when we call again, if reg 11 is still 215 from last time, then we need to reset
	CMP r11, r10					; Check if the max number of rotations has occured
	MOVEQ r11, #0 					; Reset the rotation counter if we have reached 215 sequences
	
	;check to see what direction we have specified in main
	MOV r0, r9     					;load in flag for what direction to go    		
	CMP r0, #1              		;compare flag with 1
	BEQ full_step_cycle_forwards    ;if we have set flag to 1, then we want to initaite forward movement
	CMP r0, #0						;compare flag with 0
	BEQ full_step_cycle_reverse   	;if we have set flag to 0, initiate reverse movement
	
	;update train station
	CMP r10, #21					; if we are currently at station B on way back, move stop to beginning of array, station A
	LDREQ r10, [r12]				; load start of array back to r10, r10 will now be station A and points to beginning of stops array
	LDRNE r11, [r10, #4]!			; else increment our stops to next one, stop = stops[i + 1], r11 is a placeholder
	LDRNE r10, [r11]         
	
	POP{r5,r6,r7,r8,r11}	; pop back registers
	
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


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;TRAIN_MOTOR END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LED START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;README;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	; LED(MOVEMENT SIGNAL)
	; Utilizing PORT A, PINS: PA5
	; 
	;IN ARGS: status(moving = 1, stopped = 0)
	;OUT ARGS: Nothing

    ;REGISTERS USED: r3,r4,r8
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

green_led

	PUSH {r3,r4}  ;push everything except r8(status)

    LDR r3, =GPIOA_BASE             ;load in port A
    LDR r4, [r3, #GPIO_ODR]
	
    CMP r8, #1                      ;check to see what our status(r8) is
    MOVEQ r4, #0x00000020            ;if our status is moving set ODR high(green light)
	MOVNE r4, #0x00000000			;if our status is not moving, set ODR low
    STR r4, [r3, #GPIO_ODR]         ;store back result regardless
	
	POP {r3,r4}   ;pop back registers

    BX LR
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LED END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;DELAY START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
delay
	; Delay for software debouncing
	LDR	r2, =0xE10
delayloop
	SUBS	r2, #1
	BNE	delayloop
	BX LR
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;DELAY END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;DOOR_MOTOR START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
	
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;DOOR_MOTOR END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;KEYPAD START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
keypad
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
	bl check_code1
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
		b keypad

	ENDP		

check_code1
	CMP r7, #1                   ;r7 is true flag, if 9 has been pressed we add to r7,
	BEQ check_code2				 ;if r7 = 1, 9 has been pressed and we should now check for 1
	CMP r7, #2						
	BEQ check_code3				 ;if r7 = 2, 9, 1 have been pressed and we should now check for the last 1
	PUSH{r1}
	LDR r1, [r5]                   
	CMP r1, #57
	MOVEQ r7, #1                 ;if 9 pressed, set r7 to 1
	BEQ keypad                   ; if 9 is pressed, branch back to keypad to check again
	BNE reset                    ;if not equal, exit

check_code2                      
	LDR, r1, [r5]                ;load in asci value
	CMP r1, #49					 ;if keypad press is 1, set r7 to 0 and branch back to keypad
	MOVEQ r7, #2
	BEQ keypad                   ;branch
	BNE reset					 ;if 1 was not pressed, reset

check_code3
	LDR, r1, [r5]
	CMP r1, #49
	BEQ emergency_stop           ;if 911 pressed, branch to emergency stop 
	MOVEQ r7, #0				 ;reset r7 back to 0
	BNE reset

emergency_stop
	;Print to terra term that we have stopped
	;turn off LED, turn off seven segment
	;turn off motors
End b End
	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;KEYPAD END;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;TERRA_TERM START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

printAccel
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =accel			; Load text
	MOV r1, #15    			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printDecel
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =decel			; Load text
	MOV r1, #15    			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printEmerg
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =emerg			; Load text
	MOV r1, #17  			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printAtStopA
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =atStopA		; Load text
	MOV r1, #12 			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printAtStopB
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =atStopB		; Load text
	MOV r1, #12 			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printAtStopC
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =atStopC		; Load text
	MOV r1, #12 			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printGoingToA
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =goingToStopA	; Load text
	MOV r1, #18				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printGoingToB
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =goingToStopB	; Load text
	MOV r1, #18				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch
	
printGoingToC
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =goingToStopC	; Load text
	MOV r1, #18				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printManualOverride
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =manualOverride	; Load text
	MOV r1, #18				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printEndOverride
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =endOverride	; Load text
	MOV r1, #17				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printDoorOpen
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =doorOpen	; Load text
	MOV r1, #15				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch

printDoorClose
	PUSH{r0, r1, LR}		; Push to stack
	LDR r0, =doorClose	; Load text
	MOV r1, #15				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, LR}			; Pop from stack
	BX LR					; Return from branch


stop b stop
; Replace ECE1770 with your last name

; Define stops as globals, 1 = A, 2 = B, 3 = C
stops DCD 1, 2, 3, 21                  ;defines movement stop 1, then 2, then 3, then 2(use 21 to distinguish), then restart at 1

str DCB "ECE1770",0
accel	DCB "Accelerating!\r\n", 0
decel	DCB "Decelerating!\r\n", 0
emerg	DCB "Emergency Stop!\r\n", 0
atStopA	DCB "At stop A!\r\n", 0
atStopB	DCB "At stop B!\r\n", 0
atStopC	DCB "At stop C!\r\n", 0
goingToStopA	DCB "Going to stop A!\r\n", 0
goingToStopB	DCB "Going to stop B!\r\n", 0
goingToStopC	DCB "Going to stop C!\r\n", 0
manualOverride	DCB "Manual Override!\r\n", 0
endOverride	DCB "Resuming route!\r\n", 0
doorOpen	DCB "Opening Door!\r\n", 0
doorClose	DCB "Closing Door!\r\n", 0


	END

