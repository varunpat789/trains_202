	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	IMPORT	pin_init
	IMPORT keypad_Init
	IMPORT	printAccel
	IMPORT printDecel
	IMPORT printEmerg
	IMPORT printAtStopA
	IMPORT printAtStopB
	IMPORT printAtStopC
	IMPORT printGoingToA
	IMPORT printGoingToB
	IMPORT printGoingToC
	IMPORT printManualOverride
	IMPORT printEndOverride
	IMPORT printDoorOpen
	IMPORT printDoorClose
	IMPORT print_over1
	IMPORT print_over2
	IMPORT print_over3
		
	EXPORT EXTI15_10_IRQHandler
		
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

    ;BLNE seven_segment              ; branch to seven_segment sub to display current stop

    BLNE open               		 ; branch to open doors, only if stoppingb

	MOVNE r5, #1                    ; set flag for close doors

	BLNE close               		 ; branch to close doors,

	MOVNE r5, #0					; set flag for open doors

    MOVNE r8, #1                    ; set status to 1 to indicate we're about to start moving, only update if we are moving stop to stop

    ;BLNE seven_segment              ; call seven_segment again to change display

    BL green_led                    ; turn on green led to indicate movement, keep on

    BL train_motor                  ; move the train either forward or backward

    MOVNE r8, #0                    ; set status to 0 to indicate we've stopped moving

    BLNE green_led                  ; once train is done moving, turn off green led

    B automatic                      ; continue this loop indefinitely, manual override will be interrupt

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
	
	LTORG
	
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


stop b stop
; Replace ECE1770 with your last name

; Define stops as globals, 1 = A, 2 = B, 3 = C
stops DCD 1, 2, 3, 21                  ;defines movement stop 1, then 2, then 3, then 2(use 21 to distinguish), then restart at 1

str DCB "ECE1770",0

half_step DCB 0x9, 0x8, 0xa, 0x2, 0x6, 0x4, 0x5, 0x1
	ENDP

EXTI15_10_IRQHandler PROC
    PUSH {r4-r11, lr}
	
	bl printManualOverride
	LDR r6, =GPIOA_BASE; turn on green led
    LDR r7, [r6, #GPIO_ODR]
    EOR r7, r7, #(1 << 5)
    STR r7, [r6, #GPIO_ODR]
	
    BL keypad_Init; initialize keypad pins 

    ; Clear EXTI13 interrupt pending bit
    LDR r4, =EXTI_BASE
    MOV r5, #(1 << 13)
    STR r5, [r4, #EXTI_PR1]

    ; Toggle PA5
    LDR r6, =GPIOA_BASE; turn off green led
    LDR r7, [r6, #GPIO_ODR]
    EOR r7, r7, #(1 << 5)
    STR r7, [r6, #GPIO_ODR]
	
	
    POP {r4-r11, lr}
	
	PUSH {lr}

	cmp r0, #49
	moveq r1, #1
	beq o1
	b skip1
o1
	bl print_over1
	b next
skip1
	cmp r0, #50
	moveq r1, #2
	beq o2
	b skip2
o2
	bl print_over2
	b next
skip2
	cmp r0, #51
	moveq r1, #3
	beq o3
	b skip3
o3
	bl print_over3
	b next
skip3
	
next
	bl printEndOverride
	POP{lr}
	
    BX lr
    ENDP
		
	END