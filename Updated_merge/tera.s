	INCLUDE core_cm4_constants.s		; Load Constant Definitions
	INCLUDE stm32l476xx_constants.s      

	IMPORT 	System_Clock_Init
	IMPORT 	UART2_Init
	IMPORT	USART2_Write
	
	AREA    main, CODE, READONLY
	EXPORT	printAccel
	EXPORT printChug
	EXPORT printDecel
	EXPORT printEmerg
	EXPORT printAtStopA
	EXPORT printAtStopB
	EXPORT printAtStopC
	EXPORT printGoingToA
	EXPORT printGoingToB
	EXPORT printGoingToC
	EXPORT printManualOverride
	EXPORT printEndOverride
	EXPORT printDoorOpen
	EXPORT printDoorClose
	EXPORT print_over1
	EXPORT print_over2
	EXPORT print_over3
	
	ENTRY			
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;TERRA_TERM START;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
printAccel
	PUSH{r0, r1,r12, LR}		; Push to stack
	LDR r0, =accel			; Load text
	MOV r1, #15    			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1,r12, LR}			; Pop from stack
	BX LR					; Return from branch
	
printChug
	PUSH{r0, r1,r12, LR}		; Push to stack
	LDR r0, =chug			; Load text
	MOV r1, #18    			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1,r12, LR}			; Pop from stack
	BX LR					; Return from branch

printDecel
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =decel			; Load text
	MOV r1, #15    			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printEmerg
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =emerg			; Load text
	MOV r1, #17  			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printAtStopA
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =atStopA		; Load text
	MOV r1, #19 			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printAtStopB
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =atStopB		; Load text
	MOV r1, #19 			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printAtStopC
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =atStopC		; Load text
	MOV r1, #19 			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printGoingToA
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =goingToStopA	; Load text
	MOV r1, #18				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printGoingToB
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =goingToStopB	; Load text
	MOV r1, #18				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch
	
printGoingToC
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =goingToStopC	; Load text
	MOV r1, #18				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printManualOverride
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =manualOverride	; Load text
	MOV r1, #18				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printEndOverride
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =endOverride	; Load text
	MOV r1, #17				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printDoorOpen
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =doorOpen	; Load text
	MOV r1, #15				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch

printDoorClose
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =doorClose	; Load text
	MOV r1, #15				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch
print_over1
	PUSH{r0, r1,r12, LR}		; Push to stack
	LDR r0, =over1	; Load text
	MOV r1, #33			; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12,LR}			; Pop from stack
	BX LR					; Return from branch
print_over2
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =over2	; Load text
	MOV r1, #33				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch
print_over3
	PUSH{r0, r1, r12, LR}		; Push to stack
	LDR r0, =over3	; Load text
	MOV r1, #33				; Load length of text
	BL USART2_Write			; Branch to C method to write to TeraTerm
	POP{r0, r1, r12, LR}			; Pop from stack
	BX LR					; Return from branch
	
accel	DCB "Accelerating!\r\n", 0
chug	DCB "Chugga Chugga...\r\n", 0
decel	DCB "Decelerating!\r\n", 0
emerg	DCB "Emergency Stop!\r\n", 0
atStopA	DCB "Moving to stop A!\r\n", 0
atStopB	DCB "Moving to stop B!\r\n", 0
atStopC	DCB "Moving to stop C!\r\n", 0
goingToStopA	DCB "Going to stop A!\r\n", 0
goingToStopB	DCB "Going to stop B!\r\n", 0
goingToStopC	DCB "Going to stop C!\r\n", 0
manualOverride	DCB "Manual Override!\r\n", 0
endOverride	DCB "Resuming route!\r\n", 0
over1 DCB "Override: Next Stop station A!\r\n", 0
over2 DCB "Override: Next Stop station B!\r\n", 0
over3 DCB "Override: Next Stop station C!\r\n", 0
doorOpen	DCB "Opening Door!\r\n", 0
doorClose	DCB "Closing Door!\r\n", 0
	
	END