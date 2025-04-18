LDR r0, =EXTI_BASE
	LDR r1, [r0, #EXTI_IMR1]; mask to disasble IRQ handler
	BIC r1, #0x2000; 
	STR r1, [r0, #EXTI_IMR1]

LDR r0, =EXTI_BASE
	LDR r1, [r0, #EXTI_IMR1]; unmask to trigger IRQ handler
	ORR r1, #0x2000; 
	STR r1, [r0, #EXTI_IMR1]
