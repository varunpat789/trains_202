;******************** (C) Yifeng ZHU ******************************************* 
; @file    main.s
; @author  Yifeng Zhu
; @date    May-17-2015
; @note
;           This code is for the book "Embedded Systems with ARM Cortex-M
;           Microcontrollers in Assembly Language and C, Yifeng Zhu,
;           ISBN-13: 978-0982692639, ISBN-10: 0982692633
; @attension
;           This code is provided for education purpose. The author shall not be
;           held liable for any direct, indirect or consequential damages, for any
;           reason whatever. More information can be found from book website:
;           http:;www.eece.maine.edu/~zhu/book
;*******************************************************************************


      INCLUDE core_cm4_constants.s        ; Load Constant Definitions
      INCLUDE stm32l476xx_constants.s      

      IMPORT      System_Clock_Init
      IMPORT      UART2_Init
      IMPORT      USART2_Write
      
      AREA    main, CODE, READONLY
      EXPORT      __main                        ; make __main visible to linker
      ENTRY             
      
__main      PROC

      LDR r0, =RCC_BASE; // load RCC module to r0
      
            LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
            ORR r1,r1, #0x2; // enable GPIOB
            STR r1, [r0,#RCC_AHB2ENR];
            
            LDR r1, [r0,#RCC_AHB2ENR]; // load AHB2ENR value to r1
            ORR r1,r1, #0x4; // enable GPIOC
            STR r1, [r0,#RCC_AHB2ENR];

      LDR r0,=GPIOC_BASE;//GPIOC=input
      
            LDR r1,[r0, #GPIO_MODER];
            BIC r1, r1, #0xC000000;
            ORR r1, r1, #0x0;
            STR r1, [r0,#GPIO_MODER];
            
            LDR r1, [r0, #GPIO_PUPDR];
            BIC r1,r1, #0xC000000;
            ORR r1,r1, #0x0; // set to no pull-up pull-down
            STR r1, [r0, #GPIO_PUPDR];
            
      LDR r0,=GPIOB_BASE;//GPIOB=output
      
            LDR r1,[r0, #GPIO_MODER];
            BIC r1, r1, #0xF00;// clear pins 4,5
            BIC r1, r1, #0x3C000000;// clear pins 13,14
            ORR r1, r1, #0x500;//set all pins to output
            ORR r1, r1, #0x18000000;//set all pins to output
            STR r1, [r0,#GPIO_MODER];
      
            LDR r1,[r0,#GPIO_OTYPER];
            BIC r1,r1, #0x6030;//clear pins 4,5,13,14
            ORR r1,r1, #0x0000;//set to push-pull
            STR r1, [r0,#GPIO_OTYPER];
            
            LDR r1, [r0, #GPIO_PUPDR];
            BIC r1, r1, #0xF00;// clear pins 4,5
            BIC r1, r1, #0x3C000000;// clear pins 13,14
            ORR r1,r1, #0x0000000; // set to no pull-up pull-down
            STR r1, [r0, #GPIO_PUPDR];
            
            LDR r1, [r0,#GPIO_ODR];
            BIC r1,r1, #0xFFFFFFFF;//clear pins 14-0
            b seven_segment

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
            
stop  B           stop              ; dead loop & program hangs here
            
      ENDP
                                    
      AREA myData, DATA, READWRITE
      ALIGN

      END
