.DSEG
_tmp_: .byte 2			;Manual initialize for use in scan_keypad_rising_edge_sim

.CSEG

.org 0x0
rjmp main

.org 0x10
rjmp ISR_TIMER1_OVF

.org 0x1C
rjm ADC_INTRPT_HANDLR

main: 
	ldi r24 ,(1<<TOIE1) 
	out TIMSK ,r24 
	rcall ADC_init
		
	
	ldi r24, low(RAMEND)
	out SPL, r24
	ldi r24, high(RAMEND)
	out SPH, r24 ; Stack Pointer Initialisation
	ser r24
	out DDRD, r24 ; LCD Output Initialisation
	out DDRB, r24 ; LED Initialisation
	clr r24
	ldi r24,0xF0 ; Keypad Initalisation
	out DDRC,r24 
	rcall lcd_init_sim ;LCD Initialisation and Output
	rcall timer_init
 
	clr r24
	readloop:
		rcall scan_keypad_rising_edge_sim   ;Read the first Key
		rcall keypad_to_ascii_sim
		cpi r24,0
		breq readloop
		mov r18, r24
	
	readloop2:
		
		rcall scan_keypad_rising_edge_sim	;Read the second Key
		rcall keypad_to_ascii_sim
		cpi r24,0
		breq readloop2
		mov r19, r24
		cpi r18,'0'					;First Key Check
		breq firstcorrect
		rjmp alarm_on

	firstcorrect:
		cpi r19,'8'					;Second Key Check
		breq welcome

	alarm_on:
		ldi r24,'A'					; Wrong: LCD
		rcall lcd_data_sim
		ldi r24,'L'
		rcall lcd_data_sim
		ldi r24,'A'
		rcall lcd_data_sim
		ldi r24,'R'
		rcall lcd_data_sim
		ldi r24,'M'
		rcall lcd_data_sim
		ldi r24,' '
		rcall lcd_data_sim
		ldi r24,'O'
		rcall lcd_data_sim
		ldi r24,'N'
		rcall lcd_data_sim
		ldi r22, 4				; Wrong: LED
			inloop:
			ldi r24, 0xFF
			OUT PORTB, r24
			ldi r25, 0x1
			ldi r24, 0xF4
			rcall wait_msec
			ldi r24, 0x00
			OUT PORTB, r24
			ldi r25, 0x1
			ldi r24, 0xF4
			rcall wait_msec
			dec r22
			cpi r22, 0
			brne inloop	
		ldi r24, 0x01
		rcall lcd_command_sim	;Clear Display
		ldi r25, 0
		ldi r24, 2
		rcall wait_msec
		rjmp readloop

	welcome:
		ldi r24,'W'				;Right: LCD
		rcall lcd_data_sim
		ldi r24,'E'
		rcall lcd_data_sim
		ldi r24,'L'
		rcall lcd_data_sim
		ldi r24,'C'
		rcall lcd_data_sim
		ldi r24,'O'
		rcall lcd_data_sim
		ldi r24,'M'
		rcall lcd_data_sim
		ldi r24,'E'
		rcall lcd_data_sim
		ldi r24,' '
		rcall lcd_data_sim
		ldi r24,'0'
		rcall lcd_data_sim
		ldi r24,'8'
		rcall lcd_data_sim
		ldi r24, 0xFF
		OUT PORTB, r24		;Right: LED
		ldi r25, 0xF
		ldi r24, 0xA0
		rcall wait_msec
		ldi r24, 0x01		;Clear Display
		rcall lcd_command_sim
		ldi r25, 0
		ldi r24, 2
		rcall wait_msec
		ldi r24, 0x00
		OUT PORTB, r24
		rjmp readloop	
	
	
	
	
	
	ADC_init:
		ldi r24,(1<<REFS0)
		out ADMUX,r24
		ldi r24,(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
		out ADCSRA,r24
		ret
	
	timer_init:
		
		ldi r24 ,(1<<CS12) | (0<<CS11) | (1<<CS10)
		out TCCR1B ,r24 			
		ldi r24 ,0xFC
		out TCNT1H ,r24 
		ldi r24 ,0xF3
		out TCNT1L ,r24 
		sei
		ret
	
	ISR_TIMER1_OVF:

		ldi r24,(1<<ADSC)
		out ADCSRA,r24
		rcall timer_init

	ADC_INTRPT_HANDLR:
		
	
	
	mul16:
		mul	r22, r20		 ;*	r17:r16 = r23:r22 * r21:r20
		movw	r17:r16, r1:r0
		mul	r23, r20		
		add	r17, r0
		mul	r21, r22		
		add	r17, r0
		ret

	scan_row_sim:
		out PORTC, r25  ;Check a row of the keypad
		push r24		;Input in r24 for the desired row
		push r25		;Output in 4 LSB of r24
		ldi r24,low(500) 
		ldi r25,high(500)
		rcall wait_usec
		pop r25
		pop r24 
		nop
		nop 
		in r24, PINC 
		andi r24 ,0x0f 
		ret

	scan_keypad_sim:
		push r26		;Check the whole keypad
		push r27		;Output in r25:24
		ldi r25 , 0x10 
		rcall scan_row_sim
		swap r24
		mov r27, r24 
		ldi r25 ,0x20 
		rcall scan_row_sim
		add r27, r24 
		ldi r25 , 0x40 
		rcall scan_row_sim
		swap r24 
		mov r26, r24 
		ldi r25 ,0x80 
		rcall scan_row_sim
		add r26, r24 
		movw r24, r26 
		clr r26 
		out PORTC, r26
		pop r27
		pop r26 
		ret

	scan_keypad_rising_edge_sim:
		push r22			;Check for keys that were not pressed but now are
		push r23			;Input the clicking time in ms in r24
		push r26			;Output in r25:24
		push r27
		rcall scan_keypad_sim 
		push r24 
		push r25
		ldi r24 ,15 
		ldi r25 ,0 
		rcall wait_msec
		rcall scan_keypad_sim 
		pop r23 
		pop r22
		and r24 ,r22
		and r25 ,r23
		ldi r26 ,low(_tmp_) 
		ldi r27 ,high(_tmp_) 
		ld r23 ,X+
		ld r22 ,X
		st X ,r24 
		st -X ,r25 
		com r23	
		com r22 
		and r24 ,r22
		and r25 ,r23
		pop r27 
		pop r26 
		pop r23 
		pop r22	
		ret 

	keypad_to_ascii_sim:
		push r26			;Convert previous input to ascii
		push r27			;Output the ascii code in r24
		movw r26 ,r24 
		ldi r24 ,'*'
		sbrc r26 ,0	
		rjmp return_ascii 
		ldi r24 ,'0'
		sbrc r26 ,1;
		rjmp return_ascii
		ldi r24 ,'#'
		sbrc r26 ,2
		rjmp return_ascii
		ldi r24 ,'D'
		sbrc r26 ,3 
		rjmp return_ascii 
		ldi r24 ,'7'
		sbrc r26 ,4
		rjmp return_ascii
		ldi r24 ,'8'
		sbrc r26 ,5
		rjmp return_ascii
		ldi r24 ,'9'
		sbrc r26 ,6
		rjmp return_ascii 
		ldi r24 ,'C'
		sbrc r26 ,7
		rjmp return_ascii
		ldi r24 ,'4' 
		sbrc r27 ,0 
		rjmp return_ascii
		ldi r24 ,'5'
		sbrc r27 ,1
		rjmp return_ascii
		ldi r24 ,'6'
		sbrc r27 ,2
		rjmp return_ascii
		ldi r24 ,'B'
		sbrc r27 ,3
		rjmp return_ascii
		ldi r24 ,'1'
		sbrc r27 ,4
		rjmp return_ascii 
		ldi r24 ,'2'
		sbrc r27 ,5
		rjmp return_ascii
		ldi r24 ,'3' 
		sbrc r27 ,6
		rjmp return_ascii
		ldi r24 ,'A'
		sbrc r27 ,7
		rjmp return_ascii
		clr r24
		rjmp return_ascii

		return_ascii:
			pop r27 
			pop r26
			ret

	write_2_nibbles_sim:
		push r24				;Send a byte to the LCD
		push r25				;Input the desired byte in r24
		ldi r24 ,low(6000) 
		ldi r25 ,high(6000)
		rcall wait_usec
		pop r25
		pop r24 
		push r24 
		in r25, PIND 
		andi r25, 0x0f 
		andi r24, 0xf0
		add r24, r25
		out PORTD, r24 
		sbi PORTD, PD3 
		cbi PORTD, PD3 
		push r24 
		push r25 
		ldi r24 ,low(6000) 
		ldi r25 ,high(6000)
		rcall wait_usec
		pop r25
		pop r24 
		pop r24 
		swap r24 
		andi r24 ,0xf0 
		add r24, r25
		out PORTD, r24
		sbi PORTD, PD3 
		cbi PORTD, PD3
		ret

	lcd_data_sim:
		push r24			;Send a byte of data to the LCD
		push r25			;Input the desired byte in r24
		sbi PORTD, PD2 
		rcall write_2_nibbles_sim
		ldi r24 ,43
		ldi r25 ,0
		rcall wait_usec
		pop r25
		pop r24
		ret 

	lcd_command_sim:
		push r24			;Send a command byte to the LCD
		push r25			;Input the desired command on r24
		cbi PORTD, PD2
		rcall write_2_nibbles_sim 
		ldi r24, 39
		ldi r25, 0 
		rcall wait_usec 
		pop r25 
		pop r24
		ret 

	lcd_init_sim:
		push r24			;Initialisation of LCD and options
		push r25			
		ldi r24, 40 
		ldi r25, 0 
		rcall wait_msec 
		ldi r24, 0x30 
		out PORTD, r24
		sbi PORTD, PD3 
		cbi PORTD, PD3 
		ldi r24, 39
		ldi r25, 0
		rcall wait_usec
		push r24 
		push r25 
		ldi r24,low(1000) 
		ldi r25,high(1000)
		rcall wait_usec
		pop r25
		pop r24 
		ldi r24, 0x30
		out PORTD, r24
		sbi PORTD, PD3
		cbi PORTD, PD3
		ldi r24,39
		ldi r25,0
		rcall wait_usec 
		push r24 
		push r25
		ldi r24 ,low(1000)
		ldi r25 ,high(1000)
		rcall wait_usec
		pop r25
		pop r24 
		ldi r24,0x20 
		out PORTD, r24
		sbi PORTD, PD3
		cbi PORTD, PD3
		ldi r24,39
		ldi r25,0
		rcall wait_usec
		push r24 
		push r25 
		ldi r24 ,low(1000) 
		ldi r25 ,high(1000)
		rcall wait_usec
		pop r25
		pop r24 
		ldi r24,0x28 
		rcall lcd_command_sim 
		ldi r24,0x0c 
		rcall lcd_command_sim
		ldi r24,0x01 
		rcall lcd_command_sim
		ldi r24, low(1530)
		ldi r25, high(1530)
		rcall wait_usec
		ldi r24 ,0x06
		rcall lcd_command_sim 
		pop r25 
		pop r24
		ret


	delay:
		push r24
		push r25
		ldi r24,0
		ldi r25,5
		rcall wait_msec
		pop r24
		pop r25
		ret

	wait_msec:
		push r24
		push r25
		ldi r24 ,low(998)
		ldi r25 , high(998)
		rcall wait_usec
		pop r25
		pop r24
		sbiw r24 , 1
		brne wait_msec
		ret

	wait_usec:
		sbiw r24,1	
		nop
		nop
		nop
		nop
		brne wait_usec
		ret