/*
 * Medidor_de_U__I.asm
 *
 *  Created: 24/7/2020 09:20:33
 *   Author: Bruno
 */ 

 //holis 
 .DSEG
 .ORG 0x100
	N_T0_OVF: .Byte 1

.CSEG
.ORG 0x00
	jmp INICIO

.ORG 0x20
	jmp RTI_TIMER1_OVF

.ORG 0x34
	reti


INICIO:

		ldi r16, high(ramend)  ;conf pila
		out sph, r16
		ldi r16, low(ramend)
		out spl, r16

		ldi r16, (1<<REFS0) ; voltaje de ref del adc
		sts ADMUX, r16

		ldi r16, (1<<ADEN)|(1<<ADSC)|(1<<ADPS2) ; habilito ADC, inicio conversion, prescaler en 16

		ldi r16, (1<<PB1)|(1<<PB2)	// pb1 y pb2 como salida y pd2 a pd4 como entrada (PWM)
		out DDRB,r16

		ldi r16, (1<<PORTD2)|(1<<PORTD3)|(1<<PORTD4)// full ap
		out PORTD, r16

		ldi r16, 0b1111_1111 //pull up pc0 pc1	
		out portc, r16

		ldi r16, (1<<COM1A1)|(0<<COM1A0)|(1<<WGM11)|(1<<WGM10) // pwm fast, comparacion igual no invertido 
		sts TCCR1A, r16
		ldi r16, (1<<WGM12)|(0<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10) //prescaler 8
		sts TCCR1B, r16

		ldi r17,0x03		; registro de comparador de salida en 1023
		ldi r16,0xFF
		sts OCR1AH,r17
		sts OCR1AL,r16

		ldi r16, (1<<OCIE1A)	; interrupcion de salida del temporizador/contador  
		sts TIMSK1, r16
