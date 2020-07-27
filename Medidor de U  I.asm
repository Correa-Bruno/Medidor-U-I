/*
 * Medidor_de_U__I.asm
 *
 *  Created: 24/7/2020 09:20:33
 *   Author: Bruno
 */ 
 
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

		ldi r16, (1<<ADEN)|(1<<ADIE)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) //selec divisor(ADTS), habilito ADC, interrupcion de conversion completa(ADIE), activacion auto del ADC(ADATE)  
		sts ADCSRA, r16 
		ldi r16, (1<<ADTS2)|(1<<ADTS1)|(0<<ADTS0) //Timer/Counter1 Overflow
		sts ADCSRB, r16
		ldi r16, (0<<REFS1)|(1<<REFS0)|(1<<ADLAR) // selec voltaje de referencia ADC, ajusto el resultado a la izquiera
		sts ADMUX, r16
		
		ldi r16, (0<<PB3)|(0<<PB4)|(0<<PB5)	// como entrada interrup PCIE0
		out DDRB,r16
		ldi r16, (1<<PORTB3)|(1<<PORTB4)|(1<<PORTB5)// Pull Up
		out PORTB, r16
		ldi r16, 0b0000_0001 // habilito int por cambio de pines[7:0] (PCIE0)
		sts PCICR, r16
		ldi r16, 0b0011_1000 //habilito los tres pines de interrupcion (pb3 pb4 pb5)
		sts PCMSK0, r16

		ldi r16, (1<<PB1)|(1<<PB2) // como salida pB1 pB2 (uso PWM)
		out DDRB, r16

		ldi r16, (1<<COM1A1)|(1<<COM1B1)|(0<<COM1A0)|(0<<COM1B0)|(1<<WGM11)|(1<<WGM10) // pwm fast, comparacion igual no invertido 
		sts TCCR1A, r16
		ldi r16, (1<<WGM12)|(0<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10) //prescaler 8
		sts TCCR1B, r16

		ldi r17,0x03		; registro de comparador de salida en 1023
		ldi r16,0xFF
		sts OCR1AH,r17
		sts OCR1AL,r16

		ldi r16, (1<<OCIE1A)|(1<<OCIE1B)	; interrupcion de salida del temporizador/contador  
		sts TIMSK1, r16
