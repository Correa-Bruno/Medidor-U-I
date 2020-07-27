/*
 * Medidor_de_U__I.asm
 *
 *  Created: 24/7/2020 09:20:33
 *   Author: Bruno
 */ 
 
 .DSEG
 .ORG 0x100
	VAL_PWMA: .Byte 1
	VAL_PWMB: .Byte 1

.CSEG
.ORG 0x00
	jmp INICIO

.ORG 0x20
	jmp RTI_TIMER1_OVF

.ORG 0x34
	jmp RTI_ADC

.ORG 0x40
	

.ORG 0x50
	reti


INICIO:

		ldi r16, high(ramend)  ;conf pila
		out sph, r16
		ldi r16, low(ramend)
		out spl, r16

		ldi r16, (1<<ADEN)|(1<<ADIE)|(1<<ADATE)|(0<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) //prescaler en 8, habilito ADC, interrupcion de conversion completa(ADIE), activacion auto del ADC(ADATE)  
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
		
		SEI

		RTI_ADC:
				push r16
				in r16, sreg // guardo en la pila la posicion de memoria
				push r16

				ldi r16, ADMUX
				ldi r17, ADMUX

				ANDI r16, 0b0111
				ANDI r17, 0b1111_0000
				 
				cpi r16, 0
				breq TRUE1
				cpi r16, 1
				breq TRUE2
				sts ADMUX, r17

				pop r16
				out sreg, r16
				pop r16
				reti

		TRUE1: 
				ldi r16, ADCL
				ldi r16, ADCH
				sts VAL_PWMA, r16		//guardo en variale pwm el valor del adc
				inc r17
				sts ADMUX, r17
				reti

		TRUE2:
				ldi r16, ADCL
				ldi r16, ADCH
				sts VAL_PWMB, r16	//guardo en una variable de pwm el valor de adc
				ldi r18, 0b0000_0000
				add r16, r18
				sts ADMUX, r16
				reti

		RTI_TIMER1_OVF:
				
				push r16
				in r16, sreg // guardo en la pila la posicion de memoria
				push r16

				lds r16, VAL_PWMA
				com r16
				out OCR1A, r16	// salida PWMA timer 1

				lds r16, VAL_PWMB
				com r16
				out OCR1B, r16 // salida PWMB timer 1

				pop r16
				out sreg, r16	
				pop r16
