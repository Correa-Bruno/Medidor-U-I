/*
 * Medidor_de_U__I.asm
 *
 *  Created: 24/7/2020 09:20:33
 *   Author: Bruno
 */ 
 //hola gatos
.DSEG
.ORG 0x100
	VAL_PWMAH: .Byte 1
	VAL_PWMAL: .Byte 1
	VAL_PWMBH: .Byte 1
	VAL_PWMBL: .Byte 1
.ORG 0x110
	.MACRO	PUSH_SREG
			push r20
			in r20, sreg		;guardo en la pila la posicion de memoria
			push r20
	.ENDMACRO
.ORG 0x120
	.MACRO	POP_SREG
			pop r20
			out sreg, r20		;recupero de la pila la posicion de memoria
			pop r20
	.ENDMACRO
.CSEG
.ORG 0x00
	jmp INICIO

.ORG 0x20
	jmp RTI_TIMER1_OVF

.ORG 0x34
	jmp RTI_ADC

.ORG 0x55
	jmp RTI_PCINT0

.ORG 0x100
	reti


INICIO:

		ldi r16, high(ramend)  ;conf pila
		out sph, r16
		ldi r16, low(ramend)
		out spl, r16

		ldi r16, (1<<ADEN)|(1<<ADIE)|(1<<ADATE)|(0<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) ;prescaler en 8, habilito ADC, int de conversion completa(ADIE), activacion auto del ADC(ADATE)  
		sts ADCSRA, r16 
		ldi r16, (1<<ADTS2)|(1<<ADTS1)|(0<<ADTS0)	;Timer/Counter1 Overflow
		sts ADCSRB, r16
		/*ldi r16, (1<<ADC1D)|(1<<ADC0D)	;descativo entrada digital de los pines ADC0 y ADC1
		sts DIDR0, r16*/

		ldi r16, (0<<REFS1)|(1<<REFS0)|(1<<ADLAR)	;selec voltaje de referencia ADC, ajusto el resultado a la izquiera
		sts ADMUX, r16
		
		ldi r16, (0<<PB3)|(0<<PB4)|(0<<PB5)				; pines como entrada interrupcion de PCIE0
		out DDRB,r16
		ldi r16, (1<<PORTB3)|(1<<PORTB4)|(1<<PORTB5)	; Resistencias Pull Up
		out PORTB, r16
		ldi r16, 0b0000_0001		;habilito int por cambio de pines[7:0] (PCIE0)
		sts PCICR, r16
		ldi r16, 0b0011_1000		;habilito los 3 pines de interrupcion (PB 3-4-5)
		sts PCMSK0, r16

		ldi r16, (1<<PB1)|(1<<PB2)  ;como salida PB1(OC1A) y PB2(OC1B) PWM
		out DDRB, r16

		ldi r16, (1<<PD5)|(1<<PD6)	; enciendo led prueba interrupcion externa PCEI0
		out DDRD, r16

		ldi r16, (1<<COM1A1)|(1<<COM1B1)|(0<<COM1A0)|(0<<COM1B0)|(1<<WGM11)|(1<<WGM10) ; modo fast PWM, comparacion igual no invertido 
		sts TCCR1A, r16
		ldi r16, (1<<WGM12)|(0<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10)  ;selector de reloj de timer/counter: Clock_I-O/8(from prescaler)
		sts TCCR1B, r16

		ldi r16, (1<<TOIE1)		;interrupcion de salida del temporizador/contador  
		sts TIMSK1, r16
		
		SEI

		/*
		BUCLE:
			sbic PINB,PINB5		; preg si PINB2=0
			RJMP BUCLE			; si es F retorna a BUCLE	
			CALL Retardo_20ms	;si es V espera 20ms
			sbic PINB,PINB5		;preg si PINBB=0
			rjmp BUCLE			;si es F retorna BUCLE
			sbi PIND,PIND6		;si es V prendo LED
				
		RETENCION:

			sbic PINB,PINB5		;preg si PIND2=0
			rjmp BUCLE			;si es F retorna BUCLE
			rjmp RETENCION		;si es V espera el cambio
		*/
		Retardo_20ms:

			ldi  R17, $26
  WGLOOP0:  ldi  R18, $17
  WGLOOP1:  ldi  R19, $79
  WGLOOP2:  dec  R19
			brne WGLOOP2
			dec  R18
			brne WGLOOP1
		    dec  R17
		    brne WGLOOP0
		    nop
		    nop

		    RET

		RTI_ADC:
			PUSH_SREG					;guardo en la pila la posicion de memoria
			.DEF CANAL0=r16
			.DEF CANAL1=r17

			lds CANAL0, ADMUX			;leo el registro ADEMUX para seleccionar canal
			lds CANAL1, ADMUX
			ANDI CANAL0, 0b0111
			ANDI CANAL1, 0b1111_0000	; mascara para leer canal
			 
			cpi CANAL0, 0				;si es igual al canal ADC0
			breq TRUE1
			cpi CANAL0, 1				;si es igual al canal ADC1
			breq TRUE2

		TRUE1: 
			
			lds r16, ADCL	
			lds r18, ADCH	
			sts VAL_PWMAL, r16
			sts VAL_PWMAH, r18		;guardo en variale PWMA el valor de ADC0		
			inc CANAL1				
			sts ADMUX, CANAL1		;guardo CANAL1 en 1 para leer el ADC1 en la proxima conversion
			rjmp SALIDA

		TRUE2:
			lds r16, ADCL
			lds r18, ADCH
			sts VAL_PWMBL, r16		;guardo en variable PWMB el valor de ADC1
			sts VAL_PWMBH, r18
			ldi r19, 0b0000_0000
			mul CANAL0, r19			;suma para cambiar al canal ADC0
			sts ADMUX, CANAL0		;guardo CANAL0 en 0 para leer ADC0 en la proxima conversion
			rjmp SALIDA

		SALIDA:
			POP_SREG			;recupero valor de la pila
			reti

		RTI_TIMER1_OVF:			; tratamiento de interrupcion del timer1
			PUSH_SREG			;guardo en la pila la posicion de memoria
			lds r16, VAL_PWMAH	;
			com r16
			sts OCR1AH, r16		; salida PWMA timer OC1A
			lds r16, VAL_PWMAL	
			com r16
			sts OCR1AL, r16	

			lds r16, VAL_PWMBH	
			com r16
			sts OCR1BH, r16     ;salida PWMB timer OC1B
			lds r16, VAL_PWMBL
			com r16
			sts OCR1BL, r16 

			POP_SREG			;recupero el valor de la pila
			reti
		
		RTI_PCINT0:
			sbic PINB,PINB5		; preg si PINB5=0
			sbi PIND, PIND6		; enciendo led
			reti