/*
 * MULTIMEDIDOR DE CC
 *
 * 
 *   Author: De Battista Cristian y Errecart Matias
 *	 Oyente por Zoom: Correa Bruno, Alias "el arruinau"
 */ 
 ;########################################################################################################################################## 
 ;############################################################### DECLARACION ##############################################################
 ;##########################################################################################################################################

.def	RestoL=r14
.def	RestoH=r15
.def	DividendoL=r16
.def	DividendoH=r17
.def	DivisorL=r18
.def	DivisorH=r19
.def	Contador=r20

.DSEG
.ORG 0x100
	VAL_TensionADCH: .Byte 1
	VAL_TensionADCL: .Byte 1
	VAL_CorrienteADCH: .Byte 1
	VAL_CorrienteADCL: .Byte 1
	Tension: .Byte 1
	TensionresH: .Byte 1
	TensionresL: .Byte 1
	PotenciaH: .Byte 1
	PotenciaL: .Byte 1
	Corriente: .Byte 1
	CorrienteresH: .Byte 1
	CorrienteresL: .Byte 1
	RestodivL: .Byte 1
	RestodivH: .Byte 1
	
.ORG 0x110
	.MACRO	PUSH_SREG				;Guardo en la pila la posicion de memoria
			push r13
			in r13, SREG
			push r13
			push r14
			push r15
			push r16
			push r17
			push r18
			push r19
			push r20
			push r21
			push r22
			push r23
			push r24
			push r25
	.ENDMACRO
.ORG 0x120
	.MACRO	POP_SREG
			pop r25
			pop r24
			pop r23
			pop r22
			pop r21
			pop r20
			pop r19
			pop r18
			pop r17
			pop r16
			pop r15
			pop r14
			pop r13
			out sreg, r13				;recupero de la pila la posicion de memoria
			pop r13
	.ENDMACRO

;########################################################## VECTORES DE INTERRUPCION #########################################################

.CSEG
.ORG 0x00
	jmp INICIO

.ORG 0x000A
	jmp RTI_SELECT
	
.ORG 0x001A
	jmp RTI_TIMER1_OVF

.ORG 0x0024
	jmp USART_RXC

.ORG 0x34
	reti

;################################################################################################################################## 
;########################################################## CONFIGURACION #########################################################
;##################################################################################################################################

	INICIO:

		ldi r16, high(ramend)  ;conf pila
		out sph, r16
		ldi r16, low(ramend)
		out spl, r16

;########################################################## CONFIGURACION ADC #########################################################

		ldi r16, (1<<ADC1D)|(1<<ADC0D)	;descativo entrada digital de los pines ADC0 y ADC1
		sts DIDR0, r16

;########################################################## CONFIGURACION  de PINES y PCINT0 #########################################################

		ldi r16, (0<<PD7)|(0<<PD6)|(0<<PD5)				;pines como entrada interrupcion de PCIE2
		out DDRD,r16
		ldi r16, (1<<PORTD7)|(1<<PORTD6)|(1<<PORTD5)	;Resistencias Pull Up
		out PORTD, r16
		ldi r16, (1<<PCIE2)								;habilito int por cambio de pines[7:0] (PCIE2)
		sts PCICR, r16
		ldi r16, 0b1110_0000							;habilito los 3 pines de interrupcion (PCINT 23 - 22 - 21)
		sts PCMSK2, r16

		ldi r16, (1<<PB1)|(1<<PB2)						;como salida PB1(OC1A) y PB2(OC1B) PWM
		out DDRB, r16

		ldi r16, (1<<PD5)|(1<<PD6)						;enciendo led prueba interrupcion externa PCEI0
		out DDRD, r16

;########################################################## CONFIGURACION DE TIMER/COMP 1 #########################################################
		
		ldi r16, (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(1<<WGM10) ; modo fase correcta PWM, comparacion igual no invertido, resolucion 10-bit
		sts TCCR1A, r16
		ldi r16, (1<<WGM12)|(0<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10)  ;selector de reloj de timer/counter: Clock_I-O/8(from prescaler)
		sts TCCR1B, r16

		ldi r16, (1<<TOIE1)								;interrupcion de salida del temporizador/contador  
		sts TIMSK1, r16
		
		ldi r16, 0x00
		sts OCR1AH, r16
		sts OCR1AL, r16
		sts OCR1BH, r16
		sts OCR1BL, r16

;########################################################## CONFIGURACION DE USART #########################################################

		ldi r16, 103								;Velocidad de transmicion 9600 Bd
		ldi r17, 0
		sts UBRR0H, r17
		sts UBRR0L, r16

		ldi r16, (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0)	;Habilita interrupción por recepción, Habilita recepción, Habilita transmición
		sts UCSR0B, r16
			
		ldi r16, (0<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00)	;Stop Bit 1, 8 bits
		sts UCSR0C, r16								
		
		SEI

;################################################################################################################################## 
;############################################################# PROGRAMA ###########################################################
;##################################################################################################################################

;########################################################## BUCLE PRINCIPAL #########################################################
		
		BUCLE:
			call ADC0
			call ADC1
			call CALCULO_TENSION
			call CALCULO_CORRIENTE
			jmp BUCLE

;########################################################## TRATAMIENTO DE INTERRUPCION DEL TIMER1 (salidas PWM 1-5 V) #########################################################

		RTI_TIMER1_OVF:			
			PUSH_SREG						;guardo en la pila la posicion de memoria
			
			lds r21, VAL_TensionADCH
			sts OCR1AH, r21					;salida PWMA timer OC1A
			lds r21, VAL_TensionADCL	
			sts OCR1AL, r21
			
			lds r20, VAL_CorrienteADCH
			sts OCR1BH, r20					;salida PWMB timer OC1B
			lds r20, VAL_CorrienteADCL	
			sts OCR1BL, r20
			
			POP_SREG						;recupero el valor de la pila
			reti

;########################################################## INTERRUPCION POR PCINT0 #########################################################

		RTI_SELECT:
			PUSH_SREG

			sbic PIND, PIND7				;Pregunta si PD7 esta en 0
			call MOSTRAR_POTENCIA			;Llama funcion para mostrar potencia
			sbic PIND, PIND6				;Pregunta si PD6 esta en 0
			call MOSTRAR_CORRIENTE			;Llama funcion para mostrar corriente
			sbic PIND, PIND5				;Pregunta si PD5 esta en 0
			call MOSTRAR_TENSION			;Llama funcion para mostrar tension
			
			POP_SREG
			reti

;########################################################## SUBRUTINA PARA LEER ADC0 ###################################################################
	
	ADC0:

			// *** Configuramos ADMUX ***
		ldi r16, (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0) 
		sts ADMUX, r16  
			; Referencia de Voltaje Con AVCC y Capacitor. Activamos ADC, Canal ADC0    

		// *** Iniciamos conversión en ADC0 *** 
		ldi r16, (1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) ;prescaler en 8, habilito ADC, int de conversion completa(ADIE), activacion auto del ADC(ADATE)  
		sts ADCSRA, r16 
		ldi r16, (1<<ADTS2)|(1<<ADTS1)|(0<<ADTS0)	;Timer/Counter1 Overflow
		sts ADCSRB, r16

		adcLeerADC0:
		lds r16, ADCSRA  ;Carga el control ADC y rgistro de estado A en el registro 16
		SBRC r16, 6      ;Salta si el bit 6 en el  registro r16 es limpiado
		rjmp adcLeerADC0 
			
		lds r17, ADCL  ; Cargamos parte baja del ADC pero no la guardamos.
		lds r16, ADCH  ; Cargamos parte alta del ADC en el registro 16.
		sts VAL_CorrienteADCL, r17 ; Cargamos el valor de ADCH en SRAM (a variable ValPotA)
		sts VAL_CorrienteADCH, r16
	
		ret
			
;################################################################### SUBRUTINA PARA LEER ADC1 ###################################################################

	ADC1:

			// *** Configuramos ADMUX ***
		ldi r18, (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0) 
		sts ADMUX, r18  
			; Referencia de Voltaje Con AVCC y Capacitor. Activamos ADC, Canal ADC0    

		// *** Iniciamos conversión en ADC0 *** 
		ldi r18, (1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(1<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) ;prescaler en 8, habilito ADC, int de conversion completa(ADIE), activacion auto del ADC(ADATE)  
		sts ADCSRA, r18 
		ldi r18, (1<<ADTS2)|(1<<ADTS1)|(0<<ADTS0)	;Timer/Counter1 Overflow
		sts ADCSRB, r18

		adcLeerADC1:
		lds r18, ADCSRA  ;Carga el control ADC y rgistro de estado A en el registro 16
		SBRC r18, 6      ;Salta si el bit 6 en el  registro r16 es limpiado
		rjmp adcLeerADC1 
			
		lds r18, ADCL  ; Cargamos parte baja del ADC pero no la guardamos.
		lds r19, ADCH  ; Cargamos parte alta del ADC en el registro 16.
		sts VAL_TensionADCL, r18 ; Cargamos el valor de ADCH en SRAM (a variable ValPotA)
		sts VAL_TensionADCH, r19
	
		ret	
;################################################################### CALCULO DE TENSION ###################################################################
	
	CALCULO_TENSION:
		
		lds R23, VAL_TensionADCH
		lds R22, VAL_TensionADCL			;Carga el numero VAL_Tension en r23:r22
		ldi R21,0x00
		ldi R20,0x19						;Carga el numero 25 r21:r20
		call mul16x16_16					;Llamado a rutina de multiplicacion de 16 bits x 16 bits
	
		ldi	DivisorL,0xFF					;el dividendo ya esta en r16 y r17
		ldi	DivisorH,0x03					;Dividimos por 1023, para llevar resultado de 0 a 25
		call Division16_16
		sts	Tension, DividendoL				;Valor de la tension real

		sts TensionresH, r15				;parte decimal del resto de la divison
		sts TensionresL, r14
		
		ret

;################################################################### CALCULO DE CORRIENTE ###################################################################
 
	CALCULO_CORRIENTE:

		 lds r23, VAL_CorrienteADCH			;Carga valores del ADC
		 lds r22, VAL_CorrienteADCL
		 ldi r24, 0x01						;Carga complemento de 511 para restar
		 ldi r25, 0xFE
		 add r22, r24						;Realiza suma (resta)
		 adc r23, r25
		 ldi r23, 0x00						;Ponemos 0 en la parte alta del resultado
		 ldi r21, 0x02						;Carga 587 (factor para adecuar la medicion) para multiplicar
		 ldi r20, 0x4B
		 call mul16x16_16					;Llama funcion multiplicacion

		 ldi DivisorH, 0x27					;Carga 10000 en el divisor (para obtener nuestro factor >> 0.0587)
		 ldi DivisorL, 0x10
		 call Division16_16					;Llama funcion division

		 sts Corriente, r16					;Resultado corriente
		 sts CorrienteresH, r15				;Parte decimal de la division
		 sts CorrienteresL, r14


;################################################################### FUNCION DE MULTIPLICACION ###################################################################

	mul16x16_16:

		mul	r22, r20				; al * bl
		movw	r17:r16, r1:r0
		mul	r23, r20				; ah * bl
		add	r17, r0
		mul	r21, r22				; bh * al
		add	r17, r0
		;resultado r17, r16
		ret

;############################################################ FUNCION DE MULTIPLICACION DEL RESTO ###################################################################

	mul16x16_24:
		mul		r23, r21		; ah * bh
		mov		r18, r0
		mul		r22, r20		; al * bl
		movw	r17:r16, r1:r0
		mul		r23, r20		; ah * bl
		add		r17, r0
		adc		r18, r1
		mul		r21, r22		; bh * al
		add		r17, r0
		adc		r18, r1
		; resultado r18, r17, r16
		ret


;################################################################### FUNCION DE DIVISION ###################################################################

	Division16_16:	
		clr	RestoL					; borra byte low de Resto
		sub	RestoH,RestoH			; borra el byte alto y acarreo
		ldi	Contador,17				; contador de bucle infinito
		div_1:	
		rol	DividendoL				; desplazo a la izquierda el dividendo
		rol	DividendoH
		dec	Contador				; decremento contador
		brne	div_2				;if done
		;sts	Tension, DividendoL	; este valor se multiplica por 5
		sts RestodivL, RestoL
		sts RestodivH, RestoH
		ret							; salida
		div_2:	
		rol	RestoL					;desplazo a la izquerda el resto
		rol	RestoH
		sub	RestoL,DivisorL			;resto = resto - divisor
		sbc	RestoH,DivisorH	
		brcc	div_3				;si el resultado es negativo
		add	RestoL,DivisorL			; restaurar el resto
		adc	RestoH,DivisorH
		clc							;limpia el acarreo para ser deplazado al resultado
		rjmp	div_1				;else
		div_3:	
		sec							;pone a 1 la bandera de acarreo para ser trasladado al resultado
		rjmp	div_1
		; resultado de la division r17, r16
		; resto de la division r15, r14

;########################################################## MOSTRAR POTENCIA #########################################################

		MOSTRAR_POTENCIA:



		ret

;######################################################### MOSTRAR CORRIENTE #########################################################

		MOSTRAR_CORRIENTE:



		ret

;########################################################## MOSTRAR TENSION #########################################################

		MOSTRAR_TENSION:

			call DESCOMPOSICION
					
			
			call USART_ESPERA			
			ldi r20, 0x56				; V
			sts UDR0, r20

			call USART_ESPERA
			ldi r20, 0x20				; (espacio)
			sts UDR0, r20

			call USART_ESPERA			
			ldi r20, 0x3D				; =
			sts UDR0, r20


		ret

;########################################################## USART RECEPCION #########################################################

		USART_RXC:



		reti

;########################################################## USART ESPERA #########################################################

		USART_ESPERA:

			lds r26, UCSR0A				;Espera que se limpie la bandera de transmicion
			sbrs r26, UDRE0
			rjmp USART_ESPERA
			ret

;########################################################## DESCOMPOSICION #########################################################
		
		DESCOMPOSICION:
			
			lds r16, Tension
			lds r17, TensionresH
			lds r18, TensionresL

