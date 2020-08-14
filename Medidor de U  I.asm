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

.def	ENTEROL=r12				;Subrutina mostrar
.def	ENTEROH=r13				;Subrutina mostrar
.def	RestoL=r14				;Subrutina division
.def	RestoH=r15				;Subrutina division
.def	DividendoL=r16			;Subrutina division
.def	DividendoH=r17			;Subrutina division
.def	DivisorL=r18			;Subrutina division
.def	DivisorH=r19			;Subrutina division
.def	Contador=r20			;Subrutina division
.def	rBin2L=r21				;Subrutina descomposicion
.def	rBin2H=r22				;Subrutina descomposicion
.def	rmp=r23					;Subrutina descomposicion

.MACRO	PUSH_SREG				;Guardo en la pila la posicion de memoria
		push r13
		in r13, SREG			;guardar registro de tranjo 12 tmb 
		push r13
		push r12
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
		push r26
		push r27
.ENDMACRO

.MACRO	POP_SREG
		pop r27
		pop r26
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
		pop r12
		pop r13
		out sreg, r13				;recupero de la pila la posicion de memoria
		pop r13
.ENDMACRO

.DSEG
.ORG 0x100
	VAL_TensionADCH: .Byte 1
	VAL_TensionADCL: .Byte 1
	VAL_CorrienteADCH: .Byte 1
	VAL_CorrienteADCL: .Byte 1
	TensionH: .Byte 1
	TensionL: .Byte 1
	PotenciaH: .Byte 1
	PotenciaL: .Byte 1
	CorrienteH: .Byte 1
	CorrienteL: .Byte 1
	RestodivL: .Byte 1
	RestodivH: .Byte 1
	VECTOR: .Byte 5
	DATO_RX: .Byte 1
	GRANDEH: .Byte 1
	GRANDEL: .Byte 1

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
			call USART_COMPARACION
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
			
			POP_SREG					;recupero el valor de la pila
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
		ldi R20,0x18						;Carga el numero 24 r21:r20
		call mul16x16_16					;Llamado a rutina de multiplicacion de 16 bits x 16 bits
		sts GRANDEH, r17					;Guardamos resultado de la primer multiplicacion
		sts GRANDEL, r16

		lds R23, VAL_TensionADCH
		lds R22, VAL_TensionADCL			;Carga el numero VAL_Tension en r23:r22
		ldi R21,0x00
		ldi R20,0x2B						;Carga el numero 43 r21:r20
		call mul16x16_16					;Llamado a rutina de multiplicacion de 16 bits x 16 bits
	
		ldi	DivisorL,0x64					;El dividendo ya esta en r16 y r17
		ldi	DivisorH,0x00					;Dividimos por 100
		call Division16_16

		lds r19, GRANDEH
		lds r18, GRANDEL

		add r19, DividendoH					;Sumamos primer calculo con segundo calculo
		adc r18, DividendoL
		sts	TensionH, r19					;Valor de la tension real
		sts	TensionL, r18
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

		ldi DivisorH, 0x00					;Carga 10 en el divisor (para obtener nuestro factor >> 0.0587)
		ldi DivisorL, 0x0A
		call Division16_16					;Llama funcion division

		sts CorrienteH, r17					;Resultado corriente
		sts CorrienteL, r16				
		ret

;################################################################### CALCULO DE POTENCIA ###################################################################
 
	CALCULO_POTENCIA:

		lds r23, TensionH
		lds r22, TensionL
		lds r21, CorrienteH
		lds r20, CorrienteL
		call mul16x16_16					;Llama funcion multiplicacion

		ret
		
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
		
		lds ENTEROH, CorrienteH		;Cargar valor de corriente alta
		lds ENTEROL, CorrienteL		;Cargar valor de corriente baja
		
		call DESCOMPOSICION
				
		call USART_ESPERA			
		ldi r20, 0x49				; I
		sts UDR0, r20

		call USART_ESPERA
		ldi r20, 0x20				; (espacio)
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x3D				; =
		sts UDR0, r20

		call USART_ESPERA
		ldi r20, 0x20				; (espacio)
		sts UDR0, r20

		call MOSTRAR

		call USART_ESPERA
		ldi r20, 0x20				; (espacio)
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x41				; A
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x6D				; m
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x70				; p
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x65				; e
		sts UDR0, r20
		
		call USART_ESPERA			
		ldi r20, 0x72				; r
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x0A				; (salto de linea)
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x0D				; (retorno de carro)
		sts UDR0, r20

		clr r17						;Limpiar registro de dato recibido
		sts DATO_RX, r17
	
		ret

;########################################################## MOSTRAR TENSION #########################################################

	MOSTRAR_TENSION:

		lds ENTEROH, TensionH		;Cargar valor de tension alta
		lds ENTEROL, TensionL		;Cargar valor de tension baja
		
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

		call USART_ESPERA
		ldi r20, 0x20				; (espacio)
		sts UDR0, r20

		call MOSTRAR

		call USART_ESPERA
		ldi r20, 0x20				; (espacio)
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x56				; V
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x6F				; o
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x6C				; l
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x74				; t
		sts UDR0, r20
		
		call USART_ESPERA			
		ldi r20, 0x0A				; (salto de linea)
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x0D				; (retorno de carro)
		sts UDR0, r20

		clr r17						;Limpiar registro de dato recibido
		sts DATO_RX, r17
		ret

;########################################################## USART RECEPCION #########################################################

	USART_RXC:

		PUSH_SREG
		lds r16, UDR0
		sts DATO_RX, r16
		POP_SREG
		reti

;########################################################## USART ESPERA #########################################################

	USART_ESPERA:

		lds r26, UCSR0A				;Espera que se limpie la bandera de transmicion
		sbrs r26, UDRE0
		rjmp USART_ESPERA
		ret

;########################################################## DESCOMPOSICION ENTERO #########################################################
		
	DESCOMPOSICION:

		ldi	ZL, LOW(VECTOR)
		ldi	ZH, HIGH(VECTOR)			
		ldi rmp, 0x27				;Cargamos 10000
		mov rBin2H,rmp
		ldi rmp, 0x10
		mov rBin2L,rmp
		rcall Bin2ToDigit			;Funcion para calcular digito
		ldi rmp, 0x03				;Cargamos 1000
		mov rBin2H,rmp
		ldi rmp, 0xE8
		mov rBin2L,rmp
		rcall Bin2ToDigit			;Funcion para calcular digito
		ldi rmp, 0x00				;Cargamos 100
		mov rBin2H,rmp
		ldi rmp, 0x64
		mov rBin2L,rmp
		rcall Bin2ToDigit			;Funcion para calcular digito
		ldi rmp, 0x00				;Cargamos 10
		mov rBin2H,rmp
		ldi rmp, 0x0A
		mov rBin2L,rmp
		rcall Bin2ToDigit			;Funcion para calcular digito
		st z,ENTEROL
		sbiw ZL,4					;Poner el puntero en el primer BCD
		ret

	Bin2ToDigit:
		clr rmp						;Conteo en cero

	Bin2ToDigita:
		cp ENTEROH,rBin2H			; Comparo nro con comparacion parte alta
		brcs Bin2ToDigitc			; Si Carry=1 el nro es menor a comparacion, vuelve a rutina para comparar con un valor menor
		brne Bin2ToDigitb	 
		cp ENTEROL,rBin2L			; Si es igual, Comparo nro con comparacion parte baja
		brcs Bin2ToDigitc			; Si Carry=1 nro menor a comparacion

	Bin2ToDigitb:			
		sub ENTEROL,rBin2L			; Resto partes bajas
		sbc ENTEROH,rBin2H			; Resto partes altas con carry
		inc rmp						; Incremento cuenta para digito BCD
		rjmp Bin2ToDigita			; Repito el proceso hasta que sea menor a comparacion

	Bin2ToDigitc:
		st z+,rmp					; Salva el digito 
		ret

;########################################################## MOSTRAR ENTERO #########################################################
	
	MOSTRAR:
		
		call USART_ESPERA
		ld	r27, Z+					;Cargamos decena
		ldi r29, 48
		add r27, r29				;Sumamos 48 para convertirlo en ASCII
		sts UDR0,r27				;Enviamos por puerto serie
		call USART_ESPERA
	
		call USART_ESPERA
		ld	r27, Z+					;Cargamos unidad
		ldi r29, 48
		add r27, r29				;Sumamos 48 para convertirlo en ASCII
		sts UDR0,r27				;Enviamos por puerto serie
		call USART_ESPERA

		call USART_ESPERA			
		ldi r20, 0x2C				; ,
		sts UDR0, r20				;Enviamos por puerto serie

		call USART_ESPERA
		ld	r27, Z+					;Cargamos primer decimal
		ldi r29, 48
		add r27, r29				;Sumamos 48 para convertirlo en ASCII
		sts UDR0,r27				;Enviamos por puerto serie
		call USART_ESPERA

		call USART_ESPERA
		ld	r27, Z+					;Cargamos segundo decimal
		ldi r29, 48
		add r27, r29				;Sumamos 48 para convertirlo en ASCII
		sts UDR0,r27				;Enviamos por puerto serie
		call USART_ESPERA
	
		call USART_ESPERA
		ld	r27, Z					;Cargamos tercer decimal
		ldi r29, 48
		add r27, r29				;Sumamos 48 para convertirlo en ASCII
		sts UDR0,r27				;Enviamos por puerto serie
		call USART_ESPERA

		SBIW ZL, 4					;Poner el puntero en el primer BCD
		ret
		
;########################################################## USART COMPARACION #########################################################

	USART_COMPARACION:
		
		lds r16, DATO_RX			;Cargar dato recibido
		ldi r17, 0x56				;Comparar con V
		cpse r16, r17
		rjmp I						;Si es falso, compara con I
		call MOSTRAR_TENSION		
		I:
		ldi r17, 0x49				;Comparar con I
		cpse r16, r17
		rjmp P						;Si es falso, compara con P
		call MOSTRAR_CORRIENTE
		P:
		ldi r17, 0x50				;Comparar con P
		cpse r16, r17
		ret
		call MOSTRAR_POTENCIA
		ret
