/*
 * MULTIMEDIDOR DE CC
 *
 * 
 *   Autores: Correa Bruno, De Battista Cristian, Errecart Matias
 *	 
 */ 
 ;########################################################################################################################################## 
 ;############################################################### DECLARACION ##############################################################
 ;##########################################################################################################################################

.def	ENTEROL=r12					;Subrutina mostrar
.def	ENTEROH=r13					;Subrutina mostrar
.def	RestoL=r14					;Subrutina division
.def	RestoH=r15					;Subrutina division
.def	DividendoL=r16				;Subrutina division
.def	DividendoH=r17				;Subrutina division
.def	DivisorL=r18				;Subrutina division
.def	DivisorH=r19				;Subrutina division
.def	Contador=r20				;Subrutina division
.def	rBin2L=r21					;Subrutina descomposicion
.def	rBin2H=r22					;Subrutina descomposicion
.def	rmp=r23						;Subrutina descomposicion

.MACRO	PUSH_SREG					;Guardar en la pila la posicion de memoria
		push r12
		in r12, SREG			
		push r12					;Guardar registros de trabajo
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
		pop r13
		pop r12
		out sreg, r12				;Recuperar de la pila la posicion de memoria
		pop r12
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
	Temp1: .Byte 1
	Temp2: .Byte 1
	Temp3: .Byte 1
	CorrienteH_PWM: .Byte 1
	CorrienteL_PWM: .Byte 1
	PotenciaH_PWM: .Byte 1
	PotenciaL_PWM: .Byte 1
	SELECT: .Byte 1

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

		ldi r16, high(ramend)		;Configuracion de pila
		out sph, r16
		ldi r16, low(ramend)
		out spl, r16

;########################################################## CONFIGURACION ADC #########################################################

		ldi r16, (1<<ADC1D)|(1<<ADC0D)	
		sts DIDR0, r16				;Descativo entrada digital de los pines ADC0 y ADC1

;########################################################## CONFIGURACION  de PINES y PCINT0 #########################################################

		ldi r16, (0<<DDD7)|(0<<DDD6)|(0<<DDD5)			
		out DDRD,r16				;Pines como entrada interrupcion de PCIE2
		ldi r16, (1<<PD7)|(1<<PD6)|(1<<PD5)				
		out PORTD, r16				;Resistencias Pull Up
		ldi r16, (1<<PCIE2)			;Habilito int por cambio de pines[7:0] (PCIE2)
		sts PCICR, r16
		ldi r16, 0b1110_0000		;Habilito los pines 7-6-5 de interrupcion (PCINT 23 - 22 - 21)
		sts PCMSK2, r16
		ldi r16, (1<<DDB1)|(1<<DDB2)|(1<<DDB3)|(1<<DDB5)	
		out DDRB, r16				;Como salida PB1-9(OC1A), PB2-10(OC1B)|| PB5-13(CSK), PB3-11(MOSI)
		ldi r16, (1<<DDC5)			;Pin PC5-A5 (Load) como salida
		out DDRC, r16

;########################################################## CONFIGURACION DE TIMER/COMP 1 #########################################################
		
		ldi r16, (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(1<<WGM10) 
		sts TCCR1A, r16				;Modo fase correcta PWM, comparacion igual no invertido, resolucion 10-bit
		ldi r16, (1<<WGM12)|(0<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10)  
		sts TCCR1B, r16				;Selector de reloj de timer/counter: Clock_I-O/8(from prescaler)

		ldi r16, (1<<TOIE1)			;Interrupcion de salida del temporizador/contador  
		sts TIMSK1, r16
		
		ldi r16, 0x00
		sts OCR1AH, r16
		sts OCR1AL, r16
		sts OCR1BH, r16
		sts OCR1BL, r16

;########################################################## CONFIGURACION DE USART #########################################################

		ldi r16, 103				;Velocidad de transmicion 9600 Bd
		ldi r17, 0
		sts UBRR0H, r17
		sts UBRR0L, r16

		ldi r16, (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0)	
		sts UCSR0B, r16				;Habilita interrupción por recepción, Habilita recepción, Habilita transmición
			
		ldi r16, (0<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00)	
		sts UCSR0C, r16				;Stop Bit 1, 8 bits				

;########################################################## CONFIGURACION DEL MAX #########################################################

		call CONFIG_MAX				;Inicializar MAX

		sei							;Habilitacion global de interrupciones				

;################################################################################################################################## 
;############################################################# PROGRAMA ###########################################################
;##################################################################################################################################

;########################################################## BUCLE PRINCIPAL #########################################################
		
	BUCLE:
		call ADC0
		call RETARDO
		call CALCULO_CORRIENTE
		call ADC1
		call RETARDO
		call CALCULO_TENSION
		call CALCULO_POTENCIA
		call SELECT_Magnitud
		call CALCULO_CORRIENTE_PWM
		call CALCULO_POTENCIA_PWM
		call USART_COMPARACION
		
		jmp BUCLE

;########################################################## TRATAMIENTO DE INTERRUPCION DEL TIMER1 (salidas PWM 1-5 V) #########################################################

	RTI_TIMER1_OVF:			
		PUSH_SREG					;Guardo en la pila la posicion de memoria
								
		lds r21, PotenciaH_PWM
		sts OCR1AH, r21				;Salida PWMA timer OC1A (Pin 9)
		lds r21, PotenciaL_PWM	
		sts OCR1AL, r21
			
		lds r20, CorrienteH_PWM
		sts OCR1BH, r20				;Salida PWMB timer OC1B (Pin 10)
		lds r20, CorrienteL_PWM
		sts OCR1BL, r20
			
		POP_SREG					;Recupero el valor de la pila
		reti

;########################################################## CONFIGURACION SPI Modo - MAESTRO ######################################################### 

	CONFIG_MAX:

		ldi r17,(1<<SPE)|(1<<MSTR)|(1<<SPR0)	
		out SPCR,r17				;Habilitar SPI como Master, Velocidad de reloj f/16 (1Mhz)
			
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; SETEAR BRILLO ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
					
		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x0A
		out SPDR, r17				;Entrar Set Brillo MAX
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x00
		out SPDR,r17				;Setear el brillo MAX al Minimo
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; SETEAR MODOS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x09
		out SPDR, r17				;Entrar en modo de codificacion
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x0F
		out SPDR,r17				;Setear Code B decode for digits 3–0 No decode for digits 7–4
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; SCAN LIMIT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x0B
		out SPDR, r17				;Entrar Scan Limit
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x05
		out SPDR,r17				;Display digits 0 1 2 3 4 5
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Setear Modo ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x0C
		out SPDR, r17				;Entrar MODO
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 1
		out SPDR,r17				;Normal Operation
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Setear TEST ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x0F
		out SPDR, r17				;Entrar TEST
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0
		out SPDR,r17				;Normal Operation
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Iniciar digitos en "HELLO" ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r17, (0<<PC5)			;Mando 0 a PB4 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x01
		out SPDR, r17				;Digito 0
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x00
		out SPDR,r17				;O
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x02
		out SPDR, r17				;Digito 1
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x0D
		out SPDR,r17				;L
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x03
		out SPDR, r17				;Digito 2
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x0D
		out SPDR,r17				;L
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x04
		out SPDR, r17				;Digito 3
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x0B
		out SPDR,r17				;E
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x05
		out SPDR, r17				;Digito 4
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x37
		out SPDR,r17				;H
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x06
		out SPDR, r17				;Digito 5
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0x00
		out SPDR,r17				;0
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop
		ret
		
;########################################################## INTERRUPCION POR PCINT0 #########################################################

	RTI_SELECT:
		
		PUSH_SREG
		
		ldi r17, 0xFF				;Ponemos todo el registro en 1
		in r16, PIND
		sbrs r16, 7					;Pregunta si PD7 esta en 0
		ldi r17, 0b0111_1111		;Carga 0 en el bit 7

		sbrs r16, 6					;Pregunta si PD6 esta en 0 
		ldi r17, 0b1011_1111		;Carga 0 en el bit 6

		sbrs r16, 5					;Pregunta si PD5 esta en 0
		ldi r17, 0b1101_1111		;Carga 0 en el bit 5

		sts SELECT, r17				;Guarda valor en variable
		
		POP_SREG
		reti

;########################################################## SUBRUTINA PARA LEER ADC0 ###################################################################
	
	ADC0:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Configuracion ADEMUX ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r16, (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0) 
		sts ADMUX, r16				;Referencia de Voltaje Con AVCC y Capacitor, Activar ADC, Canal ADC0  
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Iniciar conversion ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r16, (1<<ADEN)|(1<<ADSC)|(0<<ADIE)|(0<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) 
		sts ADCSRA, r16				;Prescaler en 128, Habilito ADC
		

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Leer ADC1 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	LEER_ADC0:

		lds r16, ADCSRA				;Cargar el control ADCSRA
		SBRC r16, 6					;Saltar si se completo la conversion ADCS = 0
		rjmp LEER_ADC0 
			
		lds r17, ADCL				;Cargar parte baja del ADC
		lds r16, ADCH				;Cargar parte alta del ADC
		sts VAL_CorrienteADCL, r17	;Guardar el valor de ADC en VAL_CorrienteADC
		sts VAL_CorrienteADCH, r16
		
		ret
			
;################################################################### SUBRUTINA PARA LEER ADC1 ###################################################################

	ADC1:

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Configuracion ADEMUX ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r18, (0<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0) 
		sts ADMUX, r18				;Referencia de Voltaje Con AVCC y Capacitor, Activar ADC, Canal ADC1    

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Iniciar conversion ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

		ldi r18, (1<<ADEN)|(1<<ADSC)|(0<<ADIE)|(0<<ADATE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0) 
		sts ADCSRA, r18				;Prescaler en 128, Habilito ADC  
		

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Leer ADC1 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	LEER_ADC1:

		lds r18, ADCSRA				;Cargar el control ADCSRA
		sbrc r18, 6					;Saltar si se completo la conversion ADCS = 0
		rjmp LEER_ADC1 
			
		lds r18, ADCL				;Cargar parte baja del ADC
		lds r19, ADCH				;Cargar parte alta del ADC
		sts VAL_TensionADCL, r18	;Guardar el valor de ADC en VAL_TensionADC
		sts VAL_TensionADCH, r19
		
		ret	

;################################################################### CALCULO DE TENSION ###################################################################
	
	CALCULO_TENSION:
		
		lds R23, VAL_TensionADCH
		lds R22, VAL_TensionADCL	;Carga el numero VAL_Tension en r23:r22
		ldi R21,0x00
		ldi R20,0x13				;Carga el numero 19 r21:r20
		call mul16x16_16			;Llamado a rutina de multiplicacion de 16 bits x 16 bits
		sts GRANDEH, r17			;Guardamos resultado de la primer multiplicacion
		sts GRANDEL, r16

		lds R23, VAL_TensionADCH
		lds R22, VAL_TensionADCL	;Carga el numero VAL_Tension en r23:r22
		ldi R21,0x00
		ldi R20,0x05				;Carga el numero 5 r21:r20
		call mul16x16_16			;Llamado a rutina de multiplicacion de 16 bits x 16 bits
		
		lds r19, GRANDEH
		lds r18, GRANDEL
		add r18, r16				;Sumamos primer calculo con segundo calculo
		adc r19, r17
		sts	TensionH, r19			;Valor de la tension
		sts	TensionL, r18

		lds R23, VAL_TensionADCH
		lds R22, VAL_TensionADCL	;Carga el numero VAL_Tension en r23:r22
		ldi R21,0x00
		ldi R20,0x03				;Carga el numero 3 r21:r20
		call mul16x16_16			;Llamado a rutina de multiplicacion de 16 bits x 16 bits
	
		ldi	DivisorL,0x0A			;El dividendo ya esta en r16 y r17
		ldi	DivisorH,0x00			;Dividimos por 10
		call Division16_16

		lds r19, TensionH
		lds r18, TensionL

		add r18, DividendoL			;Sumamos primer calculo con segundo calculo
		adc r19, DividendoH
		sts	TensionH, r19			;Valor de la tension real
		sts	TensionL, r18
		ret

;################################################################### CALCULO DE CORRIENTE ###################################################################
 
	CALCULO_CORRIENTE:

		lds r23, VAL_CorrienteADCH	;Carga valores del ADC
		lds r22, VAL_CorrienteADCL
		ldi R21,0x00
		ldi R20,0x1D				;Carga el numero 29 r21:r20
		call mul16x16_16			;Llamado a rutina de multiplicacion de 16 bits x 16 bits
		ldi DivisorH, 0x00			;Carga 10 en el divisor (para obtener nuestro factor >> 0.0587)
		ldi DivisorL, 0x0A
		call Division16_16			;Llama funcion division
		sts GRANDEH, r17			;Guardamos resultado de la primer multiplicacion 2969
		sts GRANDEL, r16

		lds r23, VAL_CorrienteADCH	;Carga valores del ADC
		lds r22, VAL_CorrienteADCL
		ldi R21,0x00
		ldi R20,0x1D				;Carga el numero 29 r21:r20
		call mul16x16_16			;Llamado a rutina de multiplicacion de 16 bits x 16 bits
		ldi DivisorH, 0x03			;Carga 1000 en el divisor (para obtener nuestro factor >> 0.0587)
		ldi DivisorL, 0xE8
		call Division16_16			;Llama funcion division

		lds r19, GRANDEH
		lds r18, GRANDEL

		add r18, DividendoL			;Sumamos primer calculo con segundo calculo
		adc r19, DividendoH
		sts CorrienteH, r19			;Resultado corriente
		sts CorrienteL, r18
		 				
		ret

;################################################################### CALCULO DE POTENCIA ###################################################################
 
	CALCULO_POTENCIA:

		lds DividendoH, TensionH	;Cargar valor de tension
		lds DividendoL, TensionL
		ldi DivisorH, 0x03			;Cargar 1000 en divisor
		ldi DivisorL, 0xE8
		call Division16_16			;Llamar funcion division
		sts GRANDEH, r17			;Guardar resultado
		sts GRANDEL, r16

		mov DividendoH, RestoH		;Cargar resto para dividir
		mov DividendoL, RestoL
		ldi DivisorH, 0x00			;Cargar 100 en divisor
		ldi DivisorL, 0x64
		call Division16_16			;Llamar funcion division		
		sts Temp1, r16				;Guardar resultado (primer decimal)

		mov DividendoH, RestoH		;Cargar resto para dividir
		mov DividendoL, RestoL
		ldi DivisorH, 0x00			;Cargar 10 en divisor
		ldi DivisorL, 0x0A
		call Division16_16			;Llamar funcion division
		sts Temp2, r16				;Guardar resultado (segundo decimal)
		sts Temp3, RestoL			;Guardar resto (tercer decimal)

		lds r23, CorrienteH			;Cargar valor de corriente
		lds r22, CorrienteL
		lds r21, GRANDEH			;Cargar valor entero de tension
		lds r20, GRANDEL
		call mul16x16_16			;Llamar funcion multiplicacion
		sts PotenciaH, r17			;Guardar resultado temporal de la potencia
		sts PotenciaL, r16

		lds r23, CorrienteH			;Cargar valor de corriente
		lds r22, CorrienteL
		ldi r21, 0x00				;Cargar valor del primer decimal
		lds r20, Temp1
		call mul16x16_16			;Llamar funcion multiplicacion
		ldi DivisorH, 0x00			;Cargar 10 en divisor para acomodar numero
		ldi DivisorL, 0x0A
		call Division16_16			;Llamar funcion division		
		lds r19, PotenciaH			;Cargar valor temporal de potencia
		lds r18, PotenciaL
		add r16, r18				;Sumar potencia con resultado de corriente por primer decimal
		adc r17, r19
		sts PotenciaH, r17			;Guardar resultado temporal de la potencia
		sts PotenciaL, r16

		lds r23, CorrienteH			;Cargar valor de corriente
		lds r22, CorrienteL
		ldi r21, 0x00				;Cargar valor del segundo decimal
		lds r20, Temp2
		call mul16x16_16			;Llamar funcion multiplicacion
		ldi DivisorH, 0x00			;Cargar 100 en divisor para acomodar numero
		ldi DivisorL, 0x64
		call Division16_16			;Llamar funcion division		
		lds r19, PotenciaH			;Cargar valor temporal de potencia
		lds r18, PotenciaL
		add r16, r18				;Sumar potencia con resultado de corriente por primer decimal
		adc r17, r19
		sts PotenciaH, r17			;Guardar resultado temporal de la potencia
		sts PotenciaL, r16

		lds r23, CorrienteH			;Cargar valor de corriente
		lds r22, CorrienteL
		ldi r21, 0x00				;Cargar valor del segundo decimal
		lds r20, Temp3
		call mul16x16_16			;Llamar funcion multiplicacion
		ldi DivisorH, 0x03			;Cargar 1000 en divisor para acomodar numero
		ldi DivisorL, 0xE8
		call Division16_16			;Llamar funcion division		
		lds r19, PotenciaH			;Cargar valor temporal de potencia
		lds r18, PotenciaL
		add r16, r18				;Sumar potencia con resultado de corriente por primer decimal
		adc r17, r19
		sts PotenciaH, r17			;Guardar resultado de la potencia
		sts PotenciaL, r16

		ret

;################################################################### CALCULO DE CORRIENTE PWM ###################################################################
 
	CALCULO_CORRIENTE_PWM:
		
		lds r23, CorrienteH			;Valor corriente
		lds r22, CorrienteL	
		ldi r21, 0x00				;Multiplicar por 2
		ldi r20, 0x02
		call mul16x16_16
		ldi r19, 0x00				;dividir por 10
		ldi r18, 0x0A
		call Division16_16
		sts CorrienteH_PWM, r17
		sts CorrienteL_PWM, r16

		lds r23, CorrienteH			;Valor corriente
		lds r22, CorrienteL	
		ldi r21, 0x00				;Multiplicar por 2
		ldi r20, 0x07
		call mul16x16_16
		ldi r19, 0x00				;dividir por 10
		ldi r18, 0x64
		call Division16_16
		lds r20, CorrienteH_PWM
		lds r19, CorrienteL_PWM
		add r16, r19
		adc r17, r20
		sts CorrienteH_PWM, r17
		sts CorrienteL_PWM, r16

		lds r23, CorrienteH			;Valor corriente
		lds r22, CorrienteL	
		ldi r21, 0x00				;Multiplicar por 2
		ldi r20, 0x02
		call mul16x16_16
		ldi r19, 0x03				;dividir por 10
		ldi r18, 0xE8
		call Division16_16
		lds r20, CorrienteH_PWM
		lds r19, CorrienteL_PWM
		add r16, r19
		adc r17, r20
		ldi r20, 0x00
		ldi r19, 0xCF
		add r16, r19
		adc r17, r20
		sts CorrienteH_PWM, r17
		sts CorrienteL_PWM, r16
		
		ret

;################################################################### CALCULO DE POTENCIA PWM ###################################################################
 
	CALCULO_POTENCIA_PWM:
		
		lds DividendoH, PotenciaH	;Cargar valor de potencia
		lds DividendoL, PotenciaL
		ldi DivisorH, 0x00			;Cargar 10 en divisor
		ldi DivisorL, 0x0A
		call Division16_16			;Llamar funcion division
		sts GRANDEH, r17			;Guardar resultado
		sts GRANDEL, r16

		lds r17, GRANDEH			;Valor de potencia hasta 6000
		lds r16, GRANDEL	
		ldi r19, 0x00				;dividir por 10
		ldi r18, 0x0A
		call Division16_16
		sts PotenciaH_PWM, r17
		sts PotenciaL_PWM, r16

		lds r23, GRANDEH			;Valor corriente
		lds r22, GRANDEL	
		ldi r21, 0x00				;Multiplicar por 2
		ldi r20, 0x03
		call mul16x16_16
		ldi r19, 0x00				;dividir por 100
		ldi r18, 0x64
		call Division16_16
		lds r20, PotenciaH_PWM
		lds r19, PotenciaL_PWM
		add r16, r19
		adc r17, r20
		sts PotenciaH_PWM, r17
		sts PotenciaL_PWM, r16

		lds r23, GRANDEH			;Valor corriente
		lds r22, GRANDEL	
		ldi r21, 0x00				;Multiplicar por 2
		ldi r20, 0x06
		call mul16x16_16
		ldi r19, 0x03				;dividir por 1000
		ldi r18, 0xE8
		call Division16_16
		lds r20, PotenciaH_PWM
		lds r19, PotenciaL_PWM
		add r16, r19
		adc r17, r20
		ldi r20, 0x00
		ldi r19, 0xCF				;Sumar 1 volt
		add r16, r19
		adc r17, r20
		sts PotenciaH_PWM, r17
		sts PotenciaL_PWM, r16
		
		ret
		
;################################################################### FUNCION DE MULTIPLICACION ###################################################################

	mul16x16_16:

		mul	r22, r20				;Multiplica parte baja de A con parte baja de B
		movw	r17:r16, r1:r0
		mul	r23, r20				;Multiplica parte alta de A con parte baja de B
		add	r17, r0
		mul	r21, r22				;Multiplica parte baja de A con parte alta de B
		add	r17, r0
									;Resultado r17, r16
		ret

;############################################################ FUNCION DE MULTIPLICACION DEL RESTO ###################################################################

	mul16x16_24:
		mul		r23, r21			;Multiplica parte alta de A con parte alta de B
		mov		r18, r0
		mul		r22, r20			;Multiplica parte baja de A con parte baja de B
		movw	r17:r16, r1:r0
		mul		r23, r20			;Multiplica parte alta de A con parte baja de Bah * bl
		add		r17, r0
		adc		r18, r1
		mul		r21, r22			;Multiplica parte baja de A con parte alta de B
		add		r17, r0
		adc		r18, r1
									;Resultado r18, r17, r16
		ret


;################################################################### FUNCION DE DIVISION ###################################################################

	Division16_16:	
		clr	RestoL					;Borra byte low de Resto
		sub	RestoH,RestoH			;Borra el byte alto y acarreo
		ldi	Contador,17				;Contador de bucle infinito
		div_1:	
		rol	DividendoL				;Desplazo a la izquierda el dividendo
		rol	DividendoH
		dec	Contador				;Decremento contador
		brne	div_2				;if done
		sts RestodivL, RestoL
		sts RestodivH, RestoH
		ret							;Salida
		div_2:	
		rol	RestoL					;Resplazo a la izquerda el resto
		rol	RestoH
		sub	RestoL,DivisorL			;Resto = resto - divisor
		sbc	RestoH,DivisorH	
		brcc	div_3				;Si el resultado es negativo
		add	RestoL,DivisorL			;Restaurar el resto
		adc	RestoH,DivisorH
		clc							;Limpia el acarreo para ser deplazado al resultado
		rjmp	div_1				;Else
		div_3:	
		sec							;Pone a 1 la bandera de acarreo para ser trasladado al resultado
		rjmp	div_1
									;Resultado de la division r17, r16
									;Resto de la division r15, r14

;########################################################## MOSTRAR POTENCIA #########################################################

	MOSTRAR_POTENCIA:
		
		lds ENTEROH, PotenciaH		;Cargar valor de potencia alta
		lds ENTEROL, PotenciaL		;Cargar valor de potencia baja
		
		call DESCOMPOSICION
				
		call USART_ESPERA			
		ldi r20, 0x50				; P
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
		ldi r20, 0x57				; W
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x61				; a
		sts UDR0, r20

		call USART_ESPERA			
		ldi r20, 0x74				; t
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

		lds r26, UCSR0A				;Espera que se limpie la bandera de transmision
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
		cp ENTEROH,rBin2H			;Comparo nro con comparacion parte alta
		brcs Bin2ToDigitc			;Si Carry=1 el nro es menor a comparacion, vuelve a rutina para comparar con un valor menor
		brne Bin2ToDigitb	 
		cp ENTEROL,rBin2L			;Si es igual, Comparo nro con comparacion parte baja
		brcs Bin2ToDigitc			;Si Carry=1 nro menor a comparacion

	Bin2ToDigitb:			
		sub ENTEROL,rBin2L			;Resto partes bajas
		sbc ENTEROH,rBin2H			;Resto partes altas con carry
		inc rmp						;Incremento cuenta para digito BCD
		rjmp Bin2ToDigita			;Repito el proceso hasta que sea menor a comparacion

	Bin2ToDigitc:
		st z+,rmp					;Salva el digito 
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

;############################################################# SPI ESPERA ############################################################

	SPI_ESPERA:	
		in r26, SPSR		
		sbrs r26, SPIF				; Esperar que se complete la transmisión
		rjmp SPI_ESPERA
		ret

;############################################################# MOSTRAR TENSION MAX ############################################################

	MOSTRAR_TENSION_MAX:
				
		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x06
		out SPDR, r17				;Digito 5
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0b0011_1110
		out SPDR,r17				; U
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x05
		out SPDR, r17				;Digito 5
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0b0000_1001
		out SPDR,r17				; =
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		lds ENTEROH, TensionH		;Cargar valor de tension alta
		lds ENTEROL, TensionL		;Cargar valor de tension baja
		call DESCOMPOSICION
		call TRANSMITIR_MAX
		
		ldi r17, 0xFF
		sts SELECT, r17
		ret

;############################################################# MOSTRAR CORRIENTE MAX ############################################################
	
	MOSTRAR_CORRIENTE_MAX:
		
		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x06
		out SPDR, r17				;Digito 5
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0b0011_0000
		out SPDR,r17				; I
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x05
		out SPDR, r17				;Digito 5
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0b0000_1001
		out SPDR,r17				; =
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		lds ENTEROH, CorrienteH		;Cargar valor de corriente alta
		lds ENTEROL, CorrienteL		;Cargar valor de corriente baja
		call DESCOMPOSICION
		call TRANSMITIR_MAX

		ldi r17, 0xFF
		sts SELECT, r17
		ret
		
;############################################################# MOSTRAR CORRIENTE MAX ############################################################
	
	MOSTRAR_POTENCIA_MAX:

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x06
		out SPDR, r17				;Digito 5
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0b0110_0111
		out SPDR,r17				; P
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop							;Cumplir tcss de hoja de datos MAX
		ldi r17, 0x05
		out SPDR, r17				;Digito 5
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, 0b0000_1001
		out SPDR,r17				; =
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		lds ENTEROH, PotenciaH		;Cargar valor de potencia alta
		lds ENTEROL, PotenciaL		;Cargar valor de potencia baja
		call DESCOMPOSICION
		call TRANSMITIR_MAX
		
		ldi r17, 0xFF
		sts SELECT, r17
		ret
		
;############################################################# TRANSMITIR MAX ############################################################
	
	TRANSMITIR_MAX:
			
	;ENTERO PARTE ALTA
		ldi r17, (0<<PC5)			;Mando 0 a PB0 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop
		ldi r17, 0x04
		out SPDR, r17				;Digito entero alto
		call SPI_ESPERA				;Empezar la TX de información
		nop

		ld	r17, Z+						
		out SPDR, r17				;Envio digito entero alto
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PB0 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

	;ENTERO PARTE BAJA
		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop

		ldi r17, 0x03
		out SPDR, r17				;Digito entero bajo
		call SPI_ESPERA 			;Empezar la TX de información
		nop

		ld	r17, Z+						
		ldi r16, 0xF0
		add r17, r16				;Suma para mostrar puntto decimal
		out SPDR, r17				;Envio digito entero bajo
		call SPI_ESPERA				;Empezar la TX de información
		nop
		
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

	;PRIMER DECIMAL
		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop

		ldi r17, 0x02
		out SPDR, r17				;Digito primer decimal
		call SPI_ESPERA				;Empezar la TX de información
		nop

		ld	r17, Z+					;Primer decimal
		out SPDR, r17				;Envio primer decimal
		call SPI_ESPERA				;Empezar la TX de información
		nop
		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

	;SEGUNDO DECIMAL
		ldi r17, (0<<PC5)			;Mando 0 a PC5 para indicarle a MAX que inicia transferencia de datos
		out PORTC, r17
		nop

		ldi r17, 0x1
		out SPDR, r17				;Digito segundo decimal
		call SPI_ESPERA				;Empezar la TX de información
		nop

		ld	r17, Z+					;Segundo decimal
		out SPDR, r17				;Envio segundo decimal
		call SPI_ESPERA				;Empezar la TX de información
		nop

		ldi r17, (1<<PC5)			;Mando 1 a PC5 para indicarle a MAX que finalizo transferencia
		out PORTC, r17
		nop

		ld	r17, Z
		SBIW ZL, 4
		clr r17

		ret

;############################################################# SUBRUTINA PARA MOSTRAR MAX ############################################################

	SELECT_Magnitud:
		
		lds r16, SELECT
		sbrs r16, 7					;Pregunta si PD7 esta en 0
		call MOSTRAR_POTENCIA_MAX	;Llama funcion para mostrar potencia
		sbrs r16, 6					;Pregunta si PD6 esta en 0
		call MOSTRAR_CORRIENTE_MAX	;Llama funcion para mostrar potencia
		sbrs r16, 5					;Pregunta si PD5 esta en 0
		call MOSTRAR_TENSION_MAX	;Llama funcion para mostrar tension	

		ret

;############################################################# SUBRUTINA RETARDO 100us ############################################################
	RETARDO:
		
		; delaying 1599 cycles:
				  ldi  R17, $0D
		WGLOOP0:  ldi  R18, $28
		WGLOOP1:  dec  R18
				  brne WGLOOP1
				  dec  R17
				  brne WGLOOP0
		; ----------------------------- 
		; delaying 1 cycle:
				  nop

				  ret
		; =============================
			