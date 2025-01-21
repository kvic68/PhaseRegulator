.equ		i2c_MasterBytesRX = 3				;	максимальное количество байтов для приема
.equ		i2c_MasterBytesTX = 5				;	максимальное количество байтов для передачи
; ----------------------------------------------------------------------------------------
.dseg
i2c_SlaveIndex:		.BYTE 1						;	счетчик принятых байтов
i2c_InBuff:			.BYTE(i2c_MasterBytesRX)	;	буфер приема
i2c_OutBuff:		.BYTE(i2c_MasterBytesTX)	;	буфер передачи
.cseg
; ----------------------------------------------------------------------------------------
TWIReceived:	//sbi portb,5
				PUSHF
				push r16
				push r17
				push r18
				lds r16,(i2c_SlaveIndex)
//				dec r16
//rjmp TWI_3B
					cpi r16,1
					brne TWIR_2
					rjmp TWI_1B

TWIR_2:				cpi r16,2
					brne TWIR_3
					rjmp TWI_2B

TWIR_3:				cpi r16,3
					brne TWIR_4
					rjmp TWI_3B

TWIR_4:				rjmp TWIR_Ex

// пришёл один байт, это управление режимом
TWI_1B:			lds r16,(i2c_InBuff+0)
				cpi r16,3
				brcs TWIR1BOk
				breq TWIR1BOk
				rjmp TWIR_Ex

TWIR1BOk:		lds r17,(MODE)
				cpi r17,3
				brne TWIR1BOk2
				rjmp TWIR_Ex

TWIR1BOk2:		sts (MODE),r16
				lds r16,$FF
				sts (FL_NEW_M),r16
				rjmp TWIR_Ex

// пришло два байта, это целевое значение напряжения
TWI_2B:			lds r16,(i2c_InBuff+0)
				lds r17,(i2c_InBuff+1)
				sbrc r17,7
				rjmp TWIR_Ex
				ldi r18,low (MAX_U)
				cp  r16,r18
				ldi r18,high(MAX_U)
				cpc r17,r18
				breq TWIR2BOk
				brcs TWIR2BOk
				rjmp TWIR_Ex
TWIR2BOk:		sts (TARGET_U+0),r16
				sts (TARGET_U+1),r17
				ldi r16,$FF
				sts (FL_NEW_U),r16
				rjmp TWIR_Ex

// пришло три байта, первые два - цель, третий - режим работы
TWI_3B:			lds r16,(i2c_InBuff+0)
				lds r17,(i2c_InBuff+1)
				sbrc r17,7
				rjmp TWIR3BNoU

				ldi r18,low (MAX_U)
				cp  r16,r18
				ldi r18,high(MAX_U)
				cpc r17,r18
				breq TWIR3BOk1
				brcs TWIR3BOk1
				rjmp TWIR3BNoU

TWIR3BOk1:		sts (TARGET_U+0),r16
				sts (TARGET_U+1),r17
				ldi r16,$FF
				sts (FL_NEW_U),r16
				rjmp TWIR_Ex

TWIR3BNoU:		lds r16,(i2c_InBuff+2)
				cpi r16,3
				brcs TWIR3BOk2
				//breq TWIR3BOk2
				rjmp TWIR_Ex

TWIR3BOk2:		lds r17,(MODE)
				cpi r17,3
				brne TWIR3BOk3
				rjmp TWIR_Ex

TWIR3BOk3:		sts (MODE),r16
				lds r16,$FF
				sts (FL_NEW_M),r16
				rjmp TWIR_Ex
TWIR_Ex:
				pop r18
				pop r17
				pop r16
				POPF
			ret
; ----------------------------------------------------------------------------------------
TWI:		
				//ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
				//sts TWCR,r16
				//reti
push r16
			push r17
			push ZL
			push ZH
					ldi ZL,low (TWISubsTable*2)
					ldi ZH,high(TWISubsTable*2)
			lds r16,TWSR
			andi r16,$F8
			lsr r16
			lsr r16
			lsr r16
					clr r17
					add ZL,r16	adc ZH,r17
					add ZL,r16	adc ZH,r17

					lpm r16,Z+
					lpm r17,Z
					movw ZH:ZL,r17:r16
					icall
TWI_exit:	pop ZH
			pop ZL
			pop r17
			pop r16
			reti
; ----------------------------------------------------------------------------------------
TWISubsTable:	;---  Master  ---
				.dw case_0x00			; 00		Bus Fail (автобус сломался)
				.dw TWI_NoAction		; 08		Старт был
				.dw TWI_NoAction		; 10		Повторный старт был
				.dw TWI_NoAction		; 18		Был послан SLA+W получили ACK
				.dw TWI_NoAction		; 20		Был послан SLA+W получили NACK - слейв либо занят, либо его нет дома.
				.dw TWI_NoAction		; 28		Байт данных послали, получили ACK
				.dw TWI_NoAction		; 30		Байт ушел, но получили NACK причин две. 1я передача оборвана слейвом и так надо. 2я слейв сглючил.
				.dw TWI_NoAction		; 38		Коллизия на шине. Нашелся кто то поглавней
				.dw TWI_NoAction		; 40		Послали SLA+R получили АСК.
				.dw TWI_NoAction		; 48		Послали SLA+R, но получили NACK. Видать slave занят или его нет дома. 
				.dw TWI_NoAction		; 50		Приняли байт.
				.dw TWI_NoAction		; 58		Вот мы взяли последний байт, сказали NACK слейв обиделся и отпал.
				;---  Slave  ---
				.dw case_0x60			; 60	RCV SLA+W  Incoming?				// Или просто получили свой адрес
				.dw TWI_NoAction		; 68	RCV SLA+W Low Priority				// Словили свой адрес во время передачи мастером
				.dw TWI_NoAction;case_0x70			; 70	RCV SLA+W  Incoming? (Broascast)	// Или широковещательный пакет
				.dw TWI_NoAction		; 78	RCV SLA+W Low Priority (Broadcast)	// Или это был широковещательный пакет
				.dw case_0x80			; 80	RCV Data Byte						// И вот мы приняли этот байт
				.dw case_0x88			; 88	RCV Last Byte						// Приняли последний байт
				.dw TWI_NoAction;case_0x90			; 90	RCV Data Byte (Broadcast)
				.dw TWI_NoAction;case_0x98			; 98	RCV Last Byte (Broadcast)
				.dw case_0xA0			; A0		получили Повторный старт или стоп
				.dw case_0xA8			; A8		просто словили свой адрес на чтение
				.dw TWI_NoAction		; B0		Поймали свой адрес на чтение во время передачи Мастером
				.dw case_0xB8			; B8		Послали байт, получили ACK
				.dw case_0xC0			; C0		Мы выслали последний байт, больше у нас нет, получили NACK
				.dw case_0xC8			; C8		или ACK. В данном случае нам пох. Т.к. больше байтов у нас нет
				;---         ---
				.dw TWI_NoAction		; D0
				.dw TWI_NoAction		; D8
				.dw TWI_NoAction		; E0
				.dw TWI_NoAction		; E8
				.dw TWI_NoAction		; F0
				.dw TWI_NoAction		; F8

; ----------------------------------------------------------------------------------------
case_0x00:		ldi r16,0<<TWSTA|1<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
				sts TWCR,r16
				ret
; ----------------------------------------------------------------------------------------
case_0x68:
case_0x78:
case_0x60:		;	просто получили свой адрес
case_0x70:		ldi r16,0
				sts i2c_SlaveIndex,r16
				lds r16,i2c_MasterBytesRX
				cpi r16,1
				brne case_70_1
				ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|0<<TWEA|1<<TWEN|1<<TWIE
				rjmp case_70_2
case_70_1:		ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
case_70_2:		sts TWCR,r16
				ret
; ----------------------------------------------------------------------------------------
case_0x80:		;	RCV Data Byte		
case_0x90:		clr r16
				lds r17,i2c_SlaveIndex
				LDZ i2c_InBuff
				add ZL,r17
				adc ZH,r16
				lds r16,TWDR
				st Z,r16
				inc r17
				sts i2c_SlaveIndex,r17
				ldi r16,i2c_MasterBytesRX
				dec r16
				cp r17,r16
				brne case_90_1
				ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|0<<TWEA|1<<TWEN|1<<TWIE
				rjmp case_90_2
case_90_1:		ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
case_90_2:		sts TWCR,r16
		//sbi portb,5
				ret
; ----------------------------------------------------------------------------------------
case_0x88:		;	RCV Last Byte
case_0x98:		clr r16
				lds r17,i2c_SlaveIndex
				LDZ i2c_InBuff
				add ZL,r17
				adc ZH,r16
				lds r16,TWDR
				st Z,r16
					inc r17
					sts i2c_SlaveIndex,r17
				ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
				sts TWCR,r16
				// сообщить, что приём окончен, пора обрабатывать
					lds r16,(ACTIONS_FLAGS)
					SETB r16,1
					sts (ACTIONS_FLAGS),r16
				ret
; ----------------------------------------------------------------------------------------
case_0xA0:		;	получили Повторный старт или стоп
				ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
				sts TWCR,r16
				// сообщить, что приём окончен, пора обрабатывать
					lds r16,(ACTIONS_FLAGS)
					SETB r16,1
					sts (ACTIONS_FLAGS),r16
		//sbi portb,5
				ret
; ----------------------------------------------------------------------------------------
case_0xB0:
case_0xA8:		;	словили свой адрес на чтение
				ldi r17,0
				sts i2c_SlaveIndex,r17
				LDZ i2c_OutBuff
				clr r16
				add ZL,r17
				adc ZH,r16
				ld r16,Z
				sts TWDR,r16
				lds r16,i2c_MasterBytesTX
				cpi r16,1
				brne case_A8_1
				ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|0<<TWEA|1<<TWEN|1<<TWIE
				rjmp case_A8_2
case_A8_1:		ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
case_A8_2:		sts TWCR,r16
				ret
; ----------------------------------------------------------------------------------------
case_0xB8:		;	Послали байт, получили ACK
				clr r16
				lds r17,i2c_SlaveIndex
				inc r17
				sts i2c_SlaveIndex,r17
				LDZ i2c_OutBuff
				add ZL,r17
				adc ZH,r16
				ld r16,Z
				sts TWDR,r16
				ldi r16,i2c_MasterBytesTX
				dec r16
				cp r17,r16
				brne case_B8_1
				ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|0<<TWEA|1<<TWEN|1<<TWIE
				rjmp case_B8_2
case_B8_1:		ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
case_B8_2:		sts TWCR,r16
				ret
; ----------------------------------------------------------------------------------------
case_0xC0:
case_0xC8:		ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
				sts TWCR,r16
				ret
; ----------------------------------------------------------------------------------------
TWI_NoAction:	ret
; ----------------------------------------------------------------------------------------
