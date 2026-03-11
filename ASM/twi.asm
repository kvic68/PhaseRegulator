.equ		i2c_MaxBytesRX = 3				;	максимальное количество байтов для приема
.equ		i2c_MaxBytesTX = 5				;	максимальное количество байтов для передачи
; ----------------------------------------------------------------------------------------
.dseg
i2c_RXBytesIndex:		.BYTE 1						;	счетчик принятых байтов
i2c_RXBuffer:			.BYTE(i2c_MaxBytesRX)		;	буфер приема
i2c_TXBuffer:			.BYTE(i2c_MaxBytesTX)		;	буфер передачи
.cseg
; ----------------------------------------------------------------------------------------
TWI:						PUSHF
							push r16
							push r17
							push ZL
							push ZH

							ldi ZL,low (TWISubsTable*2)	;	таблица адресов вызываемых подпрограмм
							ldi ZH,high(TWISubsTable*2)
						
							lds r16,TWSR						;	прочитаем байт состояния приемника
							andi r16,$F8
							lsr r16
							lsr r16
							lsr r16
							clr r17
							add ZL,r16	adc ZH,r17
							add ZL,r16	adc ZH,r17		;	указатель на адрес подпрограммы в Z

							lpm r16,Z+
							lpm r17,Z					;	адрес подпрограммы из таблицы в r16...17
							movw ZH:ZL,r17:r16			;	адрес подпрограммы в Z
			
							icall						;	косвенный вызов п/п, соответствующей байту состояния

TWI_exit:					pop ZH
							pop ZL
							pop r17
							pop r16
							POPF
							reti
; ----------------------------------------------------------------------------------------
TWISubsTable:	;---  Master  ---
				.dw case_0x00			; 00		Bus Fail (автобус сломался)
				.dw TWI_NoAction		; 08		Старт был
				.dw TWI_NoAction		; 10		Повторный старт был
				.dw TWI_NoAction		; 18		Был послан адрес на запись SLA+W получили ACK
				.dw TWI_NoAction		; 20		Был послан адрес на запись SLA+W получили NACK - слейв либо занят, либо его нет дома.
				.dw TWI_NoAction		; 28		Байт данных послали, получили ACK
				.dw TWI_NoAction		; 30		Байт ушел, но получили NACK причин две. 1я передача оборвана слейвом и так надо. 2я слейв сглючил.
				.dw TWI_NoAction		; 38		Коллизия на шине. Нашелся кто то поглавней
				.dw TWI_NoAction		; 40		Послали адрес на чтение SLA+R получили АСК.
				.dw TWI_NoAction		; 48		Послали адрес на чтение SLA+R получили NACK. Видать slave занят или его нет дома. 
				.dw TWI_NoAction		; 50		Приняли байт.
				.dw TWI_NoAction		; 58		Вот мы взяли последний байт, сказали NACK слейв обиделся и отпал.
			;---  Slave  ---
				.dw case_0x60			; 60		RCV SLA+W  Incoming?	// Или просто получили свой адрес на запись
				.dw TWI_NoAction		; 68		RCV SLA+W Low Priority	// Словили свой адрес на запись во время передачи мастером
				.dw TWI_NoAction		; 70		RCV SLA+W  Incoming? (Broascast)	// Или широковещательный пакет
				.dw TWI_NoAction		; 78		RCV SLA+W Low Priority (Broadcast)	// Или это был широковещательный пакет
				.dw case_0x80			; 80		RCV Data Byte			// И вот мы приняли этот байт
				.dw case_0x88			; 88		RCV Last Byte			// Приняли последний байт
				.dw TWI_NoAction		; 90		RCV Data Byte (Broadcast)
				.dw TWI_NoAction		; 98		RCV Last Byte (Broadcast)
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
case_0x00:					ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
							sts TWCR,r16
							ret
; ----------------------------------------------------------------------------------------
case_0x60:		;	получили свой адрес на запись
							ldi r16,0
							sts i2c_RXBytesIndex,r16
							ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
							sts TWCR,r16
							ret
; ----------------------------------------------------------------------------------------
case_0x80:		;	RCV Data Byte	Получили байт		
							clr r16
							lds r17,i2c_RXBytesIndex
							LDZ i2c_RXBuffer
							add ZL,r17
							adc ZH,r16
							lds r16,TWDR
							st Z,r16
							inc r17						;	увеличим счетчик принятых байтов
							sts i2c_RXBytesIndex,r17
							ldi r16,i2c_MaxBytesRX
							dec r16
							cp r17,r16				;	сравним счетчик принятых байтов с максимально допустимым количеством - 1
							brne case_RCV_NEXT
							ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|0<<TWEA|1<<TWEN|1<<TWIE
							rjmp case_RCV_LAST
case_RCV_NEXT:				ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
case_RCV_LAST:				sts TWCR,r16
							ret
; ----------------------------------------------------------------------------------------
case_0x88:		;	RCV Last Byte Получили последний байт
							clr r16
							lds r17,i2c_RXBytesIndex
							LDZ i2c_RXBuffer
							add ZL,r17
							adc ZH,r16
							lds r16,TWDR
							st Z,r16
							inc r17
							sts i2c_RXBytesIndex,r17
				// сообщить, что приём окончен, пора обрабатывать
							lds r16,(ACTIONS_FLAGS)
							SETB r16,1
							sts (ACTIONS_FLAGS),r16
							ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
							sts TWCR,r16
							ret
; ----------------------------------------------------------------------------------------
case_0xA0:		;	получили Повторный старт или стоп
				// сообщить, что приём окончен, пора обрабатывать
							lds r16,(ACTIONS_FLAGS)
							sbrc r16,1
							rjmp case_A0_1
							SETB r16,1
							sts (ACTIONS_FLAGS),r16
							ldi r16,0
							sts i2c_RXBytesIndex,r16
case_A0_1:					ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
							sts TWCR,r16
							ret
; ----------------------------------------------------------------------------------------
case_0xA8:		;	словили свой адрес на чтение
							call PrepareI2Cdata					;	подготовим данные для отправки
							ldi r17,0
							sts i2c_RXBytesIndex,r17				;	счетчик передаваемых байтов = 0
							LDZ i2c_TXBuffer
							clr r16
							add ZL,r17
							adc ZH,r16
							ld r16,Z								;	загружаем первый байт в передатчик
							sts TWDR,r16
							ldi r16,i2c_MaxBytesTX
							ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
							sts TWCR,r16
							ret
; ----------------------------------------------------------------------------------------
case_0xB8:		;	Послали байт, получили ACK
							clr r16
							lds r17,i2c_RXBytesIndex
							inc r17
							sts i2c_RXBytesIndex,r17
							LDZ i2c_TXBuffer
							add ZL,r17
							adc ZH,r16
							ld r16,Z
							sts TWDR,r16
							ldi r16,i2c_MaxBytesTX
							dec r16
							cp r17,r16
							brne case_SEND_NEXT
							ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|0<<TWEA|1<<TWEN|1<<TWIE
							rjmp case_SEND_LAST
case_SEND_NEXT:				ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
case_SEND_LAST:				sts TWCR,r16
							ret
; ----------------------------------------------------------------------------------------
case_0xC0:
case_0xC8:					ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
							sts TWCR,r16
							ret
; ----------------------------------------------------------------------------------------
TWI_NoAction:				ret
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
PrepareI2Cdata:				PUSHF
							cli
							push r16
							push r17

					// обновляем данные для iic
							lds r16,(TARGET_U+0)
							sts (i2c_TXBuffer+0),r16
							lds r16,(TARGET_U+1)
							sts (i2c_TXBuffer+1),r16
							lds r16,(AVERAGE_U+0)
							sts (i2c_TXBuffer+2),r16
							lds r16,(AVERAGE_U+1)
							sts (i2c_TXBuffer+3),r16
				// Пакет в один байт три значения
				// 0..3 - MODE, 4 - FL_RANGE, 5 - FL_MAX_U
							lds r16,(MODE)
							andi r16,$0F
							lds r17,(FL_RANGE)
							andi r17,$10
							or r16,r17
							lds r17,(FL_MAX_U)
							andi r17,$20
							or r16,r17
							sts (i2c_TXBuffer+4),r16

							pop r17
							pop r16
							POPF
							ret
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
TWIReceived:					push r16
								push r17
								push r18

								lds r16,(i2c_RXBytesIndex)	;	смотрим, сколько байтов получено
								cpi r16,1
								brne TWIReceived_2
								rjmp TWIReceived_1B

TWIReceived_2:					cpi r16,2
								brne TWIReceived_3
								rjmp TWIReceived_2B

TWIReceived_3:					cpi r16,3
								brne TWIReceived_more
								rjmp TWIReceived_3B

TWIReceived_more:				rjmp TWIReceived_Ex
// -------------------------------------------------------------
// пришёл один байт, это управление режимом

TWIReceived_1B:					call TWIReceivedChangeMode
								rjmp TWIReceived_Ex
// -------------------------------------------------------------
// пришло два байта, это целевое значение напряжения

TWIReceived_2B:					call TWIReceivedChangeTarget
								rjmp TWIReceived_Ex		
// -------------------------------------------------------------
// пришло три байта, первые два - цель, третий - режим работы

TWIReceived_3B:					call TWIReceivedChangeTarget			;	меняем цель
								lds r16,(i2c_RXBuffer+2)
								sts (i2c_RXBuffer+0),r16
								call TWIReceivedChangeMode				;	меняем режим
// -------------------------------------------------------------
TWIReceived_Ex:					ldi r16,$FF
								sts TWDR,r16

								pop r18
								pop r17
								pop r16
								ret
// -------------------------------------------------------------
TWIReceivedChangeMode:			lds r16,(i2c_RXBuffer+0)
TWIReceivedChangeModeA:			lds r17,(MODE)
								cp r16,r17							;	если требуемый режим не такой же, что и текущий
								brne TWIReceivedChangeModeRun		;	тогда идем его менять
								rjmp TWIReceivedChangeModeExit		;	иначе на выход

TWIReceivedChangeModeRun:		cpi r16,3							;	если режим не в диапазоне 0...3
								brcs TWIReceived_1B_Ok
								breq TWIReceived_1B_Ok
								rjmp TWIReceivedChangeModeExit		;	то на выход

TWIReceived_1B_Ok:				lds r17,(MODE)
								cpi r17,3							;	если режим = 3 (внешний стоп)
								brne TWIReceived_1Mode_Ok
								rjmp TWIReceivedChangeModeExit		;	то ничего не меняем и на выход

TWIReceived_1Mode_Ok:			call setMode						;	то устанавливаем запрошенный режим

TWIReceivedChangeModeExit:		ret									;	и возвращаемся к вызывающему

// -------------------------------------------------------------
TWIReceivedChangeTarget:		lds r16,(i2c_RXBuffer+0)
								lds r17,(i2c_RXBuffer+1)

TWIReceivedChangeTargetA:		sbrc r17,7							;	если затребовано отрицательное значение
								rjmp TWIReceivedChangeTargetExit	;	то на выход

								ldi r18,low (MAX_U+1)
								cp  r16,r18
								ldi r18,high(MAX_U+1)
								cpc r17,r18							;	если затребовано больше максимума (250.0)
								brcc TWIReceivedChangeTargetExit	;	то на выход

								lds r18,(TARGET_U+0)
								cp  r16,r18
								lds r18,(TARGET_U+1)
								cpc r17,r18							;	если затребованное равно текущему
								breq TWIReceivedChangeTargetExit	;	то на выход без изменений

								sts (TARGET_U+0),r16
								sts (TARGET_U+1),r17

								ldi r16,TargetV
								call setNewValFlag
								ldi r16,MainV
								call setNewValFlag

TWIReceivedChangeTargetExit:	ret

