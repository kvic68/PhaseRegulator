//--------------------------------------------------------
RX_OK:							PUSHF
								push r16
								push r17
								PUSHZ
								lds r16,(ACTIONS_FLAGS)
								sbrc r16,0
								rjmp RX_EXIT				; если предыдущее заклинание не обработано, похериваем всё

								LDZ RX_BUFFER				; Начало буфера приема
								lds r16,UDR0				; пришедший байт
								lds r17,(RX_PTR)
//--------------------------------------------------------
								cpi r16,'a'
								brlo RX_M1
								subi r16,$20				;	в верхний регистр
RX_M1:							cpi r16,'Z'+1
								brlo RX_M2
								rjmp RX_EXIT				;	все что больше Z - не обрабатываем
RX_M2:							cpi r16,'F'+1
								brlo RX_M3
								rjmp RX_PREFIX				;	G...Z - префиксы
RX_M3:							cpi r16,'A'
								brlo RX_M4
								rjmp RX_DIGITS				;	A...F - цифры
RX_M4:							cpi r16,'9'+1
								brlo RX_M5
								rjmp RX_EXIT				;	все что между 9 и A не обрабатываем
RX_M5:							cpi r16,'0'
								brlo RX_M6
								rjmp RX_DIGITS				;	0...9 - цифры
RX_M6:							cpi r16,$0D
								brne RX_M7
								rjmp RX_0D					;	$0D - признак окончания посылки
RX_M7:							rjmp RX_EXIT
//--------------------------------------------------------
RX_PREFIX:						ldi r17,0
								sts (RX_PTR),r17			;	 обнулим указатель буфера приема
								cpi r16,'S'
								breq RX_PREFIX_OK
								cpi r16,'M'
								breq RX_PREFIX_OK
								rjmp RX_EXIT
RX_PREFIX_OK:					st Z,r16					;	запомним нужный пришедший префикс в 0 ячейке буфера приема
								//inc r17
								//sts (RX_PTR),r17			;	увеличим указатель
								rjmp RX_EXIT
//--------------------------------------------------------
RX_DIGITS:						inc r17
								cpi r17,$0F					;	глубина буфера приема
								brlo RX_DIGITS_ADD
								breq RX_DIGITS_ADD
								rjmp RX_EXIT				;	в переполненный буфер цифры не кладем
RX_DIGITS_ADD:					//inc r17
								sts (RX_PTR),r17
								add ZL,r17
								clr r17
								adc ZH,r17
								st Z,r16
								rjmp RX_EXIT
//--------------------------------------------------------
; ---------- Пришел символ #0D
RX_0D:							lds r16,(ACTIONS_FLAGS)
								SETB r16,0						; устанавливаем флаг нового заклинания
								sts (ACTIONS_FLAGS),r16			; в RX_PTR количество символов после управляющего (S,M)
RX_EXIT:
								POPZ
								pop r17
								pop r16
								POPF
								reti
//--------------------------------------------------------
stringDecode:				PUSHF
							cli
								push r16
								push r17
								push r18
								PUSHZ

								LDZ RX_BUFFER
								lds r17,(RX_PTR)				;	длина принятой цифровой последовательности

								ld r16,Z+
stringDecodeCheckS:			cpi r16,'S'
								brne stringDecodeCheckM
								rjmp stringDecodeS
stringDecodeCheckM:			cpi r16,'M'
								brne stringDecodeCheckAnother
								rjmp stringDecode_M
stringDecodeCheckAnother:		rjmp stringDecodeExit
//--------------------------------------------------------
stringDecodeS:					cpi r17,3
								breq stringDecodeS_3Dig
								cpi r17,2
								ldi r18,0
								breq stringDecodeS_2Dig
								cpi r17,1
								ldi r17,0
								breq stringDecodeS_1Dig
								rjmp stringDecodeExit

stringDecodeS_3Dig:				ld r16,Z+
								call strToHex
								brcs stringDecodeS_Exit
								mov r18,r16
					
stringDecodeS_2Dig:				ld r16,Z+
								call strToHex
								brcs stringDecodeS_Exit
								swap r16
								mov r17,r16

stringDecodeS_1Dig:				ld r16,Z+
								call strToHex
								brcs stringDecodeS_Exit
								or r17,r16

								// проверка на выход за максимальное значение
								ldi r16, low(MAX_U+1)
								cp  r17,r16
								ldi r16,high(MAX_U+1)
								cpc r18,r16
								brcc stringDecodeS_Exit

stringDecodeNormalValue:		lds r16,(TARGET_U+0)
								cp  r16,r17
								lds r16,(TARGET_U+1)
								cpc r16,r18
								breq stringDecodeS_Exit		;	если новое равно старому, то не обновляем
								sts (TARGET_U+1),r18
								sts (TARGET_U+0),r17
								
								ldi r16,TargetV
								call setNewValFlag
								ldi r16,MainV
								call setNewValFlag
					
stringDecodeS_Exit:				rjmp stringDecodeExit
//--------------------------------------------------------
stringDecode_M:					cpi r17,1
								breq stringDecode_M1
								rjmp stringDecodeExit

stringDecode_M1:				ld r16,Z+
								call strToHex
								brcs stringDecodeM_Err

								cpi r16,3
								brcs stringDecode_M2
								rjmp stringDecodeExit

stringDecode_M2:				lds r17,(MODE)
								cp r16,r17
								breq stringDecodeExit		;	если режим не новый, то не меняем

								call setMode

stringDecodeM_Err:				rjmp stringDecodeExit
//--------------------------------------------------------
stringDecodeExit:				clr r16
								sts (RX_PTR),r16
								lds r16,(ACTIONS_FLAGS)
								CLRB r16,0
								sts (ACTIONS_FLAGS),r16

								POPZ
								pop r18
								pop r17
								pop r16
								POPF
								ret
//--------------------------------------------------------
//--------------------------------------------------------
UD_OK:							PUSHF
								push r16
								push r17
								PUSHZ

								LDZ TX_BUFFER
								lds r16,(TX_PTR)
								ldi r17,0
								add ZL,r16
								adc ZH,r17
								inc r16
								cpi r16,$20
								brcc StopTx

								sts (TX_PTR),r16
								ld r16,Z+

								cpi r16,0
								breq StopTx

								sts UDR0,r16

UD_Exit:						POPZ
								pop r17
								pop r16
								POPF
								reti
//--------------------------------------------------------
StopTx:							ldi r16,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0)|(0<<UDRIE0)
								sts UCSR0B,r16
								clr r16
								sts (TX_BUSY),r16
								rjmp UD_Exit
//--------------------------------------------------------
RunTX:							ldi r16,$FF
								sts (TX_BUSY),r16
								ldi r16,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0)|(1<<UDRIE0)
								sts UCSR0B,r16
								ret
//--------------------------------------------------------00
nothingToDo:					ret
//--------------------------------------------------------01
PrepareDataToSend:				PUSHF
								cli
								push r16
								push r17
								PUSHX
								PUSHY
								PUSHZ

								LDX TX_BUFFER

								ldi r16,'T'
								st X+,r16
								;---------- average U
								LDY AVERAGE_U
								ldd r16,Y+1
								call HexToAscii
								st X+,r16
								ldd r16,Y+0
								call HexToAscii
								st X+,r17
								st X+,r16
								;---------- target U
PrepareTarget:					LDY TARGET_U
								ldd r16,Y+1
								call HexToAscii
								st X+,r16
								ldd r16,Y+0
								call HexToAscii
								st X+,r17
								st X+,r16
								;---------- mode
PrepareMode:					LDY MODE
								ld r16,Y
								andi r16,$03
								call HexToAscii
							;	st X+,r17
								st X+,r16
								;----------
								ldi r16,$0D
								st X+,r16
								ldi r16,$0A
								st X+,r16

								ldi r16,$00
								st X+,r16
								st X+,r16
								sts (TX_PTR),r16

								POPZ
								POPY
								POPX
								pop r17
								pop r16
								POPF
								ret
//--------------------------------------------------------
//--------------------------------------------------------
TXmanager:						push r16
								push r17
								PUSHZ
								lds r16,(TX_BUSY)
								tst r16
								brne TXmanExit

								lds r16,(TX_QUEUE_POINTER)
								cpi r16,$00
								brne TXmanRun

								lds r16,(TX_QUEUE)
								cpi r16,$00
								brne TXmanRun
								rjmp TXmanExit

TXmanRun:						call TXprepare
								call TXQueueShifter
								call runTX	

TXmanExit:						POPZ
								pop r17
								pop r16
								ret
//--------------------------------------------------------
//--------------------------------------------------------
TXprepare:						LDZ TXVariantsTable*2
								lds r16,(TX_QUEUE)
								andi r16,$07
								clr r17
								add ZL,r16	adc ZH,r17
								add ZL,r16	adc ZH,r17
								lpm r16,Z+
								lpm r17,Z
								movw ZH:ZL,r17:r16

								icall

								ret
//--------------------------------------------------------
TXVariantsTable: 				.dw	nothingToDo				; 00 - ничего не передаем
								.dw	PrepareDataToSend		; 01 - стандартная посылка данных					
//--------------------------------------------------------
//--------------------------------------------------------
TXQueueShifter:					LDZ TX_QUEUE
								ldi r17,7

TXQueueShifterLoop:				tst r17
								breq TXQueueShifterEndLoop

								ldd r16,Z+1
								st Z+,r16
								dec r17
								rjmp TXQueueShifterLoop

TXQueueShifterEndLoop:			clr r16
								st Z,r16

								lds r16,(TX_QUEUE_POINTER)
								tst r16
								breq TXQueueShifterExit
								dec r16
								sts (TX_QUEUE_POINTER),r16
TXQueueShifterExit:
								ret
//--------------------------------------------------------
//	r16 - номер вида отправляемых данных
//--------------------------------------------------------
.equ	QUEUE_DEPTH = 8; глубина очереди передачи
TXQueueAdd:						PUSHZ
								push r17
								push r18

								tst r16							;	если запрос не 0
								brne TXQueueAddCheckDepth		;	идём на проверку заполненности буфера
								rjmp TXQueueAddExit				;	иначе на выход

TXQueueAddCheckDepth:			lds r17,(TX_QUEUE_POINTER)		;	проверим указатель очереди
								cpi r17,QUEUE_DEPTH-1			;	если он не на максимуме
								brlo TXQueueAddsCheckDuplicate	;	то идём на проверку наличия такого запроса в очереди
								rjmp TXQueueAddExit				;	иначе на выход

TXQueueAddsCheckDuplicate:		LDZ TX_QUEUE					;	в Z начало очереди, в r17 текущая глубина

TXQueueAddsCheckDuplicateCycle:	tst r17							;	если ничего в очереди не нашли
								breq TXQueueAddNew				;	то можно добавить запрос в очередь
								ld r18,Z+						;	прочитаем запрос из очереди (указатель сдвинем на следующий элемент)
								cp r16,r18						;	сравним с текущим
								breq TXQueueAddExit				;	и, если уже есть такой, выходим
								dec r17							;	переходим к следующему элементу очереди
								rjmp TXQueueAddsCheckDuplicateCycle

TXQueueAddNew:					st Z,r16						;	кладем запрос в очередь
								lds r17,(TX_QUEUE_POINTER)		;
								inc r17							;	увеличиваем указатель на первый свободный элемент очереди
								sts (TX_QUEUE_POINTER),r17		;

TXQueueAddExit:					pop r18
								pop r17
								POPZ
								ret
//--------------------------------------------------------
strToHex:						cpi r16,'0'
								brcs strToHexError
								cpi r16,'9'+1
								brcc strToHexCheck1
								subi r16,$30
								clc
								rjmp strToHexExit
strToHexCheck1:					cpi r16,'A'
								brcs strToHexError
								cpi r16,'F'+1
								brcc strToHexCheck2
								subi r16,$37
								clc 
								rjmp strToHexExit
strToHexCheck2:					cpi r16,'a'
								brcs strToHexError
								cpi r16,'f'+1
								brcc strToHexError
								subi r16,$57
								clc 
								rjmp strToHexExit
strToHexError:					sec
strToHexExit:					andi r16,$0F
								ret
//--------------------------------------------------------
; Превращение HEX в два ASCII
; R16 - входное, выход: R17 - старший, R16 - младший
HexToAscii:						PUSHF
								mov r17,r16
								andi r16,$0F
								swap r17
								andi r17,$0F
								subi r16,(-$30)
								cpi r16,$3A
								brcs HexToAsciiM1
								subi r16,(-7)
HexToAsciiM1:					subi r17,(-$30)
								cpi r17,$3A
								brcs HexToAsciiM2
								subi r17,(-7)
HexToAsciiM2:					POPF
								ret
//--------------------------------------------------------

