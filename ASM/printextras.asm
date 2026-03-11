//	----------------------------------------------------------------
// -- вывод "экстры", код в r16
//		вызывается из printString
//	----------------------------------------------------------------
printExtras:					cpi r16,OuterExtraNumber		;	номер старшей экстры плюс 1
								brcc printExtrasExit			;	если больше или равно, то на выход
								cpi r16,$80						;	номер младшей экстры
								brcs printExtrasExit			;	если меньше, то на выход

								movw X,Z						;	сохраним указатель Z в X
								LDZ printExtraRoutinesTable*2
								// ------------------------------
								LDY ExtrasConfig
								push r16

									subi r16,$80
									ldi r17,0

									add ZL,r16	adc ZH,r17
									add ZL,r16	adc ZH,r17

									add YL,r16	adc YH,r17		;	в Y указатель на конфигурацию
									add YL,r16	adc YH,r17		;	выводимой экстры

									lpm r16,Z+
									lpm r17,Z

									movw ZH:ZL,r17:r16			;	адрес программы вывода

									lds r16,(POSX)
									std Y+0,r16

									lds r17,(POSY)
									andi r17,$07				;	позиция по Y биты 0..2
									lds r16,(FontNo)
									andi r16,$03
									swap r16
									or r17,r16
									SETB r17,6					;	разрешение к выводу бит 6
									CLRB r17,7					;	новое значение обнулить бит 7
									std Y+1,r17
								pop r16
								// ------------------------------
								icall

								movw Z,X						;	восстановим указатель Z из X
printExtrasExit:				ret
//	----------------------------------------------------------------
//--------------------------------------------------------------------------------
// ---------- Таблица вызовов подпрограмм печати спецсимволов (#80-#BF) ----------
printExtraRoutinesTable:
			.dw	nothingToPrint			; #80 - конец строки	
			.dw	selectFont				; #81 - переключить на шрифт No, номер в следующем байте
			.dw	drawLine				; #82 - нарисовать линию. строка, начальный Х, конечный Х, паттерн	
			.dw	setXY					; #83 - позиционировать XY абсолютно, два следующих байта координаты	
			.dw	nothingToPrint			; #84 - 
			.dw nothingToPrint			; #85 - 	
			.dw nothingToPrint			; #86 -
			.dw nothingToPrint			; #87 - 

			.dw printRange				; #88 - Индикатор стабилизации 	
			.dw printTargetV			; #89 - Вольтметр целевого напряжения
			.dw printOutputV			; #8A - Вольтметр измеренного напряжения	

			.dw printMainV				; #8B - 
			.dw nothingToPrint			; #8C - 
			.dw nothingToPrint			; #8D - 
			.dw nothingToPrint			; #8E - 
			.dw nothingToPrint			; #8F - 

//--------------------------------------------------------------------------------
nothingToPrint:					ret
//	---------------------------------------------------------------- 81
selectFont:						movw Z,X
								lpm r16,Z+			; читаем номер шрифта
								andi r16,$03
								sts (FontNo),r16	; и записываем его в нужное место
								movw X,Z
								ret
//	---------------------------------------------------------------- 82
drawLine:						movw Z,X
								push r18
								push r19
								lpm r17,Z+	; координата Y
								lpm r18,Z+	; начало X
								lpm r19,Z+	; конец X
								lpm r16,Z+	; образец заполнения
								inc r19
								push r16
									sts (POSX),r18
									sts (POSY),r17
									call SET_LED_XY
									call softI2CStart
									lds r16,IndicatorAddr
									call softI2CSendByte
									ldi r16,datastream
									call softI2CSendByte
								pop r16

drawLineCycle:					cp r18,r19
								breq drawLineExit
								brcc drawLineExit
								push r16
									call softI2CSendByte
								pop r16
								inc r18
								rjmp drawLineCycle
drawLineExit:					call softI2CStop
								pop r19
								pop r18
								movw X,Z
								ret
//	---------------------------------------------------------------- 83
setXY:							movw Z,X
								lpm r16,Z+			; читаем X
								cpi r16,128
								brcs setXYnext
								ldi r16,127
setXYnext:						sts POSX,r16
								lpm r16,Z+			; читаем Y
								andi r16,$07
								sts POSY,r16
								movw X,Z
								call SET_LED_XY		; позиционируем
								ret
//	---------------------------------------------------------------- 88
printRange:						lds r16,(FL_RANGE)
								cpi r16,$00
								breq printNoRange
								ldi r16,$2C		call printSymbol
								ldi r16,$2D		call printSymbol
								rjmp printRangeEx
printNoRange:					ldi r16,$2A		call printSymbol
								ldi r16,$2B		call printSymbol
printRangeEx:					ret
//	---------------------------------------------------------------- 89
printTargetV:					LDY TARGET_U
								rjmp commonPrintV
//	---------------------------------------------------------------- 8A 
printOutputV:					LDY AVERAGE_U
								rjmp commonPrintV
//	---------------------------------------------------------------- 8B
printMainV:						lds r16,(FL_MAX_U)
								tst r16
								breq printTargetV
								rjmp printOutputV
//	----------------------------------------------------------------
commonPrintV:					push r22
								push r23
									push r24
									push r25
									
								ldd r22,Y+0				;	текущее значение
								ldd r23,Y+1
									ldd r24,Y+2				;	предыдущее значение
									ldd r25,Y+3
									ldd r16,Y+4				;	если уже выводилось, то там FF, нет - 00

									std Y+2,r22
									std Y+3,r23

									cp  r22,r24
									cpc r23,r25
									brne commonPrintVChanged	;	предыдущее не равно текущему
													
									tst r16						;	равно текущему
									breq commonPrintVNotPrinted	;	и ещё не выводилось
									rjmp commonPrintV_Ex		;	иначе на выход

commonPrintVNotPrinted:				ldi r16,$FF
									std Y+4,r16
									rjmp commonPrintVRun

commonPrintVChanged:				ldi r16,$00					;	не выводилось
									std Y+4,r16					;	выводим

commonPrintVRun:				call A16toBCD5				;	A16 это r22...23

								call HideZeroes

								lds r16,(DIGIT3)	call printSymbol
								lds r16,(DIGIT2)	call printSymbol
								lds r16,(DIGIT1)	call printSymbol
					
								lds r16,(FontNo)
								cpi r16,1
								brne commonPrintV_A

								lds r16,(DIGIT0)	subi r16,(-10)	call printSymbol
								rjmp commonPrintV_Ex

commonPrintV_A:					ldi r16,'.'			call printSymbol
								lds r16,(DIGIT0)	call printSymbol
								rjmp commonPrintV_Ex

commonPrintV_Ex:					pop r25
									pop r24
								pop r23
								pop r22
								ret
//	----------------------------------------------------------------
