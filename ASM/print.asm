//	----------------------------------------------------------------
// -- вывод символа из R16 шрифтом, на который ссылается переменная FontNo --
//	----------------------------------------------------------------
printSymbol:				PUSHF
							push r16
							push r17
							push r18
							push r19
							push r20
							push r0
							push r1
							push ZH
							push ZL
							cbi GPIOR0,0						;	бит пробела код $20
								push r16
									lds r16,(FontNo)
									andi r16,$03				;	шрифтов не больше 4
									ldi r17,0
									LDZ (FontAdressesTable*2)
									add ZL,r16	adc ZH,r17
									add ZL,r16	adc ZH,r17
									lpm r16,Z+
									lpm r17,Z+
									add r16,r16
									adc r17,r17
									movw Z,r16					; указатель на начало шрифта
								pop r16
								lpm r17,Z+						; код начального символа знакогенератора
								lpm r18,Z+						; код конечного символа знакогенератора (ЗГ)
								lpm r19,Z+						; ширина символа
								lpm r20,Z+						; высота символа
								cpi r16,' '						; если код = пробел, выводим в любом случае
								breq printSymbolSpace
								cpi r16,$00
								breq printSymbolHalfSpace
								cp r16,r17						; если код символа меньше нижней границы ЗГ,
								brcs printSymbolGotoExit		; то на выход
								cp r16,r18						; если код символа 
								breq printSymbolInRange			; равен верхней границе ЗГ
								brcs printSymbolInRange			; или меньше её, то выводим,
printSymbolGotoExit:			rjmp printSymbolExit			; если больше верхней границы ЗГ,то на выход

printSymbolHalfSpace:			lsr r19							;	ширина пополам
								ldi r16,' '						;	подставляем пробел
printSymbolSpace:				sbi GPIOR0,0

printSymbolInRange:				cpi r16,$80
								brlo printSymbolENG				; До $7F - латинские
								cpi r16,$C0
								brlo printSymbolGotoExit		; промежуток $80-$BF - не выводим
								cpi r17,$80						; если только русские символы, то без коррекции
								brcc printSymbolENG
printSymbolRUS:					subi r16,$40					; иначе вычитаем $40
printSymbolENG:					push r16						; сохраним код символа
									sub r16,r17
									mul r19,r20					; вычисляем шаг символов в ЗГ
									mul r16,r0					; вычисляем смещение символа от начала ЗГ
									add ZL,r0	adc ZH,r1		; получаем в Z указатель на начало образа символа
									lds r16,(POSX)
									sts (OLDX),r16			; сохраняем начальную координату X
									lds r16,(POSY)
									sts (OLDY),r16			; сохраняем начальную координату Y
								pop r16						; вспомним код символа
								cpi r16,'.'
								brne printSymbolNotComma
								clr r0
								lsr r19	
								lsr r19
								adc r19,r0								; для точки (.) выводим только левую четверть образа символа
								//-------------- цикл по строкам -----------
printSymbolNotComma:			push r20								; сохраним значение высоты символа
printSymbolRowCycle:				call softI2CStart
									lds r16,(IndicatorAddr)
									call softI2CSendByte
									ldi r16,datastream
									call softI2CSendByte
									//-------------- цикл по столбцам ----------
									push r19							; сохраним значение ширины символа
printSymbolColCycle:					lds r16,(POSX)
										cpi r16,DisplayWidth
										brcc printSymbolColCycleSkip	; если упёрлись в край экрана, печатать не будем
										inc r16
										sts (POSX),r16
										ldi r16,$00
										sbis GPIOR0,0
										lpm r16,Z+
printSymbolSendByte:						call softI2CSendByte
printSymbolColCycleCheck:				dec r19
										brne printSymbolColCycle
										rjmp printSymbolColCycleEx
printSymbolColCycleSkip:				lpm r16,Z+
										rjmp printSymbolColCycleCheck
printSymbolColCycleEx:				pop r19								; восстановим значение ширины символа
									//-------------- конец цикла по столбцам ----------
									call softI2CStop
									dec r20
									breq printSymbolRowCycleEx
									lds r16,(OLDX)
									sts (POSX),r16
									lds r16,(OLDY)
									inc r16
									andi r16,$07
									sts (POSY),r16
									sts (OLDY),r16
									call SET_LED_XY
									rjmp printSymbolRowCycle
									//--------------
printSymbolRowCycleEx:			pop r20									;	вспомним значение высоты символа
								lds r16,(POSY)
								sub r16,r20
								inc r16
								andi r16,$07
								sts (POSY),r16							;	номер строки для вывода следующего символа
								call SET_LED_XY
								//-------------- конец цикла по строкам -----------
//----
printSymbolExit:				pop ZL
								pop ZH
								pop r1
								pop r0
								pop r20
								pop r19
								pop r18
								pop r17
								pop r16
								POPF
								ret
//	----------------------------------------------------------------
// -- Вывод строки из ПЗУ на дисплей ссылка на начало строки в Z --
//	----------------------------------------------------------------
printString:					PUSHF
								push r16
								push r17
								push XL
								push XH
								push YL
								push YH

								add ZL,ZL
								adc ZH,ZH
								clr r16
								sts (StringCounter),r16

printStringCycle:				lds r16,(StringCounter)
								cpi r16,128						;	максимальная длина строки
								brcs printStringNext
								rjmp printStringExit
printStringNext:				inc r16
								sts (StringCounter),r16
								lpm r16,Z+
								cpi r16,EOS
								brne printStringNotEnd
								rjmp printStringExit

printStringNotEnd:		
								cpi r16,$80
								brcs printStringSymbol
								cpi r16,$C0
								brcc printStringSymbol
							
								call printExtras

								rjmp printStringCycle

printStringSymbol:				call printSymbol
						
								rjmp printStringCycle

printStringExit:				pop YH
								pop YL
								pop XH
								pop XL
								pop r17
								pop r16
								POPF
								ret	
//	----------------------------------------------------------------
// запрет отображения всех переменных на экране
//	----------------------------------------------------------------
disableAllOutputs:				PUSHY
								push r16
								push r17
								LDY ExtrasConfig
								ldi r16,(OuterExtraNumber-$80)	;	количество доступных экстр
disableAllOutputsCycle:			tst r16
								breq disableAllOutputsEx
								dec r16
								ldd r17,Y+1
								CLRB r17,6						;	запрет вывода бит 6
								CLRB r17,7						;	сброс признака "новое значение"
								std Y+1,r17
								adiw Y,2						;	на 2 байта вперед к следующей экстре
								rjmp disableAllOutputsCycle
disableAllOutputsEx:			pop r17
								pop r16
								POPY
								ret
//	----------------------------------------------------------------
// -- просмотр всех выводимых значений на предмет обновлённости ------
//	----------------------------------------------------------------
newValueScan:					PUSHY
								push r16
								push r17

								ldi r16,$88							;	начальный код сканирования

newValueScanCycle:				cpi r16,OuterExtraNumber			;	конечный код сканирования + 1
								brcc newValueScanEx
// -- проверка на предмет одного нового значения и вывод, если надо 
checkForOneNewValue:			LDY ExtrasConfig						
								push r16
									subi r16,$80
									clr r17
									add YL,r16	adc YH,r17
									add YL,r16	adc YH,r17
									ldd r16,Y+1
									andi r16,0b11000000			;	бит 6 - разрешен вывод, 7 - новое значение
									cpi  r16,0b11000000
									brne checkForOneNewValueEx
								pop r16
								push r16
									call getExtrasConfig		;	установка шрифта, позиции вывода
									call printExtras
checkForOneNewValueEx:			pop r16

								inc r16
								rjmp newValueScanCycle

newValueScanEx:					pop r17
								pop r16
								POPY
								ret
//	----------------------------------------------------------------
// установить флаг нового значения "экстры", в R16 - код "экстры"
//	----------------------------------------------------------------
setNewValFlag:					cpi r16,OuterExtraNumber			;	номер старшей экстры плюс 1
								brge setNewValFlagExit				;	если больше или равно, то на выход
								cpi r16,$80							;	номер младшей экстры
								brlo setNewValFlagExit				;	если меньше, то на выход

								PUSHY
								push r16
								push r17
					
								LDY ExtrasConfig
								subi r16,$80
								clr r17
								add YL,r16	adc YH,r17
								add YL,r16	adc YH,r17	
								ldd r17,Y+1	
								SETB r17,7							;	установить флаг нового значения бит 7
								std Y+1,r17

								pop r17
								pop r16
								POPY
setNewValFlagExit:				ret
//	----------------------------------------------------------------
// -- Загружаем определённые ранее параметры вывода "экстры", номер в R16 --
//	----------------------------------------------------------------
getExtrasConfig:				cpi r16,OuterExtraNumber			;	номер старшей экстры плюс 1
								brge getExtrasConfigExit				;	если больше или равно, то на выход
								cpi r16,$80							;	номер младшей экстры
								brlo getExtrasConfigExit				;	если меньше, то на выход
								
								PUSHY
								push r16
								push r17
								LDY ExtrasConfig
								subi r16,$80
								clr r17
								add YL,r16	adc YH,r17
								add YL,r16	adc YH,r17				;	Y - указатель на место отображения
								ldd r16,Y+0							;	POSX
								ldd r17,Y+1							;	POSY
								sts (POSX),r16
								sts (POSY),r17
								; ----------
								CLRB r17,7							;	сброс флага нового значения бит 7
								SETB r17,6							;	разрешение вывода на экран бит 6
								std Y+1,r17							;	
								swap r17
								andi r17,$03
								sts (FontNo),r17
		
								call SET_LED_XY

								pop r17
								pop r16
								POPY

getExtrasConfigExit:			ret

//	----------------------------------------------------------------
						

