//	----------------------------------------------------------------
//	Обработчик преобразования АЦП
//	----------------------------------------------------------------
ADC_COMPLETE:					PUSHF
								push ZL
								push ZH
								push r6
								push r7
								push r8
								push r9
								push r10
								push r11
								push r12
								push r13
								; ----------
									sbis MOC_PORT,MOC_PIN
									rjmp ADC_READ
									clr r10
									clr r11
									clr r12
									clr r13
								rjmp ADC_ADD_NEW
					; ----------
					; Считываем значение АЦП
ADC_READ:						lds r6,ADCL
								lds r7,ADCH
					; ----------
					; Возведем в квадрат значение АЦП
								movw r8,r6
								call MUL1616
					; ----------
					; Восстановим старое значение суммы квадратов
ADC_ADD_NEW:					LDZ ADC_CURRENT_SUM_QUAD
								ld r6,Z+	ld r7,Z+	ld r8,Z+	ld r9,Z+
					; ----------
					; Добавим к нему новый квадрат
								add r6,r10
								adc r7,r11
								adc r8,r12
								adc r9,r13
								; ----------
					; Сохраним сумму квадратов
								LDZ ADC_CURRENT_SUM_QUAD
								st Z+,r6	st Z+,r7	st Z+,r8	st Z+,r9
								; ----------
					; Увеличим и сохраним счетчик преобразований
								lds ZL,(ADC_CURRENT_VALUE_COUNT+0)
								lds ZH,(ADC_CURRENT_VALUE_COUNT+1)
								adiw Z,1
								sts (ADC_CURRENT_VALUE_COUNT+0),ZL
								sts (ADC_CURRENT_VALUE_COUNT+1),ZH
								; ----------
								pop r13
								pop r12
								pop r11
								pop r10
								pop r9
								pop r8
								pop r7
								pop r6
								pop ZH
								pop ZL
								POPF
								reti
//	----------------------------------------------------------------
//	Включение оптрона управления симистором 
//	Прерывание по совпадению Т1A
//	----------------------------------------------------------------
OPTO_ON:						cbi MOC_PORT,MOC_PIN
								reti
//	----------------------------------------------------------------
//	Выключение оптрона управления симистором 
//	Прерывание по совпадению Т1B
//	----------------------------------------------------------------
OPTO_OFF:						sbi MOC_PORT,MOC_PIN
								reti
//	----------------------------------------------------------------
//	
//	----------------------------------------------------------------
//	----------------------------------------------------------------
OPTO_OFF_E:						sbi MOC_PORT,MOC_PIN
								reti
//	----------------------------------------------------------------
//	Переход сетевого напряжения через 0 (прерывание INT0)
//	----------------------------------------------------------------
ZEROCROSS:		; ----------
								PUSHF
								push r16
								push r17
								push r18
								push r19
				; ----------
				; сделаем три считывания ноги с интервалом 30 мксек, 
				; если все одинаковы, то предполагаем, что это фронт, а не дребезг
								in r16,PIND
								andi r16,(1<<2)
								M_DELAY_US_RA 30,r19
								in r17,PIND
								andi r17,(1<<2)
								M_DELAY_US_RA 30,r19
								in r18,PIND
								andi r18,(1<<2)

								cp r16,r17
								brne ZEROCROSS_A_ne_B
								cp r16,r18
								brne ZEROCROSS_A_ne_B
								rjmp ZEROCROSS_OK
ZEROCROSS_A_ne_B:				rjmp ZEROCROSS_SKIP

ZEROCROSS_OK:					sts (ZEROCROSS_PIN_STATE),r16
				; ----------
				; смотрим сколько насчитал Т1 с прошлого перехода через ноль
								lds r18,TCNT1L
								lds r19,TCNT1H
								sts (TCNT1_SAVED+0),r18
								sts (TCNT1_SAVED+1),r19
				; ----------
				; гасим оптрон
								sbi MOC_PORT,MOC_PIN
				; ----------
				; Стоп АЦП
								ldi r16,ADC_OFF		
								sts ADCSRA,r16
				; ----------
				; запомним режим работы Т1	
								lds  r16, TCCR1B
								sts (TCCR1B_SAVED),r16			
				; ----------
				; останавливаем T1				
								andi r16,$F8			
								sts TCCR1B,r16	
				; ----------
				; сохраняем остальные нужные регистры
								push r20
								push ZH
								push ZL
								push YH
								push YL
								PUSH_ALL_LOW
				; ----------
				; обнуляем T1		
								clr r16					; 
								sts TCNT1H,r16			
								sts TCNT1L,r16	
				; ----------
				; перезагружаем  момент  включения оптрона
								lds r17,(OPTO_ON_TICK+1)		
								lds r16,(OPTO_ON_TICK+0)	;
								sts OCR1AH,r17				;
								sts OCR1AL,r16				; 
				; ----------
				; перезагружаем  момент  вЫключения оптрона
								lds r17,(OPTO_OFF_TICK+1)		
								lds r16,(OPTO_OFF_TICK+0)	;
								sts OCR1BH,r17				;
								sts OCR1BL,r16				; 
				; ----------
				; Сброс прескалера Т0,Т1
								ldi r16,0b00000001		
								out GTCCR,r16
				; ----------
				; вспоминаем режим работы Т1
								lds r16,(TCCR1B_SAVED)	
				; ----------
				; запускаем T1			
								sts TCCR1B,r16			
//	----------------------------------------------------------------
//	начало обработчика перехода сетевого напряжения через 0
//	----------------------------------------------------------------
ZEROCROSS_RUN:					lds r16,(ZEROCROSS_PIN_STATE)		; Выясняем, какой перепад пришел
								tst r16								
								brne RISING							; переход по фронту
								rjmp FALLING						; переход по спаду
//	----------------------------------------------------------------
//	отрабатываем фронт перехода через 0
//	----------------------------------------------------------------
RISING:							ldi r16,ADC_ON				; Старт АЦП
								sts ADCSRA,r16
								; ----------
								jmp ZEROCROSS_END
//	----------------------------------------------------------------
//	отрабатываем спад перехода через 0
//	----------------------------------------------------------------
FALLING:						ldi r16,ADC_OFF				; Стоп АЦП
								sts ADCSRA,r16
								; ----------
				; обрабатываем накопленные за положительный полупериод данные
				; Вычислим измеренное значение U_RMS
								LDZ ADC_CURRENT_SUM_QUAD
								ld r6,Z+	ld r7,Z+	ld r8,Z+	ld r9,Z+
								LDZ ADC_CURRENT_VALUE_COUNT
								ld r10,Z+	ld r11,Z+
								clr r12		clr r13
				; проверим на предмет вероятного деления на 0
								cp  r10,r12
								cpc r11,r13
								brne FALLING_A
								rjmp FALLING_B
				; ----------	
				; Умножим r9...r6 на 4 для лучшего использования разрядной сетки
FALLING_A:						lsl r6	rol r7	rol r8	rol r9	
								lsl r6	rol r7	rol r8	rol r9
				; ----------
				; Посчитаем квадрат RMS [r9...6]/[r13...10] сумма квадратов разделить на количество отсчетов
								call div32u						
				; ----------
				; Умножим r9...r6 на 4
								lsl r6	rol r7	rol r8	rol r9	 
								lsl r6	rol r7	rol r8	rol r9
				; ----------
				; вычислим корень квадратный из полученного выше
								movw r20,r6
								movw r22,r8
				; ----------
								call sqr		
				; в r9 r8 имеем TRUE RMS значение напряжения,
				; умноженное на некоторый коэффициент, определяемый внешним делителем
				; ----------
				; Сохраним текущее значение U RMS 
								sts (CURRENT_U+0),r8	; Сохраним текущее значение U RMS 
								sts (CURRENT_U+1),r9	; 
				; ----------
				; Займемся усреднением U_RMS 
								ldi ZH,high(CURRENT_U + 2 + 2*CU_HISTORY_LENGTH)
								ldi ZL,low (CURRENT_U + 2 + 2*CU_HISTORY_LENGTH)
								movw Y,Z					; Указатели на самый хвост сохраненных значений
								clr r2	clr r3	
								clr r8	clr r9
								ld r7, -Y					;	читаем самое древнее значение					
								ld r6, -Y					;	с уменьшением указателя Y
								ldi r16,CU_HISTORY_LENGTH
AVG_LOOP:						tst r16
								breq AVG_LOOP_EX
								dec r16
								ld r1,-Y					;	читаем менее древнее значение
								ld r0,-Y					;	с уменьшением указателя Y			
								add r6, r0					;	прибавляем его к сумме 
								adc r7, r1					;	более древних значений
								adc r8, r2					;
								adc r9, r3					;		 
								st -Z,r1					;	записываем менее древнее показание на место более древнего,			
								st -Z,r0					;	в конечном итоге самое древнее теряется, самое новое замещается текущим
								; ----------
								jmp AVG_LOOP
				; ----------
AVG_LOOP_EX:					ldi ZL, low(CU_HISTORY_LENGTH+1)
								ldi ZH,high(CU_HISTORY_LENGTH+1)
								movw r10,Z
								clr r12
								clr r13
								call div32u
				; ----------
								sts (AVERAGE_U+0),r6		; Сохраним среднеарифметическое значение U RMS 
								sts (AVERAGE_U+1),r7		; 

								ldi r16,OutputV
								call setNewValFlag
								ldi r16,MainV
								call setNewValFlag
				; ----------
FALLING_B:						clr r16
								; Сброс текущей суммы квадратов
								LDZ ADC_CURRENT_SUM_QUAD
								st Z+,r16	st Z+,r16	st Z+,r16	st Z+,r16	
								; Сбросим счетчик преобразований
								LDZ ADC_CURRENT_VALUE_COUNT
								st Z+,r16	st Z+,r16							

//	----------------------------------------------------------------
//	А это обратная связь по RMS напряжению
//	----------------------------------------------------------------
modeCheckStabMode:				lds r16,(MODE)
								cpi r16,StabMode
								brne modeCheckForceMode
								rjmp OOS_StabMode				; стабилизация

modeCheckForceMode:				cpi r16,ForceMode
								brne modeCheckStopMode
								rjmp modeOOSForceMode			; на разгон

modeCheckStopMode:				cpi r16,StopMode
								brne modeCheckExtOffMode
								rjmp modeOOSStopMode			; на стоп

modeCheckExtOffMode:			cpi r16,ExtOffMode
								brne modeCheckAnoterMode
								rjmp modeOOSStopMode			; на стоп
modeCheckAnoterMode:
								rjmp ZEROCROSS_END
//	---------------------------------------------------------------- разгон
modeOOSForceMode:				ldi r16,low (MIN_ON_VALUE)		;	минимальное значение задержки включения
								ldi r17,high(MIN_ON_VALUE)
								movw r8,r16
								rjmp SET_OPTO_ON_TICK
//	----------------------------------------------------------------
modeOOSStopMode:				ldi r16, low(DEFAULT_OFF_VALUE)
								ldi r17,high(DEFAULT_OFF_VALUE)
								movw r8,r16
								rjmp SET_OPTO_ON_TICK
//	----------------------------------------------------------------
OOS_StabMode:					lds r6,(CURRENT_U+0)		; текущее напряжение
								lds r7,(CURRENT_U+1)
								lds r8,(TARGET_U+0)			; целевое напряжени
								lds r9,(TARGET_U+1)
								sub r6,r8					; delta=CURRENT_U-target_u ([r7..6] = current - target) дельта
								sbc r7,r9

								lds r8,(OPTO_ON_TICK+0)
								lds r9,(OPTO_ON_TICK+1)
								add r8,r6					; opto_on_tick = opto_on_tick + delta
								adc r9,r7					; Новое значение момента включения симистора может быть отрицательным и очень большим

				; Проверка на предмет выхода за допустимый диапазон
				; от MIN_ON_VALUE до DEFAULT_OFF_VALUE

								sbrs r9,7					; переход, если неотрицательное (0000...7FFF)
								rjmp OOS_StabMode_PositiveDelta

				; отрицательное значение дельты - установка максимального напряжения на выходе

OOS_StabMode_SET_MAX_U:			ldi r16,MIN_ON_VALUE
								ldi r17,0
								movw r8,r16						; иначе OPTO_ON_TICK = MIN_ON_VALUE (максимальное напряжение)

				; задержка установки флага максимального напряжения на выходе

								lds r16,(COUNTER_A)
								cpi r16,20
								brge OOS_StabMode_SET_MAX_U_Flag
								inc r16
								sts (COUNTER_A),r16
								rjmp OOS_StabMode_SET_MAX_U_End								
								
OOS_StabMode_SET_MAX_U_Flag:	ldi r16,$FF						; И устанавливаем флаг 
								sts (FL_MAX_U),r16				; максимального напряжения на выходе
								
OOS_StabMode_SET_MAX_U_End:		jmp SET_OPTO_ON_TICK

				; положительное значение дельты

OOS_StabMode_PositiveDelta:		ldi r16, low(MIN_ON_VALUE)
								ldi r17,high(MIN_ON_VALUE)
								cp  r8,r16
								cpc r9,r17
								brlo OOS_StabMode_SET_MAX_U		; если дельта меньше MIN_ON_VALUE то это максимум напряжения
								ldi r16, low(DEFAULT_OFF_VALUE)
								ldi r17,high(DEFAULT_OFF_VALUE)
								cp  r8,r16
								cpc r9,r17
								brcs OOS_StabMode_Good_Delta	; если дельта меньше DEFAULT_OFF_VALUE, то попадаем в допустимый диапазон
								movw r8,r16						; иначе OPTO_ON_TICK = DEFAULT_OFF_VALUE по сути стоп

				; допустимый диапазон

OOS_StabMode_Good_Delta:		lds r16,(COUNTER_A)
								tst r16
								breq OOS_StabMode_Good_DeltaM1
								dec r16
								sts (COUNTER_A),r16
								rjmp SET_OPTO_ON_TICK
OOS_StabMode_Good_DeltaM1:		clr r16						; сбрасываем флаг 
								sts (FL_MAX_U),r16			; максимального напряжения на выходе
								sts (COUNTER_A),r16

				; устанавливаем момент включения симистора

SET_OPTO_ON_TICK:				sts (OPTO_ON_TICK+0),r8
								sts (OPTO_ON_TICK+1),r9

//	----------------------------------------------------------------
//	индикация вхождения усредненного выходного напряжения в допустимый коридор
//	----------------------------------------------------------------
.equ delta = 3
CHECK_RANGE:					lds r16,(MODE)
								cpi r16,0
								breq CHECK_RANGE_RUN
								rjmp CHECK_RANGE_END

CHECK_RANGE_RUN:				lds ZL,(AVERAGE_U+0)
								lds ZH,(AVERAGE_U+1)
								lds r16,(TARGET_U+0)
								lds r17,(TARGET_U+1)
								sub ZL,r16
								sbc ZH,r17
								brcc CHECK_RANGE_POSITIVE_DELTA
								com ZL
								com ZH
								adiw Z,1

CHECK_RANGE_POSITIVE_DELTA:		ldi r16, low(delta)			; +- 0.3 V
								ldi r17,high(delta)			;
								cp  ZL,r16
								cpc ZH,r17
								brcc CHECK_RANGE_NOT_RANGE

								clr r16
								sts (COUNTER_D),r16
								cbi PORTD,7			; вкл индикатор стабилизации в диапазоне
								lds r17,(FL_RANGE)
								ldi r16,$FF
								sts (FL_RANGE),r16	; флаг стабилизации установить	 
								rjmp CHECK_RANGE_EX

CHECK_RANGE_NOT_RANGE:			lds r16,(COUNTER_D)
								cpi r16,10
								brcc CHECK_RANGE_NOT_RANGE_A
								inc r16
								sts (COUNTER_D),r16
								rjmp CHECK_RANGE_END

CHECK_RANGE_NOT_RANGE_A:		sbi PORTD,7			; выкл индикатор стабилизации
								lds r17,(FL_RANGE)
								ldi r16,$00
								sts (FL_RANGE),r16	; флаг стабилизации снять

CHECK_RANGE_EX:					cp r16,r17
								breq CHECK_RANGE_END

								ldi r16,Range
								call setNewValFlag
CHECK_RANGE_END:
		; ----------
ZEROCROSS_END:					POP_ALL_LOW
								pop YL
								pop YH
								pop ZL
								pop ZH
								pop r20
ZEROCROSS_SKIP:					pop r19
								pop r18
								pop r17
								pop r16
								POPF
								reti
//	----------------------------------------------------------------

