//	----------------------------------------------------------------
//	Пауза, длительностью Z микросекунд
//	----------------------------------------------------------------
pause_Z_mks:				sbiw Z,1				;	2
							adiw Z,1 sbiw Z,1		;	2+2
							adiw Z,1 sbiw Z,1		;	2+2
							adiw Z,1 sbiw Z,1		;	2+2
							brne pause_Z_mks		;	1/2 True=2
							ret
//	----------------------------------------------------------------
printModeScreen:			call disableAllOutputs
							lds r16,(MODE)
							LDZ ModeScreenTable*2
							clr r17
							add ZL,r16	adc ZH,r17
							add ZL,r16	adc ZH,r17
							lpm r16,Z+
							lpm r17,Z
							movw ZH:ZL,r17:r16
							call printString
							ret
//	----------------------------------------------------------------
//	----------------------------------------------------------------
//	Установка режима работы из r16
//	----------------------------------------------------------------
setMode:					PUSHZ
							push r16
							push r17

							clr ZL
							sts (FL_MAX_U),ZL
							sts (AVERAGE_U+4),ZL
							sts (TARGET_U+4),ZL

							cpi r16,4					;	проверим на допустимое значение
							brcc setModeOutRange		;

							sts (MODE),r16				;	сохраним в переменную

								clr r16					;	почистим флаги
								sts (FL_RANGE),r16
								sts (FL_MAX_U),r16
								sts (COUNTER_A),r16
								sts (COUNTER_D),r16
								sbi PORTD,7				;	выкл индикатор стабилизации

							lds r16,(IndicatorAddr)
							cpi r16,IND_ADDR_A
							breq setModeOLEDRedraw
							cpi r16,IND_ADDR_B
							breq setModeOLEDRedraw
							rjmp setModeCommon			;	если нет индикатора, то и перерисовывать нечего

setModeOLEDRedraw:			call INIT_LED
							call LED_CLEAR

							call printModeScreen

setModeCommon:				lds r16,(MODE)
							cpi r16,StabMode
							breq setModeStabMode
							cpi r16,ForceMode
							breq setModeForceMode
							cpi r16,StopMode
							breq setModeStopMode
							cpi r16,ExtOffMode
							breq setModeExtOffMode

setModeOutRange:			rjmp setModeExit
// ---------- работа -------------
setModeStabMode:			AdditionalForceOff				; дополнительный разгон выкл
							rjmp setModeExit
// ---------- разгон -------------
setModeForceMode:			AdditionalForceOn				; дополнительный разгон ВКЛ
							rjmp setModeExit
// ---------- стоп ---------------
setModeStopMode:			AdditionalForceOff				; дополнительный разгон выкл
							rjmp setModeExit
// ---------- аварийный стоп -----
setModeExtOffMode:			AdditionalForceOff				; дополнительный разгон выкл
							rjmp setModeExit
// ---------------------------------
setModeExit:				pop r17
							pop r16
							POPZ
							ret
//	----------------------------------------------------------------
//	Обработка прерывания каждые 1 мсек
//	----------------------------------------------------------------
T0CompareA:					PUSHF
							push r17
							push r16

							lds r17,(TIME_FLAGS)						

//--------------------------------------------------
							SETB r17,eachmilliSecondFlag		;	1 msec

							lds r16,(milliSeconds)
							cpi r16,9
							brge millisecondsFull	
							inc r16
							sts (milliSeconds),r16
							rjmp T0CompareEnd
//--------------------------------------------------
millisecondsFull:			clr r16
							sts (milliSeconds),r16
							SETB r17,eachcentiSecondFlag		;	10 msec

T0CompareCentiSec:			lds r16,(centiSeconds)
							cpi r16,9
							brge centisecondsFull
							inc r16
							sts (centiSeconds),r16
							rjmp T0CompareEnd
//--------------------------------------------------
centisecondsFull:			clr r16
							sts (centiSeconds),r16
							SETB r17,eachdeciSecondFlag			;	100 msec

T0CompareDeciSecX:			lds r16,(decisecondsX)
							inc r16
							cpi r16,5							; 	X=5
							brge decisecondsXFull
							sts (decisecondsX),r16
							rjmp T0CompareDeciSecY
decisecondsXFull:			clr r16
							sts (decisecondsX),r16
							SETB r17,eachXdeciSecondFlag		;	X*100 = 500 msec
						; -------------------------------
T0CompareDeciSecY:			lds r16,(decisecondsY)
							inc r16
							cpi r16,2							; 	Y=2
							brge decisecondsYFull
							sts (decisecondsY),r16
							rjmp T0CompareDeciSec
decisecondsYFull:			clr r16
							sts (decisecondsY),r16
							SETB r17,eachYdeciSecondFlag		;	Y*100 = 200 msec
						; -------------------------------
T0CompareDeciSec:			lds r16,(deciSeconds)
							cpi r16,9
							brge decisecondsFull
							inc r16
							sts (deciSeconds),r16
							
							rjmp T0CompareEnd
//--------------------------------------------------
decisecondsFull:			clr r16
							sts (deciSeconds),r16
							SETB r17,eachSecondFlag				;	1000 msec

T0CompareSec:				lds r16,(Seconds)
							cpi r16,9
							brge secondsFull

							inc r16
							sts (Seconds),r16
							rjmp T0CompareEnd

secondsFull:				clr r16
							sts (Seconds),r16		
							SETB r17,eachdekaSecondFlag			;	10000 msec	
//--------------------------------------------------
T0CompareEnd:				sts (TIME_FLAGS),r17
							pop r16
							pop r17
							POPF
							reti
//	----------------------------------------------------------------

