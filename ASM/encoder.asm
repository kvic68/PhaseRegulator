//	----------------------------------------------------------------
.dseg			EncPrev		:				.BYTE 1
				EncPrevPrev	:				.BYTE 1
				EncTimeWindow:				.BYTE 1
				oldBtnState:				.BYTE 1
				btnDebounceCounter:			.BYTE 1
				btnHoldCounter:				.BYTE 1
				btnClickCount:				.BYTE 1
				Clicks:						.BYTE 1
//	----------------------------------------------------------------
.equ	StateA 	= 1;
.equ	StateB 	= 2;
.equ	StateAB = 3;
.equ	StepUpFlagNo	= 5;
.equ	StepDnFlagNo	= 6;
.equ	encPort			= PORTD
.equ	encPin			= PIND
.equ	btnPinNo		= 5						; нога порта, куда подключена кнопка энкодера
.equ	encPhaseA		= 4
.equ	encPhaseB		= 3
//	----------------------------------------------------------------
.cseg
//	----------------------------------------------------------------
//	вращение энкодера вызывается каждые 1 мс
//	----------------------------------------------------------------
encoder:						push r16
								push r17
								push r18

								lds r16,(EncTimeWindow)
								cpi r16,0
								breq encSetLittleStep
								dec r16
								sts (EncTimeWindow),r16

encSetBigStep:					ldi r16, low(DEFAULT_STEP_U*10)
								ldi r17,high(DEFAULT_STEP_U*10)
								sts (STEP_U+0),r16
								sts (STEP_U+1),r17
								rjmp encCheckFlags

encSetLittleStep:				ldi r16, low(DEFAULT_STEP_U)
								ldi r17,high(DEFAULT_STEP_U)
								sts (STEP_U+0),r16
								sts (STEP_U+1),r17

encCheckFlags:					lds r16,(ACTIONS_FLAGS)
								sbrc r16,StepUpFlagNo
								rjmp encoderExitB

								sbrc r16,StepDnFlagNo
								rjmp encoderExitB

								clr r16;	// encCur

								in r17,encPin
								sbrs r17,encPhaseA
								ori r16,StateA
								sbrs r17,encPhaseB
								ori r16,StateB
								lds r17,(encPrev)
								cp r16,r17
								breq encoderExitB

								lds r18,(encPrevPrev)
								cpi r17,StateAB
								brne encoderExitA

								cp r16,r18
								breq encoderExitA

								cpi r16,StateB
								brne encRotB
encRotA:		// step_dn
								lds r16,(ACTIONS_FLAGS)
								CLRB r16,StepUpFlagNo
								SETB r16,StepDnFlagNo
								sts (ACTIONS_FLAGS),r16
								rjmp encoderSetTW
encRotB:		// step_up
								lds r16,(ACTIONS_FLAGS)
								SETB r16,StepUpFlagNo
								CLRB r16,StepDnFlagNo
								sts (ACTIONS_FLAGS),r16

encoderSetTW:					ldi r18,20
								sts (EncTimeWindow),r18

encoderExitA:					sts (encPrevPrev),r17
								sts (encPrev),r16

encoderExitB:					pop r18
								pop r17
								pop r16
								ret
//	----------------------------------------------------------------
//	опрос кнопки и определение статуса её вызывается каждые 10 мс
//	----------------------------------------------------------------
button:							PUSH r16
								push r17

								in r16,encPin						;	состояние ножки кнопки 0 - нажата, 1 - отпущена
								ldi r17,$FF
								eor r16,r17							;	1 - нажато, 0 - отпущено
								andi r16,(1<<btnPinNo)
								lds r17,(oldBtnState)
								sts (oldBtnState),r16
								eor r17,r16							;	0 - нет изменения сигнала, 1 - есть изменение сигнала

								sbrs r17,btnPinNo
								rjmp btnIsNotChanged				;
								ldi r17,1							;	если сигнал изменился, устанавливаем счетчик антидребезга
								sts (btnDebounceCounter),r17		;	длительность антидребезга 1*10=10 мсек
								rjmp btnExit						;	и на выход
								//-----------------------------------
btnIsNotChanged:				lds r17,(btnDebounceCounter)		;	иначе вспоминаем, что он насчитал.
								sbrs r17,7							;	если он неотрицательный
								rjmp btnDebounceCounterNotNegatve	;	то прыгаем на проверку на 0

								clr r17								;	иначе обнуляем счетчик антидребезга
								sts (btnDebounceCounter),r17
								rjmp btnExit						;	и на выход
								//-----------------------------------
btnDebounceCounterNotNegatve:	cpi r17,$00							;	если счетчик равен 0, то дребезг подавлен
								breq btnStaticLevel					;	переходим на обработку уровня
	
btnDebounceCounterisZero:		dec r17								;	иначе уменьшаем счетчик
								sts (btnDebounceCounter),r17
								brne btnExit						;	и, если не 0, то на выход, иначе пойман переход сигнала
																	;	и начинаем обработку изменения сигнала
								//-----------------------------------
								sbrc r16,btnPinNo
								rjmp btnPressed						;	событие нажатия кнопки (фронт сигнала)
								rjmp btnReleased					;	событие отпускания кнопки (спад сигнала)
// произошло отпускание ---------------------------------------------
btnReleased:					lds r17,(btnClickCount)
								inc r17
								sts (btnClickCount),r17
								cpi r17,2
								brcc btnHasClicks
								
								lds r17,(btnHoldCounter)
								cpi r17,5
								brcs btnExit
								cpi r17,100
								brcc btnExit
						//флаг клика
btnHasClicks:					lds r16,(ACTIONS_FLAGS)
								SETB r16,3
								sts (ACTIONS_FLAGS),r16

//								lds r16,(btnClickCount)
//								sts (Clicks),r16
								clr r16
								sts (btnClickCount),r16
								rjmp btnExit
								//-----------------------------------
//shortClick:					lds r17,(btnClickCount)
//								inc r17
//								sts (btnClickCount),r17
//								rjmp btnExit
// произошло нажатие -------------------------------------------------
btnPressed:						clr r17
								sts (btnHoldCounter),r17			;	обнуляем счетчик нажатого состояния
								rjmp btnExit
								//-----------------------------------
btnStaticLevel:					sbrs r16,btnPinNo					;	обработчик статичного уровня
								rjmp btnExit;btnHoldReleased		;	если кнопка не нажата, то на выход
								;rjmp btnHoldPressed
btnHoldPressed:					lds r17,(btnHoldCounter)
								cpi r17,100
								brne btnHoldPressedA
								//флаг длинного нажатия
								lds r16,(ACTIONS_FLAGS)
								SETB r16,4
								sts (ACTIONS_FLAGS),r16
btnHoldPressedA:				cpi r17,$FF
								breq btnExit
								inc r17
								sts (btnHoldCounter),r17
								rjmp btnExit
								//-----------------------------------
btnExit:						pop r17
								POP r16
								ret
//	----------------------------------------------------------------
//	клик по кнопке вызывается из основного цикла по флагу
//	----------------------------------------------------------------
shortKeyPress:					PUSHF
								push r16

								lds r16,(MODE)
								andi r16,$03

shortKeyPressMode0:				cpi r16,StabMode
								brne shortKeyPressMode1
								ldi r16,StopMode		; короткое нажатие в рабочем режиме включаем режим "стоп"
								call setMode
								rjmp shortKeyPressModeExit

shortKeyPressMode1:				cpi r16,ForceMode
								brne shortKeyPressMode2
								ldi r16,StabMode		; короткое нажатие в режиме "разгон" включаем основной режим стабилизации
								call setMode
								rjmp shortKeyPressModeExit

shortKeyPressMode2:				cpi r16,StopMode
								brne shortKeyPressMode3
								ldi r16,StabMode		; короткое нажатие в режиме "стоп" включаем основной режим стабилизации
								call setMode
								rjmp shortKeyPressModeExit

shortKeyPressMode3:				cpi r16,3
								brne shortKeyPressModeExit

shortKeyPressModeExit:			pop r16
								POPF
								ret
//	----------------------------------------------------------------
//	длительное удержание кнопки вызывается из основного цикла по флагу
//	----------------------------------------------------------------
longKeyHold:					PUSHF
								push r16
								lds r16,(MODE)
								andi r16,$03

longKeyHoldMode0:				cpi r16,StabMode
								brne longKeyHoldMode2
								ldi r16,ForceMode		; длинное нажатие в рабочем режиме включаем режим разгона
								call setMode
								rjmp longKeyHoldExit

longKeyHoldMode2:				cpi r16,StopMode
								brne longKeyHoldMode3
								ldi r16,ForceMode		; длинное нажатие в режиме стоп включает режим разгона
								call setMode
								rjmp longKeyHoldExit

longKeyHoldMode3:				cpi r16,ExtOffMode
								brne longKeyHoldExit
								in r16,PINC
								sbrs r16,extForcePin	; если синал внешнего стопа активен,
								rjmp longKeyHoldExit	; то на выход, иначе
								ldi r16,StabMode		; длинное нажатие в режиме "внешний стоп" включает основной режим стабилизации
								call setMode

longKeyHoldExit:				pop r16
								POPF	
								ret
//	----------------------------------------------------------------
; --------------------------------------------------------------------------------
INCREASE_U:						PUSHF
								push r6
								push r7
								push r8
								push r9
								push r10
								push r11
								push r12
								push r13
								push r16
								push r17
								push r18
								push r19

								lds r16,(MODE)
								cpi r16,ExtOffMode
								brne INCREASE_A
								rjmp INCREASE_EX

INCREASE_A:						cpi r16,StabMode
								breq INCREASE_RUN
									lds r16,(IndicatorAddr)
									cpi r16,IND_ADDR_A
									breq INCREASE_RUN
									cpi r16,IND_ADDR_B
									breq INCREASE_RUN
									rjmp INCREASE_EX				
				; ----------
INCREASE_RUN:					lds r16,(FL_MAX_U)		; Если выходное напряжение на максимуме,
								cpi r16,$FF				; то ничего не делаем.
								brne INCREASE_NORM		; Иначе приступаем к регулировке.
								rjmp INCREASE_EX
				; ----------
INCREASE_NORM:					lds r16,(TARGET_U+0)
								lds r17,(TARGET_U+1)

								lds r18,(STEP_U+0)
								lds r19,(STEP_U+1)
					
								add r16,r18
								adc r17,r19	

								call normU

								ldi r18, low(MIN_U)
								ldi r19,high(MIN_U)
								cp  r16,r18
								cpc r17,r19
								brge INCREASE_MAX		; Если следующее значение больше или равно минимальному, то устанавливаем его.
								movw r16,r18			; Иначе следующее значение устанавливаем = MIN_U
								rjmp INCREASE_SET

INCREASE_MAX:					ldi r18, low(MAX_U)
								ldi r19,high(MAX_U)
								cp  r16,r18
								cpc r17,r19
								brcs INCREASE_SET
								breq INCREASE_SET
								movw r16,r18

INCREASE_SET:					sts (TARGET_U+0),r16
								sts (TARGET_U+1),r17

								ldi r16,TargetV
								call setNewValFlag
								ldi r16,MainV
								call setNewValFlag

				; ----------
INCREASE_EX:					pop r19
								pop r18
								pop r17
								pop r16
								pop r13
								pop r12
								pop r11
								pop r10 
								pop r9
								pop r8
								pop r7
								pop r6
								POPF
								ret
				; ----------
; --------------------------------------------------------------------------------
; --------------------------------------------------------------------------------
DECREASE_U:						PUSHF
								push r6
								push r7
								push r8
								push r9
								push r10
								push r11
								push r12
								push r13

								push r16
								push r17
								push r18
								push r19
				; ----------
								lds r16,(MODE)
								cpi r16,ExtOffMode
								brne DECREASE_A
								rjmp DECREASE_EX
DECREASE_A:						cpi r16,StabMode
								breq DECREASE_RUN
									lds r16,(IndicatorAddr)
									cpi r16,IND_ADDR_A
									breq DECREASE_RUN
									cpi r16,IND_ADDR_B
									breq DECREASE_RUN
									rjmp DECREASE_EX				
				; ----------
DECREASE_RUN:					lds ZL,(FL_MAX_U)
								cpi ZL,$FF
								brne DECREASE_NORM

								lds r16,(MODE)
								cpi r16,0
								brne DECREASE_NORM
								; ----------
									ldi r16,low (DEFAULT_STEP_U)
									ldi r17,high(DEFAULT_STEP_U)
									movw r8,r16
									ldi r16,low (20)
									ldi r17,high(20)
									movw r6,r16
									call mul1616
									movw r8,r10						;	[r9..8] = STEP*20
									lds r6,(AVERAGE_U+0)			; 
									lds r7,(AVERAGE_U+1)	
						; 
									sub r6,r8
									sbc r7,r9						;	[R7:6]=AVERAGE_U - STEP*20
									clr r8
									clr r9
									ldi r16,low (DEFAULT_STEP_U)
									ldi r17,high(DEFAULT_STEP_U)
									movw r10,r16
									clr r12
									clr r13
									call div32u						;	[R7:6]=[(AVERAGE_U - STEP*20)/STEP]

									ldi r16,low (DEFAULT_STEP_U)
									ldi r17,high(DEFAULT_STEP_U)
									movw r8,r16
									call MUL1616					;	[R10:9]=[R7:6]*STEP
									movw r16,r10					;	[R17:16]=[R7:6]*STEP
									rjmp DECREASE_CONT	
									; ----------
DECREASE_NORM:					lds r16,(TARGET_U+0)
								lds r17,(TARGET_U+1)

DECREASE_CONT:					
								lds r18,(STEP_U+0)
								lds r19,(STEP_U+1)
								sub r16,r18
								sbc r17,r19
								
								call normU

DECREASE_SKIP:					ldi r18, low(MIN_U)
								ldi r19,high(MIN_U)
								cp  r16,r18
								cpc r17,r19
								sbrc r17,7
								rjmp DECREASE_ZERO
								brcc DECREASE_SET		; Если следующее значение больше или равно минимальному, то устанавливаем его.
DECREASE_ZERO:					clr r16					; Иначе следующее значение устанавливаем = 0
								clr r17					;

DECREASE_SET:					sts (TARGET_U+0),r16
								sts (TARGET_U+1),r17

								ldi r16,TargetV
								call setNewValFlag
								ldi r16,MainV
								call setNewValFlag

DECREASE_EX:					pop r19
								pop r18
								pop r17
								pop r16

								pop r13
								pop r12
								pop r11
								pop r10 
								pop r9
								pop r8
								pop r7
								pop r6
								POPF
								ret
//-----------------------------------------------------------
normU:							movw r6,r16					;	TARGET
								clr r8
								clr r9
								ldi r16,low (DEFAULT_STEP_U)	;	STEP
								ldi r17,high(DEFAULT_STEP_U)
								movw r10,r16
								clr r12
								clr r13
								call div32u					;	TARGET/STEP
								movw r8,r16
								call mul1616
								movw r16,r10
								ret
//-----------------------------------------------------------
; сигналы на ножках порта С
EXT_SIGNALS:					PUSHF
								push r16
								push r17
								push r18

								in r16,PINC
								ldi r17,0b11111111		;	0b11111110 для инверсного входа аварии
								eor r16,r17				;	r16 = побитно 0 - нет сигнала на ноге, 1 - есть сигнал
								lds r17,(oldPINC)		;	r17 - предыдущее состояние порта С
								sts (oldPINC),r16
								eor r17,r16				;	r17 = побитно 0 - нет изменения сигнала на ноге, 1 - есть изменение

								lds r18,(MODE)
								cpi r18,ExtOffMode
								breq EXT_SIGNALS_Exit		;	если режим авария(3), то на выход

								call checkA0
								call checkA1

EXT_SIGNALS_Exit:				pop r18
								pop r17
								pop r16
								POPF
								ret
//-----------------------------------------------------------
; Аварийный сигнал
checkA0:						push r16
								push r17
								sbrs r16,ExtOFFPin
								rjmp SetPinCCounter0		;	если сигнал не активен(0), устанавливаем счетчик#0 задержки срабатывания
								lds r17,(PinCCounter0)		;	иначе вспомним, что насчитал счетчик задержки

								sbrs r17,7					;	если он неотрицательный
								rjmp PinCCounter0NotNegatve	;	то прыгаем на проверку на 0

								rjmp SetPinCCounter0		;	иначе устанавливаем счетчик

PinCCounter0NotNegatve:			cpi r17,$00					;	если счетчик равен 0
								breq checkA0Exit			;	то на выход
	
PinCCounter0isZero:				dec r17						;	иначе уменьшаем счетчик
								sts (PinCCounter0),r17
								brne checkA0Exit			;	и, если не 0, то на выход
															;	если счетчик = 0,
								dec r17						;	то делаем его отрицательным
								sts (PinCCounter0),r17
PinCA0_isHIGH:					ldi r16,ExtOffMode			;	и устанавливаем режим аварийного отключения 3
								call setMode
								rjmp checkA0Exit

SetPinCCounter0:				lds r17,(MODE)
								cpi r17,ExtOffMode
								breq checkA0Exit
								ldi r17,20					;	количество циклов задержки перед срабатыванием
								sts (PinCCounter0),r17

checkA0Exit:					pop r17
								pop r16
								ret
//-----------------------------------------------------------
; Внешний разгон
checkA1:						push r16
								push r17

									lds r18,(MODE)
									cpi r18,ExtOffMode
									breq checkA1Exit

								sbrc r17,ExtForcePin
								rjmp SetPinCCounter1		;	если сигнал изменился, устанавливаем счетчик#1 задержки срабатывания
								lds r17,(PinCCounter1)		;	вспомним, что насчитал счетчик задержки

								sbrs r17,7					;	если он неотрицательный
								rjmp PinCCounter1NotNegatve	;	то прыгаем на проверку на 0

								clr r17						;	иначе обнуляем счетчик
								sts (PinCCounter1),r17
								rjmp checkA1Exit			;	и на выход

PinCCounter1NotNegatve:			cpi r17,$00					;	если счетчик равен 0
								breq checkA1Exit			;	то на выход
	
PinCCounter1isZero:				dec r17						;	иначе уменьшаем счетчик
								sts (PinCCounter1),r17
								brne checkA1Exit			;	и, если не 0, то на выход
															;	если счетчик = 0, то начинаем обработку изменения сигнала
								lds r17,(MODE)							
								sbrc r16,ExtForcePin
								rjmp PinCA1_isHIGH

								cpi r17,ForceMode
								brne checkA1Exit
								ldi r16,StabMode			;	если сигнал = 1->0
								call setMode				
								rjmp checkA1Exit

PinCA1_isHIGH:					cpi r17,ForceMode
								breq checkA1Exit
								ldi r16,ForceMode			;	если сигнал = 0->1
								call setMode
								rjmp checkA1Exit

SetPinCCounter1:				ldi r17,5					;	количество циклов задержки перед срабатыванием
								sts (PinCCounter1),r17
checkA1Exit:					
								pop r17
								pop r16
								ret
//-----------------------------------------------------------
