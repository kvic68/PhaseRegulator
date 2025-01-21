; ------------------------------------------------------------------------
; LED BLOCK
.equ	command		= 0b10000000
.equ	cmdstream	= 0b00000000
.equ	onedata		= 0b11000000
.equ	datastream	= 0b01000000
.equ	led_1306	= 0b01111000	; 0b01111010
; ------------------------------------------------------------------------
LED_ON:				call startI2Csoft
					lds r16,LED_ADDRESS
					call outbyteI2Csoft
					ldi r16,command
					call outbyteI2Csoft
					ldi r16,$AF
					call outbyteI2Csoft
					call stopI2Csoft
					ret
; ------------------------------------------------------------------------
; очистка строки, номер в R16
LED_CLRLINE:		push r16
					push r17
					push r18
					ldi r17,0
					mov r18,r16
					call SET_LED_XY
					call startI2Csoft

					lds r16,LED_ADDRESS
					call outbyteI2Csoft
					ldi r16,datastream
					call outbyteI2Csoft

					ldi r17,132
LED_CLRL:				ldi r16,$00
						call outbyteI2Csoft
						dec r17
						brne LED_CLRL
					call stopI2Csoft

					pop r18
					pop r17
					pop r16	
					ret
; ------------------------------------------------------------------------
; очистка строки, номер в R16, образец в R18
LED_FIL_LINE:		push r16
					push r17
					push r18
					ldi r17,0
					mov r18,r16
					call SET_LED_XY
					call startI2Csoft

					lds r16,LED_ADDRESS
					call outbyteI2Csoft
					ldi r16,datastream
					call outbyteI2Csoft

					ldi r17,128
					pop r18
LED_FILL:				mov r16,r18
						call outbyteI2Csoft
						dec r17
						brne LED_FILL
					call stopI2Csoft
					pop r17
					pop r16	
					ret
; ------------------------------------------------------------------------
INIT_LED:			call startI2Csoft
					lds r16,LED_ADDRESS
					call outbyteI2Csoft
INIT_LED_CONT:		ldi r16,cmdstream	; режим потока команд
					call outbyteI2Csoft
					// передача команд
					ldi r17,(LED_INIT_STREAM_END-LED_INIT_STREAM)*2
					LDZ LED_INIT_STREAM*2
INIT_LED_CYCLE:			lpm r16,Z+
						call outbyteI2Csoft
						dec r17
						tst r17
						brne INIT_LED_CYCLE
					call stopI2Csoft
					//
					ret
; ------------------------------------------------------------------------
LED_INIT_STREAM:		.db $AE,$D5,$80,$A8,$3F,$D3,$00,$40,$8D,$14,$A1,$C8,$DA,$12,$81,$F0,$D9,$22,$DB,$30,$A4,$A6
LED_INIT_STREAM_END:	
; ------------------------------------------------------------------------
LED_CLEAR:			push r16
					push r17
					push r18
				
					ldi r18,8
LED_CLR_Outer:		ldi r17,132
					dec r18
					push r17
					ldi r17,0
					call SET_LED_XY
					pop r17
					//
					call startI2Csoft
					lds r16,LED_ADDRESS
					call outbyteI2Csoft
					ldi r16,datastream
					call outbyteI2Csoft
LED_CLR_Inner:			ldi r16,$00
						call outbyteI2Csoft
						dec r17
						brne LED_CLR_Inner
					call stopI2Csoft
					tst r18
					brne LED_CLR_Outer

					pop r18
					pop r17
					pop r16	
					ret
; ------------------------------------------------------------------------
; ------------------------------------------------------------------------
// r17 - X(0...127), r18 - Y(0...7)
SET_LED_XY:			PUSHFand r16
				
					sts LCD_X,r17
					sts LCD_Y,r18

					call startI2Csoft
					lds r16,LED_ADDRESS
					call outbyteI2Csoft
					ldi r16,cmdstream
					call outbyteI2Csoft

					mov r16,r18
					andi r16,$07
					ori r16,$B0
					call outbyteI2Csoft

					mov r16,r17
					andi r16,$0F
					call outbyteI2Csoft
					mov r16,r17
					swap r16
					andi r16,$0F
					ori r16,$10
					call outbyteI2Csoft

					call stopI2Csoft

					POPFand r16
					
					ret
; ------------------------------------------------------------------------
; ---- Вывод целевого напряжения мелким шрифтом в формате 3.1 ----
TARGET_SMALL_OUT:	PUSHFand r16
					PUSHZ
					push r17

					ldi r16,$FF
					sts (FL_LOCK),r16

					lds r16,(TARGET_U+0)
					lds r17,(TARGET_U+1)

					call bin2bcd16

					ldi r16,$0F					;	пробел
					call PRINTSmallF
					//----------
					; цифири в R11,R10
					mov r16,r11
					swap r16
					andi r16,$0F
					tst r16						;	если 3 разряд = 0, то
					brne tso_0

					ldi r16,$0F					;	выводим в 3 разряд пробел
tso_0:				call PRINTSmallF
					//----------
					mov r16,r11
					tst r16						;	если второй и третий разряды = 0, то
					brne tso_1
					ldi r16,$0F					;	выводим во 2 разряд пробел
					//rjmp tso_2
tso_1:				andi r16,$0F
					call PRINTSmallF
					//----------
					mov r16,r10
					swap r16
					andi r16,$0F
					call PRINTSmallF			;	выводим 1 разряд
					//----------
					ldi r16,$0E					;	выводим десятичную точку
					call PRINTSmallF
					//----------
					mov r16,r10
					andi r16,$0F
					call PRINTSmallF			;	выводим дробную часть
					//----------
					clr r16
					sts (FL_LOCK),r16

					pop r17
					POPZ
					POPFand r16
					ret
; ------------------------------------------------------------------------
VOLT_OUT_LED:		PUSHFand r16
					PUSHZ
					push r17

					ldi r16,$FF
					sts (FL_LOCK),r16

					ldi r17,0						; X
					ldi r18,3						; Y
					call SET_LED_XY

					lds r16,(TM_BUFFER+0)
					lds r17,(TM_BUFFER+1)
					call bin2bcd16

					; цифири в R11,R10
					mov r16,r11
					swap r16
					andi r16,$0F
					tst r16
					brne vo0

					ldi r16,20	; пробел

vo0:				call PRINTBigDigit

					mov r16,r11
					tst r16
					brne vo1

					ldi r16,20	;	пробел
					rjmp vo2

vo1:				andi r16,$0F
vo2:				call PRINTBigDigit

					mov r16,r10
					swap r16
					andi r16,$0F
					call PRINTBigDigit

					mov r16,r10
					andi r16,$0F
					subi r16,(-10)

					call PRINTBigDigit

					ldi r16,$00
					sts (FL_LOCK),r16

					pop r17
					POPZ
					POPFand r16
					ret
; ------------------------------------------------------------------------
; r16 - код символа
.equ	BigDRowCount = 5
.equ	BigDColCount = 32
PRINTBigDigit:			push r16
						push r17
						push r18

						ldi ZL, low(FONT32x40*2)
						ldi ZH,high(FONT32x40*2)

						ldi r17,BigDRowCount*BigDColCount
						mul r16,r17
						add ZL,r0
						adc ZH,r1

						lds r17,LCD_X
						lds r18,LCD_Y
						push r17
						push r18

						ldi r17,BigDRowCount						; количество рядов символа
PRINTBigDigitOuterLoop:		push r17

							call startI2Csoft
							lds r16,LED_ADDRESS
							call outbyteI2Csoft
							ldi r16,datastream
							call outbyteI2Csoft

							ldi r17,BigDColCount						; количество столбцов символа
PRINTBigDigitInnerLoop:			lpm r16,Z+
								call outbyteI2Csoft
								dec r17
								brne PRINTBigDigitInnerLoop
							call stopI2Csoft
							// восстановить X, увеличить Y	
							lds r17,LCD_X
							lds r18,LCD_Y
							inc r18
							call SET_LED_XY

							pop r17
							dec r17
							brne PRINTBigDigitOuterLoop

						// новое X, новое Y
						pop r18
						pop r17
						subi r17,(-BigDColCount)
						call SET_LED_XY

						pop r18
						pop r17
						pop r16
						ret
; ------------------------------------------------------------------------
; r16 - код символа
.equ	SmFRowCountA = 2 ;высота
.equ	SmFColCountA = 9 ;ширина
PRINTSmallF:			push r16
						push r17
						push r18
						PUSHZ

						ldi ZL, low(FONT9x16*2)
						ldi ZH,high(FONT9x16*2)

						cpi r16,$10						;	меньше $10 это цифры и символы
						brcs PRINTSmallFRun
						cpi r16,$C0						;	от $10 до $BF не печатаем
						brcs PRINTSmallFGoExit
						cpi r16,$E0						;	печатаем только большие русские символы от $C0 до $DF
						brcs PRINTSmallFRuCap
PRINTSmallFGoExit:		rjmp PRINTSmallFExit			;	всё остальное больше и равное $E0 не печатаем
PRINTSmallFRuCap:		subi r16,$B0					;	корректируем код русских символов

PRINTSmallFRun:			ldi r17,SmFRowCountA*SmFColCountA
						mul r16,r17
						add ZL,r0
						adc ZH,r1

						lds r17,LCD_X
						lds r18,LCD_Y
						push r17
						push r18

						ldi r17,SmFRowCountA			; количество рядов символа
PRINTSmallFOuterLoop:		push r17

							call startI2Csoft
							lds r16,LED_ADDRESS
							call outbyteI2Csoft
							ldi r16,datastream
							call outbyteI2Csoft
							ldi r17,SmFColCountA			; количество столбцов символа

PRINTSmallFInnerLoop:			lpm r16,Z+
								call outbyteI2Csoft
								dec r17
								brne PRINTSmallFInnerLoop

							call stopI2Csoft
							// восстановить X, увеличить Y	
							lds r17,LCD_X
							lds r18,LCD_Y
							inc r18
							call SET_LED_XY

							pop r17
							dec r17
							brne PRINTSmallFOuterLoop

						// новое X, новое Y
						pop r18
						pop r17
						subi r17,(-SmFColCountA)
						call SET_LED_XY

PRINTSmallFExit:		POPZ
						pop r18
						pop r17
						pop r16
				ret

; ------------------------------------------------------------------------
; ------------------------------------------------------------------------
; IND_SCL = SCL, IND_SDA = SDA
;***********************************
startI2Csoft:	;маркер начала посылки
					PUSHFand r18
					cli
					M_DELAY_US_RA 1,r18		;	пауза 1 мкс
					sbi	IND_PORT,IND_SCL	;
					sbi	IND_PORT,IND_SDA	;
					M_DELAY_US_RA 1,r18		;	пауза 1 мкс
					cbi	IND_PORT,IND_SDA	;	
					M_DELAY_US_RA 1,r18		;	пауза 1 мкс
					POPFand r18
					ret
;***********************************
stopI2Csoft:	;маркер конца посылки
					PUSHFand r18
					cli
					sbi	IND_PORT,IND_SCL	;
					M_DELAY_US_RA 1,r18		;	пауза 1 мкс
					sbi	IND_PORT,IND_SDA	;
					M_DELAY_US_RA 1,r18		;	пауза 1 мкс
					POPFand r18
					ret
;***********************************
outbyteI2Csoft:	;последовательный вывод. выводимый байт должен находиться в r16
					PUSHFand r17
					cli
					push r18
					; вывод восьми битов регистра r16
					ldi r17,0x08	;
outByteCycle:		cbi	IND_PORT,IND_SCL	;опускаем CLK
					M_DELAY_US_RA 1,r18		;пауза 1 мкс

					rol	r16					; \
					brcc outBitLow			;  |
					sbi	IND_PORT,IND_SDA	;   > старший бит r16 выставляем на SDA через флаг C
					rjmp outBitAny			;  |
outBitLow:			cbi	IND_PORT,IND_SDA	; /

outBitAny:			M_DELAY_US_RA 1,r18		;пауза 1 мкс

					sbi	IND_PORT,IND_SCL	;поднимаем CLK
					M_DELAY_US_RA 1,r18		;пауза 1 мкс
					dec r17		;
					brne outByteCycle		;последовательно выводим весь байт
					
					; ACK-бит (9)

					cbi	IND_PORT,IND_SCL	;опускаем CLK
					M_DELAY_US_RA 1,r18		;пауза 1 мкс

					cbi	IND_DDR,IND_SDA		;переключем SDA как вход, чтобы исключить коллизию с ACK
					//sbi IND_PORT,IND_SDA	;с подтяжкой

					sbi	IND_PORT,IND_SCL	;поднимаем CLK
					M_DELAY_US_RA 1,r18		;пауза 1 мкс

					in r17, IND_PIN			;читаем состояние ноги SDA
					clr r16
					sbrc r17,IND_SDA		;SDA 0=ACK, 1=NACK
					dec r16					;r16 = 0 - ACK, FF - NACK
					sts SOFT_I2C_ACK,r16

					cbi	IND_PORT,IND_SCL	;опускаем CLK
					cbi	IND_PORT,IND_SDA	;ноль в SDA	
					sbi	IND_DDR,IND_SDA		;окончание АСК-бита, переключаем SDA как выход
											;(выход с нулями на обеих шинах)
					M_DELAY_US_RA 1,r18		;пауза 1 мкс
					pop r18
					POPFand r17
					ret			
; ------------------------------------------------------------------------
