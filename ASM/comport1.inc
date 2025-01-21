RX_OK:		PUSHF
			PUSHZ
			push r17
			push r16
			lds r16,(ACTIONS_FLAGS)
			sbrc r16,0
			rjmp RX_EXIT				; если предыдущее заклинание не обработано, похериваем всё

			LDZ RX_BUFFER				; Начало буфера приема

			lds r16,UDR0				; пришедший байт
			cpi r16,'S'
				brne RX_cond1
			rjmp RX_SMT
RX_cond1:		cpi r16,'s'
				brne RX_cond2
				ldi r16,'S'
			rjmp RX_SMT
RX_cond2:		cpi r16,'M'
				brne RX_cond3
			rjmp RX_SMT
RX_cond3:		cpi r16,'m'
				brne RX_cond4
				ldi r16,'M'
			rjmp RX_SMT
RX_cond4:		cpi r16,'T'
				brne RX_cond5
			rjmp RX_SMT
RX_cond5:	cpi r16,$0D
				brne RX_cond6
				rjmp RX_0D
RX_cond6:		rjmp RX_HEXDIGITS
; ---------- Приход S,M,T
RX_SMT:		st Z,r16
			clr r16
			sts (RX_PTR),r16
				//call Echo	
			rjmp RX_EXIT	
; ---------- Все прочие символы кроме #0D
RX_HEXDIGITS:	cpi r16,'0'
					brlo RX_nodigit
				cpi r16,'9'
					brlo RX_good
					breq RX_good
				cpi r16,'A'
					brlo RX_nodigit
				cpi r16,'F'
					brlo RX_good
					breq RX_good
				subi r16,$20
				cpi r16,'A'
					brlo RX_nodigit
				cpi r16,'F'
					brlo RX_good
					breq RX_good
RX_nodigit:		rjmp RX_EXIT
RX_good:
				lds r17,(RX_PTR)
				inc r17
				cpi r17,$0F
				brlo RX_nofull
				breq RX_nofull
				rjmp RX_EXIT
				/*andi r17,$0F
				tst r17	
				brne RX_M3
				dec r17
				andi r17,$0F*/
RX_nofull:		sts (RX_PTR),r17
				add ZL,r17
				ldi r17,0
				adc ZH,r17
					st Z,r16
			rjmp RX_EXIT
; ---------- Пришел символ #0D
RX_0D:		//call Echo
			lds r16,(ACTIONS_FLAGS)
			SETB r16,0						; устанавливаем флаг нового заклинания
			sts (ACTIONS_FLAGS),r16			; в RX_PTR количество символов после управляющего (S,M,T)
RX_EXIT:
			pop r16
			pop r17
			POPZ
			POPF
			reti
; ----------------------------------------------------------------------------------------
stringReceived:		PUSHF
					PUSHZ
					push r16
					push r17
					push r18

					LDZ RX_BUFFER
					lds r17,(RX_PTR)

					ld r16,Z+
					cpi r16,'S'
						brne sR_1
						rjmp sR_S
sR_1:				cpi r16,'M'
						brne sR_2
						rjmp sR_M
sR_2:				cpi r16,'T'
						brne sR_3
						rjmp sR_T
sR_3:				rjmp sR_ex
; ----------
sR_S:				;lds r16,(MODE)
					;tst r16
					;breq sR_S1
					;rjmp sR_ex

sR_S1:				clr r18
;					cpi r17,4
;					breq sR_S2
					cpi r17,3
					breq sR_S2a
					cpi r17,2
					breq sR_S2b
					rjmp sR_ex

sR_S2:				ld r16,Z+
					call strToHex
					brcs sR_S3
					swap r16
					andi r16,$F0
					mov r18,r16

sR_S2a:				ld r16,Z+
					call strToHex
					brcs sR_S3
					andi r16,$0F
					or r18,r16
					
sR_S2b:				ld r16,Z+
					call strToHex
					brcs sR_S3
					swap r16
					andi r16,$F0
					mov r17,r16

					ld r16,Z+
					call strToHex
					brcs sR_S3
					andi r16,$0F
					or r17,r16

.equ	maximumU	= 2500
					ldi r16, low(maximumU+1)
					cp  r17,r16
					ldi r16,high(maximumU+1)
					cpc r18,r16
					brcc sR_S3

					sts (TARGET_U+1),r18
					sts (TARGET_U+0),r17
					ldi r16,$FF
					sts (FL_NEW_U),r16

					
sR_S3:				rjmp sR_ex
; ----------
; ----------
sR_M:				cpi r17,1
					breq sR_M1
					rjmp sR_ex

sR_M1:				ld r16,Z+
					call strToHex
					brcs sR_M3

					cpi r16,3
					brcs sR_M2
					rjmp sR_ex

sR_M2:				sts (MODE),r16
					//lds r16,(FL_NEW_M)
					ldi r16,$FF
					sts (FL_NEW_M),r16
sR_M3:				rjmp sR_ex
			
; ----------
; ----------
sR_T:				nop
					rjmp sR_ex
; ----------
sR_ex:				clr r16
					sts (RX_PTR),r16
					lds r16,(ACTIONS_FLAGS)
					CLRB r16,0
					sts (ACTIONS_FLAGS),r16

					pop r18
					pop r17
					pop r16
					POPZ
					POPF
					ret
; ----------------------------------------------------------------------------------------
TX_OK:		reti
; ----------------------------------------------------------------------------------------
UD_OK:		PUSHF
			PUSHZ
			push r17
			push r16
			LDZ TX_BUFFER
			lds r16,(TX_PTR)
			ldi r17,0
			add ZL,r16
			adc ZH,r17
			inc r16
			andi r16,$1F
			cpi r16,0
			breq StopUD
			sts (TX_PTR),r16
			ld r16,Z+

			cpi r16,0
			breq StopUD

			sts UDR0,r16

UD_Exit:	pop r16
			pop r17
			POPZ
			POPF
			reti
			; ----------
StopUD:		ldi r16,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0)|(0<<UDRIE0)
			sts UCSR0B,r16
			rjmp UD_Exit
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
PrepareDataToSend:	PUSHF
					PUSHXYZ
					push r16
					push r17

					LDX TX_BUFFER

					ldi r16,'T'
					st X+,r16
					;---------- average U
					LDY AVERAGE_U
					ldd r16,Y+1
					call HexToAscii
//						st X+,r17
					st X+,r16
					ldd r16,Y+0
					call HexToAscii
					st X+,r17
					st X+,r16
					;---------- target U
					//LDY TARGET_U
					ldd r16,Y+3
					call HexToAscii
//						st X+,r17
					st X+,r16
					ldd r16,Y+2
					call HexToAscii
					st X+,r17
					st X+,r16
					;---------- mode
					LDY MODE
					ld r16,Y
					call HexToAscii
					;	st X+,r17
					st X+,r16
					;----------
/*					LDY LED_ADDRESS
					ld r16,Y
					call HexToAscii
					st X+,r17
					st X+,r16
					;----------
					LDY SOFT_I2C_ACK
					ld r16,Y
					call HexToAscii
					st X+,r17
					st X+,r16
*/
					;----------
					ldi r16,$0D
					st X+,r16
					ldi r16,$0A
					st X+,r16

					ldi r16,0
					st X+,r16

					sts (TX_PTR),r16

					ldi r16,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0)|(1<<UDRIE0)
					sts UCSR0B,r16

					pop r17
					pop r16
					POPXYZ
					POPF
					ret
; --------------------------------------------------------
strToHex:			cpi r16,$30
					brcs sthError
					cpi r16,$3A
					brcc sthCheck1
					subi r16,$30
					clc
					rjmp sthExit
sthCheck1:			cpi r16,$41
					brcs sthError
					cpi r16,$47
					brcc sthCheck2
					subi r16,$37
					clc 
					rjmp sthExit
sthCheck2:			cpi r16,$61
					brcs sthError
					cpi r16,$67
					brcc sthError
					subi r16,$57
					clc 
					rjmp sthExit
sthError:			sec
sthExit:			ret
