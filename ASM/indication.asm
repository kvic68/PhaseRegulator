indication:		; вывод возможен только тогда, когда сброшен флаг блокировки вывода
				lds r16,(FL_LOCK)
				tst r16
				breq out_enabled
				rjmp out_disabled

out_enabled:	lds r16,(LED_ADDRESS)
				cpi r16,$FF
				breq out_TM1637
				cpi r16,$78
				breq out_OLED
				cpi r16,$7A
				breq out_OLED
				rjmp out_disabled
//*******************************************************
out_TM1637:		; вывод на индикатор ТМ1637 по флагу
				lds r16,(FL_NEW_U)
				tst r16
				breq notShowU1637
				
				clr r16
				sts (FL_NEW_U),r16

				lds r16,(MODE)

				cpi r16,0
				breq showStab1637

				cpi r16,1
				breq showCurrent1637

				cpi r16,2
				breq showStop

				cpi r16,3
				breq showExtStop

				rjmp notShowU1637
								
				clr r16
				clr r17
				rjmp showU1637

showStop:		ldi r16,$FF
				ldi r17,$D0
				rjmp showU1637hex

showExtStop:	ldi r16,$AB
				ldi r17,$A8
				rjmp showU1637hex

showStab1637:		lds r16,(FL_MAX_U)
					tst r16
				breq showTarget1637

showCurrent1637:	lds r16,(AVERAGE_U+0)
					lds r17,(AVERAGE_U+1)
				rjmp showU1637


showTarget1637:		lds r16,(TARGET_U+0)
					lds r17,(TARGET_U+1)

showU1637:			sts (TM_BUFFER+0),r16
					sts (TM_BUFFER+1),r17
					call tm1637
					rjmp notShowU1637 

showU1637hex:		sts (TM_BUFFER+0),r16
					sts (TM_BUFFER+1),r17
					call tm1637hex
					rjmp notShowU1637 
	
notShowU1637:		nop	



			rjmp out_disabled
//*****************************************
				; вывод на OLED индикатор по флагам новых значений
out_OLED:		lds r16,(FL_NEW_U)
				tst r16
				brne out_bigvolt
				rjmp chk_newmode
				
out_bigvolt:	clr r16
				sts (FL_NEW_U),r16

				lds r16,(MODE)
				cpi r16,2
				breq showCurrent;breq zero_out
				cpi r16,3
				breq no_val_out
				rjmp value_out

no_val_out:		clr r16
				clr r17
				sts (TM_BUFFER+0),r16
				sts (TM_BUFFER+1),r17
				rjmp chk_newmode

zero_out:		clr r16
				clr r17
				rjmp saveBUFFER

value_out:		lds r16,(FL_MAX_U)
				tst r16
				breq showTarget

showCurrent:	lds r16,(AVERAGE_U+0)
				lds r17,(AVERAGE_U+1)
				rjmp saveBUFFER


showTarget:		lds r16,(MODE)
				cpi r16,1
				breq showCurrent

				lds r16,(TARGET_U+0)
				lds r17,(TARGET_U+1)

saveBUFFER:		sts (TM_BUFFER+0),r16
				sts (TM_BUFFER+1),r17
				call VOLT_OUT_LED
				/////////////////////////////

chk_newmode:	lds r16,(FL_NEW_M)
				tst r16
				brne out_newmode
				rjmp chk_newtarget
				//-------------
				; изменение режима работы вывод на экран
out_newmode:	clr r16
				sts (FL_NEW_M),r16

				ldi r16,0
				call LED_CLRLINE
				ldi r16,1
				call LED_CLRLINE 
				ldi r16,2
				ldi r18,0b00010000
				call LED_FIL_LINE

				lds r16,(MODE)
				andi r16,$03
				cpi r16,3
				brne out_newmode_1
				call LED_CLEAR
out_newmode_1:	call printmode
				
				;
chk_newtarget: 	lds r16,(MODE)
				cpi r16,1
				breq out_newtarget
				cpi r16,2
				breq out_newtarget
				//cpi r16,3
				//breq out_newtarget

				rjmp chk_inrange
				//-------------
out_newtarget:	call printLittleTarget
				clr r16
				sts (FL_NEW_T),r16
				//-------------
				rjmp inrange_ex 

				//-------------
chk_inrange:	lds r16,(MODE)
				tst r16
				breq chk_inrange_a
				rjmp inrange_ex

chk_inrange_a:	lds r16,(FL_INRANGE)
				sbrc r16,7
				rjmp out_inrange
				rjmp chk_enother
				//-------------
out_inrange:	andi r16,$0F
				sts (FL_INRANGE),r16
				//andi r16,$0F
				tst r16
				breq inrange_off
				rjmp inrange_on

inrange_off:	ldi r17,107
				ldi r18,0
				call SET_LED_XY
				ldi r16,$0A		call PRINTSmallF
				ldi r16,$0B		call PRINTSmallF
					//cbi portb,5
				rjmp inrange_ex

inrange_on:		ldi r17,107
				ldi r18,0
				call SET_LED_XY
				ldi r16,$0C		call PRINTSmallF
				ldi r16,$0D		call PRINTSmallF
					//sbi portb,5
inrange_ex:
				//-------------
chk_enother:
				//-------------
out_disabled:	
				ret
//*********************************************
.equ	max_counter_b = 20;
top_rail:		
				lds r16,(MODE)
				tst r16
				breq trrA
				ldi r16,0
				sts (COUNTER_B),r16
				//
				lds r16,(MODE)
				cpi r16,3
				breq trr0
				ldi r16,0
				sts (SOUNDNUMBER),r16
trr0:			rjmp trrExit
					
trrA:			lds r16,(SOUNDNUMBER)
				tst r16
				brne trrB
				rjmp trrG

trrB:			lds r16,(FL_MAX_U)
				tst r16
				breq trrC
				rjmp trrE
trrC:			lds r16,(COUNTER_B)
				cpi r16,max_counter_b
				breq trrD
				rjmp trrF
trrD:			ldi r16,0
				sts (SOUNDNUMBER),r16
				sts (COUNTER_B),r16
				rjmp trrExit
trrE:			ldi r16,0
				sts (COUNTER_B),r16
				rjmp trrExit
trrF:			lds r16,(COUNTER_B)
				inc r16
				sts (COUNTER_B),r16
				rjmp trrExit
trrG:			lds r16,(FL_MAX_U)
				tst r16
				breq trrE
trrH:			lds r16,(COUNTER_B)
				cpi r16,max_counter_b
				breq trrI
				rjmp trrF
trrI:			ldi r16,1
				sts (SOUNDNUMBER),r16
				rjmp trrE
trrExit:	
				ret
; --------------------------------------------------------------------------------
printLittleTarget:		ldi r17,72				;	X
						ldi r18,0				;	Y
						call SET_LED_XY
						call TARGET_SMALL_OUT	
						ret
; --------------------------------------------------------------------------------
printmode:				LDZ modetextpointer*2
						clr r17
						add ZL,r16	adc ZH,r17
						add ZL,r16	adc ZH,r17
						lpm r16,Z+
						lpm r17,Z
						movw ZH:ZL,r17:r16		; в Z начало текста, первый байт X, второй - Y
						add ZL,ZL
						adc ZH,ZH
printmode_loop:				lpm r16,Z+
							tst r16
							breq printmode_exit
							cpi r16,$80
							brne printmode_char
							lpm r17,Z+
							lpm r18,Z+
							call SET_LED_XY
							rjmp printmode_loop
printmode_char:			call PRINTSmallF
						rjmp printmode_loop

printmode_exit:			ret
; --------------------------------------------------------------------------------
modetextpointer:	.dw text0,text1,text2,text3
; --------------------------------------------------------------------------------
text0:	.db $80,1,0,"ВЫХОД",$00,$00
text1:	.db $80,1,0,"РАЗГОН",$00
text2:	.db $80,1,0,"СТОП",$00
text3:	.db $80,24,0,"ОТКЛЮЧЕНО",\
			$80,33,3,"ВНЕШНИМ",\
			$80,28,6,"СИГНАЛОМ",\
			$00
; --------------------------------------------------------------------------------
