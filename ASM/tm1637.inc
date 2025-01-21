;****************************************************************
; Вывод URMS на индикатор, базирующийся на TM1637
;***************************************************
tm1637hex:	PUSHFand r16
			PUSHZ

			rcall	start1637		;маркер начала посылки
			ldi	r16,0xC0			;УСТАНОВКА НАЧАЛЬНОГО АДРЕСА (0хC0 - крайнее левое знакоместо)
			rcall	outByte1637		;вывожу адрес

			ldi ZL, low(DIGIT7*2)
			ldi ZH,high(DIGIT7*2)
		movw r12,Z

			lds r16,(TM_BUFFER+1)
			andi r16,$F0
			swap r16
			add ZL,r16
			ldi r16,0
			adc ZH,r16
			lpm r16,Z
			call outByte1637
		movw Z,r12

			lds r16,(TM_BUFFER+1)
			andi r16,$0F
			add ZL,r16
			ldi r16,0
			adc ZH,r16
			lpm r16,Z
			call outByte1637
		movw Z,r12

			lds r16,(TM_BUFFER+0)
			andi r16,$F0
			swap r16
			add ZL,r16
			ldi r16,0
			adc ZH,r16
			lpm r16,Z
			call outByte1637
		movw Z,r12

			lds r16,(TM_BUFFER+0)
			andi r16,$0F
			add ZL,r16
			ldi r16,0
			adc ZH,r16
			lpm r16,Z
			call outByte1637

			rcall	end1637		;маркер конца посылки
			POPZ
			POPFand r16
			ret
;***************************************************
tm1637:	PUSHF
		push r16
		push r17
		push ZL
		push ZH
		rcall	start1637		;маркер начала посылки
		ldi	r16,0xC0			;УСТАНОВКА НАЧАЛЬНОГО АДРЕСА (0хC0 - крайнее левое знакоместо)
		rcall	outByte1637		;вывожу адрес

			lds r16,(TM_BUFFER+0)
			lds r17,(TM_BUFFER+1)

			call bin2bcd16

			//movw r10,r16

		; цифири в R11,R10
		ldi ZL, low(DIGIT7*2)
		ldi ZH,high(DIGIT7*2)
		movw r12,Z
		clr r17
		mov r16,r11
		swap r16
		andi r16,$0F
		tst r16
		brne zz0
		ldi r16,$0F
zz0:	add ZL,r16
		adc ZH,r17
		lpm r16,Z
		call outByte1637

		movw Z,r12
		clr r17
		mov r16,r11
		tst r16
		brne zz1
		ldi r16,$FF
zz1:	andi r16,$0F
		add ZL,r16
		adc ZH,r17
		lpm r16,Z
		call outByte1637

		movw Z,r12
		clr r17
		mov r16,r10
		swap r16
		andi r16,$0F
		add ZL,r16
		adc ZH,r17
		lpm r16,Z
		ori r16,$80
		call outByte1637

		movw Z,r12
		clr r17
		mov r16,r10
		andi r16,$0F
		add ZL,r16
		adc ZH,r17
		lpm r16,Z
		call outByte1637

		rcall	end1637		;маркер конца посылки

		pop ZH
		pop ZL
		pop r17
		pop r16
		POPF
	ret
;***********************************
DIGIT7:		.db	$3F,$06,$5B,$4F,$66,$6D,$7D,$07,$7F,$6F,$77,$73,$5C,$00,$79,$71
;				  0   1   2   3   4   5   6   7   8   9   A   P   о       Е   F
;                 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
;***********************************
;инициализация	
TM1637_INIT:
						rcall	start1637			;маркер начала посылки
						ldi		r16,0x8A			;включение дисплея. Яркость минимальная.
						rcall	outByte1637			;вывожу команду
						rcall	end1637				;маркер конца посылки
						rcall	pause1637			;пауза 100 мкс
			;включение режима передачи данных с автоинкрементом адреса
						rcall	start1637			;маркер начала посылки
						ldi		r16,0x40			;режим передачи данных с автоинкрементом адреса
						rcall	outByte1637			;вывожу команду
						rcall	end1637				;маркер конца посылки
						rcall	pause1637			;пауза 100 мкс
						ret
;***********************************
start1637:	;маркер начала посылки
						rcall	pause1637			;пауза 100 мкс
						sbi		IND_PORT,IND_SCL	;
						sbi		IND_PORT,IND_SDA	;
						rcall	pause1637			;пауза 100 мкс
						cbi		IND_PORT,IND_SDA	;	
						rcall	pause1637			;пауза 100 мкс	
						ret
;***********************************
end1637:	;маркер конца посылки
						sbi		IND_PORT,IND_SCL	;
						rcall	pause1637			;пауза 100 мкс
						sbi		IND_PORT,IND_SDA	;
						rcall	pause1637			;пауза 100 мкс
						ret
;***********************************
outByte1637:	;последовательный вывод. выводимый байт должен находиться в r16
						push	r17
	
				; вывод восьми битов регистра r16
						ldi		r17,0x08	;
outByteCycle1637:		cbi	IND_PORT,IND_SCL		;опускаю CLK
						rcall	pause1637			;пауза 100 мкс

						lsr	r16						; \
						brcc	outLowBit1637		;  |
						sbi	IND_PORT,IND_SDA		;   > младший бит r16 выставляю на DIO
						rjmp	outAnyBit1637		;  |
outLowBit1637:			cbi	IND_PORT,IND_SDA		; /

outAnyBit1637:			rcall	pause1637			;пауза 100 мкс
						sbi	IND_PORT,IND_SCL		;поднимаю CLK
						rcall	pause1637			;пауза 100 мкс
						dec 	r17		;
						brne	outByteCycle1637	;последовательно вывожу весь байт

						cbi	IND_PORT,IND_SCL		;опускаю CLK
						rcall	pause1637			;пауза 100 мкс

				; ACK-бит
						cbi	IND_DDR ,IND_SDA		;переключаю SDA как вход, чтобы исключить коллизию с ACK
						sbi IND_PORT,IND_SDA		;с подтяжкой
		//***
						sbi	IND_PORT,IND_SCL		;поднимаю clk
						rcall	pause1637			;пауза 100 мкс

						in r17, IND_PIN				; здесь можно прочитать ACK или неACK
						clr r16
						sbrc r17,IND_SDA
						dec r16						; r16 = 0 - ACK, FF - NACK
						sts SOFT_I2C_ACK,r16

		//***
						rcall	pause1637			;пауза 100 мкс
						cbi	IND_PORT,IND_SCL		;опускаю CLK
						cbi	IND_PORT,IND_SDA		;	
						sbi	IND_DDR ,IND_SDA		;окончание стоп-бита, переключаю SDA как выход
													; (выхожу с нулями на обеих шинах)
						rcall	pause1637			;пауза 100 мкс
						pop	r17
	
						ret			;
		
pause1637:		;пауза 100 мкс
						push	r18		;
						M_DELAY_US_RA 40,r18;
						M_DELAY_US_RA 40,r18;
						//M_DELAY_US_RA 20,r18;
						pop	r18
						ret
;*****************************************************************
