.equ	TimeDelayIIC	= 1
//	----------------------------------------------------------------
.macro	scl_One
							cbi IND_DDR,IND_CLK			;	вход
							sbi IND_PORT,IND_CLK		;	с подтяжкой
.endmacro
//	----------------------------------------------------------------
.macro	scl_Null
							cbi IND_PORT,IND_CLK		;	с нулем на выходе
							sbi IND_DDR,IND_CLK			;	выход
.endmacro
//	----------------------------------------------------------------
.macro	sda_One
							cbi IND_DDR,IND_DIO			;	вход
							sbi IND_PORT,IND_DIO		;	с подтяжкой
.endmacro
//	----------------------------------------------------------------
.macro	sda_Null
							cbi IND_PORT,IND_DIO		;	с нулем на выходе
							sbi IND_DDR,IND_DIO			;	выход
.endmacro
//	----------------------------------------------------------------
softI2CInit:				push r18
							sda_One
							scl_One
							call softI2CStop
							pop r18
							ret
//	----------------------------------------------------------------
softI2CStart:				push r18
							sda_Null						; SDA->0
							M_DELAY_US_RA TimeDelayIIC,r18
							scl_Null						; SCL->0
							M_DELAY_US_RA TimeDelayIIC,r18
							ldi r18,$FF						;	(NACK)
							sts (SoftACK),r18
							pop r18
							ret
//	----------------------------------------------------------------
//softI2CRestart:				push r18
//							sda_One
//							M_DELAY_US_RA TimeDelayIIC,r18
//							scl_One
//							M_DELAY_US_RA TimeDelayIIC,r18
//							sda_Null
//							M_DELAY_US_RA TimeDelayIIC,r18
//							scl_Null
//							M_DELAY_US_RA TimeDelayIIC,r18
//							pop r18
//							ret
//	----------------------------------------------------------------
softI2CStop:				push r18
							scl_Null						; SCL->0
							M_DELAY_US_RA TimeDelayIIC,r18
							sda_Null						; SDA->0
							M_DELAY_US_RA TimeDelayIIC,r18
							scl_One							; SCL->1
							M_DELAY_US_RA TimeDelayIIC,r18
							sda_One							; SDA->1
							M_DELAY_US_RA TimeDelayIIC,r18
							pop r18
							ret
//	----------------------------------------------------------------
//	отправляемый байт в r16
//	на выходе в (SoftACK) - ACK(0)/NACK(FF)
//
softI2CSendByte:			PUSHF
							push r16
							push r17
							push r18
							ldi r17,8

softI2CSendByteCycle:		sbrc r16,7
							sda_One
							sbrs r16,7
							sda_Null
							rol r16

							M_DELAY_US_RA TimeDelayIIC,r18
							scl_One								; SCL->1
							M_DELAY_US_RA TimeDelayIIC,r18
							scl_Null							; SCL->0
							dec r17
							brne softI2CSendByteCycle

						//ACK bit
							sda_One								; SDA->1
							M_DELAY_US_RA TimeDelayIIC,r18
							scl_One								; SCL->1
							M_DELAY_US_RA TimeDelayIIC,r18

							in   r17,IND_PIN
							clr  r16
							sbrc r17,IND_DIO
							dec  r16							;	R16 = 0 (ACK), = FF (NACK)
							scl_Null							;	SCL->0
							sts (SoftACK),r16
							pop r18
							pop r17
							pop r16
							POPF
							ret
//	----------------------------------------------------------------
//	Проверка присутствия устройства на шине softI2C с адресом в r16
//	ACK - есть устройство, NACK - нет устройства
//	----------------------------------------------------------------
checkPresenceSoftI2C:		push r16
							call softI2CInit
							call softI2CStart
							call softI2CSendByte
							call softI2CStop
							pop r16
							ret
//	----------------------------------------------------------------
//	Блок подпрограмм управления и настройки OLED дисплея
//	----------------------------------------------------------------
.equ	cmdsingle	= 0b10000000
.equ	cmdstream	= 0b00000000
.equ	datasingle	= 0b11000000
.equ	datastream	= 0b01000000
//	----------------------------------------------------------------
LED_ON:						call softI2CStart
							lds r16,IndicatorAddr
							call softI2CSendByte
							ldi r16,cmdsingle
							call softI2CSendByte
							ldi r16,$AF
							call softI2CSendByte
							call softI2CStop
							ret
//	----------------------------------------------------------------
INIT_LED:					push r16
							push r17
							PUSHZ
							call softI2CStart
							lds r16,IndicatorAddr
							call softI2CSendByte
INIT_LED_CONT:				ldi r16,cmdstream	; режим потока команд
							call softI2CSendByte
				// передача команд
							ldi r17,(LED_INIT_STREAM_END*2-LED_INIT_STREAM*2)
							LDZ LED_INIT_STREAM*2
INIT_LED_CYCLE:				lpm r16,Z+
							call softI2CSendByte
							dec r17
							tst r17
							brne INIT_LED_CYCLE
							call softI2CStop
				//
							POPZ
							pop r17
							pop r16
							ret
//	----------------------------------------------------------------
LED_INIT_STREAM:			.db $D5,$80,$A8,$3F,$D3,$00,$40,$8D,$14,$A1,\
								$C8,$DA,$12,$81,$FF,$D9,$22,$DB,$30,$A4,$A6,$00,$10,$B0,$AF,$00
//							
LED_INIT_STREAM_END:		.db $AF,$00		; AFh; Display ON

//	----------------------------------------------------------------
LED_CLEAR:					push r16
							push r17
							push r18

							ldi r18,8
LED_CLR_OuterLoop:			ldi r17,132
							dec r18
							push r17
							ldi r17,0
							sts (POSX),r17
							sts (POSY),r18
							call SET_LED_XY
							pop r17
						//
							call softI2CStart
							lds r16,IndicatorAddr
							call softI2CSendByte
							ldi r16,datastream
							call softI2CSendByte
						//
LED_CLR_InnerLoop:			ldi r16,$00
							call softI2CSendByte
							dec r17
							brne LED_CLR_InnerLoop
							call softI2CStop
				//
							tst r18
							brne LED_CLR_OuterLoop

				
							pop r18
							pop r17
							pop r16	
							ret
//	----------------------------------------------------------------
// (POSX)->r17,(POSY)->r18
SET_LED_XY:					push r16
							push r17
							push r18

							lds r17,(POSX)
							lds r18,(POSY)
							sbrc r17,7
							rjmp SET_LED_XY_EXIT

							call softI2CStart
							lds r16,IndicatorAddr
							call softI2CSendByte
							ldi r16,cmdstream
							call softI2CSendByte
							//	set Y
							mov r16,r18
							andi r16,$07
							ori r16,$B0
							call softI2CSendByte
							//	set X
							mov r16,r17
							andi r16,$0F
							call softI2CSendByte
							mov r16,r17
							swap r16
							andi r16,$0F
							ori r16,$10
							call softI2CSendByte

							call softI2CStop

SET_LED_XY_EXIT:			pop r18
							pop r17				
							pop r16
					
							ret
//	----------------------------------------------------------------
