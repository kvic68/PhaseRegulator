	.include "m328Pdef.inc"
//	.include "m168Pdef.inc"
.include "_constants.inc"
.include "_variables.inc"
.include "_macros.asm"
//	----------------------------------------------------------------
.cseg
//	----------------------------------------------------------------
//	Таблица векторов прерываний
//	----------------------------------------------------------------
.org 	0x00		jmp START			; RESTART
.org	INT0addr 	jmp ZEROCROSS		; External Interrupt Rorgest 0		; Для отлова перехода сетевого напряжения через 0
.org	INT1addr 			reti;		; External Interrupt Rorgest 1		
.org	PCI0addr 			reti;		; Pin Change Interrupt Rorgest 0
.org	PCI1addr 			reti;		; Pin Change Interrupt Rorgest 1
.org	PCI2addr 			reti;		; Pin Change Interrupt Rorgest 2
.org	WDTaddr 			reti;		; Watchdog Time-out Interrupt
.org	OC2Aaddr 			reti;		; Timer/Counter2 Compare Match A
.org	OC2Baddr 			reti;		; Timer/Counter2 Compare Match B
.org	OVF2addr 			reti;		; Timer/Counter2 Overflow
.org	ICP1addr 			reti;		; Timer/Counter1 Capture Event
.org	OC1Aaddr 	jmp OPTO_ON;		; Timer/Counter1 Compare Match A	; включить  оптрон управления симистором
.org	OC1Baddr 	jmp OPTO_OFF;		; Timer/Counter1 Compare Match B	; выключить оптрон управления симистором
.org	OVF1addr 	jmp OPTO_OFF_E;		; Timer/Counter1 Overflow			; выключить оптрон управления симистором аварийная ситуация
.org	OC0Aaddr 	jmp T0CompareA;		; TimerCounter0 Compare Match A		; прерывание каждую 1 мс, сетка времени
.org	OC0Baddr 			reti;		; TimerCounter0 Compare Match B
.org	OVF0addr 			reti;		; Timer/Couner0 Overflow
.org	SPIaddr 			reti;		; SPI Serial Transfer Complete
.org	URXCaddr 	jmp RX_OK			; USART Rx Complete
.org	UDREaddr 	jmp UD_OK			; USART, Data Register Empty
.org	UTXCaddr 			reti		; USART Tx Complete
.org	ADCCaddr 	jmp ADC_COMPLETE	; ADC Conversion Complete						; 
.org	ERDYaddr 			reti;		; EEPROM Ready
.org	ACIaddr 			reti;		; Analog Comparator
.org	TWIaddr 	jmp TWI;			; Two-wire Serial Interface
.org	SPMRaddr 			reti;		; Store Program Memory Read
//	----------------------------------------------------------------
//	Точка входа по сбросу
//	----------------------------------------------------------------
START:						cli						;	На всякий случай выключим прерывания ибо хз, что в загрузчике
			;	Выключим watchdog
							wdr
							in r16,(MCUSR)
							andi r16,~(1<<WDRF)
							out (MCUSR),r16
							lds r16,WDTCSR
							ori r16,(1<<WDCE) | (1<<WDE)
							sts WDTCSR,r16
							ldi r16,(0<<WDE)
							sts WDTCSR,r16
			;	Переключим векторы прерываний в начало ПЗУ
							in r16,(MCUCR)
							SETB r16,IVCE
							mov r17,r16
							CLRB r16,IVSEL
							out (MCUCR),r17
							out (MCUCR),r16
			;	Проверим источник сброса
							in r16,(MCUSR)
							sbrc r16,0
							rjmp RAM_Flush			;	Переход по сбросу по включению питания
							rjmp RAM_Flush_Skip		;	Переход по всем остальным источникам сброса
//	----------------------------------------------------------------
			;	Очистка памяти и регистров 
			;	http://easyelectronics.ru/avr-uchebnyj-kurs-vazhnye-melochi-1.html
RAM_Flush:					ldi	ZL,Low (SRAM_START)		; Адрес начала ОЗУ
							ldi	ZH,High(SRAM_START)
							clr	R16						; Очищаем R16
RAM_Flush_Cycle:			st	Z+,R16					; Сохраняем 0 в ячейку памяти
							cpi	ZH,High(RAMEND+1)		; Достигли конца ОЗУ ?
							brne RAM_Flush_Cycle		; Нет? Крутимся дальше!
 
							cpi	ZL,Low(RAMEND+1)		; Младший байт достиг конца?
							brne RAM_Flush_Cycle

							st	-Z,r16
 
							ldi	ZL, 30					; Адрес самого старшего регистра	
							clr	ZH						; Здесь будет 0
RAM_Flush_Cycle2:			dec	ZL						; Уменьшая адрес,
							st	Z, ZH					; записываем в регистр 0,
							brne RAM_Flush_Cycle2		; пока не перебрали всё и успокоились.
//	----------------------------------------------------------------
RAM_Flush_Skip:				clr r16
							out (MCUSR),r16
			;	Настраиваем стек
							LDZ RAMEND
							out spl,ZL
							out sph,ZH
//	----------------------------------------------------------------
//	Настройка оборудования
//	----------------------------------------------------------------
			; порт B
			;	0 - строб  sSCL,
			;	1 - данные sSDA, это подключение индикатора
			;	4 - выбор скорости последовательного порта при старте H-9,L-38
			; 	5 - встроенный светодиод и выход на разгонный блок
							ldi r16,0b00101011
							out DDRB,r16
							ldi r16,0b11111100
							out PORTB,r16
			; ------------------------------------------------------
			; порт С
			;	0 - вход аварийного стопа, 
			;	1 - вход включения разгона
			; 	2 - , 3 - 
			; 	4 - SDA, 5 - SCL
							ldi r16,0b00000000
							out DDRC,r16
							ldi r16,0b00111111
							out PORTC,r16
			; ------------------------------------------------------
			; порт D
			;	0 - RX, 
			;	1 - TX; 
			; 	2 - детектор нуля(INT0)
			;	3,4 - сюда подключен энкодер (PhA,PhB)
			;	5 - кнопка энкодера (Btn);
			; 	6 - MOC3052
			;	7 - индикатор захвата стабилизации (светодиод)
							ldi r16,0b11000000
							out DDRD,r16
							ldi r16,0b11111000
							out PORTD,r16
//	----------------------------------------------------------------
			; Настройка таймера 0
			; прерывание раз в 1 мс
			; CTC Mode, N=64, OCR0A=249, T=(N*(OCR0A+1))/XTAL
			;	OCR0A = (T*XTAL/N)-1
							ldi r16,249
							out OCR0A,r16
							out OCR0B,r16
							ldi r16,(0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00)
							out TCCR0A,r16
							ldi r16,(0<<OCIE0B)|(1<<OCIE0A)|(0<<TOIE0)
							sts TIMSK0,r16
							ldi r16,(0<<WGM02)|(0<<CS02)|(1<<CS01)|(1<<CS00)	; (FCLK:64, CTC-mode)
							out TCCR0B,r16
			; ------------------------------------------------------
			; Настройка таймера 1 
			; режим Normal Mode WGM[13:10] = 0000
			; прескалер FCLK/8, 0,5 мкс/тик, 32,768 мс/цикл CS12..10 = 010
							ldi XH,HIGH(DEFAULT_ON_VALUE)
							ldi XL,LOW (DEFAULT_ON_VALUE)
							sts (OPTO_ON_TICK+0),XL
							sts (OPTO_ON_TICK+1),XH
							sts OCR1AH,XH
							sts OCR1AL,XL

							ldi XH,HIGH(DEFAULT_OFF_VALUE)
							ldi XL,LOW (DEFAULT_OFF_VALUE)
							sts (OPTO_OFF_TICK+0),XL
							sts (OPTO_OFF_TICK+1),XH
							sts OCR1BH,XH
							sts OCR1BL,XL

							ldi r16,(0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10)
							sts TCCR1A,r16

							ldi r16,(0<<ICIE1)|(1<<OCIE1A)|(1<<OCIE1B)|(1<<TOIE1)
							sts TIMSK1,r16

							ldi r16,(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10)	
							sts TCCR1B,r16
			; ------------------------------------------------------
			;	Таймер 2 в этой версии не используется
			; ------------------------------------------------------
//	----------------------------------------------------------------
			; Настройка АЦП 
			; ----------
			; ADMUX – ADC Multiplexer Selection Register
			; REFS1:0 = 00 - внешний ИОН 2.5V 					ADMUX[7:6]
			; ADLAR = 0 - по правому краю						ADMUX[5]
			; MUX2:0 = 0111 - ADC7 								ADMUX[3:0]
			; ----------
			; ADCSRA – ADC Control and Status Register A
			; ----------
			; ADEN: ADC Enable 									[7]			0 - стоп, 1 - работа
			; ADSC: ADC Start Conversion 						[6]			1
			; ADATE: ADC Auto Trigger Enable					[5]			1
			; ADIF: ADC Interrupt Flag							[4]			0
			; ADIE: ADC Interrupt Enable						[3]			1	
			; ADPS[2:0]: ADC Prescaler Select Bits				[2:0]		101 - Fcpu/32 = 500 кГц	
			; ----------
							ldi r16,(0<<ADEN | 1<<ADSC | 1<<ADATE | 0<<ADIF | 1<<ADIE | 1<<ADPS2 | 0<<ADPS1 | 1<<ADPS0)
							sts ADCSRA,r16
							ldi r16,(0<<ACME | 0<<ADTS2 | 0<<ADTS1 | 0<<ADTS0)
							sts ADCSRB,r16
							ldi r16,(0<<REFS1 | 0<<REFS0 | 0<<ADLAR | 0<<MUX3 | 1<<MUX2 | 1<< MUX1 | 1<< MUX0)	;	A7
							sts ADMUX,r16
//	----------------------------------------------------------------
			; Настройка i2c slave
							ldi r16,12
							sts TWBR,r16
							ldi r16,I2C_SlaveAddress
							sts TWAR,r16
							ldi r16,$FF
							sts TWDR,r16
							ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
							sts TWCR,r16
//	----------------------------------------------------------------
			; Настройка UART
							in r16,PINB
							sbrs r16,PORTB4
							rjmp SET_UART_38
							ldi r16, low(bauddivider)
							sts UBRR0L,r16
							ldi r16,high(bauddivider)
							sts UBRR0H,r16
							rjmp SKIP_SET_UART_38
SET_UART_38:				ldi r16, low(bauddivider1)
							sts UBRR0L,r16
							ldi r16,high(bauddivider1)
							sts UBRR0H,r16
			; Прерывания разрешены, прием-передача разрешен.
SKIP_SET_UART_38:			ldi r16,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0)|(0<<UDRIE0)
							sts UCSR0B,r16
			; Формат кадра - 8 бит, один стоп бит, четность не проверяется
							ldi r16,(1<<UCSZ01)|(1<<UCSZ00)
							sts UCSR0C,r16
//	----------------------------------------------------------------
			; Настройка внешнего прерывания 0 на отлов изменения уровней это детектор нуля сетевого напряжения
							ldi r16,0b00000001		; INT0 - любой фронт
							sts EICRA,r16
							ldi r16,0b00000001		; разрешаем INT0
							out EIMSK,r16
//	----------------------------------------------------------------
			;	Здесь можно вставить определение вида индикатора OLED или TM1637
//	----------------------------------------------------------------
							LDZ 50000
							call pause_Z_mks
			;	Выбор адреса OLED индикатора
							ldi r16,IND_ADDR_A
							call checkPresenceSoftI2C
							lds r17,(SoftACK)
							cpi r17,ACK
							breq indicatorAddrSet

							ldi r16,IND_ADDR_B
							call checkPresenceSoftI2C
							lds r17,(SoftACK)
							cpi r17,ACK
							breq indicatorAddrSet

							clr r16							;	индикатор отсутствует, адрес = 0
indicatorAddrSet:			sts (IndicatorAddr),r16

							tst r16
							brne indicatorIsPresent
							rjmp indicatorIsAbcent
indicatorIsPresent:			call INIT_LED
							call LED_CLEAR
							call LED_ON
indicatorIsAbcent:			nop
//	----------------------------------------------------------------
			;	Здесь можно настроить watchdog
//	----------------------------------------------------------------
							sei								;	включаем прерывания
//	----------------------------------------------------------------
							ldi r16,ExtOffMode
							in r17,PINC
							sbrc r17,ExtOFFPin
							ldi r16,StabMode
							call setMode
//	----------------------------------------------------------------
//	Основной цикл
//	----------------------------------------------------------------
Main:				
							lds r16,(ACTIONS_FLAGS)
brunch_A0:					sbrs r16,0
							rjmp brunch_A1
							CLRB r16,0
							sts (ACTIONS_FLAGS),r16
							; ------------------------------------
							; -- bit 0 set Action -- обработать принятую UART строку
							; ------------------------------------
									call stringDecode			;	расшифровываем
									ldi r16,1
									call TXQueueAdd				;	отправляем эхо
							; ------------------------------------
brunch_A1:					lds r16,(ACTIONS_FLAGS)
							sbrs r16,1
							rjmp brunch_A2
							CLRB r16,1
							sts (ACTIONS_FLAGS),r16
							; ------------------------------------
							; -- bit 1 set Action -- обработать принятую TWI строку 
							; ------------------------------------
									call TWIReceived	
							; ------------------------------------
brunch_A2:					; ------------------------------------
brunch_A3:					lds r16,(ACTIONS_FLAGS)
							sbrs r16,3
							rjmp brunch_A4
							CLRB r16,3
							sts (ACTIONS_FLAGS),r16
							; ------------------------------------
							; -- bit 3 set Action -- короткое нажатие
							; ------------------------------------
									call shortKeyPress
							; ------------------------------------
brunch_A4:					lds r16,(ACTIONS_FLAGS)
							sbrs r16,4
							rjmp brunch_A5
							CLRB r16,4
							sts (ACTIONS_FLAGS),r16
							; ------------------------------------
							; -- bit 4 set Action -- длинное удержание
							; ------------------------------------
									call longKeyHold
							; ------------------------------------
brunch_A5:					lds r16,(ACTIONS_FLAGS)
							sbrs r16,5
							rjmp brunch_A6
							CLRB r16,5
							sts (ACTIONS_FLAGS),r16
							; ------------------------------------
							; -- bit 5 set Action -- 
							; ------------------------------------
									call INCREASE_U
							; ------------------------------------
brunch_A6:					lds r16,(ACTIONS_FLAGS)
							sbrs r16,6
							rjmp brunch_A7
							CLRB r16,6
							sts (ACTIONS_FLAGS),r16
							; ------------------------------------
							; -- bit 6 set Action -- 
							; ------------------------------------
									call DECREASE_U
							; ------------------------------------
brunch_A7:					; ------------------------------------
//--------------------------------------------------------
brunch_D0:					lds r16,(TIME_FLAGS)
							sbrs r16,eachSecondFlag
							rjmp brunch_D1
							CLRB r16,eachSecondFlag
							sts (TIME_FLAGS),r16
							; ------------------------------------
							; -- eachSecondFlag -- 1000 ms
							; ------------------------------------
									ldi r16,1				;	подготовка к отправке текущего состояния в последовательный порт
									call TXQueueAdd
							; ------------------------------------
brunch_D1:					lds r16,(TIME_FLAGS)
							sbrs r16,eachdeciSecondFlag
							rjmp brunch_D2
							CLRB r16,eachdeciSecondFlag
							sts (TIME_FLAGS),r16
							; ------------------------------------
							; -- eachdeciSecondFlag -- 100 ms
							; ------------------------------------
									call TXmanager
									call EXT_SIGNALS		;	опрашиваем внешние сигналы (авария, разгон)
							; ------------------------------------
brunch_D2:					lds r16,(TIME_FLAGS)
							sbrs r16,eachcentiSecondFlag
							rjmp brunch_D3
							CLRB r16,eachcentiSecondFlag
							sts (TIME_FLAGS),r16
							; ------------------------------------
							; -- eachcentiSecondFlag -- 10 ms
							; ------------------------------------
									call Button				;	опрос кнопки энкодера
							; ------------------------------------
brunch_D3:					lds r16,(TIME_FLAGS)
							sbrs r16,eachmilliSecondFlag
							rjmp brunch_D4
							CLRB r16,eachmilliSecondFlag
							sts (TIME_FLAGS),r16
							; ------------------------------------
							; -- eachmilliSecondFlag --  1 ms
							; ------------------------------------
									call encoder				;	опрос энкодера
brunch_D4:					; ------------------------------------
brunch_D5:					; ------------------------------------
brunch_D6:					lds r16,(TIME_FLAGS)
							sbrs r16,eachYdeciSecondFlag
							rjmp brunch_D7
							CLRB r16,eachYdeciSecondFlag
							sts (TIME_FLAGS),r16
							; ------------------------------------
							; -- eachYdeciSecondFlag --  200 ms
							; ------------------------------------
									call newValueScan
brunch_D7:					; ------------------------------------
							rjmp MAIN
//	----------------------------------------------------------------
//	Конец основного цикла
//	----------------------------------------------------------------
.include "encoder.asm"
.include "math.asm"
.include "measuring.asm"
.include "print.asm"
.include "printextras.asm"
.include "serial.asm"
.include "softI2C.asm"
.include "twi.asm"
.include "unsorted.asm"
.include "fonts.inc"
.include "strings.inc"

