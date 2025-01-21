.list
.include "macro.asm"
//.include "m328Pdef.inc"
.include "m168Pdef.inc"
; ----------------------------------------------------------------------------------------
.equ		I2C_MasterAddress	= $32			; адрес регулятора на шине i2c
.equ		DISPLAY_TYPE		= 2; 0 - TM1637, 1 - LCD 84x48 (Nokia3110), 2 - LED 128x64 (sh1106)

.equ		XTAL				= 16000000
.equ 		baudrate 			= 9600
.equ 		bauddivider 		= (XTAL/(16*baudrate))-1
.equ		MOC_PORT			= PORTD			; Порт подключения MOC
.equ		MOC_PIN				= 6				; Нога подключения MOC


.equ		IND_PORT				= PORTB			; Порт подключения индикатора на TM1637
.equ		IND_DDR				= DDRB			;
.equ		IND_SCL				= 0				; TM1637 синхро SCL
.equ		IND_SDA				= 1				; TM1637 данные SDA
.equ		IND_PIN				= PINB
/*
.equ		LCD_PORT			= PORTB
.equ		LCD_SCLK			= 5
.equ		LCD_SDIN			= 3
.equ		LCD_DC				= 0
.equ		LCD_SCE				= 2
.equ		LCD_RES				= 1
*/
.equ		CU_HISTORY_LENGTH	= 15			; Длина истории значений CURRENT_U
.equ		DEFAULT_ON_VALUE 	= 19000			; момент включения оптрона по умолчанию. отсчитывается от перехода через ноль сетевого напряжения
.equ		DEFAULT_OFF_VALUE 	= 19000			; момент отключения оптрона
.equ		DEFAULT_TARGET_U	= 1250			; Значение выходного напряжения при старте с зажатой кнопкой энкодера 
.equ		DEFAULT_STEP_U		= 5				; Шаг регулировки напряжения
.equ		MIN_U				= 400			; Минимальное напряжение на выходе
.equ		MAX_U				= 2500			; Минимальное напряжение на выходе
; ----------------------------------------------------------------------------------------
.equ	btnPort			= PORTD
.equ	btnPin			= PIND
.equ	btnPinNo		= 5						; нога порта, куда подключена кнопка энкодера
.equ	encPhaseA		= 3
.equ	encPhaseB		= 4
; ----------------------------------------------------------------------------------------
.equ  BUTTON_HOLDING_TIME  	= 0  ; Пять битов   DButtonStatus[4:0] = счётчик количества полусекунд, в течение которых Кнопка удержива(лась/ется) "нажатой".  (фиксирует время до 16сек!)
.equ  BUTTON_STATUS_CODE  	= 5  ; В трёх битах DButtonStatus[7:5] = кодируется итоговый "статус-код кнопки" (см. ниже макроопределения констант).
.equ  BUTTON_HOLDEN_LONG  	= 5  ; Флаг "времени удержания" кнопки: 0-короткое или 1-длинное.
.equ  BUTTON_IS_PRESSED  	= 6  ; Флаг "зафиксировано полноценное нажатие кнопки": "0" - кнопка не нажималась, "1" - было нажатие.
.equ  BUTTON_IS_HOLDDOWN  	= 7  ; Флаг "кнопка удерживается в нажатом состоянии": "0" - сейчас кнопка "отпущена", "1" - сейчас кнопка "нажата и удерживается".

.equ  BSC_NotPressed  		= 0b000  ; "не нажата"    (исходное положение для всех кнопок - бывает только после "сброса")
.equ  BSC_ShortHold  		= 0b100  ; "короткое удержание"  (кнопка нажата, и всё ещё удерживается, пока "короткое" время)
.equ  BSC_LongHold  		= 0b101  ; "длинное удержание"  (кнопка нажата, и всё ещё удерживается, уже "длительное" время)
.equ  BSC_ShortPress  		= 0b010  ; "короткое нажатие"  (кнопка была нажата, и затем отпущена, а время её удержания было "незначительным")
.equ  BSC_LongPress  		= 0b011  ; "длинное нажатие"  (кнопка была нажата, и затем отпущена, а время её удержания было "длительным")

.equ  CShortButtonTouchDuration = 10  	; длительность длинного нажатия в х100 мс
.equ  CButtonInputChannelCount = 1		; количество кнопок

; ----------------------------------------------------------------------------------------


.dseg

DInputIntegrator:			.BYTE 1
DButtonStatus:				.BYTE 1

OPTO_ON_TICK	:			.BYTE 2				; Значение момента включения оптрона записывается в OCR1A
OPTO_OFF_TICK	:			.BYTE 2
COUNT_NEG		:			.BYTE 2
COUNT_POS		:			.BYTE 2
CORRECTION		:			.BYTE 2

REG_AB			:			.BYTE 1				; Счетчик энкодера

STEP_U			:			.BYTE 2				; Шаг изменения напряжения 1 ед = 100 мВ
FL_DATAOUT		:			.BYTE 1				; Флаг включения вывода информации в последовательный порт
FL_LOCK			:			.BYTE 1				; Флаг разрешения вывода на индикатор
FL_NEW_U		:			.BYTE 1				; Флаг нового значения U
FL_NEW_T		:			.BYTE 1				; Флаг нового значения целевого напряжения
FL_NEW_M		:			.BYTE 1				; Флаг нового значения режима работы
FL_MAX_U		:			.BYTE 1				; Флаг максимального значения U
FL_INRANGE		:			.BYTE 1				; Флаг нахождения напряжения в диапазоне
MODE			:			.BYTE 1				; Режим работы 0 - нормальный, 1 - разгон(максимум), 2 - стоп(выключено), 3 - внешний стоп

ADC_CURRENT_SUM_QUAD	:	.BYTE 4						; Сумма квадратов отсчетов АЦП в текущем периоде
ADC_CURRENT_VALUE_COUNT	:	.BYTE 2						; Количество преобразований АЦП в текущем периоде
CURRENT_QUAD_U			:	.BYTE 4						; Квадрат напряжения

AVERAGE_U				:	.BYTE 2						; Текущее значение усредненного URMS
TARGET_U				:	.BYTE 2						; Задаваемое напряжение
CURRENT_U				:	.BYTE 2						; Текущее значение URMS
CURRENT_U_HISTORY		:	.BYTE 2*CU_HISTORY_LENGTH


AVERAGE_U_OLD			:	.BYTE 2						; Старое значение усредненного URMS
T0EXTENDER				:	.BYTE 3						; Расширитель таймера 0, +0 - сотые, +1 - десятые, +2 - секунды
EXP100MS				:	.BYTE 1						; Счетчик 100 мс 
TM_BUFFER				:	.BYTE 2						; Данные для вывода на индикатор
ACTIONS_FLAGS			:	.BYTE 1		; Флаги для вызова определенных действий
										; 0 - обработка принятого по UART 
										; 1 - обработка принятого по iic
										; 2 -
										; 3 - короткое нажатие кнопки
										; 4 - длительное удержание кнопки
										; 5 - энкодер в одну   сторону +
										; 6 - энкодер в другую сторону -
										; 7 - заброс данных в уарт
DATA_FLAGS				:	.BYTE 1		; 
										; 0 -   10 мс
										; 1 -  100 мс
										; 2 - 1000 мс
										; 3 -
										; 4 -
										; 5 -
										; 6 -
										; 7 -
RX_BUFFER				:	.BYTE 32	; буфер приема
TX_BUFFER				:	.BYTE 32	; буфер передачи
RX_PTR					:	.BYTE 1		; указатель буфера приема 
TX_PTR					:	.BYTE 1		; указатель буфера передачи
LCD_X					:	.BYTE 1
LCD_Y					:	.BYTE 1
LED_ADDRESS				:	.BYTE 1
SOFT_I2C_ACK:				.BYTE 1
TONECOUNTER:				.BYTE 1
SOUNDNUMBER:				.BYTE 1
COUNTER_A				:	.BYTE 1		; таймер повторного срабатывания стопа по внешнему сигналу
COUNTER_B				:	.BYTE 1		; таймер задержки срабатывания индикации выхода из возможности стабилизации
ROTATE_COUNTER:				.BYTE 1

; ----------------------------------------------------------------------------------------

.cseg 
; ***** INTERRUPT VECTORS ************************************************
.org 	0x00		JMP START	; 	RESTART
.org	INT0addr 	JMP ZEROCROSS;	External Interrupt Rorgest 0		; Для отлова перехода сетевого напряжения через 0
.org	INT1addr 	RETI;		; External Interrupt Rorgest 1		
.org	PCI0addr 	RETI;		; Pin Change Interrupt Rorgest 0
.org	PCI1addr 	RETI;jmp PCI_ONE;		; Pin Change Interrupt Rorgest 1
.org	PCI2addr 	RETI;		; Pin Change Interrupt Rorgest 2
.org	WDTaddr 	RETI;		; Watchdog Time-out Interrupt
.org	OC2Aaddr 	RETI;		; Timer/Counter2 Compare Match A
.org	OC2Baddr 	RETI;		; Timer/Counter2 Compare Match B
.org	OVF2addr 	RETI;		; Timer/Counter2 Overflow
.org	ICP1addr 	RETI;		; Timer/Counter1 Capture Event
.org	OC1Aaddr 	jmp OPTO_ON;		; Timer/Counter1 Compare Match A	; включить  оптрон управления симистором
.org	OC1Baddr 	jmp OPTO_OFF;		; Timer/Counter1 Compare Match B	; выключить оптрон управления симистором
.org	OVF1addr 	jmp OPTO_OFF_E;		; Timer/Counter1 Overflow			; выключить оптрон управления симистором аварийная ситуация
.org	OC0Aaddr 	jmp T0COMP;			; TimerCounter0 Compare Match A
.org	OC0Baddr 	RETI;		; TimerCounter0 Compare Match B
.org	OVF0addr 	RETI;		; Timer/Couner0 Overflow
.org	SPIaddr 	RETI;		; SPI Serial Transfer Complete
.org	URXCaddr 	jmp RX_OK	;reti;			; USART Rx Complete
.org	UDREaddr 	jmp UD_OK	;reti;			; USART, Data Register Empty
.org	UTXCaddr 	jmp TX_OK	;reti;			; USART Tx Complete
.org	ADCCaddr 	jmp ADC_COMPLETE	; ADC Conversion Complete						; 
.org	ERDYaddr 	RETI;		; EEPROM Ready
.org	ACIaddr 	RETI;		; Analog Comparator
.org	TWIaddr 	jmp TWI;RETI;		; Two-wire Serial Interface
.org	SPMRaddr 	RETI;		; Store Program Memory Read

; ************************************************************************
START:
; ----------------------------------------------------------------------------------------
; переключаем векторы прерываний в нормальный вид
		in r16,(MCUCR)
		SETB r16,IVCE
		mov r17,r16
		CLRB r16,IVSEL
		out (MCUCR),r17
		out (MCUCR),r16
; ----------------------------------------------------------------------------------------

			CLEAR_ALL
; ----------------------------------------------------------------------------------------
			; порт С
			; 0 - вход аварийного стопа, 1 - вход включения разгона
			; 2 - , 3 - 
			; 4 - SDA, 5 - SCL
			ldi r16,0b00000000
			out DDRC,r16
			ldi r16,0b00111111
			out PORTC,r16
			; ----------------------------------------------------------------------------------------
			; порт D
			; 0 - RX, 1 - TX;
			; 2 - детектор нуля; 3,4 - сюда подключен энкодер; 5 - кнопка энкодера;
			; 6 - MOC3052; 7 - индикатор захвата стабилизации
			ldi r16,0b11000010
			out DDRD,r16
			ldi r16,0b11111010
			out PORTD,r16
			; ----------------------------------------------------------------------------------------
			; порт B
			; 0 - строб,1 - данные, это подключение индикатора на TM1637
			; 3 - пищалка OC2
			; 5 - встроенный светодиод и выход на разгонный блок
			ldi r16,0b00101011
			out DDRB,r16
			ldi r16,0b11111111
			out PORTB,r16
; ----------------------------------------------------------------------------------------
			; Настройка прерываний от изменения уровне на линиях порта C (PCINT8..15) PCMSK1, PCIE1
			ldi r16,0b00000011
			sts PCMSK1,r16
			ldi r16,0b00000010
			sts PCICR,r16

; ----------------------------------------------------------------------------------------
			; Настройка i2c slave
			ldi r16,I2C_MasterAddress
			sts TWAR,r16
			ldi r16,0<<TWSTA|0<<TWSTO|1<<TWINT|1<<TWEA|1<<TWEN|1<<TWIE
			sts TWCR,r16
; ----------------------------------------------------------------------------------------
			; Настройка UART
			ldi r16, low(bauddivider)
			sts UBRR0L,r16
			ldi r16,high(bauddivider)
			sts UBRR0H,r16
			; Прерывания разрешены, прием-передача разрешен.
			ldi r16,(1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<TXCIE0)|(0<<UDRIE0)
			sts UCSR0B,r16
			; Формат кадра - 8 бит, один стоп бит, четность не проверяется
			ldi r16,(1<<UCSZ01)|(1<<UCSZ00)
			sts UCSR0C,r16
; ----------------------------------------------------------------------------------------
			; ----------------------------------------------------------------------------------------
			; Настройка таймера 2
			; для извлечения звуков на OC2A
.equ		SoundOn  = 0b01000010
.equ		SoundOff = 0b00000010
			; CTC, 00000010 - normal port op 0100xx10 - toggle OC2A, 0001xx10 - toggle OC2B
			ldi r16,SoundOff		
			sts TCCR2A,r16
			ldi r16,0b00000011		; clk/32;	00000yyy: 000 - no clock, 001 - Fclk, 010 - Fclk/8, 011 - Fclk/32, 
									; 100 - Fclk/64, 101 - Fclk/128, 110 - Fclk/256, 111 - Fclk/1024
			sts TCCR2B,r16
			ldi r16,124	; 2 kHz
			sts OCR2A,r16
			ldi r16,(0<<OCIE2B)|(0<<OCIE2A)|(0<<TOIE2) 
			sts TIMSK2,r16
			; ----------------------------------------------------------------------------------------
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
			ldi r16,(0<<ICIE1)|(1<<OCIE1A)|(1<<OCIE1B)|(0<<TOIE1)
			sts TIMSK1,r16

			ldi r16,(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10)	
			sts TCCR1B,r16
			; ----------------------------------------------------------------------------------------
			; Настройка таймера 0
			; прерывание раз в 1 мс
			; CTC Mode, N=64, OCR0A=249, T=(N*(OCR0A+1))/XTAL
			ldi r16,249
			out OCR0A,r16
			out OCR0B,r16
			ldi r16,(0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00)
			out TCCR0A,r16
			ldi r16,(0<<OCIE0B)|(1<<OCIE0A)|(1<<TOIE0)
			sts TIMSK0,r16
			ldi r16,(0<<WGM02)|(0<<CS02)|(1<<CS01)|(1<<CS00)	; (FCLK:64, CTC-mode)
			out TCCR0B,r16
				ldi r16,9							; 10 мсек
				sts (T0EXTENDER+0),r16
				ldi r16,9
				sts (T0EXTENDER+1),r16				; 10*10 мсек
				ldi r16,9
				sts (T0EXTENDER+2),r16				; 10*100 мсек
			; ----------------------------------------------------------------------------------------
			; Настройка внешнего прерывания 0 на отлов изменения уровней это детектор нуля сетевого напряжения
			ldi r16,0b00001010		; INT1 - спадающий фронт, INT0 - спадающий фронт
			sts EICRA,r16
			ldi r16,0b00000001		; разрешаем INT0
			out EIMSK,r16
			; ----------------------------------------------------------------------------------------
			; Настройка АЦП 
			; ----------
			; ADMUX – ADC Multiplexer Selection Register
			; REFS1:0 = 10 - внешний ИОН 2.5V 					ADMUX[7:6]
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
			.equ		ADC_ON				= 0b11101101	; Заклинание включения  АЦП
			.equ		ADC_OFF				= 0b01101101	; Заклинание выключения АЦП
			; ----------
			ldi r16,$00
			sts ADCSRB,r16
			ldi r16,(0<<REFS1 | 0<<REFS0 | 0<<ADLAR | 0<<MUX3 | 1<<MUX2 | 1<< MUX1 | 1<< MUX0)
			sts ADMUX,r16
			ldi r16,ADC_ON
			sts ADCSRA,r16
			; ----------------------------------------------------------------------------------------


			; выбор типа дисплея
			call TM1637_INIT
			tst r16
			brne checkOLEDaddr7A
			ldi r16,$FF
			sts LED_ADDRESS,r16
			rjmp endAddrA

				
			; ----------------------------------------------------------------------------------------
			; выбор адреса OLED дисплея
checkOLEDaddr7A:	call startI2Csoft
					ldi r16,$7A
					call outbyteI2Csoft
					call stopI2Csoft
					tst r16
				brne checkOLEDaddr78
					ldi r16,$7A
					sts LED_ADDRESS,r16
				rjmp endAddr

checkOLEDaddr78:	call startI2Csoft
					ldi r16,$78
					call outbyteI2Csoft
					call stopI2Csoft
					tst r16
				brne endAddr
					ldi r16,$78
					sts LED_ADDRESS,r16
				rjmp endAddr
endAddr:
			; ----------------------------------------------------------------------------------------
			call INIT_LED
			call LED_CLEAR
			call LED_ON
endAddrA:	ldi r16,$FF
			sts (FL_NEW_M),r16
			ldi r16,$80
			sts (FL_INRANGE),r16
			; ----------------------------------------------------------------------------------------
			in r16,btnPin
			sbrc r16,btnPinNo 				; Если кнопка энкодера не нажата
			jmp START_U_0					; то стартуем с выходного напряжения = 0.
			ldi XH,HIGH(DEFAULT_TARGET_U)	; Иначе
			ldi XL,LOW (DEFAULT_TARGET_U)	; с напряжения по умолчанию.
			jmp SET_START_U
START_U_0:	clr XL
			clr XH
SET_START_U:
			sts (TARGET_U+0),XL
			sts (TARGET_U+1),XH
			; 
			ldi XH,HIGH(DEFAULT_STEP_U)
			ldi XL,LOW (DEFAULT_STEP_U)
			sts (STEP_U+0),XL
			sts (STEP_U+1),XH

			sei

			ldi r16,$FF
			sts (FL_NEW_U),r16
			call KEY_RESET_STATUS_FOR_ALL_BUTTONS
			jmp MAIN
; ----------------------------------------------------------------------------------------
MAIN:			lds r16,(ACTIONS_FLAGS)
brunch_A0:	sbrs r16,0
				rjmp brunch_A1
				CLRB r16,0
				sts (ACTIONS_FLAGS),r16
				push r16
				; -- bit 0 set Action -- обработать принятую UART строку
				call stringReceived
				pop r16
brunch_A1:	sbrs r16,1
				rjmp brunch_A2
				CLRB r16,1
				sts (ACTIONS_FLAGS),r16
				push r16
				; -- bit 1 set Action -- обработать принятую TWI строку 
					call TWIReceived	
				pop r16
brunch_A2:	sbrs r16,2
				rjmp brunch_A3
				CLRB r16,2
				sts (ACTIONS_FLAGS),r16
				push r16
				; -- bit 2 set Action -- 
					
				pop r16
brunch_A3:	sbrs r16,3
				rjmp brunch_A4
				CLRB r16,3
				sts (ACTIONS_FLAGS),r16
				push r16
				; -- bit 3 set Action -- короткое нажатие
				call KEY_RESET_STATUS_FOR_ALL_BUTTONS
				call shortKeyPress
				pop r16
brunch_A4:	sbrs r16,4
				rjmp brunch_A5
				CLRB r16,4
				sts (ACTIONS_FLAGS),r16
				push r16
				; -- bit 4 set Action -- длинное удержание
				call KEY_RESET_STATUS_FOR_ALL_BUTTONS
				call longKeyHold
				pop r16
brunch_A5:	sbrs r16,5
				rjmp brunch_A6
				CLRB r16,5
				sts (ACTIONS_FLAGS),r16
				push r16
				; -- bit 5 set Action -- вращение вправо +
				//----------------------------
					lds r16,(ROTATE_COUNTER)
					cpi r16,0
					breq brunch_A5slow
						PUSHY
							LDY 50
							STSY STEP_U
						POPY
					rjmp brunch_A5fast
brunch_A5slow:		PUSHY
						LDY DEFAULT_STEP_U
						STSY STEP_U
					POPY
brunch_A5fast:		ldi r16,2
					sts (ROTATE_COUNTER),r16
				//-----------------------------
					call INCREASE_U
				pop r16
brunch_A6:	sbrs r16,6
				rjmp brunch_A7
				CLRB r16,6
				sts (ACTIONS_FLAGS),r16
				push r16
				; -- bit 6 set Action -- вращение влево -
					//----------------------
					lds r16,(ROTATE_COUNTER)
					cpi r16,0
					breq brunch_A6slow
						PUSHY
							LDY 50
							STSY STEP_U
						POPY
					rjmp brunch_A6fast
brunch_A6slow:		PUSHY
						LDY DEFAULT_STEP_U
						STSY STEP_U
					POPY
brunch_A6fast:		ldi r16,2
					sts (ROTATE_COUNTER),r16
					//----------------------
					call DECREASE_U
				pop r16
brunch_A7:	sbrs r16,7
				rjmp brunch_D0
				CLRB r16,7
				sts (ACTIONS_FLAGS),r16
				push r16
				; -- bit 7 set Action -- 
				pop r16
brunch_D0:	lds r16,(DATA_FLAGS)
			sbrs r16,0
				rjmp brunch_D1
				CLRB r16,0
				sts (DATA_FLAGS),r16
				push r16
				; -- bit 0 set Action -- 10 ms

					lds r16,(ROTATE_COUNTER)
					cpi r16,0
					breq brunch_D0B
					cpi r16,2
					brcs brunch_D0A
					ldi r16,2
brunch_D0A:			dec r16
brunch_D0B:			sts (ROTATE_COUNTER),r16

				pop r16
brunch_D1:	sbrs r16,1
				rjmp brunch_D2
				CLRB r16,1
				sts (DATA_FLAGS),r16
				push r16
				; -- bit 1 set Action -- 100 ms
					call sub100mSec
				pop r16
brunch_D2:	sbrs r16,2
				rjmp brunch_D3
				CLRB r16,2
				sts (DATA_FLAGS),r16
				push r16
				; -- bit 2 set Action -- 1000 ms
				pop r16
brunch_D3:	sbrs r16,3
				rjmp brunch_D4
				CLRB r16,3
				sts (DATA_FLAGS),r16
				push r16
				; -- bit 3 set Action -- 
				pop r16
brunch_D4:	sbrs r16,4
				rjmp brunch_D5
				CLRB r16,4
				sts (DATA_FLAGS),r16
				push r16
				; -- bit 4 set Action -- 
				pop r16
brunch_D5:	sbrs r16,5
				rjmp brunch_D6
				CLRB r16,5
				sts (DATA_FLAGS),r16
				push r16
				; -- bit 5 set Action -- 
				pop r16
brunch_D6:	sbrs r16,6
				rjmp brunch_D7
				CLRB r16,6
				sts (DATA_FLAGS),r16
				push r16
				; -- bit 6 set Action -- 
				pop r16
brunch_D7:	sbrs r16,7
				rjmp brunch_D8
				CLRB r16,7
				sts (DATA_FLAGS),r16
				push r16
				; -- bit 1 set Action -- 
				pop r16
brunch_D8:
jmp MAIN
; ----------------------------------------------------------------------------------------
			; ----------
; --------------------------------------------------------------------------------
; --------------------------------------------------------------------------------
sub100mSec:		PUSHF
				PUSHY
				PUSHZ
				push r16
				push r17
				push r18

				; ----------

				call KEY_ENHANCE_TIME_FOR_ALL_BUTTONS

				lds r16,(EXP100MS)
				cpi r16,2
				breq this100ms
				rjmp next100ms

this100ms:		clr r16
				push r16

lds r16,(TARGET_U+0)
sts (i2c_OutBuff+0),r16
lds r16,(TARGET_U+1)
sts (i2c_OutBuff+1),r16
lds r16,(AVERAGE_U+0)
sts (i2c_OutBuff+2),r16
lds r16,(AVERAGE_U+1)
sts (i2c_OutBuff+3),r16
lds r16,(MODE)
sts (i2c_OutBuff+4),r16

call PrepareDataToSend
				call indication		; выбираем что на что и как выводить				

				pop  r16
next100ms:		inc r16
				sts (EXP100MS),r16
				; ----------
				call top_rail
				pop r18
				pop r17
				pop r16
				POPZ
				POPY
				POPF
				ret		
; --------------------------------------------------------------------------------
INCREASE_U:		PUSHFand r16
				push r17
				push r18
				push r19

				lds r16,(MODE)
				lds r17,(LED_ADDRESS)
				cpi r17,$FF
				brne INCREASE_A
				cpi r16,0
				breq INCREASE_RUN
				rjmp INCREASE_EX
INCREASE_A:		cpi r16,3
				brne INCREASE_RUN
				rjmp INCREASE_EX
				; ----------
INCREASE_RUN:	ldi r16,$FF
				sts (FL_NEW_U),r16
				lds r16,(FL_MAX_U)		; Если выходное напряжение на максимуме,
				cpi r16,$FF				; то ничего не делаем.
				brne INCREASE_NORM		; Иначе приступаем к регулировке.
				rjmp INCREASE_EX
				; ----------
INCREASE_NORM:	lds r16,(TARGET_U+0)
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
				brcc INCREASE_MAX		; Если следующее значение больше или равно минимальному, то устанавливаем его.
				movw r16,r18			; Иначе следующее значение устанавливаем = MIN_U
				rjmp INCREASE_SET

INCREASE_MAX:	ldi r18, low(MAX_U)
				ldi r19,high(MAX_U)
				cp  r16,r18
				cpc r17,r19
				brcs INCREASE_SET
				breq INCREASE_SET
				movw r16,r18

INCREASE_SET:	sts (TARGET_U+0),r16
				sts (TARGET_U+1),r17
			
				; ----------
INCREASE_EX:	pop r19
				pop r18
				pop r17
				POPFand r16
			ret
				; ----------
; --------------------------------------------------------------------------------
				; ----------
; --------------------------------------------------------------------------------
DECREASE_U:		PUSHF
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
				lds r17,(LED_ADDRESS)
				cpi r17,$FF
				brne DECREASE_A
				cpi r16,0
				breq DECREASE_RUN
				rjmp DECREASE_EX
DECREASE_A:		cpi r16,3
				brne DECREASE_RUN
				rjmp DECREASE_EX

;				lds r16,(MODE)
;				cpi r16,3
;				brne DECREASE_RUN
;				rjmp DECREASE_EX
				; ----------
DECREASE_RUN:	ldi r16,$FF
				sts (FL_NEW_U),r16
				lds ZL,(FL_MAX_U)
				cpi ZL,$FF
				brne DECREASE_NORM
				; ----------
				lds r8,(STEP_U+0)
				lds r9,(STEP_U+1)
				add r8,r8		adc r9,r9
				add r8,r8		adc r9,r9
				add r8,r8		adc r9,r9
				add r8,r8		adc r9,r9		; Y=STEP*16
				lds r6,(CURRENT_U+0)			; 
				lds r7,(CURRENT_U+1)			; 
				sub r6,r8
				sbc r7,r9						; [R7:6]=CURRENT_U-Y
				clr r8
				clr r9
				lds r10,(STEP_U+0)
				lds r11,(STEP_U+1)
				clr r12
				clr r13
				call div32u						; X=[(CURRENT_U-16*STEP)/STEP]
				lds r8,(STEP_U+0)
				lds r9,(STEP_U+1)
				call MUL1616					; X=X*STEP
				movw r16,r10
				; ----------
					ldi r18, low(5000)
					ldi r19,high(5000)
					sts (OPTO_ON_TICK+0),r18
					sts (OPTO_ON_TICK+1),r19
				; ----------
				lds r18,(STEP_U+0)
				lds r19,(STEP_U+1)
				rjmp DECREASE_CONT	
				; ----------
DECREASE_NORM:	lds r16,(TARGET_U+0)
				lds r17,(TARGET_U+1)
				lds r18,(STEP_U+0)
				lds r19,(STEP_U+1)
DECREASE_CONT:	sub r16,r18
				sbc r17,r19
				call normU
				ldi r18, low(MIN_U)
				ldi r19,high(MIN_U)
				cp  r16,r18
				cpc r17,r19
				sbrc r17,7
				rjmp DECREASE_ZERO
				brcc DECREASE_SET		; Если следующее значение больше или равно минимальному, то устанавливаем его.
DECREASE_ZERO:	clr r16					; Иначе следующее значение устанавливаем = 0
				clr r17					;

DECREASE_SET:	sts (TARGET_U+0),r16
				sts (TARGET_U+1),r17

DECREASE_EX:	pop r19
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
; ----------------------------------------------------------------------------------------
;; ----------------------------------------------------------------------------------------
; Обработчик преобразования АЦП
ADC_COMPLETE:
			PUSHF
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
			rjmp ADC_NORM
			clr r10
			clr r11
			clr r12
			clr r13
			rjmp ADC_ADD_NEW
			; ----------
			; Считываем значение АЦП
ADC_NORM:	lds r6,ADCL
			lds r7,ADCH
			; Так как частота преобразования завышена, отбросим 2 младших разряда преобразования (обнулим)
;			ldi ZL,0b11111100
;			and r6,ZL
			; ----------
			; Возведем в квадрат значение АЦП
			movw r8,r6
			call MUL1616
			; Восстановим старое значение суммы квадратов
ADC_ADD_NEW:
			lds r6,(ADC_CURRENT_SUM_QUAD+0)
			lds r7,(ADC_CURRENT_SUM_QUAD+1)
			lds r8,(ADC_CURRENT_SUM_QUAD+2)
			lds r9,(ADC_CURRENT_SUM_QUAD+3)
			; Добавим к нему новый квадрат
			add r6,r10
			adc r7,r11
			adc r8,r12
			adc r9,r13
			; Сохраним сумму квадратов
			sts (ADC_CURRENT_SUM_QUAD+0),r6
			sts (ADC_CURRENT_SUM_QUAD+1),r7
			sts (ADC_CURRENT_SUM_QUAD+2),r8
			sts (ADC_CURRENT_SUM_QUAD+3),r9
			; Увеличим счетчик преобразований
			lds ZL,(ADC_CURRENT_VALUE_COUNT+0)
			lds ZH,(ADC_CURRENT_VALUE_COUNT+1)
			adiw Z,1
			; Сохраним счетчик преобразований
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
; ----------------------------------------------------------------------------------------
; Переход сетевого напряжения через 0 (прерывание INT0)
ZEROCROSS:	PUSHF
			push r16
			push r17
			
			push r18
			push r19
			; ----------
			; запрещаем прерывание INT0
			ldi r16,$00
			sts (EIMSK),r16
			; ----------
			; гасим оптрон
			sbi MOC_PORT,MOC_PIN
			//lds r16,(MODE)
			//cpi r16,1
			//brne ZEROCROSS_A
			//cbi MOC_PORT,MOC_PIN
			; запомним режим работы Т1	
ZEROCROSS_A:	lds  r16, TCCR1B			;
				push r16
			; останавливаем T1				
			andi r16,$F8			;
			sts TCCR1B,r16	
			; запомним состояние Т1
			lds r18,TCNT1L
			lds r19,TCNT1H
			; обнуляем T1		
			clr r16					; 
			sts TCNT1H,r16			
			sts TCNT1L,r16	
			; перезагружаем  момент  включения оптрона
				lds r17,(OPTO_ON_TICK+1)		
				lds r16,(OPTO_ON_TICK+0)	;
			sts OCR1AH,r17				;
			sts OCR1AL,r16				; 
			; перезагружаем  момент  выключения оптрона
				lds r17,(OPTO_OFF_TICK+1)		
				lds r16,(OPTO_OFF_TICK+0)	;
			sts OCR1BH,r17				;
			sts OCR1BL,r16				; 
			; Сброс прескалера Т0,Т1
			ldi r16,0b00000001		
			out GTCCR,r16
			; вспоминаем режим работы Т1
				pop r16		
			; запускаем T1			
			sts TCCR1B,r16			
			; ----------
			push r20
			push ZH
			push ZL
			push YH
			push YL
			; ----------
			PUSH_ALL_LOW
			; ----------
			lds r16,EICRA			; Выясняем, какой перепад пришел
			andi r16,$03
			cpi r16,$02
			breq FALLING			; переход по спаду
			cpi r16,$03
			breq RISING				; переход по фронту

			ldi r16,0b00001011		; настраиваемся на ожидание фронта INT0
			sts EICRA,r16
			; ----------
			jmp ZEROCROSS_END
; -----------------------------------------------------------------------------------------------------------
			; отрабатываем фронт перехода через 0
RISING:		ldi r16,ADC_ON				; Старт АЦП
			sts ADCSRA,r16
				lds r16,EICRA
				andi r16,$FC
				ori  r16,$02			; настраиваемся на ожидание спада INT0
				sts EICRA,r16
				; ----------
				;sts (COUNT_NEG+0),r18	; запомним длительность
				;sts (COUNT_NEG+1),r19	; отрицательного полупериода
				; ----------
				;lds r16,(COUNT_POS+0)	; вычислим коррекцию
				;lds r17,(COUNT_POS+1)	; для следующего отрицательного полупериода
				;sub r18,r16				; она должна прибавляться к моменту включения оптрона
				;sbc r19,r17				; т.е. должна быть положительной
				;lsr r19 rol r18
				;lsr r19 rol r18
				;sts (CORRECTION+0),r18
				;sts (CORRECTION+1),r19
				; ----------
			jmp ZEROCROSS_END
; -----------------------------------------------------------------------------------------------------------
			; отрабатываем спад перехода через 0
FALLING:	ldi r16,ADC_OFF				; Стоп АЦП
			sts ADCSRA,r16
				lds r16,EICRA
				andi r16,$FC
				ori  r16,$03			; настраиваемся на ожидание фронта INT0
				sts EICRA,r16
				; ----------
				;sts (COUNT_POS+0),r18	; запомним длительность
				;sts (COUNT_POS+1),r19	; положительного полупериода
				; ----------
				;lds r16,(COUNT_NEG+0)	; вычислим коррекцию
				;lds r17,(COUNT_NEG+1)	; для следующего положительного полупериода
				;sub r18,r16				; она должна вычитаться к моменту включения оптрона
				;sbc r19,r17				; т.е. должна быть отрицательной
				;lsr r19 rol r18
				;lsr r19 rol r18
				;sts (CORRECTION+0),r18
				;sts (CORRECTION+1),r19
				; ----------
; обрабатываем накопленные за положительный полупериод данные
			; Вычислим измеренное значение U_RMS
			lds r6,(ADC_CURRENT_SUM_QUAD+0)
			lds r7,(ADC_CURRENT_SUM_QUAD+1)
			lds r8,(ADC_CURRENT_SUM_QUAD+2)
			lds r9,(ADC_CURRENT_SUM_QUAD+3)
			lds r10,(ADC_CURRENT_VALUE_COUNT+0)
			lds r11,(ADC_CURRENT_VALUE_COUNT+1)
			; ----------	
			lsl r6						; Умножим r9...r6 на 4 для лучшего использования разрядной сетки
			rol r7
			rol r8
			rol r9
			lsl r6
			rol r7
			rol r8
			rol r9
			; ----------
			clr r12
			clr r13
			call div32u					; Посчитаем квадрат RMS
			; ----------
			lsl r6						; Умножим r9...r6 на 4 
			rol r7
			rol r8
			rol r9
			lsl r6
			rol r7
			rol r8
			rol r9
			; ----------
			movw r20,r6
			movw r22,r8
			; ----------
			sts (CURRENT_QUAD_U+0),r6
			sts (CURRENT_QUAD_U+1),r7
			sts (CURRENT_QUAD_U+2),r8
			sts (CURRENT_QUAD_U+3),r9
			; ----------

			call sqr		; вычислим корень квадратный из суммы квадратов отсчетов за период измерения
							; в r9 r8 получим TRUE RMS значение напряжения,
							; умноженное на некоторый коэффициент, определяемый внешним делителем
			; ----------
			sts (CURRENT_U+0),r8	; Сохраним текущее значение U RMS 
			sts (CURRENT_U+1),r9	; 
		; ----------
			clr r16
			sts (ADC_CURRENT_SUM_QUAD+0),r16		; Сброс текущей суммы квадратов
			sts (ADC_CURRENT_SUM_QUAD+1),r16
			sts (ADC_CURRENT_SUM_QUAD+2),r16
			sts (ADC_CURRENT_SUM_QUAD+3),r16
			sts (ADC_CURRENT_VALUE_COUNT+0),r16		; Сбросим счетчик преобразований
			sts (ADC_CURRENT_VALUE_COUNT+1),r16
			; Займемся усреднением U_RMS для вывода на индикатор (дабы не скакало)
			ldi ZH,high(CURRENT_U_HISTORY + 2*CU_HISTORY_LENGTH)
			ldi ZL,low (CURRENT_U_HISTORY + 2*CU_HISTORY_LENGTH)
			movw Y,Z				; Указатели на самый хвост сохраненных значений
			clr r16
			mov r8,r16
			mov r2,r16
			mov r3,r16
			ld r7, -Y							
			ld r6, -Y
			ldi r16,CU_HISTORY_LENGTH
AVG_LOOP:	tst r16
			breq AVG_LOOP_EX
			dec r16

			ld r1,-Y							
			ld r0,-Y							
				add r6, r0							
				adc r7, r1
				adc r8, r2	
				adc r9, r3						 
			st -Z,r1							
			st -Z,r0							
			; ----------
			jmp AVG_LOOP
			; ----------
AVG_LOOP_EX:ldi ZL, low(CU_HISTORY_LENGTH+1)
			ldi ZH,high(CU_HISTORY_LENGTH+1)
			movw r10,Z
			clr r12
			clr r13
			call div32u
			; ----------
			sts (AVERAGE_U+0),r6	; Сохраним среднеарифметическое значение U RMS 
			sts (AVERAGE_U+1),r7	; 
				; ----------
; ---------------------------------------------------------------------------------
; А это обратная связь по RMS напряжению
OOS_U:		lds r16,(MODE)
				cpi r16,0
				brne modeCheck1
				rjmp OOS_NORMAL			; стабилизация
modeCheck1:		cpi r16,1
				brne modeCheck2
				rjmp modeOOS1			; на разгон
modeCheck2:		cpi r16,2
				brne modeCheck3
				rjmp modeOOS2			; на стоп
modeCheck3:		cpi r16,3
				brne modeCheck4
				rjmp modeOOS2			; на стоп
modeCheck4:
			rjmp ZEROCROSS_END
; ---------- разгон
modeOOS1:		//in r16,PORTC
				//CLRB r16,0
				//out PORTC,r16
				cbi PORTB,5
				//cbi PORTD,7
			clr r8
			clr r9
			inc r8
			ldi r16,$FF
			//sts (FL_MAX_U),r16
			sts (FL_NEW_U),r16
//				cbi MOC_PORT,MOC_PIN
			rjmp SET_OPTO_ON_TICK
			rjmp ZEROCROSS_END
; ---------- стоп
modeOOS2:		//in r16,PORTC
				//SETB r16,0
				//out PORTC,r16
				sbi PORTB,5
				//sbi PORTD,7
			ldi r16, low(DEFAULT_OFF_VALUE)
			ldi r17,high(DEFAULT_OFF_VALUE)
			movw r8,r16
			ldi r16,$FF
			//sts (FL_MAX_U),r16
			sts (FL_NEW_U),r16
//				sbi MOC_PORT,MOC_PIN
			rjmp SET_OPTO_ON_TICK
			rjmp ZEROCROSS_END
; ----------
OOS_NORMAL:		sbi PORTB,5
				//sbi PORTD,7
			lds r6,(CURRENT_U+0)		; по текущему напряжению
			lds r7,(CURRENT_U+1)
			lds r8,(TARGET_U+0)
			lds r9,(TARGET_U+1)
			sub r6,r8					; delta=CURRENT_U-target_u
			sbc r7,r9

			sbrc r7,7
			rjmp OOS_MIN
			//ldi r16,$03
			//and r7,r16
			ldi r16,$FE
			and r6,r16
			rjmp OOS_UUU
OOS_MIN:	//ldi r16,$FC
			//or  r7,r16
			ldi r16,$FE
			and r6,r16
OOS_UUU:

OOS_NORM_A:
			lds r8,(OPTO_ON_TICK+0)
			lds r9,(OPTO_ON_TICK+1)
			add r8,r6					; opto_on_tick = opto_on_tick + delta
			adc r9,r7					; Новое значение момента включения симистора

			; Проверка на предмет выхода за допустимый диапазон
			; от 1 до DEFAULT_OFF_VALUE

			sbrs r9,7					; переход, если неотрицательное (0000...7FFF)
			rjmp N1

			; отрицательное значение - установка максимального на выходе

OOS_MAX_U:	ldi r16,1
			ldi r17,0
			movw r8,r16					; иначе OPTO_ON_TICK = 1 (максимальное напряжение)

			ldi r16,$FF					; И устанавливаем флаг 
			sts (FL_MAX_U),r16			; максимального напряжения на выходе

			sts (FL_NEW_U),r16			; На индикатор выводим измеренное значение напряжения
			
			// далее включаем индикатор низкого входного напряжения
			//
			//							;
										;
										;
							jmp SET_OPTO_ON_TICK

			; 
N1:							ldi r16, low(DEFAULT_OFF_VALUE)
							ldi r17,high(DEFAULT_OFF_VALUE)
							cp  r8,r16
							cpc r9,r17
							brcs N2						; переход, если меньше DEFAULT_OFF_VALUE

			; отрабатываем минимальное значение 0
							movw r8,r16					; иначе OPTO_ON_TICK = DEFAULT_OFF_VALUE
;	clr r16						; и TARGET_U = 0
;	sts (TARGET_U+0),r16
;	sts (TARGET_U+1),r16
			; обновить индикатор
							ldi r16,$FF
							sts (FL_NEW_U),r16
							sts (FL_NEW_T),r16

			; допустимый диапазон
N2:							clr r16						; сбрасываем флаг 
							sts (FL_MAX_U),r16			; максимального напряжения на выходе


SET_OPTO_ON_TICK:			sts (OPTO_ON_TICK+0),r8
							sts (OPTO_ON_TICK+1),r9




;			rjmp NEXT_WORK
; ---------------------------------------------------------------------------------
; ---------------------------------------------------------------------------------
			; индикация вхождения усредненного выходного напряжения в допустимый коридор
NEXT_WORK:	lds ZL,(AVERAGE_U+0)
			lds ZH,(AVERAGE_U+1)
			lds r16,(TARGET_U+0)
			lds r17,(TARGET_U+1)
			sub ZL,r16
			sbc ZH,r17
			brcc NO_MINUS
				com ZL
				com ZH
				adiw Z,1
NO_MINUS:	ldi r16, low(3)			; +- 0.3 V
			ldi r17,high(3)			;
			cp  ZL,r16
			cpc ZH,r17
			brcc NOT_RANGE
				cbi PORTD,7			; вкл индикатор стабилизации в диапазоне
				lds r16,(FL_INRANGE)
				andi r16,$0F
				cpi r16,$00
				breq setINRFLAG
				rjmp RANGE_EX
setINRFLAG:		ldi r16,$8F
				sts (FL_INRANGE),r16
			rjmp RANGE_EX
NOT_RANGE:		sbi PORTD,7			; выкл индикатор стабилизации
				lds r16,(FL_INRANGE)
				andi r16,$0F
				cpi r16,$0F
				breq clrINRFLAG
				rjmp RANGE_EX
clrINRFLAG:		ldi r16,$80
				sts (FL_INRANGE),r16
RANGE_EX:
		; ----------
ZEROCROSS_END:
			; ----------
			POP_ALL_LOW
			; ----------
			pop YL
			pop YH
			pop ZL
			pop ZH
			pop r20
			pop r19
			pop r18
			; ----------
			pop r17
			pop r16
			POPF
			reti
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
; Отработка изменения сигнала на ногах PC
/*
PCI_ONE:	PUSHF
			push r16

			lds r16,(MODE)
			cpi r16,3
			brne PCI_ONE_RUN
			rjmp PCI_ONE_EX

PCI_ONE_RUN:			in r16,PINC
						andi r16,0b00000011
						sbrs r16,0	
						rjmp EXT_STOP
			
					sbrc r16,1
					rjmp EXT_FULLPOWER_OFF

					sbrs r16,1
					rjmp EXT_FULLPOWER_ON

					rjmp PCI_ONE_EX

EXT_FULLPOWER_OFF:		lds r16,(MODE)
						cpi r16,1
						brne EXT_FULLPOWER_OFF_EX
						ldi r16,0
						sts (MODE),r16
						ldi r16,$FF
						sts (FL_NEW_U),r16
						sts (FL_NEW_M),r16
EXT_FULLPOWER_OFF_EX:	rjmp PCI_ONE_EX

EXT_FULLPOWER_ON:		lds r16,(MODE)
						cpi r16,0
						brne EXT_FULLPOWER_ON_EX
						ldi r16,1
						sts (MODE),r16
						ldi r16,$FF
						sts (FL_NEW_U),r16
						sts (FL_NEW_M),r16
EXT_FULLPOWER_ON_EX:	//rjmp PCI_ONE_EX

EXT_STOP:				//ldi r16,3
						//sts (MODE),r16
						//ldi r16,$FF
						//sts (FL_NEW_U),r16
						//sts (FL_NEW_M),r16
						//ldi r16,2
						//sts (SOUNDNUMBER),r16
						//call LED_CLEAR
					//rjmp PCI_ONE_EX
PCI_ONE_EX:	
			pop r16
			POPF
			reti
			*/
; ----------------------------------------------------------------------------------------
; Включение оптрона управления симистором 
; Прерывание по совпадению Т1A
OPTO_ON:		//	pushfand r16
				//	lds r16,(MODE)
				//	tst r16
				//	breq OPTO_ON_EN				;	открываем по времени только в режиме стабилизации
				//	sbi MOC_PORT,MOC_PIN
				//	rjmp OPTO_ON_DIS

OPTO_ON_EN:			cbi MOC_PORT,MOC_PIN
OPTO_ON_DIS:	//	popfand r16
					reti
; ----------------------------------------------------------------------------------------
; Выключение оптрона управления симистором 
; Прерывание по совпадению Т1B
OPTO_OFF:			sbi MOC_PORT,MOC_PIN
			; ----------
			; разрешаем прерывание INT0 (ожидание перехода через 0)
					push r16
					ldi r16,$01
					sts (EIMSK),r16
					pop r16
					reti
; ----------------------------------------------------------------------------------------
OPTO_OFF_E:	sbi MOC_PORT,MOC_PIN
			reti
; ----------------------------------------------------------------------------------------
; Прерывание по совпадению Т0A (1 мс)
T0COMP:	
		PUSHF
		PUSHZ
		push r16
		push r17
		push r18
		push R19
				LDZ T0EXTENDER
				lds r19,(DATA_FLAGS)
				ld r16,Z+
				ld r17,Z+
				ld r18,Z+

				tst r16
					breq T0_10ms
				dec r16
				rjmp T0Comp_Ex
				;--------------------
T0_10ms:		SETB r19,0				; флаг 10 ms
					call sound
				ldi r16,9
				tst r17
					breq T0_100ms
				dec r17
				rjmp T0Comp_Ex
				;--------------------
T0_100ms:		SETB r19,1				; флаг 100 ms
				; ----------
				;	call PrepareDataToSend		; подготовим данные для передачи и запустим её
				; ----------
				ldi r17,9				; 
				tst r18
					breq T0_1000ms
				dec r18
				rjmp T0Comp_Ex
				;--------------------
T0_1000ms:		SETB r19,2				; флаг 1000 ms
				ldi r18,9				; 
				push r16
				push r17
				lds r16,(MODE)
				cpi r16,3
				brne T0CompareA_M2c

				ldi r16,0
				sts (COUNTER_A),r16
				rjmp T0CompareA_M2b
				;--------------------
				;- Аварийный сигнал -
T0CompareA_M2c:	in  r16,PINC			;	опрос ноги аварийного входа
				lds r17,(COUNTER_A)
				sbrc r16,0
				ldi r17,$FF
				inc r17
				cpi r17,5				;	5 секунд - время срабатывания (низкий уровень)
				brcs T0CompareA_M2a
					ldi r16,3
					sts (MODE),r16
					ldi r16,2
					sts (SOUNDNUMBER),r16
				ldi r16,$FF
				sts (FL_NEW_M),r16
				sts (FL_NEW_U),r16
				rjmp T0CompareA_M2b

T0CompareA_M2a:	sts (COUNTER_A),r17

T0CompareA_M2b:	pop r17
				pop r16


//
T0Comp_Ex:		st -Z,r18
				st -Z,r17
				st -Z,r16
				sts (DATA_FLAGS),r19

		call encoder
		call pressButton

; ------------------------------------------------------------
			pop r19
			pop r18
			pop r17
			pop r16
			POPZ
			POPF
			reti
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
; ------------------------------------------------------------------------
; Умножение R7..R6 * R9..R8 = R13..12..11..10
MUL1616:		PUSHF
				push r15
				push r0
				push r1

				mul r7,r9
				movw r12,r0

				mul r6,r8
				movw r10,r0

				mul r6,r9
				clr r15
				add r11,r0
				adc r12,r1
				adc r13,r15

				mul r7,r8
				clr r15
				add r11,r0
				adc r12,r1
				adc r13,r15

				pop r1
				pop r0
				pop r15
				POPF
			ret
; ----------------------------------------------------------------------------------------
; Превращение HEX в два ASCII
; R16 - входное, выход: R17 - старший, R16 - младший
HexToAscii:		PUSHF
				mov r17,r16
				andi r16,$0F
				andi r17,$F0
				swap r17
				subi r16,(-$30)
				cpi r16,$3A
				brcs HTA_m1
				subi r16,(-7)
HTA_m1:			subi r17,(-$30)
				cpi r17,$3A
				brcs HTA_m2
				subi r17,(-7)
HTA_m2:
				POPF
				ret
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
; ----------------------------------------------------------------------------------------
; *****************************************************************************
; Ниже разные подпрограммы из простторов интернета
; *****************************************************************************
;-----------------------------------------------------------------------------:
; 32bit/32bit Unsigned Division
;
; Size  = 26 words
; Clock = 549..677 cycles (+ret)
; Stack = 0 bytes
; --- делимое/результат
;r9 ...r6	
; --- делитель
;r13...r10	
; --- остаток
;r3 ....r0	

div32u:		clr	r0		;initialize variables
			clr	r1		;  mod = 0;
			clr	r2		;  lc = 32;
			clr	r3		;
			ldi	r31,32		;/
			;---- calcurating loop
			lsl	r6		;var1 = var1 << 1;
			rol	r7		;
			rol	r8		;
			rol	r9		;/
			rol	r0		;mod = mod << 1 + carry;
			rol	r1		;
			rol	r2		;
			rol	r3		;/
			cp	r0,r10	;if (mod => var2) {
			cpc	r1,r11	; mod -= var2; var1++;
			cpc	r2,r12	; }
			cpc	r3,r13	;
			brcs	PC+6		;
			inc	r6		;
			sub	r0,r10	;
			sbc	r1,r11	;
			sbc	r2,r12	;
			sbc	r3,r13	;/
			dec	r31		;if (--lc > 0)
			brne	PC-19		; continue loop;
		ret
;-----------------------------------------------------------------------------:
; https://radiokot.ru/forum/viewtopic.php?p=779071#p779071
;****************************************************************
;  //объявление функции с одним параметром - без знаковое целое 4  байта
;  unsigned short isqrt( unsigned long ul) { 
;  // объявление переменной - без знаковое целое 4  байта;  
;  unsigned long sqr = 0;   
;  // объявление переменной - без знаковое целое 4  байта
;  unsigned long temp;     
;  // объявление переменной   - без знаковое целое 4  байта. 
;  //В переменную записывается значение 0x40000000
;  unsigned long mask = 0x40000000; 
;  do{         //  цикл с пост условием (проверка произойдет после выполнения тела цикла)
;    temp = sqr | mask;  // побитное "или"  между  sqr и mask, результат записывается в temp
;    sqr >>= 1;          // сдвиг переменной sqr на 1 бит вправо
;    if( temp <= ul ){   // условный оператор если temp меньше равно  ul
;      sqr |= mask;      //  побитное "или"  между  sqr и mask, результат записывается в sqr
;      ul -= temp;       //  вычисляется разница между  ul и temp результат записывается в ul
;    }
;  }while( mask >>= 2 ); // Сдвинуть mask  на два бита в право, результат записать в mask,  выполнять цикл пока mask > 0.
;  if( sqr < ul && sqr < 0xFFFF ) ++sqr; // округление результата  (если sqr меньше ul "и" sqr меньше 0xFFFF в sqr записывается sqr+1)
;  return (unsigned short)sqr;           // возвращение результата работы функции, тип переменной приводится к без знаковое целое 1  байт

;SquareRoot32to16x16:
; r23...20 - входное  значение
; r9...r8  - выходное значение
;Регистры общего назначения
.def		sqr0		= r8
.def		sqr1		= r9
.def		sqr2		= r10
.def		sqr3		= r11

.def		mask0		= r12
.def		mask1		= r13
.def		mask2		= r14
.def		mask3		= r15

.def		temp0 		= r16
.def		temp1 		= r17
.def		temp2 		= r18
.def		temp3 		= r19

.def		src0		= r20
.def		src1		= r21
.def		src2		= r22
.def		src3		= r23

sqr:
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
		;
		clr		mask0				;  unsigned long mask = 0x40000000; 
		clr		mask1
		clr		mask2
		ldi		temp0,	0x40
		mov		mask3,	temp0
		;
		clr		sqr0				;  unsigned long sqr = 0;   
		clr		sqr1
		clr		sqr2
		clr		sqr3
		
_sr32loop:
			movw	temp0,	sqr0		;temp = sqr | mask; 
			movw	temp2,	sqr2
			or   temp0, mask0
			or   temp1, mask1
			or   temp2, mask2
			or   temp3, mask3
			lsr		sqr3	;sqr >>= 1; 
			ror		sqr2
			ror		sqr1
			ror		sqr0
			;
			cp		src0,	temp0		;if( temp <= ul )
			cpc		src1,	temp1
			cpc		src2,	temp2
			cpc		src3,	temp3
			; Если С=0, то условие выполнено
			brcs	_sr32_skipif		; пропустить, если С=1
				;							{
				or		sqr0,	mask0	; sqr |= mask
				or		sqr1,	mask1
				or		sqr2,	mask2
				or		sqr3,	mask3
				;
				sub		src0,	temp0 	; ul -= temp; 
				sbc		src1,	temp1
				sbc		src2,	temp2
				sbc		src3,	temp3
				;							}
_sr32_skipif:
			lsr		mask3		;( mask >>= 2 )
			ror		mask2
			ror		mask1
			ror		mask0
			;
			lsr		mask3
			ror		mask2
			ror		mask1
			ror		mask0
			;
			mov		temp0,	mask0   ; mask=0?
			or		temp0,	mask1
			or		temp0,	mask2
			or		temp0,	mask3
			;
			brne	_sr32loop		;while( mask=0 )
		;
		;sts		OutputSqr,	sqr0
		;sts		(OutputSqr+1), sqr1
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
ret
; *********************************************************************

; -------------------------------------------------
;***** Subroutine Register Variables
/*
.equ	10	=10		;address of r10
.equ	12	=12		;address of r13

.def	r10	=r10		;BCD value digits 1 and 0
.def	r11	=r11		;BCD value digits 3 and 2
.def	r12	=r12		;BCD value digit 4 5
;.def	r13	=r13		;BCD value digit 6 7

.def	r16	=r16		;binary value Low byte
.def	r17	=r17		;binary value High byte
;.def	r18	=r18		;binary value VHigh byte

.def	r19	=r19		;loop counter
.def	r20	=r20		;temporary value
*/
;***** Code

bin2BCD16:
			PUSHF
			push r16
			push r17
			push r19
			push r20
			ldi	r19,16	;Init loop counter	
			clr	r12		;clear result (3 bytes)
			clr	r11		
			clr	r10		
			clr	ZH		;clear ZH (not needed for AT90Sxx0x)
bBCDx_1:
			lsl	r16		;shift input value
			rol	r17		;through all bytes
			rol	r10		;
			rol	r11
			rol	r12
			dec	r19		;decrement loop counter
			brne bBCDx_2		;if counter not zero
nop
			pop r20
			pop r19
			pop r17
			pop r16
			POPF
			ret			;   return

bBCDx_2:
			ldi		r30,12+1	;Z points to result MSB + 1
bBCDx_3:
			ld		r20,-Z	;get (Z) with pre-decrement
			subi	r20,-$03	;add 0x03
			sbrc	r20,3	;if bit 3 not clear
			st		Z,r20	;	store back
			ld		r20,Z	;get (Z)
			subi	r20,-$30	;add 0x30
			sbrc	r20,7	;if bit 7 not clear
			st		Z,r20	;	store back
			cpi		ZL,10	;done all three?
			brne	bBCDx_3		;loop again if not
			rjmp	bBCDx_1		

//-----------------------------------------------------------

normU:			movw r6,r16
				clr r8
				clr r9
				lds r10,(STEP_U+0)
				lds r11,(STEP_U+1)
				clr r12
				clr r13
					call div32u						; X=[TARGET_U/STEP]
				lds r8,(STEP_U+0)
				lds r9,(STEP_U+1)
					call MUL1616					; X=X*STEP
				movw r16,r10					; R17..16 - TARGET_U

				ret
; ------------------------------------------------------------------------
// ----------------------------------------------------------------
toneIndex:	.dw notone,tone0,tone1
// ----------------------------------------------------------------
notone:
tone0:	/*.db	 94, 94,118,118, 94, 94,118,118, 88, 88,\
			 94, 94,105,105,158,158,158,158,158,158,\
			158,158,141,141,126,126,118,118,118,118,\
			118,118,$00,$00,$00,$00,$00,$00,$00,$00,\ 
		.db	238,212,189,178,158,141,126,118,105, 94,\
			 88, 79, 70, 62, 59, 59, 62, 70, 79, 88,\
			 94,105,118,126,141,158,178,189,212,238,\
			238,  0,$00,$00,$00,$00,$00,$00,$00,$00,\*/
		.db	238,238,212,212,189,189,178,178,158,158,\
			141,141,126,126,118,118,105,105, 94, 94,\
			 88, 88, 79, 79, 70, 70, 62, 62, 59, 59,\
			  0,  0,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00
// ----------------------------------------------------------------
tone1:	.db  70, 70, 70, 70, 70, 70, 70, 70, 70, 70,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			 70, 70, 70, 70, 70, 70, 70, 70, 70, 70,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			 59, 59, 59, 59, 59, 59, 59, 59, 59, 59,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00,$00,$00,$00,$00,\
			$00,$00,$00,$00,$00,$00
// ----------------------------------------------------------------
// -- Извлекалка звуков -------------------------------------------
sound:				PUSHZ
					push r16
					push r17
					lds r16,(SOUNDNUMBER)
					tst r16
					brne soundChoice
					rjmp soundOffAndDisable

soundChoice:		LDZ toneIndex*2
					clr r17
					andi r16,$03
					add ZL,r16	adc ZH,r17
					add ZL,r16	adc ZH,r17
					lpm r16,Z+
					lpm r17,Z
					movw ZH:ZL,r17:r16
					add ZL,ZL
					adc ZH,ZH
					;LDZ tone0*2
					lds r16,(TONECOUNTER)
					ldi r17,$00
					add ZL,r16	adc ZH,r17
					inc r16
					;andi r16,$3F
					cpi r16,$60
					brcs skipZeroCounter
					clr r16
skipZeroCounter:	sts (TONECOUNTER),r16
					lpm r16,Z
					tst r16
					breq soundOffOnly

					sts OCR2A,r16
					ldi r16,soundOn
					sts TCCR2A,r16
					rjmp soundExit

soundOffOnly:		ldi r16,soundOff
					sts TCCR2A,r16
					rjmp soundExit

soundOffAndDisable:	ldi r16,soundOff
					sts TCCR2A,r16
					clr r16
					sts (TONECOUNTER),r16
					rjmp soundExit

soundExit:			pop r17
					pop r16
					POPZ
					ret
// ----------------------------------------------------------------
.include "btn_subs.asm"
.include "comport.inc"
.include "f32x40.inc"
.include "f9x16.inc"
.include "indication.asm"
.include "led.inc"
.include "tm1637.inc"
.include "twi.inc"
