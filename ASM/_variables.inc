.dseg
//	----------------------------------------------------------------
OPTO_ON_TICK	:			.BYTE 2				; Значение момента включения  оптрона записывается в OCR1A
OPTO_OFF_TICK	:			.BYTE 2				; Значение момента вЫключения оптрона записывается в OCR1B
TCCR1B_SAVED	:			.BYTE 1
TCNT1_SAVED		:			.BYTE 2
//	----------------------------------------------------------------
IndicatorAddr:				.BYTE 1				;
FontNo:						.BYTE 1
POSX:						.BYTE 1
POSY:						.BYTE 1
OLDX:						.BYTE 1
OLDY:						.BYTE 1
StringCounter:				.BYTE 1
//	----------------------------------------------------------------
SoftACK:					.BYTE 1		
MODE:						.BYTE 1				;	Режим работы
//	----------------------------------------------------------------
//	таблица координат вывода значений, закодированных символами
ExtrasConfig:		.BYTE (OuterExtraNumber-$80)*2	
									;	byte+0:	координата Х (0...191)
									;	byte+1:	биты 0...2 	- координата Y (0...7)
									;	бит	 4...5	- номер шрифта (0-3)
									;	бит  6		- разрешение вывода (0-вывод выкл, 1-вывод вкл)
									; 	бит  7		- признак изменившегося значения (0-старое, 1-новое)
//	----------------------------------------------------------------
DIGITS:
DIGIT0					:	.BYTE 1
DIGIT1					:	.BYTE 1
DIGIT2					:	.BYTE 1
DIGIT3					:	.BYTE 1
DIGIT4					:	.BYTE 1
HidedZeroesCount		:	.BYTE 1
//	----------------------------------------------------------------

ACTIONS_FLAGS			:	.BYTE 1		; Флаги для вызова определенных действий
										; 0 - обработка принятого по UART 
										; 1 - обработка принятого по iic
										; 2 -
										; 3 - короткое нажатие кнопки
										; 4 - длительное удержание кнопки
										; StepUpFlagNo - энкодер в одну   сторону +
										; StepDnFlagNo - энкодер в другую сторону -
										; 7 - 
milliSeconds:	.BYTE 1
centiSeconds:	.BYTE 1
deciSeconds:	.BYTE 1
deciSecondsX:	.BYTE 1
deciSecondsY:	.BYTE 1
Seconds:		.BYTE 1

TIME_FLAGS:		.BYTE 1

RX_BUFFER				:	.BYTE 32	; буфер приема
TX_BUFFER				:	.BYTE 32	; буфер передачи
RX_PTR					:	.BYTE 1		; указатель буфера приема 
TX_PTR					:	.BYTE 1		; указатель буфера передачи
TX_BUSY					:	.BYTE 1
TX_QUEUE_POINTER		:	.BYTE 1
TX_QUEUE				:	.BYTE 8

oldPINC					:	.BYTE 1
PinCCounter0			:	.BYTE 1
PinCCounter1			:	.BYTE 1

ADC_CURRENT_SUM_QUAD	:	.BYTE 4
ADC_CURRENT_VALUE_COUNT	:	.BYTE 2

STEP_U					:	.BYTE 2

AVERAGE_U				:	.BYTE 5		; Текущее(0-1), предыдущее(2-3) значение усредненного URMS и признак(4)

TARGET_U				:	.BYTE 5		; Текущее(0-1), предыдущее(2-3) значение целевого напряжения и признак(4)

CURRENT_U				:	.BYTE 2+2*CU_HISTORY_LENGTH	; Текущее значение URMS и его история

FL_MAX_U:					.BYTE 1
FL_RANGE:					.BYTE 1

COUNTER_D:					.BYTE 1
COUNTER_A:					.BYTE 1
ZEROCROSS_PIN_STATE:		.BYTE 1
