.equ		XTAL						= 16000000
.equ		I2C_SlaveAddress			= $32			; адрес регулятора на шине i2c #32
.equ 		baudrate 					= 9600						;	скорость последовательного порта
.equ 		bauddivider 				= (XTAL/(16*baudrate))-1	;	коэфф деления для UART
.equ 		baudrate1 					= 38400						;	скорость последовательного порта
.equ 		bauddivider1 				= (XTAL/(16*baudrate1))-1	;	коэфф деления для UART
//	----------------------------------------------------------------
//	Моменты включения и отключения оптрона по умолчанию. Отсчитываются от перехода через 0. Один тик - 0.5 мкс
//	Используются таймером 1.
.equ		DEFAULT_ON_VALUE 			= 19000	;	для сети 60 Гц установить равным 15600		
.equ		DEFAULT_OFF_VALUE 			= 19000	;	для сети 60 Гц установить равным 15600
.equ		MIN_ON_VALUE				= 8	

.equ		MOC_PORT	= PORTD
.equ		MOC_PIN		= 6

.equ		ADC_ON				= 0b11101101	; Заклинание включения  АЦП
.equ		ADC_OFF				= 0b01101101	; Заклинание выключения АЦП

.equ		IND_PORT			= PORTB			; Порт подключения индикатора
.equ		IND_DDR				= DDRB			;
.equ		IND_PIN				= PINB
.equ		IND_CLK				= 0				; синхро SCL
.equ		IND_DIO				= 1				; данные SDA

.equ		IND_ADDR_A			= $78
.equ		IND_ADDR_B			= $7A
.equ		DisplayWidth		= 128

.equ		ACK					= $00
.equ		NACK				= $FF

.equ		HS					= $00			;	полупробел
//	----------------------------------------------------------------
// -- Управляющие коды для позиционирования и оформления на дисплее --
.equ EOS				= $80	; конец строки
.equ F					= $81	; шрифт No, второй байт номер шрифта
.equ Line				= $82	; линия, Y, X1, X2, Pattern
.equ XY					= $83	; позиционирование, следом идут два байта: координата X и Y
.equ dXdY				= $84	; сдвиг на X,Y (два байта следом)
// -- Управляющие коды для вывода переменных на дисплей --
.equ Range				= $88	; Индикатор вхождения напряжения в допустимый коридор
.equ TargetV			= $89	; Индикатор целевого напряжения
.equ OutputV			= $8A	; Индикатор измеренного напряжения
.equ MainV				= $8B	; Основной (большой) индикатор

.equ OuterExtraNumber	= $90	; Код, лежащий за границей возможных
//	----------------------------------------------------------------
.equ		StabMode			= 0
.equ		ForceMode			= 1
.equ		StopMode			= 2
.equ		ExtOffMode			= 3

.equ		ExtOFFPin			= 0
.equ		ExtForcePin			= 1

.equ 		DEFAULT_STEP_U		= 5;
.equ		MIN_U				= 400
.equ		MAX_U				= 2500
.equ		CU_HISTORY_LENGTH	= 15			; Длина истории значений CURRENT_U
//	----------------------------------------------------------------
.equ	eachSecondFlag		= 0		;	1 sec
.equ	eachdeciSecondFlag	= 1		;	100 msec
.equ	eachcentiSecondFlag	= 2		;	10 msec
.equ	eachmilliSecondFlag	= 3		;	1 msec
.equ	eachdekaSecondFlag	= 4		;	10 sec
.equ	eachXdeciSecondFlag	= 5		;	X*100 msec /500/
.equ	eachYdeciSecondFlag	= 6		;	Y*100 msec /200/
.equ	eachHektoSecondFlag	= 7		;	100 sec
