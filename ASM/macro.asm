.macro	PUSHFand
			push @0
			in @0,SREG
			push @0
.endmacro
;-----------
.macro	POPFand
			pop @0
			out SREG,@0
			pop @0
.endmacro
;-----------
.macro	PUSHF
			push r15
			in r15,SREG
			push r15
.endmacro
; ----------
.macro	POPF
			pop r15
			out SREG,r15
			pop r15
.endmacro
; ----------
.macro	PUSH_ALL_LOW
			push r0
			push r1
			push r2
			push r3
			push r4
			push r5
			push r6
			push r7
			push r8
			push r9
			push r10
			push r11
			push r12
			push r13
			push r14
			push r15
.endmacro
; ----------
.macro	POP_ALL_LOW
			pop r15
			pop r14
			pop r13
			pop r12
			pop r11
			pop r10
			pop r9
			pop r8
			pop r7
			pop r6
			pop r5
			pop r4
			pop r3
			pop r2
			pop r1
			pop r0
.endmacro

	.MACRO PUSHXYZ
	PUSH	XL
	PUSH	XH
	PUSH	YL
	PUSH	YH
	PUSH	ZL
	PUSH	ZH	
	.ENDM

	.MACRO POPXYZ
	POP		ZH
	POP		ZL
	POP		YH
	POP		YL
	POP		XH
	POP		XL	
	.ENDM

	.MACRO PUSHX
	PUSH	XL
	PUSH	XH
	.ENDM

	.MACRO POPX
	POP		XH
	POP		XL	
	.ENDM

	.MACRO PUSHY
	PUSH	YL
	PUSH	YH
	.ENDM

	.MACRO POPY
	POP		YH
	POP		YL
	.ENDM

	.MACRO PUSHZ
	PUSH	ZL
	PUSH	ZH	
	.ENDM

	.MACRO POPZ
	POP		ZH
	POP		ZL
	.ENDM
; ----------
.macro CLEAR_ALL
		; Очистка памяти и регистров 
		; http://easyelectronics.ru/avr-uchebnyj-kurs-vazhnye-melochi-1.html

RAM_Flush:	ldi	ZL,Low(SRAM_START)	; Адрес начала ОЗУ
			ldi	ZH,High(SRAM_START)
			clr	R16					; Очищаем R16
Flush:		st 	Z+,R16				; Сохраняем 0 в ячейку памяти
			cpi	ZH,High(RAMEND+1)	; Достигли конца ОЗУ ?
			brne	Flush			; Нет? Крутимся дальше!
 
			cpi	ZL,Low(RAMEND+1)	; Младший байт достиг конца?
			brne	Flush
			st -Z,r16
		; Настраиваем стек
			out spl,ZL
			out sph,ZH
 
			ldi	ZL, 30				; Адрес самого старшего регистра	
			clr	ZH					; Здесь будет 0
			dec	ZL					; Уменьшая адрес,
			st	Z, ZH				; записываем в регистр 0,
			brne	PC-2			; пока не перебрали всё и успокоились.
.endmacro
; ----------------------------------------------------------------------------------------
	.MACRO	LDX
	LDI		XL,low(@0)
	LDI		XH,High(@0)
	.ENDM

	.MACRO	LDY
	LDI		YL,low(@0)
	LDI		YH,High(@0)
	.ENDM

	.MACRO	LDZ
	LDI		ZL,low(@0)
	LDI		ZH,High(@0)
	.ENDM

	.MACRO	LDSX
	LDS		XL,(@0+0)
	LDS		XH,(@0+1)
	.ENDM

	.MACRO	LDSY
	LDS		YL,(@0+0)
	LDS		YH,(@0+1)
	.ENDM

	.MACRO	LDSZ
	LDS		ZL,(@0+0)
	LDS		ZH,(@0+1)
	.ENDM

	.MACRO	STSY
	STS		(@0+0),YL
	STS		(@0+1),YH
	.ENDM
; ----------
.MACRO SETB
			ori @0,EXP2(@1)
.ENDM
; ----------
.MACRO CLRB
			andi @0,$FF-EXP2(@1)
.ENDM
// ----------------------------------------------------------------
;	Макросы временных задержек
// ----------------------------------------------------------------
; http://forum.avr.ru/showthread.php?goto=nextoldest&t=36050
; Точная задержка на указанное количество тактов
; @0 -- количество тактов, @0 = 0...770
; @1 -- используемый регистр для реализации задержки
.MACRO M_DELAY_CLK_A
.if (@0) > 0                  ; Если @0 <= 0 -- нет генерации задержки
.if (@0) > 770                ; Больше 770 тактов нельзя, выводим ошибку
.error "@0 must be less or equal 770"
.endif
.if (@0)/3 > 0                ; Делим на 3, т.к. 1 итерация цикла -- 3 такта
    ldi     @1, (@0-3)/3
    subi    @1, 1             ; 1 такт
    brcc    (PC-1)            ; 2 такта
    ; На последнем шаге brcc выполнится за 1 такт, но с учетом ldi опять-таки
    ; получится 2 такта
.endif
.if (@0)%3 > 0
    nop
.endif
.if (@0)%3 > 1
    nop
.endif
.endif
.ENDMACRO
; ----------
; Задержка с погрешностью на указанное количество тактов
; @0 -- задержка в тактах, @0 = 0...262145
; @1, @2 -- регистры, используемые для реализации задержки
; Величина погрешности (округление всегда в большую сторону):
;   @0 = 1...3   -- нет погрешности;
;   @0 = 4...768 -- до двух тактов;
;   @0 > 768     -- до трех тактов
.MACRO M_DELAY_CLK
.if (@0) <= 0
    ; Если @0 <=0 -- нет герерации задержки
.elif (@0) == 1                 ; Очень короткая задержка
    nop
.elif (@0) == 2
    nop
    nop
.elif (@0) <= 768               ; Короткая задержка
    ldi     @1, (@0-1)/3        ; Округление до кратности 3 в большую сторону
    subi    @1, 1               ; 1 такт
    brcc    (PC-1)              ; 2 такта, итого 1 итерация цикла -- 3 такта
.elif (@0) <= 262145            ; Длинная задержка
    ldi     @1, Low((@0-2)/4)   ; Округление до кратности 4 в большую сторону
    ldi     @2, High((@0-2)/4)  ; "Минус 2" -- компенсация команд ldi
    subi    @1, 1               ; 1 такт
    sbci    @2, 0               ; 1 такт
    brcc    (PC-2)              ; 2 такта, итого 1 итерация цикла -- 4 такта
.else
.error "@0 must be less or equal 262145"
.endif
.ENDMACRO
; ----------
#define M_US2CLK(t) ((XTAL-1)*(t)/1000000+1)
; t -- число микросекунд
; Почему "минус 1" и "плюс 1"? "Плюс 1" значит, что число тактов округляется в
; большую сторону, то есть даже для частоты 1 Гц при t=1 результатом будет
; 1 такт. "Минус 1" нужен для того, чтобы эффекта "плюс 1" не было для "ровных"
; частот, типа 1000000, 2000000, и т.д.
; ----------
; Задержка на указанное число мкс
; Здесь надо указывать регистры
; @0 -- задержка в мкс;
; @1, @2 -- регистры, используемые для реализации задержки
.MACRO M_DELAY_US_R
    M_DELAY_CLK     M_US2CLK(@0), @1, @2
.ENDMACRO

; Задержка на указанное число мкс
; Используются регистры по умолчанию, подставьте удобные Вам
; @0 -- задержка в мкс
.MACRO M_DELAY_US
    M_DELAY_CLK     M_US2CLK(@0), R16, R17
.ENDMACRO

; Точная задержка на указанное число мкс
; @0 -- задержка в мкс;
; @1 -- используемый регистр
.MACRO M_DELAY_US_RA
    M_DELAY_CLK_A   M_US2CLK(@0), @1
.ENDMACRO

; Точная задержка на указанное число мкс
; Используется регистр по умолчанию, подставьте удобный Вам
; @0 -- задержка в мкс
.MACRO M_DELAY_US_A
    M_DELAY_CLK_A   M_US2CLK(@0), R16
.ENDMACRO
; ----------
; Задержка с погрешностью на указанное число мкс с компенсацией тактов
; @0 -- задержка в мкс минус @1 тактов
; @2, @3 -- используемые регистры для реализации задержки
.MACRO M_DELAY_US_RC
    M_DELAY_CLK     M_US2CLK(@0)-@1, @2, @3
.ENDMACRO

; Задержка с погрешностью на указанное число мкс с компенсацией тактов
; Используются регистры по умолчанию, подставьте удобные Вам
; @0 -- задержка в мкс минус @1 тактов
.MACRO M_DELAY_US_C
    M_DELAY_CLK     M_US2CLK(@0)-@1, R16, R17
.ENDMACRO
// ----------------------------------------------------------------
.MACRO START_LCD
	cbi LCD_PORT,LCD_SCE			; начало передачи
.ENDMACRO
// ------------
.MACRO STOP_LCD
	sbi LCD_PORT,LCD_SCE			; конец передачи
.ENDMACRO
// ------------
// ------------
// ------------
// ------------
