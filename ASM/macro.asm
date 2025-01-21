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
		; ������� ������ � ��������� 
		; http://easyelectronics.ru/avr-uchebnyj-kurs-vazhnye-melochi-1.html

RAM_Flush:	ldi	ZL,Low(SRAM_START)	; ����� ������ ���
			ldi	ZH,High(SRAM_START)
			clr	R16					; ������� R16
Flush:		st 	Z+,R16				; ��������� 0 � ������ ������
			cpi	ZH,High(RAMEND+1)	; �������� ����� ��� ?
			brne	Flush			; ���? �������� ������!
 
			cpi	ZL,Low(RAMEND+1)	; ������� ���� ������ �����?
			brne	Flush
			st -Z,r16
		; ����������� ����
			out spl,ZL
			out sph,ZH
 
			ldi	ZL, 30				; ����� ������ �������� ��������	
			clr	ZH					; ����� ����� 0
			dec	ZL					; �������� �����,
			st	Z, ZH				; ���������� � ������� 0,
			brne	PC-2			; ���� �� ��������� �� � �����������.
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
;	������� ��������� ��������
// ----------------------------------------------------------------
; http://forum.avr.ru/showthread.php?goto=nextoldest&t=36050
; ������ �������� �� ��������� ���������� ������
; @0 -- ���������� ������, @0 = 0...770
; @1 -- ������������ ������� ��� ���������� ��������
.MACRO M_DELAY_CLK_A
.if (@0) > 0                  ; ���� @0 <= 0 -- ��� ��������� ��������
.if (@0) > 770                ; ������ 770 ������ ������, ������� ������
.error "@0 must be less or equal 770"
.endif
.if (@0)/3 > 0                ; ����� �� 3, �.�. 1 �������� ����� -- 3 �����
    ldi     @1, (@0-3)/3
    subi    @1, 1             ; 1 ����
    brcc    (PC-1)            ; 2 �����
    ; �� ��������� ���� brcc ���������� �� 1 ����, �� � ������ ldi �����-����
    ; ��������� 2 �����
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
; �������� � ������������ �� ��������� ���������� ������
; @0 -- �������� � ������, @0 = 0...262145
; @1, @2 -- ��������, ������������ ��� ���������� ��������
; �������� ����������� (���������� ������ � ������� �������):
;   @0 = 1...3   -- ��� �����������;
;   @0 = 4...768 -- �� ���� ������;
;   @0 > 768     -- �� ���� ������
.MACRO M_DELAY_CLK
.if (@0) <= 0
    ; ���� @0 <=0 -- ��� ��������� ��������
.elif (@0) == 1                 ; ����� �������� ��������
    nop
.elif (@0) == 2
    nop
    nop
.elif (@0) <= 768               ; �������� ��������
    ldi     @1, (@0-1)/3        ; ���������� �� ��������� 3 � ������� �������
    subi    @1, 1               ; 1 ����
    brcc    (PC-1)              ; 2 �����, ����� 1 �������� ����� -- 3 �����
.elif (@0) <= 262145            ; ������� ��������
    ldi     @1, Low((@0-2)/4)   ; ���������� �� ��������� 4 � ������� �������
    ldi     @2, High((@0-2)/4)  ; "����� 2" -- ����������� ������ ldi
    subi    @1, 1               ; 1 ����
    sbci    @2, 0               ; 1 ����
    brcc    (PC-2)              ; 2 �����, ����� 1 �������� ����� -- 4 �����
.else
.error "@0 must be less or equal 262145"
.endif
.ENDMACRO
; ----------
#define M_US2CLK(t) ((XTAL-1)*(t)/1000000+1)
; t -- ����� �����������
; ������ "����� 1" � "���� 1"? "���� 1" ������, ��� ����� ������ ����������� �
; ������� �������, �� ���� ���� ��� ������� 1 �� ��� t=1 ����������� �����
; 1 ����. "����� 1" ����� ��� ����, ����� ������� "���� 1" �� ���� ��� "������"
; ������, ���� 1000000, 2000000, � �.�.
; ----------
; �������� �� ��������� ����� ���
; ����� ���� ��������� ��������
; @0 -- �������� � ���;
; @1, @2 -- ��������, ������������ ��� ���������� ��������
.MACRO M_DELAY_US_R
    M_DELAY_CLK     M_US2CLK(@0), @1, @2
.ENDMACRO

; �������� �� ��������� ����� ���
; ������������ �������� �� ���������, ���������� ������� ���
; @0 -- �������� � ���
.MACRO M_DELAY_US
    M_DELAY_CLK     M_US2CLK(@0), R16, R17
.ENDMACRO

; ������ �������� �� ��������� ����� ���
; @0 -- �������� � ���;
; @1 -- ������������ �������
.MACRO M_DELAY_US_RA
    M_DELAY_CLK_A   M_US2CLK(@0), @1
.ENDMACRO

; ������ �������� �� ��������� ����� ���
; ������������ ������� �� ���������, ���������� ������� ���
; @0 -- �������� � ���
.MACRO M_DELAY_US_A
    M_DELAY_CLK_A   M_US2CLK(@0), R16
.ENDMACRO
; ----------
; �������� � ������������ �� ��������� ����� ��� � ������������ ������
; @0 -- �������� � ��� ����� @1 ������
; @2, @3 -- ������������ �������� ��� ���������� ��������
.MACRO M_DELAY_US_RC
    M_DELAY_CLK     M_US2CLK(@0)-@1, @2, @3
.ENDMACRO

; �������� � ������������ �� ��������� ����� ��� � ������������ ������
; ������������ �������� �� ���������, ���������� ������� ���
; @0 -- �������� � ��� ����� @1 ������
.MACRO M_DELAY_US_C
    M_DELAY_CLK     M_US2CLK(@0)-@1, R16, R17
.ENDMACRO
// ----------------------------------------------------------------
.MACRO START_LCD
	cbi LCD_PORT,LCD_SCE			; ������ ��������
.ENDMACRO
// ------------
.MACRO STOP_LCD
	sbi LCD_PORT,LCD_SCE			; ����� ��������
.ENDMACRO
// ------------
// ------------
// ------------
// ------------
