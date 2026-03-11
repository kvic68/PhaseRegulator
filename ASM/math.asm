//	----------------------------------------------------------------
// Преобразование 16-битного числа в десятичное 5-позиционное
//	----------------------------------------------------------------
.equ	AtBCDLow	= 10	;address of tBCDLow
.equ	AtBCDHigh	= 12	;address of tBCDHigh

.def	tBCD0	=r10		;BCD value digits 1 and 0
.def	tBCD1	=r11		;BCD value digits 3 and 2
.def	tBCD2	=r12		;BCD value digit 4 5

.def	fbinB0	=r22		;binary value byte 0
.def	fbinB1	=r23		;binary value byte 1

.def	cnt16a	=r20		;loop counter
.def	tmp16a	=r21		;temporary value


A16toBCD5:					push cnt16a
							push tmp16a
							push fbinB0
							push fbinB1
							push tBCD0
							push tBCD1
							push tBCD2
							push ZH
							push ZL
							ldi	cnt16a,16	;Init loop counter	
							clr	tBCD2		;clear result (3 bytes)
							clr	tBCD1		
							clr	tBCD0		
							clr	ZH		

bBCDx_1:					lsl	fbinB0		;shift input value
							rol	fbinB1		;through all bytes
							rol	tBCD0		;
							rol	tBCD1
							rol	tBCD2

							dec	cnt16a		;decrement loop counter
						breq bBCDx_4		;if counter not zero

							ldi		r30,AtBCDHigh+1	;Z points to result MSB + 1

bBCDx_3:					ld		tmp16a,-Z	;get (Z) with pre-decrement
							subi	tmp16a,-$03	;add 0x03
							sbrc	tmp16a,3	;if bit 3 not clear
							st		Z,tmp16a	;	store back
							ld		tmp16a,Z	;get (Z)
							subi	tmp16a,-$30	;add 0x30
							sbrc	tmp16a,7	;if bit 7 not clear
							st		Z,tmp16a	;	store back
							cpi		ZL,AtBCDLow	;done all three?
							brne	bBCDx_3		;loop again if not
						rjmp	bBCDx_1		

bBCDx_4:
							mov  tmp16a,tBCD0
							andi tmp16a,$0F
							ori  tmp16a,$30
							sts (DIGIT0),tmp16a

							mov  tmp16a,tBCD0
							swap tmp16a
							andi tmp16a,$0F
							ori  tmp16a,$30
							sts (DIGIT1),tmp16a

							mov  tmp16a,tBCD1
							andi tmp16a,$0F
							ori  tmp16a,$30
							sts (DIGIT2),tmp16a
							mov  tmp16a,tBCD1

							swap tmp16a
							andi tmp16a,$0F
							ori  tmp16a,$30
							sts (DIGIT3),tmp16a

							pop ZL
							pop ZH
							pop tBCD2
							pop tBCD1
							pop tBCD0
							pop fbinB1
							pop fbinB0
							pop tmp16a
							pop cnt16a
							ret		
//	----------------------------------------------------------------
//
//	----------------------------------------------------------------
.equ	digitsCount 	= 4
.equ	digitsDecimal	= 1
//	----------------------------------------------------------------
HideZeroes:
							push r16
							push r17
							push r18
							push r19
							PUSHZ

							LDZ DIGITS+digitsCount
							clr r18
							ldi r19,digitsCount-digitsDecimal-1	;	два младших нуля не убираем
							ldi r17,'0'

HZCycle:					tst r19
							breq HZCycleEx
							ld r16,-Z
							cp r16,r17				;	если старшая цифра 0, то
							brne HZCycleEx			;		если нет - то дальше ничего делать не надо, на выход
							ldi r16,' '				;	заменяем её на пробел
							inc r18					;	увеличиваем счетчик скрытых
							dec r19
							st Z,r16				;	сохраняем пробел вместо нуля в соответствующей цифре
							rjmp HZCycle

HZCycleEx:					POPZ
							pop r19
							pop r18
							pop r17
							pop r16
							ret
//	----------------------------------------------------------------
; Умножение R7..R6 * R9..R8 = R13..12..11..10
MUL1616:					push r0
							push r1

							mul  r7,r9
							movw r12,r0

							mul  r6,r8
							movw r10,r0

							mul r6,r9
							add r11,r0
							adc r12,r1
							clr r0
							adc r13,r0

							mul r7,r8
							add r11,r0
							adc r12,r1
							clr r0
							adc r13,r0

							pop r1
							pop r0
							ret 
//	----------------------------------------------------------------
; 32bit/32bit Unsigned Division
;
; Register Variables
;  Call:  var1[3:0] = dividend (0x00000000..0xffffffff)
;         var2[3:0] = divisor (0x00000001..0x7fffffff)
;         mod[3:0]  = <don't care>
;         r31        = <don't care> (high register must be allocated)
;
;  Result:var1[3:0] = var1[3:0] / var2[3:0]
;         var2[3:0] = <not changed>
;         mod[3:0]  = var1[3:0] % var2[3:0]
;         r31        = 0
;
; Size  = 26 words
; Clock = 549..677 cycles (+ret)
; Stack = 0 bytes
; --- делимое/результат
//.def r6	= r6	; младший разряд
//.def r7	= r7
//.def r8	= r8
//.def r9	= r9	;
; --- делитель
//.def r10	= r10	; младший разряд
//.def r11	= r11
//.def r12	= r12
//.def r13	= r13
; --- остаток
//.def mod0	= r2	; младший разряд
//.def mod1	= r3
//.def mod2	= r4
//.def mod3	= r5

//.def r31		= r31

div32u:						push r31
							push r2
							push r3
							push r4
							push r5

							clr	r2		;initialize variables
							clr	r3		;  mod = 0;
							clr	r4		;  r31 = 32;
							clr	r5		;
							ldi	r31,32		;/
				;---- calcurating loop
div32cycle:					lsl	r6		;var1 = var1 << 1;
							rol	r7		;
							rol	r8		;
							rol	r9		;/
							rol	r2		;mod = mod << 1 + carry;
							rol	r3		;
							rol	r4		;
							rol	r5		;/
							cp	r2,r10	;if (mod => var2) {
							cpc	r3,r11	; mod -= var2; var1++;
							cpc	r4,r12	; }
							cpc	r5,r13	;
							brcs div32skip	//	PC+6		;
							inc	r6		;
							sub	r2,r10	;
							sbc	r3,r11	;
							sbc	r4,r12	;
							sbc	r5,r13	;/
div32skip:					dec	r31		;if (--r31 > 0)
							brne div32cycle	//	PC-19		; continue loop;
					//---
							lsl r2
							rol r3
							rol r4
							rol r5
							cp  r2,r10
							cpc r3,r11
							cpc r4,r12
							cpc r5,r13
							brcs div32ex

							ldi r31,1
							add r6,r31
							ldi r31,0
							adc r7,r31
							adc r8,r31
							adc r9,r31
					//---
div32ex:					pop r5
							pop r4
							pop r3
							pop r2
							pop r31
							ret
//	----------------------------------------------------------------

//	----------------------------------------------------------------
; https://radiokot.ru/forum/viewtopic.php?p=779071#p779071
//	----------------------------------------------------------------
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

sqr:
							push r10
							push r11
							push r12
							push r13
							push r14
							push r15
							push r16
							push r17
							push r18
							push r19
							push r20
							push r21
							push r22
							push r23
							;
							clr	r12				;  unsigned long mask = 0x40000000; 
							clr	r13
							clr	r14
							ldi	r16,0x40
							mov	r15,r16
							;
							clr	r8				;  unsigned long sqr = 0;   
							clr	r9
							clr	r10
							clr	r11
		
sqr32loop:
							movw r16,r8		;temp = sqr | mask; 
							movw r18,r10
							or   r16,r12
							or   r17,r13
							or   r18,r14
							or   r19,r15
							lsr	r11			;sqr >>= 1; 
							ror	r10
							ror	r9
							ror	r8
							;
							cp	r20,r16		;if( temp <= ul )
							cpc	r21,r17
							cpc	r22,r18
							cpc	r23,r19

							; Если С=0, то условие выполнено

							brcs sqr32_skipif		; пропустить, если С=1
							;							{
							or	r8,	r12		; sqr |= mask
							or	r9,	r13
							or	r10,r14
							or	r11,r15
							;
							sub	r20,r16 	; ul -= temp; 
							sbc	r21,r17
							sbc	r22,r18
							sbc	r23,r19
							;							}
sqr32_skipif:
							lsr	r15			;( mask >>= 2 )
							ror	r14
							ror	r13
							ror	r12
							;
							lsr	r15
							ror	r14
							ror	r13
							ror	r12
							;
							mov	r16,r12   	; mask=0?
							or	r16,r13
							or	r16,r14
							or	r16,r15
							;
							brne sqr32loop	;while( mask=0 )
		;
							pop r23
							pop r22
							pop r21
							pop r20
							pop r19
							pop r18
							pop r17
							pop r16
							pop r15
							pop r14
							pop r13
							pop r12
							pop r11
							pop r10

							ret
//	----------------------------------------------------------------
