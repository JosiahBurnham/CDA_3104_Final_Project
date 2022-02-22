;----------------------------------------------------------
; File    : fibonacci.asm
; Author  : J.Burnham, T.Beam, C.Apostoli
; Purpose : To display all the fibonacci number that fit in a byte, onto a LCD display
;----------------------------------------------------------


; Global Defines
;----------------------------------------------------------

; LCD Defines
.equ LCD_DPRT = PORTD         ; LCD data port
.equ LCD_DDDR = DDRD          ; Dir reg for data
.equ LCD_DPIN = PIND          ; LCD data Pin
.equ LCD_CPRT = PORTB         ; LCD command PORT
.equ LCD_CDDR = DDRB          ; LCD Command DDR
.equ LCD_CPIN = PINB          ; LCD command pin
.equ LCD_RS   = 0
.equ LCD_RW   = 1
.equ LCD_EN   = 2


; Fib Sequence Defines
.def RESULT_LOW  = r0         ; Result of Fib Calc 
.def RESULT_HIGH = r1
.def FIRST_LOW = r2           ; First number of Fib Seq
.def FIRST_HIGH = r3
.def SECOND_LOW = r4          ; Second Number of Fib Seq
.def SECOND_HIGH = r5

; Division Algorithm Defines
.def TEMP_LOW = r7            ; temp var for division algorithm
.def TEMP_HIGH = r8

.DEF VALUE_LOW  = R10         ; value of to be divided by in division algoritm
.DEF VALUE_HIGH = R11       
.DEF MOD10_LOW  = R12         ; the value of the remainder in divistion algorithm
.DEF MOD10_HIGH = R13
.def ZERO = r9                ; used as the value zero

; Storage Defines
.equ FIRST_NUM = $0100        ; THE FIRST NUMBER OF THE STORED CALCULATED FIB NUMBER (MOST SIGNIFICANT NUM)
.equ LAST_NUM = $0105         ; THE LAST NUMBER OF THE STORED CALCULATED FIB NUMBER (LEAST SIGNIFICANT NUM)

; Macros
;----------------------------------------------------------
.MACRO __INIT_STACK
ldi r16, high(RAMEND)
out SPH, r16
ldi r16, low(RAMEND)
out SPL, r16
clr r16
.ENDMACRO

; Vector Table
; ---------------------------------------------------------
.org 0x00                     ; reset
     jmp  main
 
.org PCI1addr                 ; Pin Change Interrupt Request 1
     jmp  pci1ISR
 
.org INT_VECTORS_SIZE         ; end of v-table
 

; ---------------------------------------------------------
Main:
     __INIT_STACK

     call INIT_FIB            ; initialize the first two number of the fib sequence
     call INIT_BUTTONS        ; setup the GPIO for the buttons
     call INIT_INTERUPTS      ; setup pin-change interupts for the buttons
     call DISPLAY_INIT        ; initialize the display

End_Main:  rjmp End_Main

;####################################################################
; SUBROUTINES
;####################################################################

; LCD SUBROUTINES
;==========================================================

DISPLAY_INIT:
;----------------------------------------------------------
; Initialize the display to turn on and print a welcome message
;----------------------------------------------------------

     ldi  r21, 0xFF;
     out  LCD_DDDR, r21  ; LCD data port is output
     out  LCD_CDDR, r21  ; LCD command port is output
     cbi  LCD_CPRT,LCD_EN
     call DELAY_2ms      ; wait for turn on
     call DELAY_2ms      ; wait for tunr on

     ldi  r16, 0x38      ; init LCD 2 lines, 5X7 matrix
     call CMNDWRT
     call DELAY_2ms

     ldi  r16, 0x0E      ; turn cursor on
     call CMNDWRT
     call DELAY_2ms

     ldi  r16, 0x01      ; clear LCD
     call CMNDWRT
     call DELAY_2ms

     ldi  r16, 0x06      ; shift LCD to the right
     call CMNDWRT

     ldi  r16, 'F'
     call DATAWRT
     ldi  r16, 'I'
     call DATAWRT
     ldi  r16, 'B'
     call DATAWRT
     ldi  r16, 'O'
     call DATAWRT
     ldi  r16, 'N'
     call DATAWRT
     ldi  r16, 'A'
     call DATAWRT
     ldi  r16, 'C'
     call DATAWRT
     ldi  r16, 'C'
     call DATAWRT
     ldi  r16, 'I'
     call DATAWRT

     ldi  r16, $C0            ; Strart on next LCD ROW
     call CMNDWRT
     call DELAY_2ms

     ldi  r16, 'S'
     call DATAWRT
     ldi  r16, 'E'
     call DATAWRT
     ldi  r16, 'Q'
     call DATAWRT
     ldi  r16, 'U'
     call DATAWRT
     ldi  r16, 'E'
     call DATAWRT
     ldi  r16, 'N'
     call DATAWRT
     ldi  r16, 'C'
     call DATAWRT
     ldi  r16, 'E'
     call DATAWRT

     ret

CMNDWRT:
;----------------------------------------------------------
; Write a command from r16 to the LCD
;----------------------------------------------------------
     out  LCD_DPRT,R16        ; LCD data port = r16
     cbi  LCD_CPRT,LCD_RS     ; RS=0 for a command
     cbi  LCD_CPRT,LCD_RW     ; RS=0 for write
     sbi  LCD_CPRT,LCD_EN     ; EN = 1
     call SDELAY              ; make a wide EN pulse
     cbi  LCD_CPRT,LCD_EN     ; High to low pulse
     call DELAY_100us         ; wait 100 us
     ret

DATAWRT:
;----------------------------------------------------------
; Write data to LCD from r16, an ACII char
;----------------------------------------------------------
     out  LCD_DPRT,r16
     sbi  LCD_CPRT,LCD_RS
     cbi  LCD_CPRT,LCD_RW
     sbi  LCD_CPRT,LCD_EN
     call SDELAY
     cbi  LCD_CPRT,LCD_EN
     call DELAY_100us
     ret

GEN_WHITESPACE:
;----------------------------------------------------------
; USING R17 AS THE AMMOUNT OF WHITESPACE, GENERATE WHITESPACE TO DISPLAY
;----------------------------------------------------------
IF_WHITESPACE:
     ldi r16, ' '
     call DATAWRT
     dec r17
     brne IF_WHITESPACE
     ret

     OVERFLOW_DISP:
;----------------------------------------------------------
; The LCD Display for when an overflow occurrs
;----------------------------------------------------------

     ldi  r16, 0x01      ; clear LCD
     call CMNDWRT
     call DELAY_2ms


     ldi  r16, 'O'
     call DATAWRT
     ldi  r16, 'V'
     call DATAWRT
     ldi  r16, 'E'
     call DATAWRT
     ldi  r16, 'R'
     call DATAWRT
     ldi  r16, 'F'
     call DATAWRT
     ldi  r16, 'L'
     call DATAWRT
     ldi  r16, 'O'
     call DATAWRT
     ldi  r16, 'W'
     call DATAWRT
     ldi  r16, '!'
     call DATAWRT
     call DELAY_200ms

ret


; DELAY SUBROUTINES
;==========================================================

SDELAY:
;----------------------------------------------------------
; Simple delay
;----------------------------------------------------------
     nop
     nop
     ret

DELAY_100us:
;----------------------------------------------------------
; 100 micro second delay
;----------------------------------------------------------
     push r20
     
     ldi  r20, $E7            ; 256 - (100 us with prescaler of 64)
     out  TCNT0, r20

     clr  r20
     out  TCCR0A, r20         ; normal mode

     ldi  r20, (1<<CS01) | (1 << CS00)
     out  TCCR0B, r20         ; normal mode / prescaler of 64
DR0:
     sbis TIFR0, TOV0
     rjmp DR0

     clr  r20
     out  TCCR0B, r20         ; stop timer
     
     sbi  TIFR0, TOV0         ; clrea flag 

     pop r20
     ret

DELAY_2ms:
;----------------------------------------------------------
; 2 milisecond delay
;----------------------------------------------------------
     push r17
     ldi  r17,20
LDR0:
     call DELAY_100us
     dec  r17
     brne LDR0
     pop r17
     ret


DELAY_200ms:
;----------------------------------------------------------
; 200 milisecond delay
;----------------------------------------------------------
     push r17
     ldi r17, 100
DL200:
     call DELAY_2ms
     dec r17
     brne DL200
     pop r17
     ret
; GPIO SUBROUTINES
;==========================================================
INIT_BUTTONS:
;----------------------------------------------------------
; General setup for two buttons 
;----------------------------------------------------------
     cbi  DDRC, DDC0          ; input
     cbi  DDRC, DDC1
     sbi  PORTC, PC0          ; with pullup
     sbi  PORTC, PC1
     ret

; INTERRUPT SUBROUTINES
;==========================================================

INIT_INTERUPTS:
;----------------------------------------------------------
; initialize pin-change intterupts on PC0, and PC1
;----------------------------------------------------------
     push r20
     ; set up pin chnage interupts for on and off buttons
     ldi  r20, (1<<PCINT9) | (1<<PCINT8)
     sts  PCMSK1, r20    ; set up PORTC pins 0 and 1 for interrupts

     ldi r20, (1<<PCIE1) ; set PORTC for intterupts
     sts PCICR, r20
     
     sei                 ; enable global intterupts

     pop r20
     ret

pci1ISR:
;----------------------------------------------------------
; PORTC Intterupt routine
;----------------------------------------------------------

     in r20, PINC

     ; first check the next button PC1
     ldi  r21, (1<< PC1)
     and  r20, r21

     clr  r0
     cp   r20,r0
     breq D1_LOW

     in r20, PINC

     ; second check the off button PC0
     ldi  r21, (1<< PC0)
     and  r20, r21

     clr  r0
     cp   r20,r0
     breq D0_LOW
     
     rjmp END_LOGIC
     
D0_LOW:

     call DISPLAY_INIT
     call INIT_FIB            ; start the sequence over
     rjmp END_LOGIC

D1_LOW:
     call DELAY_200ms
     call FIB_CALL

     rjmp END_LOGIC


END_LOGIC:
     reti
 
; Fibonacci Subroutines
;==========================================================

CALC_FIB:
;----------------------------------------------------------
; Calculate the next fibonacci number
;----------------------------------------------------------

     add  RESULT_LOW, FIRST_LOW
     adc  RESULT_HIGH, FIRST_HIGH

     add  RESULT_LOW, SECOND_LOW
     adc  RESULT_HIGH, SECOND_HIGH

     mov FIRST_LOW,SECOND_LOW
     mov FIRST_HIGH, SECOND_HIGH

     mov SECOND_LOW, RESULT_LOW
     mov SECOND_HIGH, RESULT_HIGH

     ret

STORE_NUM:
;----------------------------------------------------------
; Divides the calculated Fibonacci number into 5 of it's decimal value palces
; and stores each place individually
;----------------------------------------------------------
     CLR ZERO                 ; MAKE ZERO ZERO

     ; initialize VALUE to the number we want to convert
     MOV  R16, RESULT_HIGH
     MOV  VALUE_HIGH, R16

     MOV  R16, RESULT_LOW
     MOV VALUE_LOW, R16

     ; setup the array in which the digits will be stored in ascii format
     LDI XH, HIGH(LAST_NUM)
     LDI XL, LOW(LAST_NUM)
     
     LDI R16, 5

DIVIDE:
     ; initialize the remainder to be zero
     CLR  MOD10_HIGH
     CLR  MOD10_LOW
     CLC                      ; CLEAR THE CARRY BIT
     
     LDI R17, 16              ; LOOP 16 TIMES

DIVLOOP:

     ; rotate quotient and remainder, with the carry flag
     ROL  VALUE_LOW
     ROL  VALUE_HIGH
     ROL  MOD10_LOW
     ROL  MOD10_HIGH
     
     MOV TEMP_LOW, MOD10_LOW        ; temp storage for mod10_low
     MOV TEMP_HIGH, MOD10_HIGH       ; temp storage for mod10_high

     ; R20, R21 = dividend - divisor    
     SEC
     LDI  R18, 10
     SUB  TEMP_LOW, R18
     SBC  TEMP_HIGH, ZERO

     CP   R18, TEMP_LOW             
     BRLO IGNORE_RESULT       ; BRANCH IF DIVIDEND_LOW < DIVISOR
     CP   TEMP_HIGH, ZERO
     BRNE IGNORE_RESULT       ; IF THERE IS ANYTHING IN DIVIDEND_HIGH IT IS LARGER THAN THE DIVISOR

     MOV MOD10_LOW, TEMP_LOW
     MOV MOD10_HIGH, TEMP_HIGH
     RJMP SAVE_RESULT

IGNORE_RESULT:
     CLC
     RJMP DEC_LOOP_CTR

SAVE_RESULT:
     SEC

DEC_LOOP_CTR:
     DEC R17
     BRNE DIVLOOP

     MOV TEMP_LOW, MOD10_LOW
     ROL  VALUE_LOW
     ROL  VALUE_HIGH

     LDI  R18, $30           ; THE ASCII NUMBER FOR 0
     ADD  TEMP_LOW, R18             ; MAKE THE VALUE_LOW AN ASCII DIGIT
     ; STORE VALUE TO ARRAY
     ST -X, TEMP_LOW


     DEC R16
     BRNE DIVIDE    ; BRANCH IF VALUE NOT ZERO

     clr  RESULT_LOW
     clr  RESULT_HIGH

     ret

OUTPUT_NUM:

     ldi  r16, 0x01      ; clear LCD
     call CMNDWRT
     call DELAY_2ms

     ldi r17, 5
IF_ELEMENTS:
     LD   r16, X+
     call DATAWRT
     dec r17
     brne IF_ELEMENTS
     ret

INIT_FIB:
;----------------------------------------------------------
; initialize the first two numbers of the fibonacci sequence
;----------------------------------------------------------

     clr FIRST_LOW
     clr FIRST_HIGH
     ldi r16, 1
     mov SECOND_LOW, r16
     clr SECOND_HIGH
     ret
     
FIB_CALL:
;----------------------------------------------------------
; The logic to handle the calculation, storage, and display of the 
; fibonacci sequence
;----------------------------------------------------------

     call CALC_FIB
     BRCS OVERFLOW            ; if the carry flag is set, the addtion is greater than a two byte value
     call STORE_NUM
     call OUTPUT_NUM
     rjmp END_CALL

OVERFLOW:
     call STORE_NUM           ; you have to call STORE NUM here or else it messes with what's in the registers
     call OVERFLOW_DISP
     call INIT_FIB
END_CALL:
     ret

