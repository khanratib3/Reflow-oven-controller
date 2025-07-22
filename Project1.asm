; This program is an Oven Reflow Controller
; ELEC 291 B11
; Alvin Li, Ben Coronado, David Park, Julia Wadey, Ki Broome, Ratib Khan

$NOLIST
$MODN76E003
$LIST

;  N76E003 pinout:
;                               -------
;       PWM2/IC6/T0/AIN4/P0.5 -|1    20|- P0.4/AIN5/STADC/PWM3/IC3
;               TXD/AIN3/P0.6 -|2    19|- P0.3/PWM5/IC5/AIN6
;               RXD/AIN2/P0.7 -|3    18|- P0.2/ICPCK/OCDCK/RXD_1/[SCL]
;                    RST/P2.0 -|4    17|- P0.1/PWM4/IC4/MISO
;        INT0/OSCIN/AIN1/P3.0 -|5    16|- P0.0/PWM3/IC3/MOSI/T1
;              INT1/AIN0/P1.7 -|6    15|- P1.0/PWM2/IC2/SPCLK
;                         GND -|7    14|- P1.1/PWM1/IC1/AIN7/CLO
;[SDA]/TXD_1/ICPDA/OCDDA/P1.6 -|8    13|- P1.2/PWM0/IC0
;                         VDD -|9    12|- P1.3/SCL/[STADC]
;            PWM5/IC7/SS/P1.5 -|10   11|- P1.4/SDA/FB/PWM1
;                               -------
;

CLK               EQU 16600000 ; Microcontroller system frequency in Hz
BAUD              EQU 115200 ; Baud rate of UART in bps
TIMER1_RELOAD     EQU (0x100-(CLK/(16*BAUD)))
TIMER0_RELOAD_1MS EQU (0x10000-(CLK/1000))
TIMER2_RATE       EQU 1000
TIMER2_RELOAD     EQU ((65536)-(CLK/TIMER2_RATE))
SampleSize        EQU 250 ; amount of readings
SampleSpeed       EQU 1 ; number of time between gathered samples.
TIMER0_RATE   EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD EQU ((65536-(CLK/TIMER0_RATE)))
ORG 0x0000
	ljmp main

    
; External interrupt 0 vector (not used in this code)
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
	ljmp Timer0_ISR
ORG 0x002B
    ljmp Timer2_ISR


;                     1234567890123456    <- This helps determine the location of the counter
clear_message: db    '                ', 0
preheat_soak_message: db 'PREHEAT/SOAK' , 0
test_message:     db 'State        ', 0
value_message:    db 'Temp.        ', 0
reset_message:    db 'PRESS TO START', 0
cooling_message:  db 'COOLING', 0
ramp_to_peak_message: db 'RAMP TO PEAK', 0
ramp_to_soak_message: db 'RAMP TO SOAK', 0
time_message: db 'time is', 0
reflow_message: db 'refl:', 0
soak_message: db 'soak:', 0
reflow_long_message: db 'reflow' , 0
finish_message: db 'process is done!',0
finish_message2: db '-by ELEC 291 B11',0

cseg
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3
PB6 equ P1.6
ABORT_BUTTON equ P1.2
SOUND_OUT equ P0.5
PWM_OUT equ p1.0


$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$LIST

; These register definitions needed by 'math32.inc'
;DSEG at 30H
DSEG at 0x30

; Variables
pwm_counter:  ds 1 ; Free running counter 0, 1, 2, ..., 100, 0
pwm:          ds 2 ; pwm percentage
seconds:      ds 1 ; a seconds counter attached to Timer 2 ISR
x:   ds 4
y:   ds 4
state: ds 1
temp_soak: ds 1
time_soak: ds 1
temp_refl: ds 1
time_refl: ds 1
temp_cool: ds 1
FSM1_state: ds 1
sec: ds 2
thot: ds 4
bcd: ds 5
VAL_LM4040: ds 2
temp: ds 4
temp_new: ds 1
Count1ms: ds 2
a_store: ds 1
sum: ds 4
counter: ds 1

; Flags
BSEG
mf: dbit 1
temp_mode: dbit 1 ; 0 = fahrenheit, 1 = celsius
seconds_flag: dbit 1
s_flag: dbit 1; pwm seconds flag

; Push buttons
pb_temp_soak: dbit 1		; push button for setting soak TEMP
pb_soak_time: dbit 1		; push button for setting soak TIME
pb_temp_refl: dbit 1		; push button for setting refl TEMP
pb_refl_time: dbit 1		; push button for setting refl TIME
pb_free_for_now: dbit 1		; FREE BUTTON FOR NOW



$NOLIST
$include(math32.inc)
$LIST

;---------------------------------;
; intialize for timer 0           ;
;---------------------------------;
Timer0_Init:
	orl CKCON, #0b00001000 ; Input for timer 0 is sysclk/1
	mov a, TMOD
	anl a, #0xf0 ; 11110000 Clear the bits for timer 0
	orl a, #0x01 ; 00000001 Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    ;setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
ret


;---------------------------------;
; ISR for timer 0.  Set to execute;
; every 1/4096Hz to generate a    ;
; 2048 Hz wave at pin SOUND_OUT   ;
;---------------------------------;
Timer0_ISR:
	; Timer 0 doesn't have 16-bit auto-reload, so
	clr TR0
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	setb TR0
	cpl SOUND_OUT ; Connect speaker the pin assigned to 'SOUND_OUT'!
reti

;---------------------------------;
; intialize for timer 2           ;
;---------------------------------;
Timer2_Init:
    mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
    mov TH2, #high(TIMER2_RELOAD)
    mov TL2, #low(TIMER2_RELOAD)
    ; Set the reload value

    orl T2MOD, #0x80 ; Enable timer 2 autoreload
    ;mov T2MOD, #0b1010_0000 ;--changed above line of code Enable timer 2 autoreload, and clock divider is 16

    mov RCMP2H, #high(TIMER2_RELOAD)
    mov RCMP2L, #low(TIMER2_RELOAD)
    ; Init One millisecond interrupt counter.  It is a 16-bit variable made with two 8-bit parts
    clr a
    mov Count1ms+0, a
    mov Count1ms+1, a

    mov pwm_counter, #0
    ; Enable the timer and interrupts
    orl EIE, #0x80 ; Enable timer 2 interrupt ET2=1
    setb TR2  ; Enable timer 2
ret

;---------------------------------;
; ISR for timer 2                 ;
;---------------------------------;

Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in the ISR.  It is bit addressable.
	cpl P0.4 ; To check the interrupt rate with oscilloscope. It must be precisely a 1 ms pulse.
	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw
	
    ; Logic for incrementing seconds
	; Increment the 16-bit one mili second counter
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1

Inc_Done:
    clr C
    mov a , pwm + 0
    subb a, Count1ms+0
    mov a, pwm + 1
    subb a, Count1ms+1

    mov PWM_OUT, c
	; Check if half second has passed
	mov a, Count1ms+0
	cjne a, #low(1000), Timer2_ISR_done ; Warning: this instruction changes the carry flag! if not 1 second the isr is done and can procced
	mov a, Count1ms+1
	cjne a, #high(1000), Timer2_ISR_done
	
	; 1000 milliseconds have passed.  Set a flag so the main program knows
	setb seconds_flag ; Let the main program know half second had passed
	cpl TR0 ;Enable/disable timer/counter 0. This line creates a beep-silence-beep-silence sound.
	; Reset to zero the milli-seconds counter, it is a 16-bit variable
    
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Increment the BCD counter
	mov a, sec
	add a, #0x01

	sjmp Timer2_ISR_da

Timer2_ISR_da:
	da a ; Decimal adjust instruction.  Check datasheet for more details!
	mov sec, a
		
Timer2_ISR_done:
	pop psw
	pop acc
reti


Init_All:
	; Configure all the pins for biderectional I/O
	mov	P3M1, #0x00
	mov	P3M2, #0x00
	mov	P1M1, #0x40
	mov	P1M2, #0x00
	mov	P0M1, #0x00
	mov	P0M2, #0x00
	
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD ; TH1=TIMER1_RELOAD;
	setb TR1
	
	; Using timer 0 for delay functions.  Initialize here:
	clr	TR0 ; Stop timer 0
	orl	CKCON,#0x08 ; CLK is the input for timer 0
	anl	TMOD,#0xF0 ; Clear the configuration bits for timer 0
	orl	TMOD,#0x01 ; Timer 0 in Mode 1: 16-bit timer
	
	; Initialize the pins used by the ADC (P1.1, P1.7) as input.
	orl	P1M1, #0b10000010
	anl	P1M2, #0b01111101
	; Initialize the pins used by the ADC (P0.4) as input.
	orl P0M1, #0b00010000
	anl P0M2, #0b11101111
	; Initialize and start the ADC:
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	; AINDIDS select if some pins are analog inputs or digital I/O:
	mov AINDIDS, #0x00 ; Disable all analog inputs
	orl AINDIDS, #0b10100001 ; Activate AIN0 and AIN7 and AIN5 analog inputs
	orl ADCCON1, #0x01 ; Enable ADC

    lcall Timer2_Init
	
    Set_Cursor(1,13)
    Display_BCD(sec +1)
    Display_BCD(sec +0)
    setb seconds_flag
    mov sec, #0
    mov pwm, #0
    setb EA

	; Initial temp/time soak/reflow
	mov temp_cool, #60
    mov temp_soak, #150
    mov temp_refl, #220
    
    mov time_soak, #75
    mov time_refl, #55

	; temporary value register
	mov a_store, #0x00
ret
	
wait_1ms:
	clr	TR0 ; Stop timer 0
	clr	TF0 ; Clear overflow flag
	mov	TH0, #high(TIMER0_RELOAD_1MS)
	mov	TL0,#low(TIMER0_RELOAD_1MS)
	setb TR0
	jnb	TF0, $ ; Wait for overflow
ret

; Wait the number of miliseconds in R2
waitms:
	lcall wait_1ms
	djnz R2, waitms
ret

; We can display a number any way we want.  In this case with
; four decimal places.
Display_formated_BCD:
    Display_BCD(bcd+2)
	Display_BCD(bcd+1)
	Display_char(#'.')
	Display_BCD(bcd+0)
ret

;===================================;
;									;
;		LCD Push button logic		;
;									;
;===================================;

LCD_PB:
	; Set variables to 1: 'no push button pressed'
	setb pb_temp_soak
	setb pb_soak_time
	setb pb_temp_refl
	setb pb_refl_time
	setb pb_free_for_now
	; The input pin used to check set to '1'
	setb P1.5
	
	; Check if any push button is pressed
	clr P0.0
	clr P0.1
	clr P0.2
	clr P0.3
	clr P1.3
	jb P1.5, LCD_PB_Done

	; Debounce
    Wait_Milli_Seconds(#35)
	jb P1.5, LCD_PB_Done

	; Set the LCD data pins to logic 1
	setb P0.0
	setb P0.1
	setb P0.2
	setb P0.3
	setb P1.3
	
	; Check the push buttons one by one
	clr P1.3
	mov c, P1.5

	mov pb_free_for_now, c
	setb P1.3

	clr P0.0
	mov c, P1.5
	mov pb_refl_time, c
	setb P0.0
	
	clr P0.1
	mov c, P1.5
	mov pb_temp_refl, c
	setb P0.1
	
	clr P0.2
	mov c, P1.5
	mov pb_soak_time, c
	setb P0.2
	
	clr P0.3
	mov c, P1.5
	mov pb_temp_soak, c
	setb P0.3

LCD_PB_Done:		
ret

state_0_check:
    mov a_store, a
	check_temp_soak:
	clr a
	jb pb_temp_soak, check_soak_time
	
	
	mov a, temp_soak
	add a, #0x01
	mov temp_soak, a
    lcall display_state_0_variables
	clr a

	check_soak_time:
    clr a
	jb pb_soak_time, check_refl_temp
	mov a, time_soak
	add a, #0x01
	mov time_soak, a
    lcall display_state_0_variables
	clr a

    check_refl_temp:
    clr a
	jb pb_temp_refl, check_time_reflow
	mov a, temp_refl
	add a, #0x01
	mov temp_refl, a
    lcall display_state_0_variables
	clr a

    check_time_reflow:
    clr a
	jb pb_refl_time, finish
	mov a, time_refl
	add a, #0x01
	mov time_refl, a
    lcall display_state_0_variables
	clr a

finish:
    mov a, a_store
    mov a_store, #0x00
ret

display_state_0_variables:
	Set_Cursor(1,1)

	; Clear x
	mov x + 0, #0
	mov x + 1, #0
	mov x + 2, #0
	mov x + 3, #0

	; Insert soak message string
	mov dptr, #soak_message
	lcall ?Send_Constant_String

	; Insert soak time value
	mov x, time_soak
	lcall hex2bcd
	Display_BCD(BCD + 1)
	Display_BCD(BCD + 0)
	Display_char(#'s')
	Display_char(#' ')

	; Insert soak temp value
	mov x, temp_soak
	lcall hex2bcd
	Display_BCD(BCD + 1)
	Display_BCD(BCD + 0)
	Display_char(#'C')

	; Insert reflow message string
	Set_Cursor(2,1)
	mov dptr, #reflow_message
	lcall ?Send_Constant_String

	; Insert reflow time value
	mov x, time_refl
	lcall hex2bcd
	Display_BCD(BCD + 1)
	Display_BCD(BCD + 0)
	Display_char(#'s')
	Display_char(#' ')

	; Insert reflow temp value
	mov x, temp_refl
	lcall hex2bcd
	Display_BCD(BCD + 1)
	Display_BCD(BCD + 0)
	Display_char(#'C')
ret

display_top_row:
	Set_Cursor(1,1)
	Display_char(#'T')
	lcall gathertemp
	Set_Cursor(1,2)
	Display_char(#':')
	Set_Cursor(1,8)
	Display_char(#' ')

	Display_char(#'S')
	Display_char(#'E')
	Display_char(#'C')
	Display_char(#':')
	Display_BCD(sec + 1)
	Display_BCD(sec + 0)
ret

display_state_1_variables:
	lcall display_top_row
	Set_Cursor(2,1)
	Display_char(#'S')
	Display_char(#'T')
	Display_char(#'1')
	Display_char(#' ')

	mov dptr, #ramp_to_soak_message
	lcall ?Send_Constant_String
ret

display_state_2_variables:
	lcall display_top_row
	Set_Cursor(2,1)
	mov dptr, #clear_message
	lcall ?Send_Constant_String
	Set_Cursor(2,1)
	Display_char(#'S')
	Display_char(#'T')
	Display_char(#'2')
	Display_char(#' ')

	mov dptr, #preheat_soak_message
	lcall ?Send_Constant_String
ret

display_state_3_variables:
	lcall display_top_row
	Set_Cursor(2,1)
	mov dptr, #clear_message
	lcall ?Send_Constant_String
	Set_Cursor(2,1)
	Display_char(#'S')
	Display_char(#'T')
	Display_char(#'3')
	Display_char(#' ')

	mov dptr, #ramp_to_peak_message
	lcall ?Send_Constant_String
ret

display_state_4_variables:
	lcall display_top_row
	Set_Cursor(2,1)
	mov dptr, #clear_message
	lcall ?Send_Constant_String
	Set_Cursor(2,1)
	Display_char(#'S')
	Display_char(#'T')
	Display_char(#'4')
	Display_char(#' ')

	mov dptr, #reflow_long_message
	lcall ?Send_Constant_String
ret

display_state_5_variables:
	lcall display_top_row
	Set_Cursor(2,1)
	Display_char(#'S')
	Display_char(#'T')
	Display_char(#'5')
	Display_char(#' ')

	mov dptr, #cooling_message
	lcall ?Send_Constant_String
ret

Display_Var:
	Display_char(#'c')
	;Display_BCD(bcd+1)
	Display_BCD(bcd+0)
ret

Read_ADC:
	clr ADCF
	setb ADCS ;  ADC start trigger signal
    jnb ADCF, $ ; Wait for conversion complete
    
    ; Read the ADC result and store in [R1, R0]
    mov a, ADCRL
    anl a, #0x0f
    mov R0, a
    mov a, ADCRH   
    swap a
    push acc
    anl a, #0x0f
    mov R1, a
    pop acc
    anl a, #0xf0
    orl a, R0
    mov R0, A
ret

putchar:
    jnb TI, putchar
    clr TI
    mov SBUF, a
ret


send_string:
    clr a
    movc a, @a+dptr
    jz sendstring_done
    lcall putchar
    inc dptr
    sjmp send_string

sendstring_done:
ret


take_sample:
;=======================================;
;										;
; 		Reference Voltage reading		;
;										;
;=======================================;

	; Read the 2.08V LM4040 voltage connected to AIN0 on pin 6
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0

	lcall Read_ADC
	; Save result for later use
	mov VAL_LM4040+0, R0
	mov VAL_LM4040+1, R1
	
;=======================================================;
;														;
; 		K-wire temp calculation with ref voltage		;
;														;
;=======================================================;

	; Read the signal connected to AIN7
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	lcall Read_ADC
    
    ; Take op amp reading and store R1 = x1 and R0 = x0
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	
	; From resistor value gain
    Load_y(33000)
    lcall mul32
	
	; Divide by ref voltage
	; Retrive the ADC LM4040 value
	mov y+0, VAL_LM4040+0
	mov y+1, VAL_LM4040+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32		

	; Store hot temp into thot 
	mov thot + 0, x + 0
	mov thot + 1, x + 1
	mov thot + 2, x + 2
	mov thot + 3, x + 3

	mov x+0, sum+0
    mov x+1, sum+1
    mov x+2, sum+2
    mov x+3, sum+3

	mov y+0, thot+0
    mov y+1, thot+1
    mov y+2, thot+2
    mov y+3, thot+3
	lcall add32
	mov sum+0, x+0
    mov sum+1, x+1
    mov sum+2, x+2
    mov sum+3, x+3
ret

gathertemp:
	mov sum+0, #0
    mov sum+1, #0
    mov sum+2, #0
    mov sum+3, #0
    mov counter, #SampleSize

loop_sample:
	lcall take_sample
	Wait_Milli_Seconds(#SampleSpeed)
    djnz counter, loop_sample
	
	; Take the average of the reading
 	mov x+0, sum+0
    mov x+1, sum+1
    mov x+2, sum+2
    mov x+3, sum+3
    Load_y(SampleSize)
    lcall div32

	; Clear thot
	mov thot + 0, #0
	mov thot + 1, #0
	mov thot + 2, #0
	mov thot + 3, #0

	; Move average to thot
	mov thot + 0, x + 0
	mov thot + 1, x + 1
	mov thot + 2, x + 2
	mov thot + 3, x + 3
	
;=======================================================;
;														;
;		Ambient temp calculation with ref voltage		;
;														;
;=======================================================;
	
	; Read pin 20 (Ain5)
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x05
	lcall Read_ADC
    
    ; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(40320) ; The MEASURED voltage reference: 4.0959V, with 4 decimal places
	lcall mul32
	; Retrive the ADC LM4040 value
	mov y+0, VAL_LM4040+0
	mov y+1, VAL_LM4040+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32
    
    ; Convert to C ( - 273)
    Load_y(27300)
    lcall sub32
    
;===================================;
;									;
;		thot + ambibent temp		;
;									;
;===================================;		

	mov y+0, thot + 0
	mov y+1, thot + 1
	mov y+2, thot + 2
	mov y+3, thot + 3
	lcall add32
	
	mov temp+0, x+0
    mov temp+1, x+1
    mov temp+2, x+2
    mov temp+3, x+3

	Send_BCD(bcd+2)		;_ _ X X . X X
    Send_BCD(bcd+1)		;X X _ _ . X X

    mov a, #'.'      	; Decimal point
    lcall putchar

    Send_BCD(bcd+0)		;X X X X . _ _

	; For putty terminal
    mov a, #'\r'     	; Carriage return
    lcall putchar
    mov a, #'\n'     	; New line
    lcall putchar
	; Convert to BCD and display
	lcall hex2bcd
    lcall Display_formated_BCD
    
	; Moving decimal point
    Load_y(100)
    lcall div32
    mov temp_new, x    
ret


main:
    mov sp, #0x7f
	lcall Init_All
    lcall LCD_4BIT
    mov FSM1_state, #0 ;

Forever:
    Wait_Milli_Seconds(#250)
    Wait_Milli_Seconds(#200)

;===================================;
;									;
;		State Machine Start			;
;									;
;===================================;
FSM1:
	mov a, FSM1_state

FSM1_state0:
	; If FSM1_state is NOT 0, go to next state
	cjne a, #0, FSM1_state1
	mov pwm + 0, #low(1000) ;pwm is inverted
    mov pwm + 1, #high(1000)
    lcall LCD_PB
    lcall display_state_0_variables
    lcall state_0_check
	
    jb PB6, FSM1_state0
    Wait_Milli_Seconds(#20)
    jb PB6, FSM1_state0
    jnb PB6, $ ; Wait for key release

    mov sec + 0, #0
    mov sec + 1, #0

FSM1_state0_done:
    mov FSM1_state, #1
	lcall LCD_4BIT
	ljmp Forever
	
; If restart button pressed
restart_1: 
	mov FSM1_state, #0
	mov sec, #0x0000 
	lcall LCD_4BIT
	Set_Cursor(2,3)
	ljmp Forever
	
FSM1_state1:

	; If abort button is pressed
	jnb ABORT_BUTTON, restart_1
	Wait_Milli_Seconds(#20)
	jnb ABORT_BUTTON, restart_1

	; If FSM1_state is NOT 1, go to next state
	cjne a, #1, FSM1_state2
	
	lcall display_state_1_variables
    
	mov pwm + 0, #low(0) ;pwm is inverted
    mov pwm + 1, #high(0)
    
    mov a, sec
    clr c
    subb a, #0x0060 ; If it pass abort_time => If 'a' is bigger than 20 c=0 if a is less c=1
    jnc check_abort 
    
	mov a, temp_soak
	clr c
	subb a, temp_new ; temp soak - temp
	jnc FSM1_state1_done ; No carry (temp_soak >= temp), jump to FSM_state1_done
	mov FSM1_state, #2
    
    Set_Cursor(2,1)
    mov dptr, #clear_message
    lcall ?Send_Constant_String
	
	mov sec + 0, #0
	mov sec + 1, #0
	sjmp FSM1_state1_done

check_abort: 
	mov a, temp_new
	clr c
	subb a, #0x0050 ; If 'a' is bigger than 50 c=0 less than c=-1
	jnc FSM1_state1 ; Jumps back to state 1 if c =0
	
	; Therefore, if a is less than 50
	mov FSM1_state, #0
	mov sec, #0
	ljmp Forever
		
FSM1_state1_done:
	ljmp Forever
	

FSM1_state2:
	jnb ABORT_BUTTON, restart_2
	Wait_Milli_Seconds(#20)
	jnb ABORT_BUTTON, restart_2
    
	cjne a, #2, FSM1_state3
    lcall display_state_2_variables

	mov pwm + 0, #low(800) ; pwm is inverted
    mov pwm + 1, #high(800)

    mov x + 0, time_soak
    mov x + 1, #0
    mov x + 2, #0
    mov x + 3, #0
    lcall hex2bcd
	mov a, bcd + 0
	clr c
    
	subb a, sec + 0
    subb a, #1 ; Correct for equal value edge case
	jnc FSM1_state2_done ; No carry (time_soak >= sec), jump to FSM_state1_done
    
    mov FSM1_state, #3
    mov sec + 0, #0
	mov sec + 1, #0
	
    Set_Cursor(2,1)
    mov dptr, #clear_message
    lcall ?Send_Constant_String
    sjmp FSM1_state2_done

restart_2:
	mov FSM1_state, #0
	mov sec, #0x0000 
	ljmp Forever
    
FSM1_state2_done:
	ljmp Forever


FSM1_state3:
	jnb ABORT_BUTTON, restart_2
	Wait_Milli_Seconds(#20)
	jnb ABORT_BUTTON, restart_2
	
	cjne a, #3, FSM1_state4
    lcall display_state_3_variables
	mov pwm + 0, #low(0) ;pwm is inverted
    mov pwm + 1, #high(0)
	mov a, temp_refl
	clr c
	subb a, temp_new
	jnc FSM1_state3_done
	mov FSM1_state, #4
	
    mov sec + 0, #0
	mov sec + 1, #0

    Set_Cursor(2,1)
    mov dptr, #clear_message
    lcall ?Send_Constant_String

FSM1_state3_done:
	ljmp Forever

FSM1_state4:
	jnb ABORT_BUTTON, restart_2
	Wait_Milli_Seconds(#20)
	jnb ABORT_BUTTON, restart_2

	cjne a, #4, FSM1_state5
    lcall display_state_4_variables
	mov pwm + 0, #low(800) ;pwm is inverted
    mov pwm + 1, #high(800)
	clr c

    mov x + 0, time_refl
    mov x + 1, #0
    mov x + 2, #0
    mov x + 3, #0
    lcall hex2bcd
	mov a, bcd + 0

	subb a, sec + 0
    subb a, #1 ;correct for equal value edge case
    jnc FSM1_state4_done
    mov FSM1_state, #5
    
    mov sec + 0, #0
	mov sec + 1, #0

    Set_Cursor(2,1)
    mov dptr, #clear_message
    lcall ?Send_Constant_String
    sjmp FSM1_state4_done

restart_3:
	mov FSM1_state, #0
	mov sec, #0x0000 
	ljmp Forever
	
FSM1_state4_done:
	ljmp Forever

FSM1_state5:
	jnb ABORT_BUTTON, restart_3
	Wait_Milli_Seconds(#20)
	jnb ABORT_BUTTON, restart_3

	cjne a, #5, FSM1_state5_done
    lcall display_state_5_variables
	mov pwm + 0, #low(1000) ;pwm is inverted
    mov pwm + 1, #high(1000)
	mov a, temp_new
	clr c
    
	subb a, temp_cool 
	jnc FSM1_state5_done
    mov FSM1_state, #0
	mov pwm + 0, #low(1000) ;pwm is inverted
    mov pwm + 1, #high(1000)
    lcall LCD_4BIT
    Wait_Milli_Seconds(#200)
    setb ET0
    Set_Cursor(1,1)
    mov dptr, #finish_message
	lcall ?Send_Constant_String
    Wait_Milli_Seconds(#200)
    Set_Cursor(2,1)
    mov dptr, #finish_message2
	lcall ?Send_Constant_String
    Wait_Milli_Seconds(#200)
    Wait_Milli_Seconds(#200)
    Wait_Milli_Seconds(#200)
    Wait_Milli_Seconds(#200)
    Wait_Milli_Seconds(#200)
    cpl ET0

FSM1_state5_done:
	ljmp Forever	

Restart_FSM:
	mov FSM1_state, #0
	ljmp FSM1_state0
	
;=============== State Machine End ===============;
END
	