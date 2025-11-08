;********************************************************************
;* F2025 - COE538 Lab 4.3 - Timer Alarms (HCS12, 9S32C)             *
;*                                                                  *
;*    A three-stage alarm with displays on the LCD. The program     *
;* displays A at the start of the program, B after 1 second, C      *
;* after a further 2 seconds. Tests hardware functionality,         *
;*                                                                  *
;*                                                                  *
;* Author: TERRY CHEN                                               *
;********************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry                  ; for absolute assembly: mark this as application entry point


; Include derivative-specific definitions 
		        INCLUDE 'derivative.inc' 
		        
;*****************************************************************
;* Register, Variable Definitions                                *
;*****************************************************************

              ORG $3850
              
TOF_COUNTER   RMB 1                   ; The timer, incremented at 23Hz
AT_DEMO       RMB 1                   ; The alarm time for this demo

;TESTA         RMB 1
;TESTB         RMB 1
;TESTC         RMB 1

OneSec        EQU 23                  ; 1 second delay (at 23Hz)
TwoSec        EQU 46                  ; 2 second delay (at 23Hz)

LCD_DAT       EQU PORTB               ; LCD data port, bits - PB7,...,PB0
LCD_CNTR      EQU PTJ                 ; LCD control port, bits - PJ7(E),PJ6(RS)
LCD_E         EQU $80                 ; LCD E-signal pin
LCD_RS        EQU $40                 ; LCD RS-signal pin
                 

;********************************************************************
;* Code section                                                     *
;********************************************************************
              ORG     $4000
Entry:
_Startup:


              LDS     #$4000          ; initialize the stack pointer
              JSR     initLCD         ; initialize the LCD
              JSR     clrLCD          ; clear LCD & home cursor
              JSR     ENABLE_TOF      ; Jump to TOF initialization
              CLI                     ; Enable global interrupt
              
              LDAA    #$0
              STAA    TOF_COUNTER     ; initialize counter to be non-garbage value
              
              LDAA    #'A'            ; Display A (for 1 sec)
              JSR     putcLCD         ; --"--
              
              LDAA    TOF_COUNTER     ; Initialize the alarm time
              ADDA    #OneSec         ; by adding on the 1 sec delay
              STAA    AT_DEMO         ; and save it in the alarm
              
CHK_DELAY_1   LDAA    TOF_COUNTER     ; If the current time
              CMPA    AT_DEMO         ; equals the alarm time  (CMPA sets Z=1 if A-M = 0)
              BEQ     A1              ; then display B  (BEQ Z=1)
              BRA     CHK_DELAY_1     ; and check the alarm again
              
A1            LDAA    #'B'            ; Display B (for 2 sec)
              JSR     putcLCD         ; --"--
              
              LDAA    AT_DEMO         ; Initialize the alarm time  
              ADDA    #TwoSec         ; by adding on the 2 sec delay
              STAA    AT_DEMO         ; and save it in the alarm

CHK_DELAY_2   LDAA    TOF_COUNTER     ; If the current time
              CMPA    AT_DEMO         ; equals the alarm time
              BEQ     A2              ; then display C
              BRA     CHK_DELAY_2     ; and check the alarm again

A2            LDAA    #'C'            ; Display C (forever)
              JSR     putcLCD         ; --"--
              
              SWI

                       

; Interrupt Service Routine
;***************************************************************
;* Timer Overflow Interrupt Service Routine
;* This routine is called on interrupt each time the free-running
;* 16 bit counter overflows.
;* Assuming that the timer prescaler bits PR0, PR1 and PR2 have
;* been changed, the basic rate of the 16 bit free running
;* counter is 667 nanoseconds, so overflows occur about 1/23
;* second apart. The TOF_COUNTER may be used in time delays.


TOF_ISR       INC     TOF_COUNTER       ; Increment the overflow count
              LDAA    #%10000000        ; Clear the TOF flag
              STAA    TFLG2             ; --"--
              RTI
              
;*********************************************************
;* Enable Timer Overflow
;*
;* This routine sets up the ISR vector and enables
;* the TOF interrupt mask bit.

ENABLE_TOF    LDD     #TOF_ISR          ; Setup the interrupt vector for timer overflow
              STD     $FFDE            
              LDAA    #%10000000
              STAA    TSCR1             ; Enable TCNT by setting bit 7
              
              ;* When enabling the timer overflow interrupt, it is prudent to clear
              ;* the TOF flag so than an interrupt does not occur immediately, but
              ;* rather on the next timer overflow.
              ;* (FOR DISABLE_TOF), re-enabling TOF

              STAA    TFLG2             ; Clear the TOF flag by writing to bit 7 
              LDAA    #%10000100        ; Turn timer overflow interrupt on by setting bit 7
              STAA    TSCR2             ; in TSCR2 and select prescale factor equal to 16
              RTS

;******************************************************************
;* Disable the TOF interrupt
;*
;* The routine to disable the timer overflow interrupt is useful
;* during debugging since the monitor ’trace’ function doesn’t
;* work otherwise.
              
DISABLE_TOF   LDAA    #%00000100        ; Turn timer overflow interrupt off by clearing bit 7
              STAA    TSCR2             ; in TSCR2 and leave prescale factor at 16
              RTS                     



; LCD subroutines
;*******************************************************************
;* Initialization of the LCD: 4-bit data width, 2-line display,    *
;* turn on display, cursor and blinking off. Shift cursor right.   *
;*******************************************************************
initLCD       BSET  DDRB, %11110000   ; configure pins PB7,PB6,PB5,PB4 for output       
              BSET  DDRJ, %11000000   ; configure pins PJ7,PJ6 for output
              LDY   #2000             ; wait for LCD to be ready
              JSR   del_50us          ; -"-
              
              LDAA  #$28              ; set 4-bit data, 2-line display
              JSR   cmd2LCD           ; -"-
              LDAA  #$0C              ; display on, cursor off, blinking off
              JSR   cmd2LCD           ; -"-
              LDAA  #$06              ; move cursor right after entering a character
              JSR   cmd2LCD           ; -"-
              RTS
            
;*******************************************************************
;* Clear display and home cursor *                                              
;*******************************************************************
clrLCD        LDAA  #$01              ; clear cursor and return to home position
              JSR   cmd2LCD           ; -"-
              LDY   #40               ; wait until "clear cursor" command is complete
              JSR   del_50us          ; -"-
              RTS
            
;*******************************************************************
;* This function sends a command in accumulator A to the LCD *
;*******************************************************************
cmd2LCD:      BCLR  LCD_CNTR, LCD_RS  ; select the LCD Instruction Register (IR)
              JSR   dataMov           ; send data to IR
              RTS
     
;*******************************************************************
;* This function outputs a NULL-terminated string pointed to by X *               
;*******************************************************************
putsLCD       LDAA  1,X+              ; get one character from the string
              BEQ   donePS            ; reach NULL character?
              JSR   putcLCD
              BRA   putsLCD
donePS        RTS

;*******************************************************************
;* This function outputs the character in accumulator in A to LCD *                        
;*******************************************************************
putcLCD       BSET  LCD_CNTR,LCD_RS    ; select the LCD Data register (DR)
              JSR   dataMov            ; send data to DR
              RTS  
            
;*******************************************************************
;* This function sends data to the LCD IR or DR depening on RS *
;*******************************************************************
dataMov       BSET  LCD_CNTR,LCD_E    ; pull the LCD E-signal high
              STAA  LCD_DAT           ; send the upper 4 bits of data to LCD
              BCLR  LCD_CNTR,LCD_E    ; pull the LCD E-signal low to complete the write oper.
              LSLA                    ; match the lower 4 bits with the LCD data pins
              LSLA                    ; -"-
              LSLA                    ; -"-
              LSLA                    ; -"-
              BSET  LCD_CNTR,LCD_E    ; pull the LCD E signal high
              STAA  LCD_DAT           ; send the lower 4 bits of data to LCD
              BCLR  LCD_CNTR,LCD_E    ; pull the LCD E-signal low to complete the write oper.
              LDY   #1                ; adding this delay will complete the internal
              JSR   del_50us          ; operation for most instructions
              RTS
              
;*******************************************************************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns.                   *
;*******************************************************************
del_50us:     PSHX                    ;2 E-clk
eloop:        LDX   #30               ;2 E-clk -
iloop:        PSHA                    ;2 E-clk |
              PULA                    ;3 E-clk |        ; total = 5 E-clk
              PSHA                    ;        |
              PULA						        ;        |        ; total = 10 E-clk
              PSHA                    ;        |
              PULA              		  ;        | 	      ; total = 15 E-clk
              PSHA                    ;        |
              PULA              			;        |        ; total = 20 E-clk
              PSHA                    ;        |
              PULA              			;        |        ; total = 25 E-clk
              PSHA                    ;        |
              PULA              			;        |        ; total = 30 E-clk                             
              PSHA                    ;2 E-clk | 50us
              PULA                    ;3 E-clk |        ; total = 35 E-clk
              NOP                     ;1 E-clk |
              NOP                     ;1 E-clk |
              DBNE  X, iloop          ;3 E-clk -        ; total = 40 E-clk
              DBNE  Y, eloop          ;3 E-clk            Dec and BNE = 0
              PULX                    ;3 E-clk
              RTS    
                            
;**************************************************************
;* Interrupt Vectors                                          *
;**************************************************************
              ORG     $FFDE 
              FDB     TOF_ISR
              
              ORG     $FFFE
              DC.W    Entry               ; Reset Vector
