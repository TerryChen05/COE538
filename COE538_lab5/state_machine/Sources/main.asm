;********************************************************************
;* F2025 - COE538 Lab 5 - Robot Roaming Program (HCS12, 9S32C)      *
;*                                                                  *
;*    This program implements the robots behavior as a sequence     *
;* of states using a state machine:                                 *
;*                                                                  *
;*  (START, ALL_STOP, FORWARD, REVERSE, FORWARD_TURN,REVERSE_TURN)  *
;*                                                                  *
;*    A central controlling DISPATCHER routine calls the            *
;* current states. State transitions are triggered by bumper input  *
;* or timer alarms. An interrupt-driven Timer Overflow  is used     *
;* for timekeeping. The LCD display will update its current state   *
;* and battery voltage constantly as the program runs.              *
;*                                                                  *
;*                                                                  *
;* Author: TERRY CHEN                                               *
;********************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry                  ; mark this as application entry point


; Include derivative-specific definitions 
		        INCLUDE 'derivative.inc' 
		        
;*******************************************************************
; Equates section, Definitions of the various states
;*******************************************************************

START         EQU 0
FORWARD       EQU 1
REVERSE       EQU 2
ALL_STOP      EQU 3
FORWARD_TURN  EQU 4
REVERSE_TURN  EQU 5

LCD_DAT       EQU PORTB               ; LCD data port, bits - PB7,...,PB0
LCD_CNTR      EQU PTJ                 ; LCD control port, bits - PJ7(E),PJ6(RS)
LCD_E         EQU $80                 ; LCD E-signal pin
LCD_RS        EQU $40                 ; LCD RS-signal pin
FWD_INT       EQU 69                  ; 3 second delay (at 23Hz)
REV_INT       EQU 69                  ; 3 second delay (at 23Hz)
FWD_TRN_INT   EQU 29                  ; 2 second delay (at 23Hz)
REV_TRN_INT   EQU 29                  ; 2 second delay (at 23Hz)


              ORG   $3850
              
TOF_COUNTER   RMB  1                  ; The timer, incremented at 23Hz
CRNT_STATE    dc.b 3                  ; Current state register
T_FWD         ds.b 1                  ; FWD time
T_REV         ds.b 1                  ; REV time
T_FWD_TRN     ds.b 1                  ; FWD_TURN time
T_REV_TRN     ds.b 1                  ; REV_TURN time

TEN_THOUS     ds.b 1                  ; 10,000 digit
THOUSANDS     ds.b 1                  ; 1,000 digit
HUNDREDS      ds.b 1                  ; 100 digit
TENS          ds.b 1                  ; 10 digit
UNITS         ds.b 1                  ; 1 digit
NO_BLANK      ds.b 1                  ; Used in ’leading zero’ blanking by BCD2ASC              
BCD_SPARE     RMB  2                  ; Extra space for decimal point and string terminator
  
  
msg1          dc.b  "Battery volt ",0
msg2          dc.b  "State ",0
tab           dc.b  "START  ",0
              dc.b  "FWD    ",0
              dc.b  "REV    ",0
              dc.b  "ALL_STP",0
              dc.b  "FWD_TRN",0
              dc.b  "REV_TRN",0
           
           
;********************************************************************
;* Code section                                                     *
;********************************************************************
              ORG   $4000
Entry:
_Startup:
                                                   
              CLI                               ; Enable interrupts                         
              LDS   #$4000                      ; Initialize the stack pointer
                                                                                
              BSET  DDRA,%00000011              ; STAR_DIR, PORT_DIR                       
              BSET  DDRT,%00110000              ; STAR_SPEED, PORT_SPEED                    
                                                                                 
              JSR   initAD                      ; Initialize ATD converter                  
                                                                                  
              JSR   initLCD                     ; Initialize the LCD                        
              JSR   clrLCD                      ; Clear LCD & home cursor                  
                                                                                  
              LDX   #msg1                       ; Display msg1                             
              JSR   putsLCD                                                        
                                                                                 
              LDAA  #$C0                        ; Move LCD cursor to the 2nd row            
              JSR   cmd2LCD                                                         
              LDX   #msg2                       ; Display msg2 
              JSR   putsLCD               
                                       
              JSR   ENABLE_TOF                  ; Jump to TOF initialization
MAIN          JSR   UPDT_DISPL           
              LDAA  CRNT_STATE        
              JSR   DISPATCHER      
              BRA   MAIN              

;*******************************************************************
; Subroutines
;*******************************************************************

DISPATCHER    CMPA  #START                      ; If it’s the START state 
              BNE   NOT_START             
              JSR   START_ST                    ; then call START_ST routine 
              BRA   DISP_EXIT                   ; and exit 

NOT_START     CMPA  #FORWARD                    ; Else if it’s the FORWARD state
              BNE   NOT_FWD_ST
              JSR   FWD_ST                      ; then call the FORWARD routine
              JMP   DISP_EXIT                   ; and exit

NOT_FWD_ST    CMPA  #REVERSE                    ; repeat for other states
              BNE   NOT_REV_ST 
              JSR   REV_ST 
              JMP   DISP_EXIT              
                            
NOT_REV_ST    CMPA  #ALL_STOP
              BNE   NOT_ALL_STOP
              JSR   ALL_STP_ST
              JMP   DISP_EXIT  
               
NOT_ALL_STOP  CMPA  #FORWARD_TURN
              BNE   NOT_FWD_TRN
              JSR   FWD_TRN_ST
              JMP   DISP_EXIT                                                                    
              
NOT_FWD_TRN   CMPA  #REVERSE_TURN               ; Else if it’s the REV_TRN state 
              BNE   NOT_REV_TRN             
              JSR   REV_TRN_ST                  ; then call REV_TRN_ST routine 
              BRA   DISP_EXIT                   ; and exit 
                                            
NOT_REV_TRN   SWI                               ; Else the CRNT_ST is not defined, so stop 

DISP_EXIT     RTS                               ; Exit from the state dispatcher 

;*******************************************************************      

START_ST      BRCLR PORTAD0,$04,NO_FWD          ; If FWD_BUMP (0 = on, 1 = off) -> start2
              JSR   INIT_FWD                    ; Initialize the FORWARD state
              MOVB  #FORWARD,CRNT_STATE         ; Go into the FORWARD state
              BRA   START_EXIT

NO_FWD        NOP                               ; Else
START_EXIT    RTS                               ; return to the MAIN routine

;*******************************************************************    

FWD_ST        BRSET PORTAD0,$04,NO_FWD_BUMP     ; If FWD_BUMP then
              JSR   INIT_REV                    ; initialize the REVERSE routine
              MOVB  #REVERSE,CRNT_STATE         ; set the state to REVERSE
              JMP   FWD_EXIT                    ; and return
              
NO_FWD_BUMP   BRSET PORTAD0,$08,NO_REAR_BUMP    ; If REAR_BUMP, then we should stop
              JSR   INIT_ALL_STP                ; so initialize the ALL_STOP state
              MOVB  #ALL_STOP,CRNT_STATE        ; and change state to ALL_STOP
              JMP   FWD_EXIT                    ; and return
              
NO_REAR_BUMP  LDAA  TOF_COUNTER                 ; If Tc>Tfwd then
              CMPA  T_FWD                       ; the robot should make a turn
              BNE   NO_FWD_TRN                  ; so
              JSR   INIT_FWD_TRN                ; initialize the FORWARD_TURN state
              MOVB  #FORWARD_TURN,CRNT_STATE    ; and go to that state
              JMP   FWD_EXIT

              
NO_FWD_TRN    NOP                               ; Else
FWD_EXIT      RTS                               ; return to the MAIN routine 

;*******************************************************************

REV_ST        LDAA  TOF_COUNTER                 ; If Tc>Trev then
              CMPA  T_REV                       ; the robot should make a FWD turn
              BNE   NO_REV_TRN                  ; so
              JSR   INIT_REV_TRN                ; initialize the REV_TRN state
              MOVB  #REVERSE_TURN,CRNT_STATE    ; set state to REV_TRN
              BRA   REV_EXIT                    ; and return
NO_REV_TRN    NOP                               ; Else
REV_EXIT      RTS                               ; return to the MAIN routine

;*******************************************************************

ALL_STP_ST    BRSET PORTAD0,$04,NO_START        ; If FWD_BUMP   
              BCLR  PTT,%00110000               ; initialize the START state (both motors off)
              MOVB  #START,CRNT_STATE           ; set the state to START
              BRA   ALL_STP_EXIT                ; and return
NO_START      NOP                               ; Else
ALL_STP_EXIT  RTS                               ; return to the MAIN routine

;*******************************************************************

FWD_TRN_ST    LDAA  TOF_COUNTER                 ; If Tc>Tfwdturn then
              CMPA  T_FWD_TRN                   ; the robot should go FWD
              BNE   NO_FWD_FT                   ; so
              JSR   INIT_FWD                    ; initialize the FWD state
              MOVB  #FORWARD,CRNT_STATE         ; set state to FWD
              BRA   FWD_TRN_EXIT                ; and return
NO_FWD_FT     NOP                               ; Else
FWD_TRN_EXIT  RTS                               ; return to the MAIN routine
                       
                       
;*******************************************************************

REV_TRN_ST    LDAA  TOF_COUNTER                 ; If Tc>Trevturn then
              CMPA  T_REV_TRN                   ; the robot should go FWD
              BNE   NO_FWD_RT                   ; so
              JSR   INIT_FWD                    ; initialize the FWD state
              MOVB  #FORWARD,CRNT_STATE         ; set state to FWD
              BRA   REV_TRN_EXIT                ; and return
NO_FWD_RT     NOP                               ; Else
REV_TRN_EXIT  RTS                               ; return to the MAIN routine   

;*******************************************************************

INIT_FWD      BCLR  PORTA,%00000011             ; Set FWD direction for both motors
              BSET  PTT,%00110000               ; Turn on the drive motors
              LDAA  TOF_COUNTER                 ; Mark the fwd time Tfwd
              ADDA  #FWD_INT
              STAA  T_FWD
              RTS      

;*******************************************************************

INIT_REV      BSET  PORTA,%00000011             ; Set REV direction for both motors
              BSET  PTT,%00110000               ; Turn on the drive motors
              LDAA  TOF_COUNTER                 ; Mark the fwd time Tfwd
              ADDA  #REV_INT
              STAA  T_REV
              RTS
  
;*******************************************************************

INIT_ALL_STP  BCLR  PTT,%00110000               ; Turn off the drive motors
              RTS
              
;*******************************************************************

INIT_FWD_TRN  BSET  PORTA,%00000010             ; Set REV dir. for STARBOARD (right) motor
              LDAA  TOF_COUNTER                 ; Mark the fwd_turn time Tfwdturn
              ADDA  #FWD_TRN_INT
              STAA  T_FWD_TRN
              RTS            
              
;*******************************************************************

INIT_REV_TRN  BCLR  PORTA,%00000010             ; Set FWD dir. for STARBOARD (right) motor
              LDAA  TOF_COUNTER                 ; Mark the fwd time Tfwd
              ADDA  #REV_TRN_INT
              STAA  T_REV_TRN
              RTS 
;*******************************************************************
;* Update Display (Battery Voltage + Current State) *
;*******************************************************************
UPDT_DISPL    MOVB  #$90,ATDCTL5                ; R-just., uns., sing. conv., mult., ch=0, start
              BRCLR ATDSTAT0,$80,*              ; Wait until the conver. seq. is complete
              LDAA  ATDDR0L                     ; Load the ch0 result - battery volt - into A (not 4 bc we want battery not pot)                 
              LDAB  #$27                        ; AccB = 39   (0x27, 0b0010 0111)                              
              MUL                               ; AccD = 1st result x 39
              ADDD  #$258                       ; AccD = 1st result x 39 + 600
                                        
              JSR   int2BCD                     ; convert value obtained from equation to BCD (uses reg D and X), stored in buffer variables
              JSR   BCD2ASC                     ; convert BCD values to ASCII, stored back in buffer variables
              
              LDAA  #$8D                        ; move LCD cursor to the 1st row, end of msg1 (msg1 uses 13 chars) (volts will use 4: xx.x)
              JSR   cmd2LCD                     ; "
              
              LDAA  TEN_THOUS                   ; output the TEN_THOUS ASCII character
              JSR   putcLCD                 
              LDAA  THOUSANDS                   ; output the THOUS ASCII character
              JSR   putcLCD
              
              LDAA  #'.'                        ; output the decimal
              JSR   putcLCD
              LDAA  HUNDREDS                    ; output the HUNDREDS ASCII character
              JSR   putcLCD                     ; Display the battery voltage

              LDAA  #$C8                        ; Move LCD cursor to the 2nd row, end of msg2
              JSR   cmd2LCD 
              LDAB  CRNT_STATE                  ; Display current state
              LSLB                              ; "
              LSLB                              ; "
              LSLB                              ; "
              LDX   #tab                        ; "
              ABX                               ; "
              JSR   putsLCD                     ; "
              RTS
              
;*******************************************************************
; LCD subroutines
;*******************************************************************
;* Initialization of the LCD: 4-bit data width, 2-line display,    *
;* turn on display, cursor and blinking off. Shift cursor right.   *
;*******************************************************************
initLCD       BSET  DDRB, %11110000             ; configure pins PB7,PB6,PB5,PB4 for output       
              BSET  DDRJ, %11000000             ; configure pins PJ7,PJ6 for output
              LDY   #2000                       ; wait for LCD to be ready
              JSR   del_50us                    ; -"-
              
              LDAA  #$28                        ; set 4-bit data, 2-line display
              JSR   cmd2LCD                     ; -"-
              LDAA  #$0C                        ; display on, cursor off, blinking off
              JSR   cmd2LCD                     ; -"-
              LDAA  #$06                        ; move cursor right after entering a character
              JSR   cmd2LCD                     ; -"-
              RTS
;*******************************************************************
;* Clear display and home cursor *                                              
;*******************************************************************
clrLCD        LDAA  #$01                        ; clear cursor and return to home position
              JSR   cmd2LCD                     ; -"-
              LDY   #40                         ; wait until "clear cursor" command is complete
              JSR   del_50us                    ; -"-
              RTS
;*******************************************************************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns.                   * ; 4 E-clks * 300 = 1200 E-clks, 1200 * 41.67ns = 50.004us
;*******************************************************************
del_50us      PSHX                              ; (2 E-clk) Protect the X register
eloop         LDX   #300                        ; (2 E-clk) Initialize the inner loop counter
iloop         NOP                               ; (1 E-clk) No operation
              DBNE X,iloop                      ; (3 E-clk) If the inner cntr not 0, loop again
              DBNE Y,eloop                      ; (3 E-clk) If the outer cntr not 0, loop again
              PULX                              ; (3 E-clk) Restore the X register
              RTS                           
;*******************************************************************
;* This function sends a command in accumulator A to the LCD *
;*******************************************************************
cmd2LCD:      BCLR  LCD_CNTR, LCD_RS            ; select the LCD Instruction Register (IR)
              JSR   dataMov                     ; send data to IR
              RTS
     
;*******************************************************************
;* This function outputs a NULL-terminated string pointed to by X *               
;*******************************************************************
putsLCD       LDAA  1,X+                        ; get one character from the string
              BEQ   donePS                      ; reach NULL character?
              JSR   putcLCD
              BRA   putsLCD
donePS        RTS

;*******************************************************************
;* This function outputs the character in accumulator in A to LCD *                        
;*******************************************************************
putcLCD       BSET  LCD_CNTR,LCD_RS              ; select the LCD Data register (DR)
              JSR   dataMov                      ; send data to DR
              RTS  
;*******************************************************************
;* This function sends data to the LCD IR or DR depening on RS *
;*******************************************************************
dataMov       BSET  LCD_CNTR,LCD_E              ; pull the LCD E-signal high
              STAA  LCD_DAT                     ; send the upper 4 bits of data to LCD
              BCLR  LCD_CNTR,LCD_E              ; pull the LCD E-signal low to complete the write oper.
              LSLA                              ; match the lower 4 bits with the LCD data pins
              LSLA                              ; -"-
              LSLA                              ; -"-
              LSLA                              ; -"-
              BSET  LCD_CNTR,LCD_E              ; pull the LCD E signal high
              STAA  LCD_DAT                     ; send the lower 4 bits of data to LCD
              BCLR  LCD_CNTR,LCD_E              ; pull the LCD E-signal low to complete the write oper.
              LDY   #1                          ; adding this delay will complete the internal
              JSR   del_50us                    ; operation for most instructions
              RTS
              
;*******************************************************************
;* Initialization of the A/D Converter                             *
;*******************************************************************
initAD        MOVB    #$C0,ATDCTL2              ; power up AD, select fast flag clear  ($C0 = 0b1100 0000)
              JSR     del_50us                  ; wait for 50 us
              MOVB    #$00,ATDCTL3              ; 8 conversions in a sequence, results put in corresponding registers. 
              MOVB    #$85,ATDCTL4              ; res=8 bits, conv-clks=2 periods, prescal=12 bits  ($85 = 0b1000 0101)
              BSET    ATDDIEN,$0C               ; configure pins AN03,AN02 to be enabled as digital inputs  ($0C = 0b0000 1100)
              RTS
;***********************************************************************
; Binary 16 to BCD Conversion Routine
; ref: b16todec.asm
;***********************************************************************

;* This routine converts a 16 bit binary number in .D into
;* BCD digits in BCD_BUFFER.
;* Peter Hiscocks
;* Algorithm:
;* Because the IDIV (Integer Division) instruction is available on
;* the HCS12, we can determine the decimal digits by repeatedly
;* dividing the binary number by ten: the remainder each time is
;* a decimal digit. Conceptually, what we are doing is shifting
;* the decimal number one place to the right past the decimal
;* point with each divide operation. The remainder must be
;* a decimal digit between 0 and 9, because we divided by 10.
;* The algorithm terminates when the quotient has become zero.
;* Bug note: XGDX does not set any condition codes, so test for
;* quotient zero must be done explicitly with CPX.

int2BCD       XGDX             		      ; Save input number in X (exchange D and X)
              LDAA    #0        		    ; Clear BCD_buffer
              STAA    TEN_THOUS
              STAA    THOUSANDS
              STAA    HUNDREDS
              STAA    TENS
              STAA    UNITS
              STAA    BCD_SPARE
              STAA    BCD_SPARE+1	      ; Filled all bytes with 0

              CPX     #0       			    ; Check for zero input
              BEQ     CON_EXIT  	      ; If zero, exit

            ; UNITS digit
              XGDX              	      ; Get binary number back to D (as dividend)
              LDX     #10       	      ; Divisor = 10
              IDIV              	    	; IDIV = (D / X) Quotient in X, Remainder in D(0-9)
              STAB    UNITS    	        ; Store remainder(D) in UNITS(1 byte), B = lower byte of D
              CPX     #0                ; Quotient = 0? (dividend < 10)
              BEQ     CON_EXIT

            ; TENS digit
              XGDX                      ; Swap Quotient from X to D (so we can do IDIV)
              LDX     #10               
              IDIV                      ; Div by 10
              STAB    TENS              ; Store Tens place remainder in TENS
              CPX     #0                
              BEQ     CON_EXIT

            ; HUNDREDS digit
              XGDX                      ; Swap Quotient from X to D
              LDX     #10
              IDIV
              STAB    HUNDREDS          ; Store Hundreds place remainder
              CPX     #0
              BEQ     CON_EXIT

            ; THOUSANDS digit
              XGDX
              LDX     #10
              IDIV
              STAB    THOUSANDS         ; Store Thousands place remainder
              CPX     #0
              BEQ     CON_EXIT
              
            ; TEN_THOUS digit  
              XGDX
              LDX     #10                
              IDIV                      ; Guaranteed 0 Quotient in X, Ten Thousands digit in D (max Vbatt value = 10545)
              STAB    TEN_THOUS

CON_EXIT      RTS                       ; Return from subroutine

;***********************************************************************
; BCD to ASCII Conversion Routine (V2)     
; ref: bcdtoasc.asm
;***********************************************************************

;* This routine converts the BCD number in the BCD_BUFFER
;* into ascii format, with leading zero suppression.
;* Leading zeros are converted into space characters.
;* The flag ’NO_BLANK’ starts cleared and is set once a non-zero
;* digit has been detected.
;* The ’UNITS’ digit is never blanked, even if it and all the
;* preceding digits are zero.
;* Peter Hiscocks

BCD2ASC      	LDAA    #0         			; Initialize the blanking flag
             	STAA    NO_BLANK

            ; CHECK DIGITS
            ; Ten-thousands digit     
C_TTHOU      	LDAA 	  TEN_THOUS   		; Load digit
              ORAA 	  NO_BLANK        ; OR with NO_BLANK (#0)
              BNE  		NOT_BLANK1  		; Check if not zero

ISBLANK1    	LDAA 	  #' '        		; If zero, replace with a blank space
              STAA 	  TEN_THOUS
              BRA  		C_THOU          ; Move to next digit (thousands)

NOT_BLANK1   	LDAA 	  TEN_THOUS   		 
              ORAA 	  #$30            ; Convert to ASCII (0x0[0-9] + 0b0011 0000)
              STAA 	  TEN_THOUS       ; Store the ASCII hex code back in its buffer variable
              LDAA 	  #$1         		
              STAA 	  NO_BLANK        ; Signal that we have seen a ’non-blank’ digit (0x01 = true)

            ; Thousands digit
C_THOU       	LDAA 	  THOUSANDS       ; Check the thousands digit for blankness
              ORAA 	  NO_BLANK        ; If it’s blank and ’no-blank’ is still zero
              BNE  		NOT_BLANK2

ISBLANK2     	LDAA 	  #' '            ; Only runs if 10 000s and 1000s is 0
              STAA 	  THOUSANDS       ; if THOUS dig = 0, but TEN THOUS != 0, we dont want a blank here (ex. "10000" instead of "1    ")
              BRA  		C_HUNS

NOT_BLANK2   	LDAA 	  THOUSANDS       ; ASCII conversion
              ORAA 	  #$30
              STAA 	  THOUSANDS
              LDAA    #$1
              STAA    NO_BLANK

            ; Hundreds digit
C_HUNS       	LDAA 	  HUNDREDS
              ORAA 	  NO_BLANK
              BNE  		NOT_BLANK3

ISBLANK3     	LDAA 	  #' '            ; Only runs if 10 000s, 1000s. and 100s is 0
              STAA 	  HUNDREDS
              BRA  		C_TENS

NOT_BLANK3   	LDAA 	  HUNDREDS        ; ASCII conversion
              ORAA 	  #$30
              STAA 	  HUNDREDS
              LDAA 	  #$1
              STAA 	  NO_BLANK

            ; Tens digit
C_TENS      	LDAA 	  TENS            
              ORAA 	  NO_BLANK
              BNE  		NOT_BLANK4

ISBLANK4      LDAA 	  #' '            ; Only runs if 10 000s. 1000s, 100s, and 10s is 0
              STAA 	  TENS
              BRA  		C_UNITS

NOT_BLANK4   	LDAA 	  TENS            ; ASCII conversion
              ORAA 	  #$30
              STAA 	  TENS

            ; Units digit
C_UNITS      	LDAA 	  UNITS           ; No blank check necessary, ALWAYS convert to ascii.
              ORAA 	  #$30
              STAA 	  UNITS

             	RTS              			  ; Return from subroutine   
             	
; Interrupt Service Routine
;***************************************************************
;* Timer Overflow Interrupt Service Routine
;* This routine is called on interrupt each time the free-running
;* 16 bit counter overflows.
;* Assuming that the timer prescaler bits PR0, PR1 and PR2 have
;* been changed, the basic rate of the 16 bit free running
;* counter is 667 nanoseconds, so overflows occur about 1/23
;* second apart. The TOF_COUNTER may be used in time delays.
;******************************************************************


TOF_ISR       INC     TOF_COUNTER       ; Increment the overflow count
              LDAA    #%10000000        ; Clear the TOF flag
              STAA    TFLG2             ; --"--
              RTI
              
;*********************************************************
;* Enable Timer Overflow
;*
;* This routine sets up the ISR vector and enables
;* the TOF interrupt mask bit.
;******************************************************************

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
;******************************************************************
              
DISABLE_TOF   LDAA    #%00000100        ; Turn timer overflow interrupt off by clearing bit 7
              STAA    TSCR2             ; in TSCR2 and leave prescale factor at 16
              RTS                	           
                           

;**************************************************************
;* Interrupt Vectors                                          *
;**************************************************************
              
              ORG   $FFFE
              DC.W  Entry ; Reset Vector
              
              ORG   $FFDE
              DC.W  TOF_ISR ; Timer Overflow Interrupt Vector
