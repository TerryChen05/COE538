;********************************************************************
;* F2025 - COE538 Lab 3 - Battery, Bumper Displays (HCS12, 9S32C)   *
;*                                                                  *
;*    The program reads the potentiometer and shows the equivalent  *
;* battery voltage reading on the LCD (displays to 1 decimal place) *
;* Also reads the bumpers switches and puts symbols on the LCD to   *
;* indicate whether the switches are open or closed.                *
;*                                                                  *
;*                                                                  *
;* Author: TERRY CHEN                                               *
;********************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry                  ; for absolute assembly: mark this as application entry point


; Include derivative-specific definitions 
		        INCLUDE 'derivative.inc' 
		        
;****************************************************************
;* Displaying battery voltage and bumper states (s19c32) *
;*****************************************************************
; Definitions

LCD_DAT       EQU   PORTB                   ; LCD data port, bits - PB7,...,PB0
LCD_CNTR      EQU   PTJ                     ; LCD control port, bits - PJ7(E) , PJ6(RS)
LCD_E         EQU   $80                     ; LCD Enable-signal pin   PJ7 = 1000 0000
LCD_RS        EQU   $40                     ; LCD Reset-signal pin    PJ6 = 0100 0000

; Variable/data section
              ORG   $3850

BCD_BUFFER    EQU   *                       ; The following registers are the BCD buffer area

TEN_THOUS     RMB   01                      ; 10,000 digit
THOUSANDS     RMB   01                      ; 1,000 digit
HUNDREDS      RMB   01                      ; 100 digit
TENS          RMB   01                      ; 10 digit
UNITS         RMB   01                      ; 1 digit
NO_BLANK      RMB   01                      ; Used in ’leading zero’ blanking by BCD2ASC

BCD_SPARE     RMB   02                      ; Extra space for decimal point and string terminator
BCD_SPARE2    RMB   10                      ; Extra space for decimal point and string terminator

ADDATA        RMB   8                       ; Storage for A/D converter results
CONSTANT      FDB   $258                    ; 600

msg1          dc.b  "Battery volt ",0
msg2          dc.b  "Sw status ",0

;********************************************************************
;* Code section                                                     *
;********************************************************************
              ORG   $4000
Entry:
_Startup:
              LDS   #$4000                  ; initialize the stack pointer
              JSR   initAD                  ; initialize ATD converter
              JSR   initLCD                 ; initialize LCD
              JSR   clrLCD                  ; clear LCD & home cursor

              LDX   #msg1                   ; display msg1
              JSR   putsLCD                 ; "
              LDAA  #$C0                    ; move LCD cursor to the 2nd row ($C0 = line 2 index 1, $80 = line 1 index 1)
              JSR   cmd2LCD
              LDX   #msg2                   ; display msg2
              JSR   putsLCD                 ; "

lbl           MOVB  #$90,ATDCTL5            ; r.just., unsign., sing.conv., mult., ch0, start conv. (0b1001 0000)
                                            ; 0bxxx1 xxxx = 8 conversion on 8 succesive channels
                                            ; 0bxx0x xxxx = conversions start on request
              BRCLR ATDSTAT0,$80,*          ; wait until the conversion sequence is complete
                                            ; mask ATDSTAT0 with 0b1000 0000, continue when SCF != 0
              
              LDAA  ATDDR4L                 ; load the ch4 result into AccA (ATDDRy), reads in 8-bit data NAD
              LDAB  #$27                    ; AccB = 39   (0x27, 0b0010 0111)
              MUL                           ; AccD = 1st result x 39
              ADDD  CONSTANT                ; AccD = 1st result x 39 + 600
                                        
              JSR   int2BCD                 ; convert value obtained from equation to BCD (uses reg D and X), stored in buffer variables
              JSR   BCD2ASC                 ; convert BCD values to ASCII, stored back in buffer variables
              
              LDAA  #$8D                    ; move LCD cursor to the 1st row, end of msg1 (msg1 uses 13 chars) (volts will use 4: xx.x)
              JSR   cmd2LCD                 ; "
              
              LDAA  TEN_THOUS               ; output the TEN_THOUS ASCII character
              JSR   putcLCD                 
              LDAA  THOUSANDS               ; output the THOUS ASCII character
              JSR   putcLCD
              
              LDAA  #'.'                    ; output the decimal
              JSR   putcLCD
              LDAA  HUNDREDS                ; output the HUNDREDS ASCII character
              JSR   putcLCD

              
              LDAA  #$CB                    ; move LCD cursor to the 2nd row, end of msg2  (msg2 uses 10 chars)
              JSR   cmd2LCD                 ; "
              
              BRCLR PORTAD0,$04,bowON       ; 0b0000 0100 (AN2)
              LDAA  #$31                    ; output ’1’ if bow sw OFF   (bow = front)
              BRA   bowOFF    
bowON         LDAA  #$30                    ; output ’0’ if bow sw ON
bowOFF        JSR   putcLCD
              LDAA  #' '                    ; output a space character in ASCII
              JSR   putcLCD

              BRCLR PORTAD0,$08,sternON     ; 0b0000 1000 (AN3)
              LDAA  #$31                    ; output ’1’ if stern sw OFF
              BRA   sternOFF
sternON       LDAA  #$30                    ; output ’0’ if stern sw ON
sternOFF      JSR   putcLCD

              JMP lbl


; Subroutine section

;*******************************************************************
;* Initialization of the LCD: 4-bit data width, 2-line display,    *
;* turn on display, cursor and blinking off. Shift cursor right.   *
;*******************************************************************
initLCD       BSET  DDRB, %11110000         ; configure pins PB7,PB6,PB5,PB4 for output       
              BSET  DDRJ, %11000000         ; configure pins PJ7,PJ6 for output
              LDY   #2000                   ; wait for LCD to be ready
              JSR   del_50us                ; -"-
              LDAA  #$28                    ; set 4-bit data, 2-line display
              JSR   cmd2LCD                 ; -"-
              LDAA  #$0C                    ; display on, cursor off, blinking off
              JSR   cmd2LCD                 ; -"-
              LDAA  #$06                    ; move cursor right after entering a character
              JSR   cmd2LCD                 ; -"-
              RTS
            
;*******************************************************************
;* Clear display and home cursor *                                              
;*******************************************************************
clrLCD        LDAA  #$01                    ; clear cursor and return to home position
              JSR   cmd2LCD                 ; -"-
              LDY   #40                     ; wait until "clear cursor" command is complete
              JSR   del_50us                ; -"-
              RTS
            
;*******************************************************************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns.                   *
;*******************************************************************
del_50us:     PSHX                          ;2 E-clk
eloop:        LDX   #30                     ;2 E-clk -
iloop:        PSHA                          ;2 E-clk |
              PULA                          ;3 E-clk |        
              PSHA                          ;        |
              PULA						              ;        |       
              PSHA                          ;        |
              PULA              		        ;        | 	     
              PSHA                          ;        |
              PULA              		      	;        |        
              PSHA                          ;        |
              PULA              			      ;        |        
              PSHA                          ;        |
              PULA              			      ;        |                                   
              PSHA                          ;2 E-clk | 50us
              PULA                          ;3 E-clk |        
              NOP                           ;1 E-clk |
              NOP                           ;1 E-clk |
              DBNE  X, iloop                ;3 E-clk -        ; total = 40 E-clk
              DBNE  Y, eloop                ;3 E-clk           
              PULX                          ;3 E-clk
              RTS    

;*******************************************************************
;* This function sends a command in accumulator A to the LCD *
;*******************************************************************
cmd2LCD:      BCLR  LCD_CNTR, LCD_RS        ; select the LCD Instruction Register (IR)
              JSR   dataMov                 ; send data to IR
              RTS
            
;*******************************************************************
;* This function outputs a NULL-terminated string pointed to by X *               
;*******************************************************************
putsLCD       LDAA  1,X+                    ; get one character from the string
              BEQ   donePS                  ; reach NULL character?
              JSR   putcLCD
              BRA   putsLCD
donePS        RTS

;*******************************************************************
;* This function outputs the character in accumulator in A to LCD *                        
;*******************************************************************
putcLCD       BSET  LCD_CNTR,LCD_RS          ; select the LCD Data register (DR)
              JSR   dataMov                  ; send data to DR
              RTS  
            
;*******************************************************************
;* This function sends data to the LCD IR or DR depening on RS *
;*******************************************************************
dataMov       BSET  LCD_CNTR,LCD_E          ; pull the LCD E-signal high
              STAA  LCD_DAT                 ; send the upper 4 bits of data to LCD
              BCLR  LCD_CNTR,LCD_E          ; pull the LCD E-signal low to complete the write oper.
              LSLA                          ; match the lower 4 bits with the LCD data pins
              LSLA                          ; -"-
              LSLA                          ; -"-
              LSLA                          ; -"-
              BSET  LCD_CNTR,LCD_E          ; pull the LCD E signal high
              STAA  LCD_DAT                 ; send the lower 4 bits of data to LCD
              BCLR  LCD_CNTR,LCD_E          ; pull the LCD E-signal low to complete the write oper.
              LDY   #1                      ; adding this delay will complete the internal
              JSR   del_50us                ; operation for most instructions
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

BCD2ASC      	LDAA    #0         			   ; Initialize the blanking flag
             	STAA    NO_BLANK

            ; CHECK DIGITS
            ; Ten-thousands digit     
C_TTHOU      	LDAA 	  TEN_THOUS   	      ; Load digit
              ORAA 	  NO_BLANK            ; OR with NO_BLANK (#0)
              BNE  		NOT_BLANK1  		    ; Check if not zero

ISBLANK1    	LDAA 	  #' '        		    ; If zero, replace with a blank space
              STAA 	  TEN_THOUS
              BRA  		C_THOU              ; Move to next digit (thousands)

NOT_BLANK1   	LDAA 	  TEN_THOUS   		 
              ORAA 	  #$30                ; Convert to ASCII (0x0[0-9] + 0b0011 0000)
              STAA 	  TEN_THOUS           ; Store the ASCII hex code back in its buffer variable
              LDAA 	  #$1         		
              STAA 	  NO_BLANK            ; Signal that we have seen a ’non-blank’ digit (0x01 = true)

            ; Thousands digit
C_THOU       	LDAA 	  THOUSANDS           ; Check the thousands digit for blankness
              ORAA 	  NO_BLANK            ; If it’s blank and ’no-blank’ is still zero
              BNE  		NOT_BLANK2

ISBLANK2     	LDAA 	  #' '                ; Only runs if 10 000s and 1000s is 0
              STAA 	  THOUSANDS           ; if THOUS dig = 0, but TEN THOUS != 0, we dont want a blank here (ex. "10000" instead of "1    ")
              BRA  		C_HUNS

NOT_BLANK2   	LDAA 	  THOUSANDS           ; ASCII conversion
              ORAA 	  #$30
              STAA 	  THOUSANDS
              LDAA    #$1
              STAA    NO_BLANK

            ; Hundreds digit
C_HUNS       	LDAA 	  HUNDREDS
              ORAA 	  NO_BLANK
              BNE  		NOT_BLANK3

ISBLANK3     	LDAA 	  #' '                ; Only runs if 10 000s, 1000s. and 100s is 0
              STAA 	  HUNDREDS
              BRA  		C_TENS

NOT_BLANK3   	LDAA 	  HUNDREDS            ; ASCII conversion
              ORAA 	  #$30
              STAA 	  HUNDREDS
              LDAA 	  #$1
              STAA 	  NO_BLANK

            ; Tens digit
C_TENS      	LDAA 	  TENS            
              ORAA 	  NO_BLANK
              BNE  		NOT_BLANK4

ISBLANK4      LDAA 	  #' '                ; Only runs if 10 000s. 1000s, 100s, and 10s is 0
              STAA 	  TENS
              BRA  		C_UNITS

NOT_BLANK4   	LDAA 	  TENS                ; ASCII conversion
              ORAA 	  #$30
              STAA 	  TENS

            ; Units digit
C_UNITS      	LDAA 	  UNITS               ; No blank check necessary, ALWAYS convert to ascii.
              ORAA 	  #$30
              STAA 	  UNITS

             	RTS              			      ; Return from subroutine
                          
;*******************************************************************
;* Initialization of the A/D Converter                             *
;*******************************************************************
initAD        MOVB    #$C0,ATDCTL2           ; power up AD, select fast flag clear  ($C0 = 0b1100 0000)
              JSR     del_50us               ; wait for 50 us
              MOVB    #$00,ATDCTL3           ; 8 conversions in a sequence, results put in corresponding registers. 
              MOVB    #$85,ATDCTL4           ; res=8 bits, conv-clks=2 periods, prescal=12 bits  ($85 = 0b1000 0101)
              BSET    ATDDIEN,$0C            ; configure pins AN03,AN02 to be enabled as digital inputs  ($0C = 0b0000 1100)
              RTS

;**************************************************************
;* Interrupt Vectors                                          *
;**************************************************************
            ORG   $FFFE
            DC.W  Entry           ; Reset Vector
