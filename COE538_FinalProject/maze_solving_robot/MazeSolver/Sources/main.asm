;********************************************************************
;* F2025 - COE538 Final Project - Maze Solving Robot (HCS12, 9S32C) *
;*                                                                  *
;*    This program allows the eebot to autonomously solve the maze  *  
;* through the use of line tracking via photoresistor sensors, and  *
;* state machine logic to control the flow of its movement patterns *
;* and situational responses.                                       *
;*                                                                  *
;*    eebot #102940 was used, the tolerances were set such that the *
;* specific sensor readings corresponded to desired line tracking   *
;* behaviour. Sensors A, C, and E/F were used to maintain proper    *                                          
;* centering of the eebot body in relation to the maze track. The   *                                                                  
;* sensors B and D were used to determine the path viabilities at   *
;* junctions, with the two sensors corresponding to the Port and    *
;* Starboard sensors respectively.                                  *
;*                                                                  *
;*    The program favours left turns at all junctions, and foward   *
;* continuations at right facing L junctions.                       *                                           
;*                                                                  *
;* note: battery voltage display removed                            *
;* Author: TERRY CHEN                                               *
;********************************************************************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry                  ; absolute assembly: application entry point


; Include derivative-specific definitions 
		        INCLUDE 'derivative.inc'

;********************************************************************
; Equates section                                                   *
;********************************************************************

; LCD Equates
;----------------------
CLEAR_HOME    EQU   $01                   ; Clear the display and home the cursor
INTERFACE     EQU   $38                   ; 8 bit interface, two line display
CURSOR_OFF    EQU   $0C                   ; Display on, cursor off
SHIFT_OFF     EQU   $06                   ; Address increments, no character shift
LCD_SEC_LINE  EQU   64                    ; Starting addr. of 2nd line of LCD (decimal value)

; LCD Addresses
;----------------------
LCD_CNTR      EQU   PTJ                   ; LCD Control Register: E = PJ7, RS = PJ6
LCD_DAT       EQU   PORTB                 ; LCD Data Register: D7 = PB7, ... , D0 = PB0
LCD_E         EQU   $80                   ; LCD E-signal pin
LCD_RS        EQU   $40                   ; LCD RS-signal pin


; Timers
;----------------------
T_LEFT        EQU   8
T_RIGHT       EQU   8

; States
;----------------------
START         EQU   0
FWD           EQU   1
ALL_STOP      EQU   2
LEFT_TRN      EQU   3
RIGHT_TRN     EQU   4
REV_TRN       EQU   5                     
L_ALIGN       EQU   6                     
R_ALIGN       EQU   7  

; Other Equates
;----------------------
NULL          EQU   00                    ; The string null terminator
CR            EQU   $0D                   ; Carriage return character
SPACE         EQU   ' '                   ; The space character                   

; Variables (RAM space)
;---------------------
              ORG   $3850

; Constant Values 
;-------------------------------------------------------
                                          ; Base dark values (when aligned with track)
BASE_BOW      FCB   $CD                   ; A
BASE_PORT     FCB   $CA                   ; B
BASE_MID      FCB   $CD                   ; C
BASE_STBD     FCB   $CA                   ; D
BASE_LINE     FCB   $A9                   ; E/F

                                          ; Tolerance values
TOL_BOW       FCB   $30                   ; A
TOL_PORT      FCB   $22                   ; B  
TOL_MID       FCB   $20                   ; C
TOL_STBD      FCB   $16                   ; D
TOL_LINE      FCB   $18                   ; E/F


TOP_LINE      RMB   20                    ; LCD top line
              FCB   NULL                  ; null terminator
              
BOT_LINE      RMB   20                    ; LCD bottom line
              FCB   NULL                  ; null terminator

CLEAR_LINE    FCC   '                  '  ; Clear display line
              FCB   NULL                  ; null terminator

TEMP          RMB   1                     ; Temp location

; Sensor Variables 
;------------------------------------------------------
SENSOR_LINE   FCB   $01                   ; Storage for guider sensor readings
SENSOR_BOW    FCB   $23                   ; test values
SENSOR_PORT   FCB   $45
SENSOR_MID    FCB   $67
SENSOR_STBD   FCB   $89
SENSOR_NUM    RMB   1 

; Other Variables
;------------------------------------------------------
TEN_THOUS     ds.b  1                     ; 10,000 digit
THOUSANDS     ds.b  1                     ; 1,000 digit
HUNDREDS      ds.b  1                     ; 100 digit
TENS          ds.b  1                     ; 10 digit
UNITS         ds.b  1                     ; 1 digit
NO_BLANK      ds.b  1                     
BCD_SPARE     RMB   2
                                          ; Where our TOF counter register lives
TOF_COUNTER   dc.b  0                     ; The timer, incremented at 23Hz
CRNT_STATE    dc.b  2                     ; Current state variable
T_TURN        ds.b  1                     ; Turn timer variable


;********************************************************************
;* Code section                                                     *
;********************************************************************
              ORG   $4000
Entry:                                                                       
_Startup: 

              LDS   #$4000                 ; Initialize the stack pointer
              CLI                          ; Enable interrupts
              JSR   INIT                   ; Initialize ports
              JSR   openADC                ; Initialize the ATD
              JSR   initLCD                ; Initialize the LCD
              JSR   CLR_LCD_BUF            ; Clears LCD buffer
              BSET  DDRA,%00000011         ; STAR_DIR, PORT_DIR                        
              BSET  DDRT,%00110000         ; STAR_SPEED, PORT_SPEED                    
              JSR   initAD                 ; Initialize ATD converter                  
              JSR   initLCD                ; Initialize the LCD                        
              JSR   clrLCD                 ; Clear LCD & home cursor                   
              JSR   ENABLE_TOF             ; Jump to TOF initialization

MAIN        
              JSR   G_LEDS_ON              ; Enable the guider LEDs   
              JSR   READ_SENSORS           ; Read the 5 guider sensors
              JSR   G_LEDS_OFF             ; Disable the guider LEDs                   
              
              LDAA  #$80
              JSR   cmd2LCD
              LDX   #msg1                  ; Display msg1                              
              JSR   putsLCD                                                                          ;       "      
              JSR   UPDT_DISPL         
              JSR   DISPLAY_SENSORS
              
              LDAA  CRNT_STATE         
              JSR   DISPATCHER         
              BRA   MAIN               

;********************************************************************
;* States                                                           *
;********************************************************************
msg1          dc.b  "State",0              ; Display messages

tab           dc.b  "Start  ",0
              dc.b  "Fwd    ",0
              dc.b  "All_Stp",0
              dc.b  "L_Turn ",0
              dc.b  "R_Turn ",0
              dc.b  "RevTrn ",0
              dc.b  "L_Align",0     
              dc.b  "R_Align",0  

;********************************************************************
;* Subroutines                                                      *
;********************************************************************
DISPATCHER        

IF_START          CMPA  #START                          ; check if in START state
                  BNE   IF_FORWARD                      ; else check if FWD state
                  JSR   START_ST                        ; start START state
                  RTS                                         

IF_FORWARD        CMPA  #FWD                            ; check if in FWD state
                  BNE   IF_STOP                         ; else check if ALL_STOP state
                  JSR   FWD_ST                          ; start FWD state
                  RTS
                  
IF_STOP           CMPA  #ALL_STOP                       ; check if in ALL_STOP state
                  BNE   IF_LEFT_TRN                     ; else check if LEFT_TRN state
                  JSR   ALL_STOP_ST                     ; start ALL_STOP state
                  RTS
                  
IF_LEFT_TRN       CMPA  #LEFT_TRN                       ; check if in LEFT_TRN state
                  BNE   IF_RIGHT_TRN                    ; else check if RIGHT_TRN state
                  JSR   GO_LEFT                         ; start LEFT_TRN state
                  RTS    
                  
IF_RIGHT_TRN      CMPA  #RIGHT_TRN                      ; check if in RIGHT_TRN state
                  BNE   IF_REV_TRN                      ; else check if REV_TRN state
                  JSR   GO_RIGHT                        ; start RIGHT_TRN state                                                             
                  
IF_REV_TRN        CMPA  #REV_TRN                        ; check if REV_TRN state
                  BNE   IF_LEFT_ALIGN                   ; else check if LEFT_ALIGN state
                  JSR   REV_TRN_ST                      ; start REV_TRN state
                  RTS                                           

IF_LEFT_ALIGN     CMPA  #L_ALIGN                        ; check if L_ALIGN state
                  BNE   IF_RIGHT_ALIGN                  ; else check if RIGHT_ALIGN state
                  JSR   LEFT_ALIGN_DONE                 ; start L_ALIGN state
                  RTS         

IF_RIGHT_ALIGN    CMPA  #R_ALIGN                        ; check if R_ALIGN state
                  JSR   RIGHT_ALIGN_DONE                ; start R_ALIGN state
                  
                  RTS                                   ; else INVALID state


;********************************************************************
;* States                                                           *
;********************************************************************

; START state
; ---------------------

START_ST          BRCLR   PORTAD0, %00000100,RELEASE    ; move from START -> FWD on bumper release                               
                  JSR     INIT_FWD                                                               
                  MOVB    #FWD, CRNT_STATE

RELEASE           RTS                                                                                                                                  

; FWD state
; --------------------

FWD_ST            BRSET   PORTAD0, $04, NO_FWD_BUMP           ; Checks if bow bumper is hit                           
                  MOVB    #REV_TRN, CRNT_STATE                ; if true, enter the                                 
                                                              ; REV_TURN state                             
                  JSR     UPDT_DISPL                          ; Update the display                                
                  JSR     INIT_REV                                                                
                  LDY     #6000                                                                   
                  JSR     del_50us                                                                
                  JSR     INIT_RIGHT                                                              
                  LDY     #6000                                                                   
                  JSR     del_50us                                                                
                  LBRA    EXIT                                                                    

NO_FWD_BUMP       BRSET   PORTAD0, $04, NO_FWD_REAR_BUMP      ; Checks if the stern bumper 
                  MOVB    #ALL_STOP, CRNT_STATE               ; if true, enter the                   
                  JSR     INIT_STOP                           ; ALL_STOP state
                  LBRA    EXIT 
                  
NO_FWD_REAR_BUMP  LDAA    SENSOR_BOW                                                              
                  ADDA    TOL_BOW                                                               
                  CMPA    BASE_BOW
                  BPL     NOT_ALIGNED                         ; Sensor A within tolerance 
                                                                                  
                  LDAA    SENSOR_MID                                                              
                  ADDA    TOL_MID                                                                
                  CMPA    BASE_MID 
                  BPL     NOT_ALIGNED                         ; Sensor C within tolerance
                                                                                 
                  LDAA    SENSOR_LINE                                                             
                  ADDA    TOL_LINE                                                                
                  CMPA    BASE_LINE 
                  BPL     CHECK_RIGHT_ALIGN                   ; Sensor E/F left leaning 
                                                                            
                  LDAA    SENSOR_LINE                                                             
                  SUBA    TOL_LINE                                                               
                  CMPA    BASE_LINE                                                              
                  BMI     CHECK_LEFT_ALIGN                    ; Sensor E/F right leaning

; Tracking Line (left turn priority)
; ----------------------------------

NOT_ALIGNED       LDAA    SENSOR_PORT                                                            
                  ADDA    TOL_PORT                                                               
                  CMPA    BASE_PORT                                                              
                  BPL     PARTIAL_LEFT_TRN                    ; Sensor B within tolerance (nearing a line)                                    
                  BMI     NO_PORT                                                             

NO_PORT           LDAA    SENSOR_BOW                                                             
                  ADDA    TOL_BOW                                                                 
                  CMPA    BASE_BOW                                                                
                  BPL     EXIT                                ; Cont. FWD                                    
                  BMI     NO_BOW                                                              

NO_BOW            LDAA    SENSOR_STBD                                                             
                  ADDA    TOL_STBD                                                               
                  CMPA    BASE_STBD                           ; Sensor D within tolerance (nearing a line)                                     
                  BPL     PARTIAL_RIGHT_TRN                                                         
                  BMI     EXIT 

; Partial Left
; ----------------------------------

PARTIAL_LEFT_TRN  LDY     #7000                                                                 
                  jsr     del_50us                                                                
                  JSR     INIT_LEFT                                                              
                  MOVB    #LEFT_TRN, CRNT_STATE               ; LEFT TRN state                                     
                  LDY     #7000                                                                   
                  JSR     del_50us                                                                
                  BRA     EXIT    
                                                                                  
; Turning Left
; ----------------------------------

CHECK_LEFT_ALIGN  JSR     INIT_LEFT                                                             
                  MOVB    #L_ALIGN, CRNT_STATE                ; LEFT ALIGN state                                  
                  BRA     EXIT

; Partial Right
; ----------------------------------

PARTIAL_RIGHT_TRN LDY     #7000                                                                  
                  jsr     del_50us                                                                
                  JSR     INIT_RIGHT                                                              
                  MOVB    #RIGHT_TRN, CRNT_STATE              ; RIGHT TRN state                                    
                  LDY     #7000                                                                   
                  JSR     del_50us                                                                
                  BRA     EXIT     
                                                                                
; Turning Right
; ----------------------------------

CHECK_RIGHT_ALIGN JSR     INIT_RIGHT                                                              
                  MOVB    #R_ALIGN, CRNT_STATE                ; RIGHT ALIGN state                                 
                  BRA     EXIT                                                                                                                                                         

EXIT              RTS 

; LEFT TRN state
; --------------------

GO_LEFT           LDAA    SENSOR_BOW                                                              
                  ADDA    TOL_BOW                                                                 
                  CMPA    BASE_BOW                                                              
                  BPL     LEFT_ALIGN_DONE                     ; Stay until BOW aligned                                    
                  BMI     EXIT
                  
; LEFT ALIGN state
; --------------------

LEFT_ALIGN_DONE   MOVB    #FWD, CRNT_STATE                    ; Cont. FWD                                    
                  JSR     INIT_FWD                                                                
                  BRA     EXIT
                                                                                      
; RIGHT TRN state
; --------------------

GO_RIGHT          LDAA    SENSOR_BOW                                                              
                  ADDA    TOL_BOW                                                                
                  CMPA    BASE_BOW                                                                
                  BPL     RIGHT_ALIGN_DONE                    ; Stay until BOW aligned                                     
                  BMI     EXIT 
                  
; RIGHT ALIGN state
; --------------------

RIGHT_ALIGN_DONE  MOVB    #FWD, CRNT_STATE                    ; Cont. FWD                                   
                  JSR     INIT_FWD                                                                
                  BRA     EXIT                                                                    

; REVERSE TRN state
; --------------------

REV_TRN_ST        LDAA    SENSOR_BOW                                                              
                  ADDA    TOL_BOW                                                                
                  CMPA    BASE_BOW                                                                
                  BMI     EXIT                                ; Stay until BOW aligned
                                                                                     
                  JSR     INIT_LEFT                                                               
                  MOVB    #FWD, CRNT_STATE                                                        
                  JSR     INIT_FWD                            ; Cont. FWD                                     
                  BRA     EXIT                                                                    
; ALL STOP state
; --------------------

ALL_STOP_ST       BRSET   PORTAD0, %00000100, NO_START_BUMP                                       
                  MOVB    #START, CRNT_STATE                  ; START state                                   

NO_START_BUMP     RTS                                                                             

;********************************************************************
;* Initalization Subroutines                                        *
;********************************************************************

INIT_RIGHT        BSET    PORTA,%00000010          
                  BCLR    PORTA,%00000001           
                  LDAA    TOF_COUNTER               ; Mark the fwd_turn time Tfwdturn
                  ADDA    #T_RIGHT
                  STAA    T_TURN
                  RTS

INIT_LEFT         BSET    PORTA,%00000001         
                  BCLR    PORTA,%00000010          
                  LDAA    TOF_COUNTER               ; Mark TOF time
                  ADDA    #T_LEFT                   ; Add left turn
                  STAA    T_TURN                    
                  RTS

INIT_FWD          BCLR    PORTA, %00000011          ; Set FWD dir. for both motors
                  BSET    PTT, %00110000            ; Turn on the drive motors
                  RTS 

INIT_REV          BSET    PORTA,%00000011           ; Set REV direction for both motors
                  BSET    PTT,%00110000             ; Turn on the drive motors
                  RTS

INIT_STOP         BCLR    PTT, %00110000            ; Turn off the drive motors
                  RTS


;********************************************************************
;* PORT DDR Initializations                                         *
;********************************************************************

INIT              BCLR    DDRAD,$FF  ; Make PORTAD an input (DDRAD @ $0272)
                  BSET    DDRA, $FF  ; Make PORTA an output (DDRA @ $0002)
                  BSET    DDRB, $FF  ; Make PORTB an output (DDRB @ $0003)
                  BSET    DDRJ, $C0  ; Make pins 7,6 of PTJ outputs (DDRJ @ $026A)
                  RTS


;********************************************************************
;* ADC Initialization                                               *
;********************************************************************   
         
openADC           MOVB   #$80,ATDCTL2     ; Turn on ADC (ATDCTL2 @ $0082)
                  LDY    #1               ; Wait for 50 us for ADC to be ready
                  JSR    del_50us         ; - " -
                  MOVB   #$20,ATDCTL3     ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                  MOVB   #$97,ATDCTL4     ; 8-bit resolution, prescaler=48 (ATDCTL4 @ $0084)
                  RTS
                  
initAD            MOVB   #$C0,ATDCTL2     ;power up AD, select fast flag clear
                  JSR    del_50us         ;wait for 50 us
                  MOVB   #$00,ATDCTL3     ;8 conversions in a sequence
                  MOVB   #$85,ATDCTL4     ;res=8, conv-clks=2, prescal=12
                  BSET   ATDDIEN,$0C      ;configure pins AN03,AN02 as digital inputs
                  RTS                  

;*******************************************************************************
;* Clear LCD Buffer                                                            *
;*                                                                             *
;* This routine writes space characters to the buffer in order to              *
;* prepare it for the building of a new display buffer.                        *
;* This needs only to be done once at the start of the program. Thereafter the *
;* display routine should maintain the buffer properly.                        *
;*******************************************************************************

CLR_LCD_BUF       LDX   #CLEAR_LINE
                  LDY   #TOP_LINE
                  JSR   STRCPY

CLB_SECOND        LDX   #CLEAR_LINE
                  LDY   #BOT_LINE
                  JSR   STRCPY

CLB_EXIT          RTS

;*******************************************************************************
;* String Copy                                                                 *
;*                                                                             *
;* Copies a null-terminated string from one location to another.               *
;*                                                                             *
;* Passed: X contains starting address of null-terminated string               *
;* Y contains first address of destination                                     *
;*******************************************************************************

STRCPY            PSHX                  ; Protect the registers used                 
                  PSHY
                  PSHA

STRCPY_LOOP       LDAA  0,X             ; Get a source character
                  STAA  0,Y             ; Copy it to the destination
                  BEQ   STRCPY_EXIT     ; If it was the null, then exit
                  INX                   ; Else increment the pointers
                  INY
                  BRA   STRCPY_LOOP     ; and do it again

STRCPY_EXIT       PULA                  ; Restore the registers
                  PULY
                  PULX
                  RTS  
                  
;*******************************************************************************
;* Guider LEDs ON                                                              *  
;*                                                                             *
;* This routine enables the guider LEDs such that readings of the sensor       *                 
;* correspond to the illuminated situation.                                    * 
;*                                                                             *
;* Passed: Nothing                                                             *             
;* Returns: Nothing                                                            *              
;* Side: PORTA bit 5 is changed                                                *  
;*******************************************************************************
                
G_LEDS_ON         BSET PORTA,%00100000 ; Set bit 5                                              
                  RTS                                                                           

;*******************************************************************************
;* Guider LEDs OFF                                                             *  
;*                                                                             *
;* This routine disables the guider LEDs. Readings of the sensor               *                 
;* correspond to the ambient lighting situation.                               * 
;*                                                                             *
;* Passed: Nothing                                                             *             
;* Returns: Nothing                                                            *              
;* Side: PORTA bit 5 is changed                                                *  
;*******************************************************************************

G_LEDS_OFF        BCLR PORTA,%00100000 ; Clear bit 5                                              
                  RTS                                                                            

;*******************************************************************************
;* Reading Sensors                                                             *
;*******************************************************************************


READ_SENSORS      CLR   SENSOR_NUM     ; Select sensor number 0
                  LDX   #SENSOR_LINE   ; Point at the start of the sensor array

RS_MAIN_LOOP      LDAA  SENSOR_NUM     ; Select the correct sensor input
                  JSR   SELECT_SENSOR  ; on the hardware
                  LDY   #200           ; 20 ms delay to allow the
                  JSR   del_50us       ; sensor to stabilize
                  LDAA  #%10000001     ; Start A/D conversion on AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done
                  LDAA  ATDDR0L        ; A/D conversion is complete in ATDDR0L
                  STAA  0,X            ; so copy it to the sensor register
                  CPX   #SENSOR_STBD   ; If this is the last reading
                  BEQ   RS_EXIT        ; Then exit
                  INC   SENSOR_NUM     ; Else, increment the sensor number
                  INX                  ; and the pointer into the sensor array
                  BRA   RS_MAIN_LOOP   ; and do it again

RS_EXIT           RTS


;*******************************************************************************
;* Select Sensors                                                              *
;*******************************************************************************  
   
SELECT_SENSOR     PSHA                ; Save the sensor number for the moment
                  LDAA PORTA          ; Clear the sensor selection bits to zeros
                  ANDA #%11100011
                  STAA TEMP           ; and save it into TEMP
                  PULA                ; Get the sensor number
                  ASLA                ; Shift the selection number left, twice
                  ASLA 
                  ANDA #%00011100     ; Clear irrelevant bit positions
                  ORAA TEMP           ; OR it into the sensor bit positions
                  STAA PORTA          ; Update the hardware
                  RTS


;*******************************************************************************
;* Display Sensors                                                             *
;*******************************************************************************

DP_FRONT_SENSOR   EQU BOT_LINE+0
DP_PORT_SENSOR    EQU BOT_LINE+3
DP_MID_SENSOR     EQU BOT_LINE+6
DP_STBD_SENSOR    EQU BOT_LINE+9
DP_LINE_SENSOR    EQU BOT_LINE+12

DISPLAY_SENSORS   LDAA  SENSOR_BOW        ; Get the FRONT sensor value
                  JSR   BIN2ASC           ; Convert to ascii string in D
                  LDX   #DP_FRONT_SENSOR  ; Point to the LCD buffer position
                  STD   0,X               ; and write the 2 ascii digits there
                  LDAA  SENSOR_PORT       ; Repeat for the PORT value
                  JSR   BIN2ASC
                  LDX   #DP_PORT_SENSOR
                  STD   0,X
                  LDAA  SENSOR_MID        ; Repeat for the MID value
                  JSR   BIN2ASC
                  LDX   #DP_MID_SENSOR
                  STD   0,X
                  LDAA  SENSOR_STBD       ; Repeat for the STARBOARD value
                  JSR   BIN2ASC
                  LDX   #DP_STBD_SENSOR
                  STD   0,X
                  LDAA  SENSOR_LINE       ; Repeat for the LINE value
                  JSR   BIN2ASC
                  LDX   #DP_LINE_SENSOR
                  STD   0,X
                  LDAA  #$C0              ; Move cursor to start of line 2
                  JSR   cmd2LCD

                  LDAA  #LCD_SEC_LINE     ; Position the LCD cursor on the second line
                  JSR   LCD_POS_CRSR
                  LDX   #BOT_LINE         ; Copy the buffer bottom line to the LCD
                  JSR   putsLCD
                  RTS

;*******************************************************************************
;* Update Display                                                              *                   
;*******************************************************************************

UPDT_DISPL        LDAA    #$88            ;  
                  JSR     cmd2LCD         ;
                  LDAB    CRNT_STATE      ; Display current state
                  LSLB                    ; "
                  LSLB                    ; "
                  LSLB
                  LDX     #tab            ; "
                  ABX                     ; "
                  JSR     putsLCD         ; "
                  RTS
                  
;*******************************************************************************
;* Timer Subroutines                                                           *                   
;*******************************************************************************

ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1           ; Enable TCNT
                  STAA    TFLG2           ; Clear TOF
                  LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS

TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000      ; Clear
                  STAA    TFLG2           ; TOF
                  RTI


;*******************************************************************************
;* LCD Subroutines                                                             *                   
;*******************************************************************************

; LCD Initialization
; --------------------
initLCD           BSET    DDRB,%11111111  ; configure pins PS7,PS6,PS5,PS4 for output
                  BSET    DDRJ,%11000000  ; configure pins PE7,PE4 for output
                  LDY     #2000
                  JSR     del_50us
                  LDAA    #$28
                  JSR     cmd2LCD
                  LDAA    #$0C
                  JSR     cmd2LCD
                  LDAA    #$06
                  JSR     cmd2LCD
                  RTS

; LCD clear
; --------------------
clrLCD            LDAA  #$01
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us
                  RTS
; LCD delay (50us)
; --------------------
del_50us          PSHX                   ; (2 E-clk) Protect the X register
eloop             LDX   #300             ; (2 E-clk) Initialize the inner loop counter
iloop             NOP                    ; (1 E-clk) No operation
                  DBNE X,iloop           ; (3 E-clk) If the inner cntr not 0, loop again
                  DBNE Y,eloop           ; (3 E-clk) If the outer cntr not 0, loop again
                  PULX                   ; (3 E-clk) Restore the X register
                  RTS                    ; (5 E-clk) Else return

; LCD config. commands
; --------------------
cmd2LCD           BCLR  LCD_CNTR, LCD_RS ; select the LCD instruction
                  JSR   dataMov          ; send data to IR
                  RTS

; Print string to LCD
; --------------------
putsLCD           LDAA  1,X+             ; get one character from string
                  BEQ   donePS           ; get NULL character
                  JSR   putcLCD
                  BRA   putsLCD

donePS            RTS

; Print char to LCD
; --------------------
putcLCD           BSET  LCD_CNTR, LCD_RS  ; select the LCD data register (DR)c
                  JSR   dataMov           ; send data to DR
                  RTS

; Send data to LCD
; --------------------
dataMov           BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the upper 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LSLA                    ; match the lower 4 bits with LCD data pins
                  LSLA                    ; ""
                  LSLA                    ; ""
                  LSLA                    ; ""
                  BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the lower 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LDY   #1                ; adding this delay allows
                  JSR   del_50us          ; completion of most instructions
                  RTS
                  
; LCD position cursor
; --------------------                  
LCD_POS_CRSR      ORAA #%10000000         ; Set the high bit of the control word
                  JSR cmd2LCD             ; and set the cursor address
                  RTS                  

;*******************************************************************************
;* Binary 16 to BCD Conversion Routine                                         *
;*                                                                             *
;* Because the IDIV (Integer Division) instruction is available on the HCS12,  *
;* we can determine the decimal digits by repeatedly dividing the binary       *
;* number by ten: the remainder each time is a decimal digit.                  *
;*                                                                             *
;* Conceptually, what we are doing is shifting the decimal number one place    *
;* to the right past the decimal point with each divide operation.             *
;*                                                                             *
;* The remainder must be a decimal digit between 0 and 9, because we divided   *
;* by 10.The algorithm terminates when the quotient has become zero.           *
;*******************************************************************************

int2BCD           XGDX             		      ; Save input number in X (exchange D and X)
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

CON_EXIT          RTS                       ; Return from subroutine

;*******************************************************************************
;* BCD to ASCII Conversion Routine                                             *
;*                                                                             *
;* This routine converts the BCD number in the BCD_BUFFER into ascii format,   *
;* Leading zeros are converted into space characters. The flag ’NO_BLANK’      *
;* starts cleared and is set once a non-zero digit has been detected.          *
;*                                                                             *
;* The ’UNITS’ digit is never blanked,                                         *
;* even if it and all the preceding digits are zero.                           *
;*******************************************************************************

BCD2ASC      	    LDAA    #0         			   ; Initialize the blanking flag
                	STAA    NO_BLANK

            ; CHECK DIGITS
            ; Ten-thousands digit     
C_TTHOU      	    LDAA 	  TEN_THOUS   	      ; Load digit
                  ORAA 	  NO_BLANK            ; OR with NO_BLANK (#0)
                  BNE  		NOT_BLANK1  		    ; Check if not zero

ISBLANK1    	    LDAA 	  #' '        		    ; If zero, replace with a blank space
                  STAA 	  TEN_THOUS
                  BRA  		C_THOU              ; Move to next digit (thousands)

NOT_BLANK1   	    LDAA 	  TEN_THOUS   		 
                  ORAA 	  #$30                ; Convert to ASCII (0x0[0-9] + 0b0011 0000)
                  STAA 	  TEN_THOUS           ; Store the ASCII hex code back in its buffer variable
                  LDAA 	  #$1         		
                  STAA 	  NO_BLANK            ; Signal that we have seen a ’non-blank’ digit (0x01 = true)

            ; Thousands digit
C_THOU       	    LDAA 	  THOUSANDS           ; Check the thousands digit for blankness
                  ORAA 	  NO_BLANK            ; If it’s blank and ’no-blank’ is still zero
                  BNE  		NOT_BLANK2

ISBLANK2        	LDAA 	  #' '                ; Only runs if 10 000s and 1000s is 0
                  STAA 	  THOUSANDS           ; if THOUS dig = 0, but TEN THOUS != 0, we dont want a blank here (ex. "10000" instead of "1    ")
                  BRA  		C_HUNS

NOT_BLANK2   	    LDAA 	  THOUSANDS           ; ASCII conversion
                  ORAA 	  #$30
                  STAA 	  THOUSANDS
                  LDAA    #$1
                  STAA    NO_BLANK

            ; Hundreds digit
C_HUNS       	    LDAA 	  HUNDREDS
                  ORAA 	  NO_BLANK
                  BNE  		NOT_BLANK3

ISBLANK3     	    LDAA 	  #' '                ; Only runs if 10 000s, 1000s. and 100s is 0
                  STAA 	  HUNDREDS
                  BRA  		C_TENS

NOT_BLANK3   	    LDAA 	  HUNDREDS            ; ASCII conversion
                  ORAA 	  #$30
                  STAA 	  HUNDREDS
                  LDAA 	  #$1
                  STAA 	  NO_BLANK

            ; Tens digit
C_TENS      	    LDAA 	  TENS            
                  ORAA 	  NO_BLANK
                  BNE  		NOT_BLANK4

ISBLANK4          LDAA 	  #' '                ; Only runs if 10 000s. 1000s, 100s, and 10s is 0
                  STAA 	  TENS
                  BRA  		C_UNITS

NOT_BLANK4   	    LDAA 	  TENS                ; ASCII conversion
                  ORAA 	  #$30
                  STAA 	  TENS

            ; Units digit
C_UNITS      	    LDAA 	  UNITS               ; No blank check necessary, ALWAYS convert to ascii.
                  ORAA 	  #$30
                  STAA 	  UNITS

             	    RTS              			      ; Return from subroutine
             	
;*******************************************************************************
;* Binary to ASCII routine                                                     *
;*                                                                             *
;* Converts an 8 bit binary value in ACCA to the equivalent ASCII character 2, *
;* character string in accumulator D.                                          *
;* Uses a table-driven method rather than various tricks.                      *
;*                                                                             *
;* Passed: Binary value in ACCA                                                *
;* Returns: ASCII Character string in D                                        *
;* Side Fx: ACCB is destroyed                                                  *
;*******************************************************************************

HEX_TABLE         FCC   '0123456789ABCDEF'    ; Table for converting values

BIN2ASC           PSHA                        ; Save a copy of the input number
                  TAB            
                  ANDB #%00001111             ; Strip off the upper nibble
                  CLRA                        ; D now contains 000n where n is the LSnibble
                  ADDD #HEX_TABLE             ; Set up for indexed load
                  XGDX                
                  LDAA 0,X                    ; Get the LSnibble character
                          
                  PULB                        ; Retrieve the input number into ACCB
                  PSHA                        ; and push the LSnibble character in its place
                  RORB                        ; Move the upper nibble of the input number
                  RORB                        ;  into the lower nibble position.
                  RORB
                  RORB 
                  ANDB #%00001111             ; Strip off the upper nibble
                  CLRA                        ; D now contains 000n where n is the MSnibble 
                  ADDD #HEX_TABLE             ; Set up for indexed load
                  XGDX                                                               
                  LDAA 0,X                    ; Get the MSnibble character into ACCA
                  PULB                        ; Retrieve the LSnibble character into ACCB
                  RTS

;***************************************************************************************************
;* Interrupt Vectors                                                                               *
;***************************************************************************************************
                  ORG     $FFFE
                  DC.W    Entry           ; Reset Vector
                      
                  ORG     $FFDE
                  DC.W    TOF_ISR         ; Timer Overflow Interrupt Vector