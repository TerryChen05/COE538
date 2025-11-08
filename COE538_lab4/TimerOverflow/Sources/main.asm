;********************************************************************
;* F2025 - COE538 Lab 4.2 - Timer Overflow (HCS12, 9S32C)           *   
;*                                                                  *                                               
;*    This program tests the function of the hardware timer.        *
;* Creates a 5 second delay                                         *
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
DT_DEMO       EQU 115                 ; 5 second delay (at 23Hz)
                 

;********************************************************************
;* Code section                                                     *
;********************************************************************
              ORG     $4000
Entry:
_Startup:


START         JSR     ENABLE_TOF          ; Start the TOF interrupt
              CLI                         ; Enable global interrupts
              LDAA    #$0
              STAA    TOF_COUNTER         ; initialize counter to be non-garbage value
              
INIT_DELAY    LDAA    TOF_COUNTER         ; Initialize the alarm time
              ADDA    #DT_DEMO            ; by adding on the delay
              STAA    AT_DEMO             ; and save it in the alarm
              
CHK_DELAY     LDAA    TOF_COUNTER         ; If the current time
              CMPA    AT_DEMO             ; equals the alarm time
              BEQ     STOP_HERE           ; then stop here
              
              NOP                         ; Do something during the display
              BRA     CHK_DELAY           ; and check the alarm again
              
STOP_HERE     SWI                         ; Done, break to the monitor      
                       
                                          

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
              STD     $FFDE             ; 
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

;**************************************************************
;* Interrupt Vectors                                          *
;**************************************************************
              ORG     $FFDE 
              FDB     TOF_ISR
              
              ORG     $FFFE
              DC.W    Entry               ; Reset Vector
