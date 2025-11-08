;********************************************************************
;* F2025 - COE538 Lab 4.1 - Motor Control (HCS12, 9S32C)            *
;*                                                                  *
;*    This program creates subroutines to control the eebot drive   *
;* motors                                                           *
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

              ORG     $3850
             
;********************************************************************
;* Code section                                                     *
;********************************************************************
              ORG     $4000
Entry:
_Startup:
                                                    ; P  S
              BSET    DDRA,%00000011                ; A0,A1 : 0=input, 1=output , dir(fwd/rev) 
              BSET    DDRT,%00110000                ; T4,T5 : 0=input, 1=output , spd(on/off)
              
              JSR     STARFWD
              JSR     PORTFWD
              
              JSR     STARON
              JSR     PORTON
              
              ;JSR     STARREV
              ;JSR     PORTREV
              
              ;JSR     STAROFF
              ;JSR     PORTOFF
              
              BRA     *
              
              SWI

;************************************************************
;* Motor Control Routines                                   *
;************************************************************
              
              
STARON        LDAA PTT                           ; Starboard ON (T5)
              ORAA #%00100000                    ; 1 = on
              STAA PTT
              RTS
STAROFF       LDAA PTT                           ; Starboard OFF            
              ANDA #%11011111                    ; 0 = off
              STAA PTT
              RTS
              

PORTON        LDAA PTT                           ; Port ON (T4)
              ORAA #%00010000                    ; 1 = on
              STAA PTT
              RTS
PORTOFF       LDAA PTT                           ; Port OFF
              ANDA #%11101111                    ; 0 = off
              STAA PTT
              RTS
              
                                                              
PORTFWD       LDAA PORTA                         ; Port FWD (A0)
              ANDA #%11111110                    ; 0 = fwd
              STAA PORTA
              RTS
PORTREV       LDAA PORTA
              ORAA #%00000001                    ; Port REV
              STAA PORTA                         ; 1 = rev
              RTS
                       

STARFWD       LDAA PORTA                         ; Starboard FWD (A1)
              ANDA #%11111101                    ; 0 = fwd
              STAA PORTA
              RTS
STARREV       LDAA PORTA
              ORAA #%00000010                    ; Starboard REV
              STAA PORTA                         ; 1 = rev
              RTS

;**************************************************************
;* Interrupt Vectors                                          *
;**************************************************************
              
              ORG     $FFFE
              DC.W    Entry               ; Reset Vector
