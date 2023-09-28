; output driver Viktor Peter Kovacs (kvp in 2018)
; Licence:
; Creative Commons Attribution Non-Commercial Share Alike (CC-BY-NC-SA-4.0)

; pins: red1, red2, white1, input (low: go), state out [active: go], (white2)

; configuration
    include P12F508.inc
    __CONFIG (_CP_OFF & _WDT_OFF & _MCLRE_OFF & _IntRC_OSC)
    radix decimal

; --- defines ---

#define PIN_IN (3)

; --- macros ---

nop2 macro
    goto $+1
    endm

nop4 macro
    nop2
    nop2
    endm

; --- data ---
; 0-6 : i/o registers, 7 : scratch pad
tmp      equ 7
cnt1     equ 8
cnt2     equ 9

; --- code ---
    org 0x00
rst                     ; reset vector
    movwf   OSCCAL
    movlw   64+128
    option              ; enable gp2 output mode (+disable timer)
    clrw                
    movwf   GPIO        ; set output pins to low
    movlw   8           ; set pin 3 to input, 0,1,2,4,5 to output
    tris    GPIO        ; set output mode
    clrw
    movwf   GPIO
wait_rdy
    btfss   GPIO, PIN_IN
    goto    wait_rdy
    goto    main

; 500 msec wait
blink
    movlw   177
    movwf   cnt1
blink2
    clrw
    movwf   cnt2
blink3
    nop4
    nop4
    decfsz  cnt2, F
    goto    blink3
    decfsz  cnt1, F
    goto    blink2
    retlw   0

main
stop
    movlw   1
    movwf   GPIO
    call    blink
    btfss   GPIO, PIN_IN
    goto    go
    nop2
    movlw   2
    movwf   GPIO
    call    blink
    btfss   GPIO, PIN_IN
    goto    go
    goto    stop
go
    movlw   4 + 16
    movwf   GPIO
    call    blink
    btfsc   GPIO, PIN_IN
    goto    stop
    nop2
    movlw   32 + 16
    movwf   GPIO
    call    blink
    btfsc   GPIO, PIN_IN
    goto    stop
    goto    go

    end
