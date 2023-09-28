; output driver Viktor Peter Kovacs (kvp in 2018)
; Licence:
; Creative Commons Attribution Non-Commercial Share Alike (CC-BY-NC-SA-4.0)

; notes:
; 4 Mhz, 1 mips, 9600 baud
; 1 bit = 104.166 inst ~ 104 inst
; 1/3 bit = 34.722 inst ~ 35 inst
;
; packet format:
;  [addr7 | 128], [data7], [chksum7]

; configuration
    include P12F508.inc
    __CONFIG (_CP_OFF & _WDT_OFF & _MCLRE_OFF & _IntRC_OSC)
    radix decimal

; --- defines ---

#define ADDRESS (0x42)

#define PIN_SERIAL (3)

; --- macros ---

nop2 macro
    goto $+1
    endm

nop4 macro
    goto $+1
    goto $+1
    endm
     
wait1_3bit macro
    local wait1_3bit_1
    movlw   10
    movwf   tmp
wait1_3bit_1
    decfsz  tmp, F
    goto    wait1_3bit_1
    nop
    endm
     
wait1bit macro
    local wait1bit_1
    movlw   31
    movwf   tmp
wait1bit_1
    decfsz  tmp, F
    goto    wait1bit_1
    nop2
    endm

; --- data ---
; 0-6 : i/o registers, 7 : scratch pad
tmp      equ 7
recv     equ 8
pkt0     equ 9
pkt1     equ 10
pkt2     equ 11

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
wait_serial_rdy
    btfss   GPIO, PIN_SERIAL
    goto    wait_serial_rdy
    clrw
    movwf   GPIO
    goto    main

recv_bit
    wait1bit
    clrc
    btfsc   GPIO, PIN_SERIAL
    setc
    rrf     recv, F
    retlw   0

recv_byte
    btfsc   GPIO, PIN_SERIAL
    goto    recv_byte
    wait1_3bit
    btfsc   GPIO, PIN_SERIAL
    goto    recv_byte
    clrf    recv
    call    recv_bit
    call    recv_bit
    call    recv_bit
    call    recv_bit
    call    recv_bit
    call    recv_bit
    call    recv_bit
    call    recv_bit
recv_byte_stop
    btfss   GPIO, PIN_SERIAL
    goto    recv_byte_stop
    retlw   0

main
    call    recv_byte
    ; check for start byte
    btfss   recv, 7
    goto    main
main_1
    movfw   recv
    movwf   pkt0
    ; check start bit and address
    xorlw   ADDRESS + 128
    skpz
    goto    main
    call    recv_byte
    ; check for start byte
    btfsc   recv, 7
    goto    main_1
    movfw   recv
    movwf   pkt1
    call    recv_byte
    ; check for start byte
    btfsc   recv, 7
    goto    main_1
    movfw   recv
    movwf   pkt2
    ; check checksum: (addr + data + chks) & 127 == 0
    movfw   pkt0
    addwf   pkt1, W
    addwf   pkt2, W
    andlw   127
    skpz
    goto    main
    ; move bits 01234xxx to 012x54xx
    movfw   pkt1
    andlw   1+2+4
    btfsc   pkt1, 3
    iorlw   32
    btfsc   pkt1, 4
    iorlw   16
    movwf   GPIO
    goto    main

    end
