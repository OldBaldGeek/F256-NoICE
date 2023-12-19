; 65C02 Debug monitor for Foenix F256jr, for use with NOICE02
;
; This monitor uses only 65C02 instructions supported by WDC and Rockwell
;
; Copyright (c) 2023 by John Hartman
;
; Modification History:
;   8-Dec-23 JLH Ported to run on Foenix F256jr board
;
;============================================================================
; This file has been assembled with the Merlin32 assembler:
;   merlin32 -V Mon6502_Merlin.asm
;
; This version of the monitor is designed to run in RAM on the F256jr.
; Run it with
;      python fnxmgr.zip --run-pgz Mon6502_F256_Merlin.pgz
; The monitor proper is built at physical address 7/Fxxx.
; It includes a boot chunk that runs at $0200. This maps the 7/Fxxx code
; to processor address F000, so that the monitor can access the IRQ/BRK vector,
; which is used for breakpoints and single-step.
; It contains code to revector IRQ and NMI interrupts through RAM vectors
; to user code. See RAMVEC below
;
; To customize for other targets or configurations, refer to the NoICE help
; file monitor.htm
;
; This monitor uses no page0 RAM, since it is a scarce resource.
; It tries to minimize stack usage for the same reason. In some cases
; this may result in code that looks clunky. See "HACKBUF" below...
;
; Some notes on interrupts and the BRK instructions
; - Even though BRK is a one byte instruction, when a BRK occurs the PC
;   is incremented by 2 before being pushed.  If this is not true for your
;   processor, change the "SBC #2" instruction after INT_ENTRY to "SBC #1"
; - When a BRK occurs, the "B" bit is set BEFORE the status register is
;   pushed.  If this is not true for your processor, remove the "PLA/PHA"
;   after .IRQ and .NMI and repalce them with "PHP/PLA"
; - If an interrupt is pending when a BRK is executed, PC will be loaded
;   with the vector for the INTERRUPT, not for BRK.  Thus, both .IRQ
;   and .NMI check for the break bit.  (This occurs on the 740.  According
;   to the Rockwell databook, it occurs on the 6502, but not the 65C02)
;
;============================================================================
        mx      %11
        org     $0
        dsk     Mon6502_F256_Merlin.pgz
        db      'Z'             ; PGZ header upper case Z means 24 bit size/length fields

; Where in physical RAM to load the monitor
PAGE_BASE       equ     $7E000
MAP_PAGE        equ     PAGE_BASE/$2000

RAM_START       EQU     PAGE_BASE+$1800   ;START OF MONITOR RAM
CODE_START      EQU     PAGE_BASE+$1900   ;START OF MONITOR CODE
HARD_VECT       EQU     PAGE_BASE+$1FFA   ;START OF HARDWARE VECTORS

; NoICE supports one "window" for banked memory, while the F256jr has 8 8K
; windows. This monitor is configured with a single memory window, generally
; 8K or 16K in size. If your program doesn't need to load or manipulate
; memory pages, set both BANK_LOW and BANK_HIGH to 0.
;
; For more information on NoICE banked memory, refer to the NoICE help
; file 2bitmmu.htm
BANK_LOW        EQU     $A000   ;Bottom of memory window (must be a multiple of 8K)
BANK_HIGH       EQU     $BFFF   ;top of memory window
BANK_IN_MMU     equ     BANK_LOW/$2000

; STACK RAM (PAGE 1)
; Monitor use is at most 7 bytes.
; This stack is shared with user programs.
; This should need to be changed only on system with less than a full
; page of stack RAM.
INITSTACK       EQU     $01FF   ;TOP OF STACK RAM

;==========================================================================
; Equates for the F256jr's MMU and I/O mapping
MMU_MEM_CTRL    equ     $0000
MMU_EDIT_EN     equ     $80     ; Set to allow editing of MLUTS
MMU_EDIT_MASK   equ     $30     ; Mask for LUT to be edited
MMU_ACT_MASK    equ     $03     ; Mask for active LUT

MMU_IO_CTRL     equ     $0001
MMU_IO_DISABLE  equ     $04     ; Clear for I/O in bank 6, set for memory
MMU_IO_PAGE_MASK equ    $03     ; Mask for I/O page

MMU_LUTS        equ     $08     ; Start of LUT registers

;==========================================================================
; Equates for F256jr's Buzzer and Status LEDs
SYS0    equ     $D6A0           ; Bit0 is the power LED, bit1 is the SD LED
SYS1    equ     $D6A1

;==========================================================================
; Equates for F256jr's 16750-compatible serial port
;
S16750  equ     $D630           ;Base of 16750-compatible UART
RXR     equ     0               ;  Receiver buffer register
TXR     equ     0               ;  Transmitter buffer register
IER     equ     1               ;  Interrupt enable register
LCR     equ     3               ;  Line control register
MCR     equ     4               ;  Modem control register
LSR     equ     5               ;  Line status register
;
RXRDY   equ     $01             ; LSR bit mask for RX buffer full
TXRDY   equ     $20             ; LSR bit mask for TX buffer empty

;===========================================================================
; HARDWARE PLATFORM INDEPENDENT EQUATES
;
; Communications function codes.
FN_GET_STATUS   equ     $FF     ;reply with device info
FN_READ_MEM     equ     $FE     ;reply with data
FN_WRITE_MEM    equ     $FD     ;reply with status (+/-)
FN_READ_REGS    equ     $FC     ;reply with registers
FN_WRITE_REGS   equ     $FB     ;reply with status
FN_RUN_TARGET   equ     $FA     ;reply (delayed) with registers
FN_SET_BYTES    equ     $F9     ;reply with data (truncate if error)
FN_IN           equ     $F8     ;input from port
FN_OUT          equ     $F7     ;output to port
;
FN_MIN          equ     $F0     ;MINIMUM RECOGNIZED FUNCTION CODE
FN_ERROR        equ     $F0     ;error reply to unknown op-code
;
; 6502 OP-CODE EQUATES for code building in HACKBUF
B               equ     $10     ;Break bit in condition codes
LDA_OP          equ     $AD     ;LDA AAA
STA_OP          equ     $8D     ;STA AAA
CMP_OP          equ     $CD     ;CMP AAA
LDAY_OP         equ     $B9     ;LDA AAA,Y
STAY_OP         equ     $99     ;STA AAA,Y
CMPY_OP         equ     $D9     ;CMP AAA,Y
RTS_OP          equ     $60     ;RTS

;============================================================================
; Bootstrap from low RAM
        org     $0
        adr     boot_start              ; Address to load into memory
        adr     boot_end - boot_start   ; Length of data to load there

        org     $200
boot_start
        SEI
        CLD
        LDX     #INITSTACK & $FF
        TXS
;
; Map the monitor to $E000 (windows 7) of the current LUT
        LDA     MMU_MEM_CTRL
        AND     #MMU_ACT_MASK
        STA     BOOT_TEMP        ;active MLUT
        ASL
        ASL
        ASL
        ASL
        ORA     BOOT_TEMP
        ORA     #MMU_EDIT_EN    ;Enable MMU edit of the active LUT
        STA     MMU_MEM_CTRL
        LDA     #MAP_PAGE
        STA     MMU_LUTS + 7

        STZ     MMU_IO_CTRL
        LDA     #$03
        STA     SYS0

; Jump to the monitor in its new location
        JMP     RESET

BOOT_TEMP  DS   0
boot_end

;============================================================================
; RAM definitions
        org     $0
        adr     data_start              ; Address to load into memory
        adr     data_end - data_start   ; Length of data to load there

        org     RAM_START
data_start
;
; RAM interrupt vectors
; (first in segment for easy addressing, else move to their own segment)
RAMVEC          ds      2*3
;
; Target registers: order must match that used by NoICE on the PC
TASK_REGS
REG_STATE       ds      1
REG_PAGE        ds      1
REG_SP          ds      2
REG_Y           ds      1
REG_X           ds      1
REG_A           ds      1
REG_CC          ds      1
REG_PC          ds      2
TASK_REG_END
TASK_REGS_SIZE  EQU    TASK_REG_END-TASK_REGS
;
; In order that we need no page zero RAM, we do memory access via an
; instruction built into RAM.  Build instruction and RTS here
HACKBUF         ds      4        ;Room for "LDA xxxx,Y; RTS"
;
; Store a counter for input timeout
RXTIMER         ds      2
RXWIGGLE        ds      1
;
; Save MMU values while monitor is active
MMU_TEMP         ds     1
SAVE_MMU_MEM_CTL ds     1
SAVE_MMU_IO_CTL  ds     1
;
; Communications buffer
; (Must be at least as long as TASK_REG_SZ.  At least 19 bytes recommended.
; Larger values may improve speed of NoICE memory commands.)
COMBUF_SIZE     equ     128             ;DATA SIZE FOR COMM BUFFER
COMBUF          ds      2+COMBUF_SIZE+1 ;BUFFER ALSO HAS FN, LEN, AND CHECK
;
data_end
;
;===========================================================================
; Code
        org     $0
        adr     main_code_start                   ; Address to load into memory
        adr     main_code_end - main_code_start   ; Length of data to load into their

        org     CODE_START
main_code_start
;
; Power on reset
RESET

; TODO: we may need to include a signature of "KERNEL" and some other
; code from F256_MicroKernel-master\f256\jr.asm
; If run-from RAM is mediated by the Flash kernel...

;
; Set CPU mode to safe state
        SEI                     ;INTERRUPTS OFF
        CLD                     ;USE BINARY MODE
;
; INIT STACK
        LDX     #INITSTACK & $FF
        TXS

;===========================================================================
; INITIALIZE F256jr HARDWARE
        JSR     MAP_IO
        STZ     RXWIGGLE
;
; Initialize 16750 UART
;
; Access baud generator, no parity, 1 stop bit, 8 data bits
        LDA     #$83            ;10000011B
        STA     S16750+LCR
;
; Table 15.6 Shows divisors for different baud rates
        LDA     #13             ;115.2 kbaud
        STA     S16750+RXR      ;lsb
        LDA     #0
        STA     S16750+RXR+1    ;msb
;
; Access data registers, no parity, 1 stop bits, 8 data bits
        LDA     #$03            ;00000011B
        STA     S16750+LCR
;
; No loopback, OUT2 on, OUT1 on, RTS on, DTR on
        LDA     #$0F            ;00001111B
        STA     S16750+MCR
;
; Disable all interrupts: modem, receive error, transmit, and receive
        LDA     #$00            ;00000000B
        STA     S16750+IER
        JSR     RESTORE_IO

;===========================================================================
; INIT RAM INTERRUPT VECTORS TO DUMMY HANDLERS
        LDA     #_NMIX & $FF    ;NMI
        STA     RAMVEC+0
        LDA     #_NMIX / $100
        STA     RAMVEC+0+1
;
        LDA     #_IRQX & $FF    ;IRQ
        STA     RAMVEC+4
        LDA     #_IRQX / $100
        STA     RAMVEC+4+1
;
; Initialize user registers
        LDX     #INITSTACK & $FF
        STX     REG_SP          ;INIT USER'S STACK POINTER
        LDX     #INITSTACK / $100
        STX     REG_SP+1
        STZ     REG_PC
        STZ     REG_PC+1
        STZ     REG_A
        STZ     REG_X
        STZ     REG_Y
        LDA     #$04
        STA     REG_CC          ;interrupts disabled
        STZ     REG_STATE       ;STATE IS 0 = RESET
        JSR     SET_MMU         ;Set REG_PAGE with current MMU content
        JSR     RESTORE_MMU
;
; Set function code for "GO".  Then if we reset after being told to
; GO, we will come back with registers so user can see the crash
        LDA     #FN_RUN_TARGET
        STA     COMBUF
        JMP     RETURN_REGS     ;DUMP REGS, ENTER MONITOR

;===========================================================================
; Save user's MMU setting and enable editing (but don't set LUT)
SET_MMU
        LDA     MMU_MEM_CTRL
        STA     SAVE_MMU_MEM_CTL ;Remember user's setting

        AND     #MMU_ACT_MASK
        STA     MMU_TEMP        ;active MLUT
        ASL
        ASL
        ASL
        ASL
        ORA     MMU_TEMP
        ORA     #MMU_EDIT_EN    ;Enable MMU edit - we are editing the active
        STA     MMU_MEM_CTRL
        LDA     MMU_LUTS + BANK_IN_MMU
        STA     REG_PAGE        ;Remember user's mapping
        ; TODO: handle multi-8K window if desired
        RTS

;===========================================================================
; Restore user's LUT and MMU settings
RESTORE_MMU
        LDA     REG_PAGE
        STA     MMU_LUTS + BANK_IN_MMU
        ; TODO: handle multi-8K window if desired

        LDA     SAVE_MMU_MEM_CTL
        STA     MMU_MEM_CTRL
        RTS

;===========================================================================
; Save user's I/O configuration and map our I/O, bank 0
MAP_IO  LDA     MMU_IO_CTRL
        STA     SAVE_MMU_IO_CTL
        STZ     MMU_IO_CTRL
        RTS

;===========================================================================
; Restore user's I/O configuration
RESTORE_IO
        LDA     SAVE_MMU_IO_CTL
        STA     MMU_IO_CTRL
        RTS

;===========================================================================
; Get a character to A
;
; Return A=char, CY=0 if data received
;        CY=1 if timeout (0.5 seconds)
;
; Uses 2 bytes of stack including return address
;
GETCHAR LDA     #0              ;LONG TIMEOUT
        STA     RXTIMER
        STA     RXTIMER+1
GC_10   DEC     RXTIMER
        BNE     GC_20           ;BR IF NOT TIMEOUT
        DEC     RXTIMER+1       ;ELSE DEC HIGH HALF
        BEQ     GC_90           ;EXIT IF TIMEOUT
GC_20   LDA     S16750+LSR      ;READ DEVICE STATUS
        AND     #RXRDY
        BEQ     GC_10           ;NOT READY YET.
;
; Data received:  return CY=0. data in A
        LDA     S16750+RXR      ;READ DATA
        CLC
        RTS
;
; Timeout: toggle the power LED as a sign of life, return CY=1
GC_90   LDA     RXWIGGLE
        INC
        AND     #1
        STA     RXWIGGLE
        STA     SYS0
        SEC
        RTS
;
;===========================================================================
; Output character in A
;
; Uses 3 bytes of stack including return address
;
PUTCHAR PHA
PC_10   LDA     S16750+LSR      ;CHECK TX STATUS
        AND     #TXRDY          ;RX READY ?
        BEQ     PC_10
        PLA
        STA     S16750+TXR      ;TRANSMIT CHAR.
        RTS
;
;===========================================================================
; Response string for GET TARGET STATUS request
; Reply describes target:
TSTG    db      7                   ;2: PROCESSOR TYPE = 65(C)02
        db      COMBUF_SIZE         ;3: SIZE OF COMMUNICATIONS BUFFER
        db      $80                 ;4: has CALL
        dw      BANK_LOW,BANK_HIGH  ;5-8: LOW AND HIGH LIMIT OF MAPPED MEMORY
        db      B1-B0               ;9 BREAKPOINT INSTR LENGTH
;
; Define either the BRK or JSR BRKE instruction for use as breakpoint
; TODO: Arnold assembles BRK as two bytes: 00 EA.  We want a ONE byte breakpoint
; so we do it by hand.
B0      db      $00                  ;10+ BREKAPOINT INSTRUCTION (BRK)
B1      asc     '6502 monitor for F256jr V3.11111'    ;DESCRIPTION, ZERO
        db      0
        db      0                   ;page of CALL breakpoint
        dw      B0                  ;address of CALL breakpoint in native order
B2 
TSTG_SIZE       EQU     B2-TSTG     ;SIZE OF STRING
;
;===========================================================================
; Enter here via JSR for breakpoint:  PC is stacked.
; Stacked PC points at JSR+1
;;BRKE  STA     REG_A           ;SAVE ACCUM FROM DIRECT ENTRY
;;      PHP                     ;SAVE CC'S AS IF AFTER A BRK INSTRUCTION
;;      SEC
;
; Common handler for default interrupt handlers
; Enter with A=interrupt code = processor state
; PC and CC are stacked.
; REG_A has pre-interrupt accumulator
; Stacked PC points at BRK+2 if BRK, else at PC if entry from interrupt
; A  has state
INT_ENTRY
;
; Set CPU mode to safe state
        SEI
        CLD
;
; Save registers in reg block for return to PC
        STA     REG_STATE       ;SAVE MACHINE STATE
        PLA                     ;CONDITION CODES
        STA     REG_CC
        PLA                     ;LSB OF PC OF BREAKPOINT
        STA     REG_PC
        PLA                     ;MSB OF PC OF BREAKPOINT
        STA     REG_PC+1
;
; If this is a breakpoint (state = 1), then back up PC to point at BRK
        LDA     REG_STATE       ;SAVED STATUS FOR TESTING
        CMP     #1
        BNE     B99             ;BR IF NOT BREAKPOINT: PC IS OK
;
; On the 6502, BRK leaves PC at break address +2:  back it up by 2
; If your core leaves PC at break address +1, back up by 1
        SEC
        LDA     REG_PC          ;BACK UP PC TO POINT AT BREAKPOINT
        SBC     #2
        STA     REG_PC
        LDA     REG_PC+1
        SBC     #0
        STA     REG_PC+1
B99     JMP     ENTER_MON       ;REG_PC POINTS AT BREAKPOINT OPCODE
;
;===========================================================================
;
; Main loop:  wait for command frame from PC
;
; Uses 4 bytes of stack before jump to functions
;
MAIN
;
; Save user's I/O mapping and enable I/O acess to the UART
        JSR     MAP_IO
;
; Since we have only part of a page for stack, we run on the target's
; stack.  Thus, reset to target SP, rather than our own.
MA_10   LDX     REG_SP
        TXS
        LDX     #0              ;INIT INPUT BYTE COUNT
;
; First byte is a function code
        JSR     GETCHAR         ;GET A FUNCTION
        BCS     MA_10           ;JIF TIMEOUT: RESYNC
        CMP     #FN_MIN
        BCC     MA_10           ;JIF BELOW MIN: ILLEGAL FUNCTION
        STA     COMBUF,X        ;SAVE FUNCTION CODE
        INX
;
; Second byte is data byte count (may be zero)
        JSR     GETCHAR         ;GET A LENGTH BYTE
        BCS     MA_10           ;JIF TIMEOUT: RESYNC
        CMP     #COMBUF_SIZE+1
        BCS     MA_10           ;JIF TOO LONG: ILLEGAL LENGTH
        STA     COMBUF,X        ;SAVE LENGTH
        INX
        CMP     #0
        BEQ     MA_80           ;SKIP DATA LOOP IF LENGTH = 0
;
; Loop for data
        TAY                     ;SAVE LENGTH FOR LOOP
MA_20   JSR     GETCHAR         ;GET A DATA BYTE
        BCS     MA_10           ;JIF TIMEOUT: RESYNC
        STA     COMBUF,X        ;SAVE DATA BYTE
        INX
        DEY
        BNE     MA_20
;
; Get the checksum
MA_80   JSR     GETCHAR         ;GET THE CHECKSUM
        BCS     MA_10           ;JIF TIMEOUT: RESYNC
        STA     HACKBUF         ;SAVE CHECKSUM
;
; Compare received checksum to that calculated on received buffer
; (Sum should be 0)
        JSR     CHECKSUM
        CLC
        ADC     HACKBUF
        BNE     MA_10           ;JIF BAD CHECKSUM
        JSR     RESTORE_IO
;
; Process the message.
        LDA     COMBUF+0        ;GET THE FUNCTION CODE
        CMP     #FN_GET_STATUS
        BEQ     TARGET_STATUS
        CMP     #FN_READ_MEM
        BEQ     JREAD_MEM
        CMP     #FN_WRITE_MEM
        BEQ     JWRITE_MEM
        CMP     #FN_READ_REGS
        BEQ     JREAD_REGS
        CMP     #FN_WRITE_REGS
        BEQ     JWRITE_REGS
        CMP     #FN_RUN_TARGET
        BEQ     JRUN_TARGET
        CMP     #FN_SET_BYTES
        BEQ     JSET_BYTES
        CMP     #FN_IN
        BEQ     JIN_PORT
        CMP     #FN_OUT
        BEQ     JOUT_PORT
;
; Error: unknown function.
        LDA     #FN_ERROR
        STA     COMBUF          ;SET FUNCTION AS "ERROR"
        LDA     #1
        JMP     SEND_STATUS     ;VALUE IS "ERROR"
;
; long jumps to handlers too far for BEQ
JREAD_MEM       JMP     READ_MEM
JWRITE_MEM      JMP     WRITE_MEM
JREAD_REGS      JMP     READ_REGS
JWRITE_REGS     JMP     WRITE_REGS
JRUN_TARGET     JMP     RUN_TARGET
JSET_BYTES      JMP     SET_BYTES
JIN_PORT        JMP     IN_PORT
JOUT_PORT       JMP     OUT_PORT

;===========================================================================
;
; Target Status:  FN, len
TARGET_STATUS
        LDX     #0              ;DATA FOR REPLY
        LDY     #TSTG_SIZE      ;LENGTH OF REPLY
        STY     COMBUF+1        ;SET SIZE IN REPLY BUFFER
TS_10   LDA     TSTG,X          ;MOVE REPLY DATA TO BUFFER
        STA     COMBUF+2,X
        INX
        DEY
        BNE     TS_10
;
; Compute checksum on buffer, and send to PC, then return
        JMP     SEND

;===========================================================================
;
; Read Memory:  FN, len, page, Alo, Ahi, Nbytes
;
READ_MEM
;
; Build "LDA  AAAA,Y" in RAM
        LDA     #LDAY_OP
        STA     HACKBUF+0
;
; Insert address to be read into the instruction in RAM
        LDA     COMBUF+3
        STA     HACKBUF+1
        LDA     COMBUF+4
        STA     HACKBUF+2
        BEQ     RM_10
;
; Address is not on page 0: make the MMU visible, set page
        JSR     SET_MMU
        LDA     COMBUF+2
        STA     MMU_LUTS + BANK_IN_MMU
;
; Set return after LDA
RM_10   LDA     #RTS_OP
        STA     HACKBUF+3
;
; Prepare return buffer: FN (unchanged), LEN, DATA
        LDX     COMBUF+5        ;NUMBER OF BYTES TO GET
        STX     COMBUF+1        ;RETURN LENGTH = REQUESTED DATA
        BEQ     RM_90           ;JIF NO BYTES TO GET
;
; Read the requested bytes from local memory
        LDY     #0              ;INITIAL OFFSET
RM_20   JSR     HACKBUF         ;GET BYTE AAAA,Y TO A
        STA     COMBUF+2,Y      ;STORE TO RETURN BUFFER
        INY
        DEX
        BNE     RM_20
;
RM_90   LDA     HACKBUF+2
        BEQ     RM_99
        JSR     RESTORE_MMU     ;back to user's MMU settings
  
; Compute checksum on buffer, and send to PC, then return
RM_99   JMP     SEND

;===========================================================================
;
; Write Memory:  FN, len, page, Alo, Ahi, (len-3 bytes of Data)
;
; Uses 5 bytes of stack including return address
;
WRITE_MEM
;
; Build "STA  AAAA,Y" in RAM
        LDA     #STAY_OP
        STA     HACKBUF+0
;
; Set address into RAM
        LDA     COMBUF+3
        STA     HACKBUF+1
        LDA     COMBUF+4
        STA     HACKBUF+2
        BEQ     WM_10
;
; Address is not on page 0: make the MMU visible, set page
        JSR     SET_MMU
        LDA     COMBUF+2
        STA     MMU_LUTS + BANK_IN_MMU
;
; Set return after LDA
WM_10   LDA     #RTS_OP
        STA     HACKBUF+3
;
; Prepare return buffer: FN (unchanged), LEN, DATA
        LDX     COMBUF+1        ;NUMBER OF BYTES TO PUT
        DEX                     ;LESS PAGE, ADDRLO, ADDRHI
        DEX
        DEX
        BEQ     WM_50           ;JIF NO BYTES TO PUT
;
; Write the specified bytes to local memory
        LDY     #0              ;INITIAL OFFSET
WM_20   LDA     COMBUF+5,Y      ;GET BYTE TO WRITE
        JSR     HACKBUF         ;STORE THE BYTE AT AAAA,Y
        INY
        DEX
        BNE     WM_20
;
; Build "CMP  AAAA,Y" in RAM
        LDA     #CMPY_OP
        STA     HACKBUF+0
;
; Compare to see if the write worked
        LDX     COMBUF+1        ;NUMBER OF BYTES TO PUT
        DEX                     ;LESS PAGE, ADDRLO, ADDRHI
        DEX
        DEX
        LDY     #0              ;INITIAL OFFSET
WM_30   LDA     COMBUF+5,Y      ;GET BYTE JUST WRITTEN
        JSR     HACKBUF         ;COMPARE THE BYTE AT AAAA,Y
        BNE     WM_80           ;BR IF WRITE FAILED
        INY
        DEX
        BNE     WM_30
;
; Write succeeded:  return status = 0
WM_50   LDA     #0              ;RETURN STATUS = 0
        JMP     WM_90
;
; Write failed:  return status = 1
WM_80   LDA     #1
;
WM_90   PHA
        LDA     HACKBUF+2
        BEQ     RM_99
        JSR     RESTORE_MMU     ;back to user's MMU settings
;
; Compute checksum on buffer, and send to PC, then return
WM_99   PLA
        JMP     SEND_STATUS

;===========================================================================
;
; Read registers:  FN, len=0
;
; Uses 5 bytes of stack including return address
;
READ_REGS
;
; Enter here from "RUN" and "STEP" to return task registers
RETURN_REGS
        LDX     #0              ;REGISTER LIVE HERE
        LDY     #TASK_REGS_SIZE ;NUMBER OF BYTES
        STY     COMBUF+1        ;SAVE RETURN DATA LENGTH
;
; Copy the registers
RR_10   LDA     TASK_REGS,X     ;GET BYTE TO A
        STA     COMBUF+2,X      ;STORE TO RETURN BUFFER
        INX
        DEY
        BNE     RR_10
;
; Compute checksum on buffer, and send to PC, then return
        JMP     SEND

;===========================================================================
;
; Write registers:  FN, len, (register image)
;
; Uses 5 bytes of stack including return address
;
WRITE_REGS
;
        LDX     #0              ;POINTER TO DATA
        LDY     COMBUF+1        ;NUMBER OF BYTES
        BEQ     WR_90           ;JIF NO REGISTERS
;
; Copy the registers
WR_10   LDA     COMBUF+2,X      ;GET BYTE TO A
        STA     TASK_REGS,X     ;STORE TO REGISTER RAM
        INX
        DEY
        BNE     WR_10
;
; Update the MMU, in case REG_PAGE has changed
        JSR     SET_MMU         ;Make MMU visible
        JSR     RESTORE_MMU     ;Set page to REG_PAGE
;
; Reload SP, in case it has changed
        LDX     REG_SP
        TXS
;
; Return OK status
WR_90   LDA     #0
        JMP     SEND_STATUS

;===========================================================================
;
; Run Target:  FN, len
;
; Uses 3 bytes of stack for user PC and CC before RTI
;
RUN_TARGET
        LDX     REG_SP          ;BACK TO USER STACK
        TXS
        LDA     REG_PC+1        ;PUSH USER PC FOR RTI
        PHA
        LDA     REG_PC
        PHA
        LDA     REG_CC          ;PUSH USER CONDITION CODES FOR RTI
        PHA
;
; Restore registers
        LDX     REG_X
        LDY     REG_Y
        LDA     REG_A
;
; Return to user
        RTI
;
;===========================================================================
;
; Common continue point for all monitor entrances
; REG_STATE, REG_A, REG_CC, REG_PC set; X, Y intact; SP = user stack
ENTER_MON
        STX     REG_X
        STY     REG_Y
        TSX
        STX     REG_SP          ;SAVE USER'S STACK POINTER (LSB)
        LDA     #1              ;STACK PAGE ALWAYS 1
        STA     REG_SP+1        ;(ASSUME PAGE 1 STACK)
;
        JSR     SET_MMU         ;Make MMU visible, update REG_PAGE
        JSR     RESTORE_MMU     ;Restore user settings
;
; Return registers to PC
        JMP     RETURN_REGS

;===========================================================================
;
; Set target byte(s):  FN, len { (page, alow, ahigh, data), (...)... }
;
; Return has FN, len, (data from memory locations)
;
; If error in insert (memory not writable), abort to return short data
;
; This function is used primarily to set and clear breakpoints
;
; Uses 5 bytes of stack including return address
;
SET_BYTES
        LDY     COMBUF+1        ;LENGTH = 4*NBYTES
        BEQ     SB_90           ;JIF NO BYTES
;
; Loop on inserting bytes
        LDX     #0              ;INDEX INTO INPUT BUFFER
        LDY     #0              ;INDEX INTO OUTPUT BUFFER
;
; Build "LDA  AAAA" in RAM
SB_10   LDA     #LDA_OP
        STA     HACKBUF+0
;
; Set address
        LDA     COMBUF+3,X
        STA     HACKBUF+1
        LDA     COMBUF+4,X
        STA     HACKBUF+2
        BEQ     SB_20
;
; Address is not on page 0: make the MMU visible, set page
        JSR     SET_MMU
        LDA     COMBUF+2,X
        STA     MMU_LUTS + BANK_IN_MMU
;
; Set return after LDA
SB_20   LDA     #RTS_OP
        STA     HACKBUF+3
;
; Read current data at byte location
        JSR     HACKBUF         ;GET BYTE AT AAAA
        STA     COMBUF+2,Y      ;SAVE IN RETURN BUFFER
;
; Insert new data at byte location
;
; Build "STA  AAAA" in RAM
        LDA     #STA_OP
        STA     HACKBUF+0
        LDA     COMBUF+5,X      ;BYTE TO WRITE
        JSR     HACKBUF
;
; Verify write
        LDA     #CMP_OP
        STA     HACKBUF+0
        LDA     COMBUF+5,X
        JSR     HACKBUF
        PHP                     ;SAVE SUCCESS/FAIL
;
SB_40   LDA     HACKBUF+2
        BEQ     SB_50
        JSR     RESTORE_MMU     ;back to user's MMU settings
SB_50   PLP
        BNE     SB_90           ;BR IF INSERT FAILED: ABORT AT Y BYTES
;
; Loop for next byte
        INY                     ;COUNT ONE INSERTED BYTE
        INX                     ;STEP TO NEXT BYTE SPECIFIER
        INX
        INX
        INX
        CPX     COMBUF+1
        BNE     SB_10           ;LOOP FOR ALL BYTES
;
; Return buffer with data from byte locations
SB_90   STY     COMBUF+1        ;SET COUNT OF RETURN BYTES
;
; Compute checksum on buffer, and send to PC, then return
        JMP     SEND

;===========================================================================
;
; Input from port:  FN, len, PortAddressLo, PAhi (=0)
;
; While the 6502 has no input or output instructions, we retain these
; to allow write-without-verify, useful for I/O devices
;
; Uses 5 bytes of stack including return address
;
IN_PORT
;
; Build "LDA  AAAA" in RAM
        LDA     #LDA_OP
        STA     HACKBUF+0
;
; Set port address
        LDA     COMBUF+2
        STA     HACKBUF+1
        LDA     COMBUF+3
        STA     HACKBUF+2
;
; Set return after LDA
        LDA     #RTS_OP
        STA     HACKBUF+3
;
; Read the requested byte from local memory
        JSR     HACKBUF         ;GET BYTE TO A
;
; Return byte read as "status"
        JMP     SEND_STATUS

;===========================================================================
;
; Output to port:  FN, len, PortAddressLo, PAhi (=0), data
;
; While the 6502 has no input or output instructions, we retain these
; to allow write-without-verify, useful for I/O devices
;
; Uses 5 bytes of stack including return address
;
OUT_PORT
;
; Build "STA  AAAA" in RAM
        LDA     #STA_OP
        STA     HACKBUF+0
;
; Set port address
        LDA     COMBUF+2
        STA     HACKBUF+1
        LDA     COMBUF+3
        STA     HACKBUF+2
;
; Set return after STA
        LDA     #RTS_OP
        STA     HACKBUF+3
;
; Get data
        LDA     COMBUF+4
;
; Write value to port
        JSR     HACKBUF         ;PUT BYTE FROM A
;
; Do not read port to verify (some I/O devices don't like it)
;
; Return status of OK
        LDA     #0
        JMP     SEND_STATUS

;===========================================================================
; Build status return with value from "A"
;
; Uses 5 bytes of stack including return address
;
SEND_STATUS
        STA     COMBUF+2        ;SET STATUS
        LDA     #1
        STA     COMBUF+1        ;SET LENGTH
        JMP     SEND

;===========================================================================
; Append checksum to COMBUF and send to PC
;
; Uses 5 bytes of stack including return address
;
SEND    JSR     MAP_IO          ;Save user's I/O mapping, enable ours
        JSR     CHECKSUM        ;GET A=CHECKSUM, X->checksum location
        EOR     #$FF
        CLC
        ADC     #1
        STA     COMBUF,X        ;STORE NEGATIVE OF CHECKSUM
;
; Send buffer to PC
        LDX     #0              ;POINTER TO DATA
        LDY     COMBUF+1        ;LENGTH OF DATA
        INY                     ;PLUS FUNCTION, LENGTH, CHECKSUM
        INY
        INY
SE_10   LDA     COMBUF,X
        JSR     PUTCHAR         ;SEND A BYTE
        INX
        DEY
        BNE     SE_10
;
        JSR     RESTORE_IO
        JMP     MAIN            ;BACK TO MAIN LOOP

;===========================================================================
; Compute checksum on COMBUF.  COMBUF+1 has length of data,
; Also include function byte and length byte
;
; Returns:
;      A = checksum
;      X = pointer to next byte in buffer (checksum location)
;      Y is scratched
;
; Uses 2 bytes of stack including return address
;
CHECKSUM
        LDX     #0              ;pointer to buffer
        LDY     COMBUF+1        ;length of message
        INY                     ;plus function, length
        INY
        LDA     #0              ;init checksum to 0
CK_10   CLC
        ADC     COMBUF,X
        INX
        DEY
        BNE     CK_10           ;loop for all
        RTS                     ;return with checksum in A

;**********************************************************************
;
; VECTORS THROUGH RAM
;
; IRQ/BREAK.  VECTOR THROUGH RAM
; First, check for BRK (breakpoint) interrupt
_IRQ    CLD
        STA     REG_A           ;SAVE A
;
; Test B bit in STACKED condition codes
        PLA
        PHA                     ;GET COPY OF PRE-INT STATUS REG
        AND     #B
        BNE     GOBREAKP        ;SET: DO BREAKPOINT CODE
        LDA     REG_A           ;ELSE RESTORE ACC.
        JMP     (RAMVEC+4)      ;JUMP THROUGH RAM VECTOR
;
; IRQ NOT SET BY USER CODE (ADDRESS PLACED IN RAMVEC+04H BY INIT)
_IRQX   LDA     #2              ;TARGET STOP TYPE
        JMP     GOBREAK         ;COMPLAIN, ENTER MONITOR
;
; NMI.  VECTOR THROUGH RAM
; First, check for BRK (breakpoint) interrupt
_NMI    CLD
        STA     REG_A           ;SAVE A
;
; Test B bit in STACKED condition codes
        PLA
        PHA                     ;GET COPY OF PRE-INT STATUS REG
        AND     #B
        BNE     GOBREAKP        ;SET: DO BREAKPOINT CODE
        LDA     REG_A           ;ELSE RESTORE ACC.
        JMP     (RAMVEC+0)      ;JUMP THROUGH RAM VECTOR
;
; NMI NOT SET BY USER CODE (ADDRESS PLACED IN RAMVEC+00H BY INIT)
_NMIX   LDA     #3              ;TARGET STOP TYPE
        JMP     GOBREAK         ;ALWAYS DO BREAKPOINT CODE
;
; BREAK ENTRY.
GOBREAKP
        LDA     #1              ;STATE IS "BREAKPOINT"
GOBREAK
        JMP     INT_ENTRY       ;ENTER MONITOR

main_code_end
;
================================================================================
; INTERRUPT VECTORS
        org     $0
        adr     vec_start       ; Address to load into memory
        adr     6               ; Length: Math of vec_end - vec_start blows up, since vec_end wraps

        org     HARD_VECT
vec_start
        dw      _NMI            ;FFFA NMI
        dw      RESET           ;FFFC RESET
        dw      _IRQ            ;FFFE IRQ, BRK
vec_end
;
        org     $0
        adr     boot_start      ; Start address
        adr     0               ; length of zero means "run me"

        END     RESET
