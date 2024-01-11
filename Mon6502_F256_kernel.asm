; 65C02 Debug monitor for Foenix F256jr, for use with NOICE02
;
; Copyright (c) 2024 by John Hartman
;
; Modification History:
;  10-Jan-2024 JLH Ported to run on Foenix F256jr board
;
;============================================================================
; This file may be assembled with the Merlin32 assembler:
;    merlin32 -V Mon6502_Merlin.asm
;
; This version of the monitor is designed to run in conjunction with the
; micorkernel on the F256jr.
; The pgx contains
; - a stub, orged at $00200 to configure and run the monitor
; - the body of the monitor, orged at 0F800 (a hole in the kernel)
; - a start block specifying the stub's start address.
;
; We note that the top 8K of the kernel has about 3K of unused space below
; the soft and hard vectors. We tuck the monitor there so that we don't have
; to deal with mapping hassles, since the top 8K of the kernel is always mapped.
;
; The stub doctors the IRQ and NMI vectors to pass to the monitor.
; It then remaps blocks 7 in LUTs 0, 1, and 3 to use the RAM copy of the
; kernel rather than the Flash.
;
; Start the monitor with
;    python fnxmgr.zip --run-pgz Mon6502_F256_kernel.pgz
; This will usually be done by NoICE's Options, Extensions
;
; To customize for other targets or configurations, refer to the NoICE help
; file monitor.htm
;
; This monitor uses no page zero RAM, since that is a scarce resource belonging
; to the program being debugged. It tries to minimize stack usage for the same
; reason. In some cases this may result in code that looks clunky.
; See "HACKBUF" below...
;
; * We use BRK for breakpoints and single-step.
; * We use NMI to force entry to the monitor. NMI may be caused by a RESTORE
;   key on a CBM keyboard, or a push-button switch on J5 between RESTORE (pin 3)
;   and ground (pin 1).
;
; Information about F256jr ports used here is from f256jr_ref.pdf,
; dated 31-Jan-2023.
;
;============================================================================
        mx      %11
        org     $0
        dsk     Mon6502_F256_kernel.pgz
        db      'Z'             ;PGZ header upper case Z means 24 bit size/length fields

; MMU pages for tops of Flash and RAM
FLASH_TOP       EQU     $7F     ;Physical addresses FE000-FFFFF
RAM_TOP         EQU     $07     ;Physical addresses 0E000-0FFFF

; Where in physical RAM to load the monitor
MON_START       EQU     $0F800  ;Start of monitor variables and code

; STACK RAM (PAGE 1)
; This monitor uses at most ? bytes of stack.
; This stack is shared with user programs.
INITSTACK       equ     $01FF   ;TOP OF STACK RAM

;==========================================================================
; Equates for the F256jr's MMU and I/O mapping
mmu_ctrl        equ     $0000
MMU_EDIT_EN     equ     $80     ; Set to allow editing of MLUTS
MMU_EDIT_MASK   equ     $30     ; Mask for LUT to be edited
MMU_ACT_MASK    equ     $03     ; Mask for active LUT

io_ctrl         equ     $0001
MMU_IO_DISABLE  equ     $04     ; Clear for I/O in bank 6, set for memory
MMU_IO_PAGE_MASK equ    $03     ; Mask for I/O page

mmu             equ     $08     ; Start of LUT registers

VKY_MSTR_CTRL_0 equ     $D000   ; VICKY master control registers
VKY_MSTR_CTRL_1 equ     $D001

;============================================================================
; Boot loader: prepare the monitor, and start it.
        org     $0
        adr     boot_start              ;Address to load into memory
        adr     boot_end - boot_start   ;Length of data to load there

; Unlike the monitor, boot loader can use DP
save_mmu_ctrl equ $20
save_io_ctrl  equ $21
save_mmu_1    equ $22
boot_temp     equ $24

SRC_ADDR    equ  $26
DST_ADDR    equ  $28
LIMIT_ADDR  equ  $2A
text_cursor equ  $2C
line_start  equ  $2E
MEM_ADDR    equ  $30

        org     $200
boot_start
        sei
        cld
        ldx     #<INITSTACK
        txs
;
; Remember initial conditions to be restored below
        lda     mmu_ctrl
        sta     save_mmu_ctrl
        lda     io_ctrl
        sta     save_io_ctrl
;
; Show initial MMU and I/O state
        jsr     InitTextDisplay

        ldx     #>mon_title
        lda     #<mon_title
        jsr     StringToScreen

        ldx     #>mmu_title
        lda     #<mmu_title
        jsr     StringToScreen
        lda     save_mmu_ctrl
        jsr     HexToScreen

        ldx     #>io_title
        lda     #<io_title
        jsr     StringToScreen
        lda     save_io_ctrl
        jsr     HexToScreen
;
; Show the contents of the four MMU LUTS
        jsr     DumpMMU
;
; Enable the active LUT for editing
        lda     mmu_ctrl
        and     #MMU_ACT_MASK
        sta     boot_temp
        asl
        asl
        asl
        asl
        ora     boot_temp
        ora     #MMU_EDIT_EN
        sta     mmu_ctrl
;
; The LUTs are only initialized by a power-on. Reset doesn't affect them.
; Experience shows that an F256 configured "boot from Flash" will come up
; using LUT 3, and one configured "boot from RAM" will come up using LUT 0.
; We use these to distinguish boot type, and how we operate.
        and     #MMU_ACT_MASK
        beq     :boot_ram
        cmp     #3
        beq     :boot_flash
;
; Unsupported configuration
        ldx     #>nobo_title
        lda     #<nobo_title
        jsr     StringToScreen
:b_ram  bra     :b_ram

;------------------------------------------------------------------------------
; Boot from RAM
:boot_ram
        ldx     #>rambo_title
        lda     #<rambo_title
        jsr     StringToScreen
;
; The monitor has been loaded into 0E000. Map it via MMU 7
        lda     #RAM_TOP
        sta     mmu+7
;
; The reset vector was most likely set to boot_start by fnxmgr.
; Set it to the monitor's reset, so that reset button will run the monitor.
        lda     #<main_code_start
        sta     $FFFC
        lda     #>main_code_start
        sta     $FFFD

; Set NMI and IRQ vectors
        lda     #<NMI_HAN
        sta     $FFFA
        lda     #>NMI_HAN
        sta     $FFFB

        lda     #<IRQ_HAN
        sta     $FFFE
        lda     #>IRQ_HAN
        sta     $FFFF

        ldx     #>vec_title
        lda     #<vec_title
        jsr     StringToScreen
;
; Without the monitor, user code would take control out of reset, with
; interrupts disabled. Tell the monitor to start user programs that way.
        lda     #$04
        sta     INIT_PS
;
; Map all four LUTS - same as for Flash case
        jmp     update_luts

;------------------------------------------------------------------------------
; Boot from Flash.
:boot_flash
;
; Top block may or may not be Flash, since only power-on reset initializes it.
; 
; The monitor has been loaded into physical memory at 0E000. Map it via MMU 7
        lda     #RAM_TOP
        sta     mmu+7
;
; Map the top of the kernel's flash at 2000 so we can access it
        lda     mmu + 1
        sta     save_mmu_1      ;remember the original mapping for restore later
        lda     #FLASH_TOP
        sta     mmu + 1
;
; The monitor was loaded into the RAM block as part of this PGZ.
; Merge in the portion of the Flash kernel below the monitor.
        lda     #>$2000                 ;Kernel in Flash
        sta     SRC_ADDR+1
        stz     SRC_ADDR

        lda     #>MON_START - $C000     ;Convert $E000 to $2000
        sta     LIMIT_ADDR+1
        lda     #<MON_START - $C000
        sta     LIMIT_ADDR

        lda     #>$E000
        sta     DST_ADDR+1
        stz     DST_ADDR
        jsr     copy_mem
;
; Verify that there are enough zeros in the kernel to accomodate the monitor
        lda     #>MON_END - $C000
        sta     LIMIT_ADDR+1
        lda     #<MON_END - $C000
        sta     LIMIT_ADDR
        jsr     test_mem_zeros
        beq     :b_20
        
        ; Complain about lack of space, and stop
        ldx     #>error_title
        lda     #<error_title
        jsr     StringToScreen
        lda     SRC_ADDR+1
        ora     #$E0
        jsr     HexToScreen
        lda     SRC_ADDR
        jsr     HexToScreen
:die    bra     :die
;
; Merge the rest of the kernel, up to FFFF
:b_20   lda     SRC_ADDR+1
        ora     #$E0            ;Convert 3XXX to FXXX
        sta     DST_ADDR+1
        lda     SRC_ADDR
        sta     DST_ADDR
        lda     #>$4000
        sta     LIMIT_ADDR+1
        lda     #<$4000
        sta     LIMIT_ADDR
        jsr     copy_mem
        ldx     #>merge_title
        lda     #<merge_title
        jsr     StringToScreen
;
; Copy the kernel's IRQ vector into the monitor's IRQ redirect.
        ldx     #>irq_title
        lda     #<irq_title
        jsr     StringToScreen
        lda     $3FFE           ;IRQ vector in kernel Flash
        sta     MON_IRQ
        jsr     HexToScreen
        lda     $3FFF
        sta     MON_IRQ+1
        jsr     HexToScreen
;
; Copy the monitor's IRQ and NMI handler addresses into the hardware vectors.
; Leave the kernel's reset vector (copied above) alone, so that reset will
; restart the kernel.
        lda     #<NMI_HAN
        sta     $FFFA
        lda     #>NMI_HAN
        sta     $FFFB

        lda     #<IRQ_HAN
        sta     $FFFE
        lda     #>IRQ_HAN
        sta     $FFFF

        ldx     #>vec_title
        lda     #<vec_title
        jsr     StringToScreen
;
; The kernel has to run with interrupts enabled, so our first thought was
; to initialize user code that way. But that seems to prevent startup.
; For now, require user program to explicitly enable interrupts.
        lda     #$04
        sta     INIT_PS
;
; Restore the mapping of slot 1, used above
        lda     mmu_ctrl        ;configured to edit the current LUT
        lda     save_mmu_1
        sta     mmu + 1
;
; Update slot 7 of ALL FOUR LUTs to map our RAM, since BRK or NMI can happen anywhere.
update_luts
        ldx     #RAM_TOP
        stx     mmu + 7

        lda     mmu_ctrl
        eor     #$10
        sta     mmu_ctrl
        stx     mmu + 7

        lda     mmu_ctrl
        eor     #$30
        sta     mmu_ctrl
        stx     mmu + 7

        lda     mmu_ctrl
        eor     #$10
        sta     mmu_ctrl
        stx     mmu + 7

        lda     save_mmu_ctrl
        sta     mmu_ctrl
;
; Show the modified LUTS
        ldx     #>map_title
        lda     #<map_title
        jsr     StringToScreen
        jsr     DumpMMU

        ldx     #>go_title
        lda     #<go_title
        jsr     StringToScreen
;
; Restore MMU and IO, then start the monitor
        lda     save_mmu_ctrl
        sta     mmu_ctrl
        lda     save_io_ctrl
        sta     io_ctrl

        jmp     main_code_start

mon_title asc   'NoICE 65C02 monitor for F256jr. V1.0',00
mmu_title ASC   '  mmu_ctrl:',00
io_title  ASC   '  io_ctrl:',00

rambo_title ASC   0A,'Booting monitor in RAM',00
nobo_title  ASC   0A,'ERROR: Cannot boot from that LUT/mmu_ctrl',00

error_title ASC   0A,'ERROR: Flash Kernel does not have expected zeros at ',00
merge_title ASC   0A,'Merged Flash kernel with monitor',00

irq_title   ASC   0A,'Copied kernel IRQ handler ',00
vec_title   ASC   0A,'Updated vectors',00
map_title   ASC   0A,'Updated slot 7 in all four LUT',00
go_title    ASC   0A,'Starting NoICE monitor',00

;============================================================================
; Increment SRC_ADDR, test against LIMIT_ADDR
; Return Z=1 if limit is reached
; Preserves x,y
inc_src_addr
        inc     SRC_ADDR
        bne     :isa_10
        inc     SRC_ADDR+1
:isa_10 lda     SRC_ADDR
        cmp     LIMIT_ADDR
        bne     :isa_90
        lda     SRC_ADDR+1
        cmp     LIMIT_ADDR+1
:isa_90 rts

;============================================================================
; Copy memory between SRC_ADDR and LIMIT_ADDR to DST_ADDR 
; Preserves x,y
copy_mem
:cm_10  lda     (SRC_ADDR)
        sta     (DST_ADDR)
        inc     DST_ADDR
        bne     :cm_20
        inc     DST_ADDR+1
:cm_20  jsr     inc_src_addr
        bne     :cm_10
        rts

;============================================================================
; Test memory between SRC_ADDR and LIMIT_ADD for zeros
; Exit with Z=1 if all bytes are zero; else Z=0
; Preserves x,y
test_mem_zeros
:tm_10  lda     (SRC_ADDR)
        bne     :tm_90
        jsr     inc_src_addr
        bne     :tm_10
:tm_90  rts

;============================================================================
; Fill memory between SRC_ADDR and LIMIT_ADD with the value in X
; Preserves x,y
fill_mem
:fm_10  txa
        sta     (SRC_ADDR)
        jsr     inc_src_addr
        bne     :fm_10
        rts

;============================================================================
; Dump 16 bytes of memory from x:a
; Uses MEM_ADDR as a memory pointer
dump_hex
        stx     MEM_ADDR+1
        sta     MEM_ADDR
        jsr     new_line
        lda     MEM_ADDR+1
        jsr     HexToScreen
        lda     MEM_ADDR
        jsr     HexToScreen
        ldy     #0
:dh_10  lda     #$20
        jsr     CharToScreen
        lda     (MEM_ADDR),y
        jsr     HexToScreen
        iny
        cpy     #16
        bne     :dh_10
        rts

;============================================================================
; Initialize for text display
; Returns with I/O page 2 selected to allow display access
InitTextDisplay
        stz     io_ctrl         ; main I/O page
        lda     #$01
        sta     VKY_MSTR_CTRL_0 ; VICKY in text mode
        stz     VKY_MSTR_CTRL_1 ; single height/width characters

        lda     #2
        sta     io_ctrl         ; swap in the text memory
;
; Fill text display memory with spaces
        lda     #>$C000
        sta     SRC_ADDR+1
        stz     SRC_ADDR

        lda     #>$D000
        sta     LIMIT_ADDR+1
        stz     LIMIT_ADDR

        ldx     #' '
        jsr     fill_mem
;
; Init cursor and line pointers
        lda     #>$C000
        sta     line_start+1
        sta     text_cursor+1
        stz     line_start
        stz     text_cursor
        rts

;============================================================================
; Start a new line
; Preserves x,y
;
; We COULD check for end of screen, and scroll or reset to the top, but
; we stick with simple limited output for boot diagnostics
new_line
        lda     line_start
        clc
        adc     #80
        sta     line_start
        sta     text_cursor
        lda     line_start+1
        adc     #0
        sta     line_start+1
        sta     text_cursor+1
        rts

;============================================================================
; Show the character in A on the text screen, advance cursor
; Handles newline ($0A)
; Preserves x,y
CharToScreen
        cmp     #$0A
        beq     new_line
        sta     (text_cursor)
        ; Fall thru to inc_cursor

;============================================================================
; Advance screen cursor
; Preserves a,x,y
inc_cursor
        inc     text_cursor
        bne     :rts
        inc     text_cursor+1
:rts    rts

;============================================================================
; Show the null-terminated string X:A on the text screen
; Uses MEM_ADDR as a string pointer
StringToScreen
        sta     MEM_ADDR
        stx     MEM_ADDR+1
:lp     lda     (MEM_ADDR)
        beq     :exit
        jsr     CharToScreen
        inc     MEM_ADDR
        bne     :lp
        inc     MEM_ADDR+1
        bra     :lp
:exit   rts

;============================================================================
; Show A as two hex characters on the text screen
; Preserves x,y
HexToScreen
        pha
        lsr
        lsr
        lsr
        lsr
        ora     #'0'
        cmp     #'9'+1
        bcc     :hs_10          ;BR IF DIGIT 0 TO 9
        adc     #6              ;ELSE A-F: ADD 6 + CY = 7
:hs_10  jsr     CharToScreen
        pla
        and     #$0F
        ora     #'0'
        cmp     #'9'+1
        bcc     :hs_20
        adc     #6
:hs_20  jmp     CharToScreen

;============================================================================
; Dump all four LUTs
; Returns mmu_ctrl unchanged
;
DumpMMU lda     mmu_ctrl
        pha
        stz     boot_temp
:dm_10  jsr     new_line

        lda     boot_temp
        lsra
        lsra
        lsra
        lsra
        ora     #'0'
        jsr     CharToScreen
        lda     #':'
        jsr     CharToScreen

        lda     mmu_ctrl
        and     #MMU_ACT_MASK
        ora     #MMU_EDIT_EN
        ora     boot_temp
        sta     mmu_ctrl

        ldx     #0
:dm_20  lda     #' '
        jsr     CharToScreen
        lda     8,x
        jsr     HexToScreen
        inx
        cpx     #8
        bne     :dm_20

        lda     boot_temp
        clc
        adc     #$10
        sta     boot_temp
        cmp     #$40
        bne     :dm_10

        pla
        sta     mmu_ctrl
        rts

boot_end

;==========================================================================
;==========================================================================
; Equates for F256jr's Buzzer and Status LEDs
SYS0    equ     $D6A0           ; Bit0 is the power LED, bit1 is the SD LED
SYS1    equ     $D6A1

;==========================================================================
; Equates for F256jr's 16750-compatible serial port
S16750  equ     $D630           ;Base of 16750-compatible UART
RXR     equ     0               ;  Receiver buffer register
TXR     equ     0               ;  Transmitter buffer register
IER     equ     1               ;  Interrupt enable register
LCR     equ     3               ;  Line control register
MCR     equ     4               ;  Modem control register
LSR     equ     5               ;  Line status register
;
RXRDY   equ     $01             ;LSR bit mask for RX buffer full
TXRDY   equ     $20             ;LSR bit mask for TX buffer empty

;============================================================================
; NoICE monitor in high memory
        org     $0
        adr     MON_START               ;Address to load into memory
        adr     MON_END - MON_START     ;Length of data to load there

        org     MON_START
;
; Re-vector for IRQ that are not BRK.
; If boot-from-Flash, this will be set to the kernel's IRQ handler
; If boot-from-RAM, UNHAN_IRQ is a dummy handler that enters the monitor.
; If boot-from-RAM, user code can change this in order to handle non-BRK IRQ
MON_IRQ         dw      UNHAN_IRQ       ; IRQ re-vector if IRQ isn't BRK
;
; Initial value of user's PS register (may be changed by boot stub)
INIT_PS         db      $04     ;Default to interrupts disabled

;
NMI_NESTING     ds      1       ;poor-man's mutex to control NMI processing
NMI_COUNT       ds      1       ;Total number of NMI see (for debounce evaluation)
;
; Target registers: order must match that used by NoICE on the PC
TASK_REGS
REG_STATE       ds      1
REG_PAGE        ds      1
REG_SP          ds      2
REG_Y           ds      1
REG_X           ds      1
REG_A           ds      1
REG_PS          ds      1
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
; Communications buffer. If you can't afford 128 bytes, COMBUF_SIZE must be
; at least as long as the larger of TASK_REG_SIZE and TSTG_SIZE.
; Larger values may improve speed of NoICE memory commands.
COMBUF_SIZE     equ     128             ;DATA SIZE FOR COMM BUFFER
COMBUF          ds      2+COMBUF_SIZE+1 ;BUFFER ALSO HAS FN, LEN, AND CHECK
;
;===========================================================================
; Monitor Code
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
; 65C02 OP-CODE EQUATES for code building in HACKBUF
B               equ     $10     ;Break bit in condition codes
LDA_OP          equ     $AD     ;LDA AAA
STA_OP          equ     $8D     ;STA AAA
CMP_OP          equ     $CD     ;CMP AAA
LDAY_OP         equ     $B9     ;LDA AAA,Y
STAY_OP         equ     $99     ;STA AAA,Y
CMPY_OP         equ     $D9     ;CMP AAA,Y
RTS_OP          equ     $60     ;RTS

main_code_start
;
; Set CPU mode to safe state
        SEI                     ;INTERRUPTS OFF
        CLD                     ;USE BINARY MODE
;
; INIT STACK
        LDX     #INITSTACK & $FF
        TXS
;
; Ignore any NMI that occur while the monitor is active
        STZ     NMI_NESTING
        STZ     NMI_COUNT

;===========================================================================
; INITIALIZE F256jr HARDWARE (since we may take control upon reset)
;
; Map I/O page 0. Turn on SD and Power LEDs, turn off L1 and L0
        STZ     io_ctrl
        LDA     #$03
        STA     SYS0
;
; Stop blinking L1/L0
        LDA     SYS1
        ORA     #$03
        STA     SYS1
        STZ     RXWIGGLE
;
; Initialize 16750 UART
;
; Access baud generator, no parity, 1 stop bit, 8 data bits
        LDA     #$83            ;10000011B
        STA     S16750+LCR
;
; F256jr manual Table 15.6 Shows divisors for different baud rates
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

;===========================================================================
;
; Initialize user registers
        LDA     #INITSTACK & $FF
        STA     REG_SP          ;INIT USER'S STACK POINTER
        LDA     #INITSTACK / $100
        STA     REG_SP+1
        STZ     REG_PC          ;Init PC to 200, since 0000 is the MMU
        LDA     #2
        STA     REG_PC+1
        STZ     REG_A
        STZ     REG_X
        STZ     REG_Y
        LDA     INIT_PS         ;initial value to control interrupt enable
        STA     REG_PS
        STZ     REG_STATE       ;STATE 0 = RESET
;
; Set function code for "GO".  Then if we reset after being told to
; GO, we will come back with registers so user can see the crash
        LDA     #FN_RUN_TARGET
        STA     COMBUF
        JMP     RETURN_REGS     ;DUMP REGS, ENTER MONITOR

;===========================================================================
; Save user's I/O configuration and map our I/O, bank 0
;
; Uses 2 bytes of stack including return address
;
MAP_IO  LDA     io_ctrl
        STA     SAVE_MMU_IO_CTL
        STZ     io_ctrl
        RTS

;===========================================================================
; Restore user's I/O configuration
;
; Uses 2 bytes of stack including return address
;
RESTORE_IO
        LDA     SAVE_MMU_IO_CTL
        STA     io_ctrl
        RTS

;===========================================================================
; Get a character to A
;
; Assumes that I/O page 0 is mapped
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
        BNE     GC_20
        DEC     RXTIMER+1
        BEQ     GC_90           ;EXIT IF TIMEOUT
GC_20   LDA     S16750+LSR
        AND     #RXRDY
        BEQ     GC_10           ;NOT READY YET.
;
; Data received:  return CY=0. data in A
        LDA     S16750+RXR
        CLC
        RTS
;
; Timeout: toggle the power LED as a sign of life, return CY=1
GC_90   LDA     SYS0
        AND     #$0E            ;Preserve L1,L0,SD LED states
        ORA     RXWIGGLE
        STA     SYS0
        INC                     ;Flip PWR bit for next time
        AND     #1
        STA     RXWIGGLE
        SEC
        RTS
;
;===========================================================================
; Output character in A
;
; Assumes that I/O page 0 is mapped
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
        dw      0,0                 ;5-8: NoICE memory banking not used
        db      B1-B0               ;9 BREAKPOINT INSTR LENGTH
;
; Define either the BRK or JSR BRKE instruction for use as breakpoint
B0      db      $00                 ;10+ BREKAPOINT INSTRUCTION (BRK)
B1      asc     '65C02 monitor for F256jr RAM Boot. V3.2',00 ;DESCRIPTION, ZERO
        db      0
        db      0                   ;page of CALL breakpoint
        dw      B0                  ;address of CALL breakpoint in native order
B2 
TSTG_SIZE       EQU     B2-TSTG     ;SIZE OF STRING
;
;===========================================================================
; Common handler for default interrupt handlers
; Enter with:
; - A = interrupt code = processor state
; - REG_A has pre-interrupt accumulator
; - 3 bytes on stack: PC and CC
; - Stacked PC points at BRK+2 if BRK, else at next instruction if from interrupt
INT_ENTRY
;
; Save registers in reg block for return to PC
        STA     REG_STATE       ;SAVE MACHINE STATE
        PLA                     ;CONDITION CODES
        STA     REG_PS
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
; On the 65C02, BRK leaves PC at break address +2: back it up to the op-code
        SEC
        LDA     REG_PC
        SBC     #2
        STA     REG_PC
        LDA     REG_PC+1
        SBC     #0
        STA     REG_PC+1
B99     JMP     ENTER_MON
;
;===========================================================================
;
; Main loop:  wait for command frame from PC
;
; Uses 2 bytes of stack before jump to functions
;
MAIN
;
; Save user's I/O mapping and enable I/O acess to the UART
        JSR     MAP_IO          ;2 stack
;
; Since we have only part of a page for stack, we run on the target's
; stack. Thus, reset to target's SP, rather than our own.
MA_10   LDX     REG_SP
        TXS
        LDX     #0              ;INIT INPUT BYTE COUNT
;
; First byte is a function code
        JSR     GETCHAR         ;2 stack. GET A FUNCTION
        BCS     MA_10           ;JIF TIMEOUT: RESYNC
        CMP     #FN_MIN
        BCC     MA_10           ;JIF BELOW MIN: ILLEGAL FUNCTION
        STA     COMBUF,X        ;SAVE FUNCTION CODE
        INX
;
; Second byte is data byte count (may be zero)
        JSR     GETCHAR         ;2 stack. GET A LENGTH BYTE
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
MA_20   JSR     GETCHAR         ;2 stack. GET A DATA BYTE
        BCS     MA_10           ;JIF TIMEOUT: RESYNC
        STA     COMBUF,X        ;SAVE DATA BYTE
        INX
        DEY
        BNE     MA_20
;
; Get the checksum
MA_80   JSR     GETCHAR         ;2 stack. GET THE CHECKSUM
        BCS     MA_10           ;JIF TIMEOUT: RESYNC
        STA     HACKBUF         ;SAVE CHECKSUM
;
; Compare received checksum to that calculated on received buffer
; (Sum should be 0)
        JSR     CHECKSUM        ;2 stack
        CLC
        ADC     HACKBUF
        BNE     MA_10           ;JIF BAD CHECKSUM
        JSR     RESTORE_IO      ;2 stack
;
; Process the message.
        LDA     COMBUF+0        ;FUNCTION CODE
        CMP     #FN_GET_STATUS
        BEQ     TARGET_STATUS
        CMP     #FN_READ_MEM
        BEQ     READ_MEM
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
;
; Uses 3 bytes of stack
;
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
; Uses 3 bytes of stack
;
READ_MEM
;
; Build "LDA  AAAA,Y" in RAM
        LDA     #LDAY_OP
        STA     HACKBUF+0
;
; Insert address to be read into the instruction in RAM
; NoICE memory banking is not used
        LDA     COMBUF+3
        STA     HACKBUF+1
        LDA     COMBUF+4
        STA     HACKBUF+2
;
; Set return after LDA
        LDA     #RTS_OP
        STA     HACKBUF+3
;
; Prepare return buffer: FN (unchanged), LEN, DATA
        LDX     COMBUF+5        ;NUMBER OF BYTES TO GET
        STX     COMBUF+1        ;RETURN LENGTH = REQUESTED DATA
        BEQ     RM_99           ;JIF NO BYTES TO GET
;
; Read the requested bytes from local memory
        LDY     #0              ;INITIAL OFFSET
RM_20   JSR     HACKBUF         ;GET BYTE AAAA,Y TO A
        STA     COMBUF+2,Y      ;STORE TO RETURN BUFFER
        INY
        DEX
        BNE     RM_20
;
; Compute checksum on buffer, and send to PC, then return
RM_99   JMP     SEND

;===========================================================================
;
; Write Memory:  FN, len, page, Alo, Ahi, (len-3 bytes of Data)
;
; Uses 3 bytes of stack
;
WRITE_MEM
;
; Build "STA  AAAA,Y" in RAM
        LDA     #STAY_OP
        STA     HACKBUF+0
;
; Set address into RAM
; NoICE memory banking is not used
        LDA     COMBUF+3
        STA     HACKBUF+1
        LDA     COMBUF+4
        STA     HACKBUF+2
        CMP     #$E0
        BCS     WM_80           ;block attempt to write to E000 or above
;
; Set return after LDA
        LDA     #RTS_OP
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
        BRA     WM_99
;
; Write failed:  return status = 1
WM_80   LDA     #1
;
; Compute checksum on buffer, and send to PC, then return
WM_99   JMP     SEND_STATUS

;===========================================================================
;
; Read registers:  FN, len=0
;
; Uses 3 bytes of stack
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
; Uses 3 bytes of stack
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
; Force high half of SP, in case user is being mean
; Reload SP, in case it has changed
        LDA     #INITSTACK / $100
        STA     REG_SP+1
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
        LDA     REG_PS          ;PUSH USER CONDITION CODES FOR RTI
        PHA
;
; Restore registers
        LDX     REG_X
        LDY     REG_Y
        LDA     REG_A
;
; Arm our NMI debouncer
        DEC     NMI_NESTING
;
; Return to user
        RTI
;
;===========================================================================
;
; Common continue point for all monitor entrances
; REG_STATE, REG_A, REG_PS, REG_PC set; X, Y intact; SP = user stack
ENTER_MON
        STX     REG_X
        STY     REG_Y
        TSX
        STX     REG_SP          ;SAVE USER'S STACK POINTER (LSB)
        LDA     #1              ;STACK PAGE ALWAYS 1
        STA     REG_SP+1        ;(ASSUME PAGE 1 STACK)
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
; Uses 3 bytes of stack
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
; NoICE memory banking is not used
        LDA     COMBUF+3,X
        STA     HACKBUF+1
        LDA     COMBUF+4,X
        STA     HACKBUF+2
;
; Set return after LDA
        LDA     #RTS_OP
        STA     HACKBUF+3
;
; Read current data at byte location
        JSR     HACKBUF         ;GET BYTE AT AAAA
        STA     COMBUF+2,Y      ;SAVE IN RETURN BUFFER
;
; Attempt to insert new data at byte location
;
; Block attempt to write to E000 or above, since the kernel does bank switching
; and other things that are likely to cause us trouble.
; BUT allow breakpoints on the system calls above FF00: otherwise continuing
; from a breakpoint in user code on a system call will fail when NoICE tries to
; step off the breakpoint.
        LDA     HACKBUF+2
        CMP     #$FF
        BEQ     SB_20
        CMP     #$E0
        BCS     SB_90
;
; Build "STA  AAAA" in RAM
SB_20   LDA     #STA_OP
        STA     HACKBUF+0
        LDA     COMBUF+5,X      ;BYTE TO WRITE
        JSR     HACKBUF
;
; Verify write
        LDA     #CMP_OP
        STA     HACKBUF+0
        LDA     COMBUF+5,X
        JSR     HACKBUF
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
; Uses 3 bytes of stack
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
; Uses 3 bytes of stack
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
        BRA     SEND_STATUS

;===========================================================================
; Build status return with value from "A"
;
; Uses 3 bytes of stack
;
SEND_STATUS
        STA     COMBUF+2        ;SET STATUS
        LDA     #1
        STA     COMBUF+1        ;SET LENGTH
        BRA     SEND

;===========================================================================
; Append checksum to COMBUF and send to PC
;
; If called, uses 5 bytes of stack including return address
; If jumped to (normal case), uses 3 bytes of stack
;
SEND    JSR     CHECKSUM        ;2 stack. GET A=CHECKSUM, X->checksum location
        EOR     #$FF
        CLC
        ADC     #1
        STA     COMBUF,X        ;STORE NEGATIVE OF CHECKSUM
;
; Send buffer to PC
        JSR     MAP_IO          ;2 stack. Save user's I/O mapping, enable ours
        LDX     #0              ;POINTER TO DATA
        LDY     COMBUF+1        ;LENGTH OF DATA
        INY                     ;PLUS FUNCTION, LENGTH, CHECKSUM
        INY
        INY
SE_10   LDA     COMBUF,X
        JSR     PUTCHAR         ;3 stack. SEND A BYTE
        INX
        DEY
        BNE     SE_10
;
        JSR     RESTORE_IO      ;2 stack
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
; NMI. Enter the monitor, typically by grounding the RESTORE pin
NMI_HAN
        INC     NMI_COUNT       ;Total number of NMI seen
        INC     NMI_NESTING
        BEQ     NMI_10          ;First NMI: enter the monitor
        DEC     NMI_NESTING
        RTI                     ;Ignore nested NMI

NMI_10  STA     REG_A           ;SAVE A
        LDA     #3
        JMP     INT_ENTRY       ;ENTER MONITOR: STATE IS "NMI"
;
; IRQ/BRK interrupt
; First, check for BRK (breakpoint) interrupt
IRQ_HAN STA     REG_A           ;SAVE A
        PLA
        PHA                     ;GET COPY OF PRE-INT STATUS REG
        AND     #B
        BNE     GOBREAKP        ;SET: DO BREAKPOINT CODE
        LDA     REG_A           ;ELSE RESTORE ACC.
        JMP     (MON_IRQ)       ;JUMP THROUGH RAM VECTOR
;
; BREAK ENTRY.
GOBREAKP LDA     #1             ;ENTER MONITOR: STATE IS "BREAKPOINT"
        BRA     IRQ_90
;
; IRQ NOT SET BY USER CODE (THIS ADDRESS PLACED IN RAMVEC+04H BY INIT)
UNHAN_IRQ
        LDA     #2              ;ENTER MONITOR: STATE IS "UNHANDLED IRQ"
;
; Ignore any NMI that occur while the monitor is active
IRQ_90  STZ     NMI_NESTING
        JMP     INT_ENTRY

MON_END
;
================================================================================
        org     $0
        adr     boot_start      ;Start address
        adr     0               ;length of zero means "run me"

        end
