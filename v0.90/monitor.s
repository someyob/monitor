;/usr/bin/cl65 --listing monitor.lst --cpu 65c02 --target none -C firmware.cfg monitor.s
.PC02
.include "via.inc"


.code 

; Monitor v 0.90
; VIA1 is at $7000, LED array on PORTA 
; VIA2 is at $7100, LCD on PORTA
; ACIA is at $7200
;
; Display fully functional, converted to using display memory
; debounced push buttons.
; Tick timer counts at 20 mSec intervals
; LCD cursor control, left and right with wrap around
; Read-only memory monitor

.macro copy_lcd_string _str_ptr, _line, _pos
    pha
    lda #<_str_ptr
    sta STRG_PTR
    lda #>_str_ptr
    sta STRG_PTR+1
    lda #_line
    sta LCD_LINE_POS
    lda #_pos
    sta LCD_CURSOR_POS
    pla
    jsr lcd_copy_string
.endmacro


T1LSB1 = VIA1_T1CL ; VIA1 + $4
T1MSB1 = VIA1_T1CH  ; VIA1 + $5
ACR1   = VIA1_ACR ; VIA1 + $B
IER1   = VIA1_IER ; VIA1 + $E

LED_DATA    = VIA1_PORTA
LED_DDR     = VIA1_DDRA

TIMER1_CONT	= %01000000		; TIMER1 continuous, PB7 toggle disabled (ACR)
ENABLE_T1	  = %11000000      ; enable interrupt TIMER1 (IER)

LCD_DATA    = VIA2_PORTA
LCD_DDR     = VIA2_DDRA
PB_DATA     = VIA2_PORTB
PB_DDR	    = VIA2_DDRB

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ACIA / serial port
ACIA_MODE		= %00001001		; no parity rcv interrupt enabled  (CMD)
ACIA_8N1_BAUD	= %00011110     ; 9600 baud						(CTRL)
ACIA_DTR		= %00000001		; Data Terminal Ready bit

;											PA7 PA6 PA5 PA4 PA3 PA2 PA1 PA0
;  All I/O to LCD through VIA2 PORT A        D7  D6  D5  D4  NC   E  RW  RS
;    

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  LCD constants
;;
LCD_E  = %00000100		; PA2, enable, toggle to latch in data to LCD
LCD_RW = %00000010		; PA1, set bit = 1 if reading, = 0 for writing to LCD
LCD_RS = %00000001		; PA0, set bit = 1 if character data, otherwise 0 for
	    				;   instruction or command	
LCD_COMMAND_MODE = 0
LCD_CHAR_MODE	 = LCD_RS

LCD_IN 	       = %00001111  ; PA2-PA0 LCD control lines, PA3 NC
LCD_OUT 	   = %11111111  ; PA7-PA4 = 1 (write), = 0 (read)

LCD_CLEAR	    = %00000001
LCD_BUSY 	    = %10000000
LCD_RESET	    = %01100000
LCD_4BITMODE    = %00100000
LCD_2LINES 	    = %00101000
LCD_CURSOR_R    = %00010100
LCD_CURSOR_L    = %00010000
LCD_CURSORON	= %00001110		; 00001110 Display on, cursor on, blink off
LCD_CURSOROFF 	= %00001100		; 00001100 Display on, cursor off, blink off
LCD_ENTRYMODE	= %00000110
LCD_LINE0       = %10000000	; first line, position 0
LCD_LINE1       = %11000000	; second line, position 0
LCD_POSITIONX   = %10000010

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  Pushbutton keypad constants
;;
BTN_LEFT	= 0
BTN_DOWN	= 1
BTN_UP		= 2
BTN_SEL		= 3
BTN_RIGHT	= 4
BTN_C		= 5
BTN_B		= 6
BTN_A		= 7

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  Global variables
;;
UMEM	 = $00	; start of page user mem locations
TICKS   = UMEM+0        ; 4 bytes, tick counter for TIMER1 on VIA1

; Temp area for brk interrupt (registers)
PCH     = UMEM+4    ; Program counter HIGH byte
PCL     = UMEM+5    ; Program counter LOW byte
AREG    = UMEM+6
XREG    = UMEM+7
YREG    = UMEM+8
SPREG   = UMEM+9
PREG    = UMEM+10    ; processor status reg

; LCD variables
LCD_BUFF = UMEM+11		; 1 byte
LCD_MODE = UMEM+12		; 1 byte
LCD_STAT = UMEM+13       ; 1 byte
STRG_PTR = UMEM+14		; 2 bytes, string pointer for LCD print routine
		 ; UMEM+15

LCD_BYTE = UMEM+16      ; 1 byte, general purpose

LCD_CURSOR_POS   = UMEM+17
LCD_LINE_POS     = UMEM+18
LCD_USER  	     = UMEM+19   ; user cursor, variable depending on current screen
LCD_REFRESH		 = UMEM+20   ; 1 byte, refresh LCD countdown timer

LCD_MEMORY		= UMEM+21	; 32 bytes, LCD display memory
;					....
;				= UMEM+36
;				= UMEM+37
;					....
;				= UMEM+52

PB_KEY   = UMEM+53		; 1 byte, pb keypad from VIA2 Port B
PB_BUFF	 = UMEM+54		; 8 bytes, circ buffer for debouncing keypad
;          UMEM+61

MEMPTR      = UMEM+62       ; 2 bytes, MEM ptr for LCD monitor
;             UMEM+63
MEMINC      = UMEM+64       ; 2 bytes, increment when scrolling through memory
;           = UMEM+65
; not used MENUS       = UMEM+66       ; 2 bytes x number of menu items = 8 bytes
;                       = UMEM+73

; General purpose registers
STRING_PTR      = UMEM + 74    ; 2 byte pointer

;  ACIA read and write pointers
RD_PTR	    = UMEM + 76	; 1 byte read pointer
WR_PTR      = UMEM + 77	    ; 1 byte write pointer
COMMAND_BUFF    = UMEM+80       ; buffer for incoming keyb command, 64 byte
            ;   = UMEM+144
; Wozmon variables
WOZ_MODE    = UMEM + 145        ; $00=XAM, $7F=STOR, $AE=BLOCK XAM
WOZ_L       = UMEM + 146        ; Hex value parsing Low
WOZ_H       = UMEM + 147        ; Hex value parsing High
WOZ_YSAV    = UMEM + 148        ; Used to see if hex value is given
WOZ_STL     = UMEM + 149        ; store address low
WOZ_STH     = UMEM + 150        ; store address high
WOZ_XAML    = UMEM + 151        ; last opened location low
WOZ_XAMH    = UMEM + 152        ; last opened location high
;
ACIABUF     = UMEM + $0300      ; 256 bytes circ buffer for ACIA


welcomemsg: 
	.asciiz "Monitor v0.90g"
releasemsg: 
	.asciiz "KJ 7 Nov 2022"
hexascii: 
	.asciiz "0123456789ABCDEF"
regdumplabel:
	.asciiz "A X Y  PC   P SP"
terminalmsg:
    .asciiz "Connect Serial  "
terminalSettings:
    .asciiz " 9600 baud 8N1  "
wozmonhelp:
    .asciiz "Wozmon (adapted), '?' for help"
; memory increments for LCD monitor:
memincrementslo:
    .byte $00
    .byte $00
    .byte $10
    .byte $08
memincrementshi:
    .byte $10
    .byte $01
    .byte $00
    .byte $00

menu1:
	.asciiz "A: Mon   B: Term"
menu2:
	.asciiz "C: Ticks v: Next"
menuexit:
    .asciiz "'C' to exit     "


NUM_MENUS = 4
    
reset:
    cld
    sei

;; clear page zero ram
    ldx #$00
clear_page0:
    stz $00,X
    inx 
    bne clear_page0

;; init stack    
    ldx #$ff
    txs             ; initialize stack pointer

;; set up VIAs
    lda #$ff
    sta LED_DDR     ; set LED port data direction
    sta LED_DATA	; turn on all LEDs
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; set up LCD 
            ; PB7-PB4 PORTA is data io to the LCD
            ; set bits data direction PORTA as OUTPUT
    lda #LCD_OUT	; PORTA is mapped as PA5=RS, PA6=RW, and PA7=E of LCD
    sta LCD_DDR

    jsr init_push_buttons

    jsr init_lcd
    jsr clear_lcd

    copy_lcd_string welcomemsg, 0, 0
    copy_lcd_string releasemsg, 1, 0
    jsr lcd_print_display_mem

    jsr init_timer1
    jsr init_acia

    cli             ; enable interrupts

    jsr delay
    jsr delay
    jsr delay
    jsr delay
    lda #$0
    sta LED_DATA	; turn off LEDs
    jsr delay
    jsr delay
    jsr delay

run_menus:

.scope menu_system
    lda #0
    sta LCD_USER
display_menu:
    jsr clear_lcd
    copy_lcd_string menu1, 0, 0
    copy_lcd_string menu2, 1, 0
    jsr lcd_print_display_mem
    lda LCD_USER
    sta LED_DATA

busy_loop:
    ldx #0
check_pb:
    lda PB_BUFF,X       ; check each of 8 buttons
    dec a               ; pattern was %00000001
    beq button_pushed   ; then debounced
    inx
    cpx #8
    bne check_pb
    jmp busy_loop
button_pushed:
    lda #%11111111
    sta PB_BUFF,X       ; clear the pb buffer
    txa                 ; x is index of button pushed
    ;sta LED_DATA
; ===========================
check_btn_A:
    cmp #BTN_A
    bne check_btn_B
    jsr run_monitor
    jmp display_menu
check_btn_B:
    cmp #BTN_B
    bne check_btn_C
    jsr run_terminal
    jmp display_menu
check_btn_C:
    cmp #BTN_C
    bne end_btn_check
    jsr blink_leds
    jmp display_menu
end_btn_check:
    jmp display_menu

    ;;   code to handle menus continues here

.endscope  ; menu_system


    jsr run_monitor

do_nothing1:
    jmp do_nothing1
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; Blink LEDs
    

blink_leds:
.scope BlinkLEDs
    pha
    jsr clear_lcd
    ; copy_lcd_string xxxx, 0, 0
    copy_lcd_string menuexit, 1, 0
    jsr lcd_print_display_mem
busy_loop:
    lda TICKS
    sta LED_DATA
    jsr display_ticks
    
;busy_loop:
    ldx #0
check_pb:
    lda PB_BUFF,X       ; check each of 8 buttons
    dec a               ; pattern was %00000001
    beq button_pushed   ; then debounced
    inx
    cpx #8
    bne check_pb 
    jmp busy_loop
    
button_pushed:
    lda #%11111111
    sta PB_BUFF,X       ; clear the pb buffer
    txa                 ; x is index of button pushed
    ;sta LED_DATA
; ===========================
;check_C:
    cmp #BTN_C
    bne check_pb
    lda #0
    sta LED_DATA
    pla
    rts
.endscope

    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; Show About

display_welcome:
.scope ShowAbout
    pha
    jsr clear_lcd

    copy_lcd_string welcomemsg, 0, 0
    copy_lcd_string releasemsg, 1, 0
    jsr lcd_print_display_mem
busy_loop:
    ldx #0
check_pb:
    lda PB_BUFF,X       ; check each of 8 buttons
    dec a               ; pattern was %00000001
    beq button_pushed   ; then debounced
    inx
    cpx #8
    bne check_pb 
    jmp busy_loop
    
button_pushed:
    lda #%11111111
    sta PB_BUFF,X       ; clear the pb buffer
    txa                 ; x is index of button pushed
    ;sta LED_DATA
; ===========================
check_C:
    cmp #BTN_C
    bne check_pb
    jsr clear_lcd
    pla
    rts
.endscope
    
    

    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; MONITOR on LCD (read only)
run_monitor:
.scope RunMonitor
    pha
    lda #$00
    sta MEMPTR
    sta MEMPTR+1        ; set Memory ptr to location zero, zero page
    lda #$08
    sta MEMINC
    lda #$00
    sta MEMINC+1
    lda #$3             ; user cursor under mem address
    sta LCD_USER

mon_print:
    lda #10             ; 200 mSec LCD referesh cycle
    sta LCD_REFRESH

    jsr clear_lcd
    lda LCD_USER
    sta LCD_CURSOR_POS
    lda #1
    sta LCD_LINE_POS
    lda #$5e                ;"^"
    sta LCD_BYTE
    jsr lcd_copy_byte
    ; set increment here
    ldx LCD_USER
    lda memincrementslo,x
    sta MEMINC
    lda memincrementshi,x
    sta MEMINC+1
            
    lda #0
    sta LCD_LINE_POS
    sta LCD_CURSOR_POS
    lda MEMPTR+1
    sta LCD_BYTE
    jsr lcd_copy_byte_hex       ; print high byte of MEMPTR
    inc LCD_CURSOR_POS
    inc LCD_CURSOR_POS
    lda MEMPTR
    sta LCD_BYTE
    jsr lcd_copy_byte_hex      ; print low byte of MEMPTR
    inc LCD_CURSOR_POS
    inc LCD_CURSOR_POS
    inc LCD_CURSOR_POS
    ldy #0
mon_print_next_byte:
    lda (MEMPTR),y
    sta LCD_BYTE
    jsr lcd_copy_byte_hex
    inc LCD_CURSOR_POS
    inc LCD_CURSOR_POS
    inc LCD_CURSOR_POS
    iny
    cpy #8
    beq exit_mon_print
    cpy #4
    bne mon_print_next_byte
    lda #1
    sta LCD_LINE_POS
    lda #5
    sta LCD_CURSOR_POS
    jmp mon_print_next_byte
exit_mon_print:
    jsr lcd_print_display_mem

busy_loop:
    lda LCD_REFRESH
    ;sta LED_DATA
    beq mon_print

    ldx #0
check_pb:
    lda PB_BUFF,X       ; check each of 8 buttons
    dec a               ; pattern was %00000001
    beq button_pushed   ; then debounced
    inx
    cpx #8
    bne check_pb 
    jmp busy_loop

button_pushed:
    lda #%11111111
    sta PB_BUFF,X       ; clear the pb buffer
    txa                 ; x is index of button pushed
    ;sta LED_DATA
; ===========================
check_right:
    cmp #BTN_RIGHT
    bne check_left
    lda LCD_USER
    inc a
    cmp #4              ; maximum of 4 positions, wrap
    bne end_check_right
    lda #0
end_check_right:
    sta LCD_USER
    jmp mon_print
; ========================    
check_left:
    cmp #BTN_LEFT
    bne check_up
    lda LCD_USER
    dec a
    bpl end_check_left
    lda #3
end_check_left:
    sta LCD_USER
    jmp mon_print
; ========================
check_up:
    cmp #BTN_UP
    bne check_down
    ; (inc mon_ptr)
    clc 
    lda MEMPTR
    adc MEMINC
    sta MEMPTR
    lda MEMPTR+1
    adc MEMINC+1
    sta MEMPTR+1
    jmp mon_print
; =========================
check_down:
    cmp #BTN_DOWN
    bne check_btn_C
    ;  (dec mon_ptr)
    sec
    lda MEMPTR
    sbc MEMINC
    sta MEMPTR
    lda MEMPTR+1
    sbc MEMINC+1
    sta MEMPTR+1
    jmp mon_print

check_btn_C:    ; 'C' to exit
    cmp #BTN_C
    bne check_other
    jmp end_run_monitor
check_other:



end_check:

    jmp busy_loop
end_run_monitor:
    pla
    rts
.endscope  ; RunMonitor

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  Display tick timer in hex on the LCD
display_ticks:
.scope DisplayTicks
    pha
    lda #0
    sta LCD_LINE_POS
busy_loop:
    lda #0
    sta LCD_CURSOR_POS
    lda TICKS+3
    sta LCD_BYTE
    jsr lcd_copy_byte_hex
    lda #2
    sta LCD_CURSOR_POS
    lda TICKS+2
    sta LCD_BYTE
    jsr lcd_copy_byte_hex
    lda #4
    sta LCD_CURSOR_POS
    lda TICKS+1
    sta LCD_BYTE
    jsr lcd_copy_byte_hex
    lda #6
    sta LCD_CURSOR_POS
    lda TICKS
    sta LCD_BYTE
    jsr lcd_copy_byte_hex

    jsr lcd_print_display_mem

    pla
    rts
.endscope   ; DisplayTicks

    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; TERMINAL
run_terminal:
.scope RunTerminal
    copy_lcd_string terminalmsg, 0, 0
    copy_lcd_string terminalSettings, 1, 0
    jsr lcd_print_display_mem

    jsr acia_print_crlf
    jsr acia_print_crlf
    lda #<welcomemsg
    ldx #>welcomemsg
    jsr acia_print_string
    jsr acia_print_crlf
    lda #<releasemsg
    ldx #>releasemsg
    jsr acia_print_string
    jsr acia_print_crlf
    jsr acia_print_crlf
    lda #<wozmonhelp
    ldx #>wozmonhelp
    jsr acia_print_string
    jsr acia_print_crlf
    jsr acia_print_crlf
main_loop:
    jsr acia_print_prompt
    ldx #0
    lda #0
    sta COMMAND_BUFF,x
busy_loop:
    jsr acia_buff_diff
    beq busy_loop
    jsr read_from_acia_buffer
    cmp #$1B            ; esc?
    bne not_esc
    jsr acia_print_crlf
    jmp main_loop
not_esc:
    cmp #$0D                ; cr ?
    bne normal_char
    sta COMMAND_BUFF,x      ; add the cr to the end before exit
    inx
    lda #0
    sta COMMAND_BUFF,x      ; keeps string null terminated
    jsr acia_print_crlf

    jsr parse_input         ; parse and execute command
    jmp main_loop
normal_char:
    jsr acia_print_char
    sta COMMAND_BUFF,x
    ; sta LED_DATA
    inx
    lda #0
    sta COMMAND_BUFF,x      ; keeps string null terminated
    jmp busy_loop
.endscope ; RunTerminal

init_timer1:
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; set up timer interrupts on T1 (VIA2) for PB polling
    lda #TIMER1_CONT
    sta ACR1
    lda #ENABLE_T1
    sta IER1
    lda #$1E        ; subtract 2 from the following:  (ref https://www.youtube.com/watch?v=g_koa00MBLg)
    sta T1LSB1		; $4e20 = 20,000 cycles = 20 mSec interrupts @ 1 MHz
    lda #$4E		; $2710 = 10,000 cycles = 10 mSec interrupts @ 1 MHz
    sta T1MSB1		; $1388 = 5,000 cycles = 5 mSec interrupts @ 1 MHz
    rts



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  General purpose delay routine	
delay:
    phx
    phy
    ldy #$FF
outerloop:
    ldx #$FF
innerloop:
    dex
    bne innerloop
    dey
    bne outerloop
    ply
    plx
    rts


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; set up ACIA
init_acia:
.scope _INIT_ACIA
	stz ACIA_STAT			; reset the ACIA
	lda #ACIA_MODE
	sta ACIA_CMD
	lda #ACIA_8N1_BAUD
	sta ACIA_CTRL
	stz RD_PTR				; init circ buffer pointers
	stz WR_PTR
    rts
.endscope ; _INIT_ACIA

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  Delay for 1046 uSec for trnsmit ACIA 1 char at 9600 baud
delay9600:
.scope _DELAY9600
	phx
	phy
	ldy #$2
outerloop:
	ldx #$68
innerloop:
	dex
	bne innerloop
	dey
	bne outerloop
	ply
	plx
	rts
.endscope ; _DELAY9600

write_to_acia_buffer:
.scope _WRITE_TO_ACIA_BUFFER
    pha
	phx
	ldx WR_PTR
	lda ACIA_DATA
	sta ACIABUF,X
	inc WR_PTR
	plx
	pla
	rts
.endscope ; _WRITE_TO_ACIA_BUFFER

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
read_from_acia_buffer:
.scope _READ_FROM_ACIA_BUFFER
	phx
	ldx RD_PTR
	lda ACIABUF,X		; routines assumes byte to read goes into A
	inc RD_PTR
	plx
	rts
.endscope ; _READ_FROM_ACIA_BUFFER

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
acia_buff_diff:
	lda WR_PTR
	sec
	sbc RD_PTR
	rts					; ends with A showing number of bytes to read

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
acia_print_char:
	sta ACIA_DATA
	jsr delay9600
    rts



acia_print_byte:
    ; byte stored in A
    pha
    and #%11110000
    tax
    lda hexascii,x
    jsr acia_print_char
    pla        ; grab copy of value again
    asl
    asl
    asl
    asl
    tax
    lda hexascii,x
    jsr acia_print_char
    rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;  Print null terminated string oon ACIA
;  A is low byte, X is high byte
acia_print_string:
.scope _ACIA_PRINT_STRING
    sta STRING_PTR
    stx STRING_PTR+1
	ldy #0
next_char:
	lda (STRING_PTR),y
	beq exit
    jsr acia_print_char
	iny					; increment to next char
	jmp next_char
exit:
	rts
.endscope ; _ACIA_PRINT_STRING

acia_print_command_string:
.scope _ACIA_PRINT_COMMAND_STRING
    phy
    pha
	ldy #0
next_char:
	lda COMMAND_BUFF,y
	beq exit
    jsr acia_print_char
	iny					; increment to next char
	jmp next_char
exit:
    jsr acia_print_crlf
    pla
    ply
	rts
.endscope ; _ACIA_PRINT_STRING

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
acia_print_crlf:
    pha
    lda #13
    jsr acia_print_char
    lda #10
    jsr acia_print_char
    pla
    rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
acia_print_prompt:
    lda #'\'                ; the classic wozmon prompt
    jsr acia_print_char
    jsr acia_print_crlf
    rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Parse input a-la Wozmon
parse_input:
.scope _PARSE_INPUT
    ; jsr acia_print_command_string
	ldy #$FF
	lda #0
	ldx #0
setstor:
    asl                 ; Leaves $7B if setting STOR mode.
setmode:
    sta WOZ_MODE        ; $00=XAM $74=STOR $2E=BLOK XAM
blskip:                 ; space skip.  any character less than '.' is considered a space
    iny
nextitem:
	lda COMMAND_BUFF,y  ; KBD b7..b0 are inputs, b6..b0 is ASCII input, b7 is constant high
                        ;     Programmed to respond to low to high KBD strobe

	; ora #%10000000      ; set bit 7 as if PIA bit 7 set to mimic wozmon (see * below)

	;cmp #$8D            ; CR + bit 7 (*)
    cmp #$0D            ; CR?
    bne notcr
	jmp exit_parse      ; We're done if it's CR!
notcr:
    cmp #'.'            ; any char < '.', treat as whitespace/delimiter
    bcc blskip
    beq setmode         ;   Set BLOCK XAM mode ("." = $2E)

    cmp #':'            ; ':'=$3A=0011 1010, asl = 01110100 = $74
    beq setstor
    cmp #'R'
    beq run
    stz WOZ_L           ; Clear the input value
    stz WOZ_H
    sty WOZ_YSAV
nexthex:                ; Here we're trying to parse a new hex value
    lda COMMAND_BUFF, y ; Get character for hex test
    eor #$30            ; map digits to 0-9
    cmp #9+1            ; is it a decimal digit?
    bcc dig
    adc #$88
    cmp #$FA
    bcc nothex
dig:
    asl
    asl
    asl
    asl

    ldx #4
hexshift:
    asl                 ;Hex didgit left, MSB to carry
    rol WOZ_L           ; rotate into LSD
    rol WOZ_H           ; rotate into MSD
    dex                 ; done 4 shifts?
    bne hexshift
    iny
    bne nexthex

nothex:
    cpy WOZ_YSAV        ; was at least 1 hex digit given?
    beq escape          ; no, ignore all, start from scratch
    bit WOZ_MODE        ; test mode byte
    bvc notstor         ; B6=0 is STOR, 1 is XAM or BLOCK XAM
    lda WOZ_L           ; lsd of hex data
    sta (WOZ_STL,x)     ; store current 'store index' (X=0)
    inc WOZ_STL         ; increment store index
    bne nextitem
    inc WOZ_STH
tonextitem:
    jmp nextitem

run:
    ; jmp (WOZ_XAML)
    brk

notstor:
    bmi xamnnext        ; B7 = 0 for XAM, 1 for BLOCK XAM
                        ; we're in XAM mode now
    ldx #2              ; copy 2 bytes
setadr:
    lda WOZ_L-1,x       ; copy hex data to
    sta WOZ_STL-1,x     ;    to 'store index'
    sta WOZ_XAML-1,x    ;    and to 'XAM index'
    dex                 ; next of 2 bytes
    bne setadr
nextprnt:               ; print address and data from this address
    bne prdata
    lda #13             ; print cr
    jsr acia_print_char
    lda WOZ_XAMH
    jsr acia_print_byte
    lda WOZ_XAML
    jsr acia_print_byte
    lda #':'
    jsr acia_print_char

prdata:
    lda #' '
    jsr acia_print_char
    lda (WOZ_XAML,x)    ; get data from address
    jsr acia_print_byte
xamnnext:
    stx WOZ_MODE        ; 0-> MODE (XAM mode)
    lda WOZ_XAML        ; see if there's more to print
    cmp WOZ_L
    lda WOZ_XAMH
    sbc WOZ_H
    bcs tonextitem      ; not less! no more data to output

    inc WOZ_XAML        ; increment 'examine index'
    bne mod8chk         ; no carry!
    inc WOZ_XAMH

mod8chk:
    lda WOZ_XAML
    and #%00001111
    bpl nextprnt        ; always taken


;	iny					; increment to next char
;	jmp next_char
exit_parse:
	rts
.endscope ; _PARSE_INPUT


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  Wait for LCD to be ready
lcd_wait:
;;;;;;;;;;;;;;;;;;;;;;;;;
    pha				; save A reg on stack
    phx
    phy
    lda #LCD_IN		; %00000111; set PB2-PB0 to OUTPUT, 
    sta LCD_DDR		;                  rest to input
    ldx #$FF
lcd_is_busy:
    dex
    beq lcd_timeout
    lda #LCD_RW			; trigger read from PORTA
    sta LCD_DATA
    lda #(LCD_RW | LCD_E)		; toggle enable
    sta LCD_DATA

    lda LCD_DATA		; read PORTA, LCD hi nibble
    and #%11110000		; mask off lo nibble
    sta LCD_STAT
    
    lda #LCD_RW				; trigger another read from PORTA
    sta LCD_DATA
    lda #(LCD_RW | LCD_E)	; toggle enable
    sta LCD_DATA

    lda LCD_DATA		; read PORTA, low nibble
    and #%11110000		; mask off low nibble
    lsr
    lsr
    lsr
    lsr
    ora LCD_STAT
    sta LCD_STAT
    and #LCD_BUSY
    bne lcd_is_busy		; flag is set, keep looking
lcd_timeout:
    lda #LCD_OUT		; set PB7-PB4 to OUTPUT again
    sta LCD_DDR
    lda #LCD_RW
    sta LCD_DATA
    ply
    plx
    pla
    rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; write 4 bits to LCD port (4 bits wide)
;;   A Reg contains data or command 
;;   LCD_MODE must be set to either
;;        LCD_COMMAND_MODE or LCD_CHAR_MODE
write4_lcd:
    jsr lcd_wait
    ora LCD_MODE	; set to data or command mode
            ; LCD_MODE will be either 0 or RS
    pha				; save a copy of A
    sta LCD_DATA    ; write to port
    ora #LCD_E
    sta LCD_DATA    ; toggle enable line
    pla
    sta LCD_DATA    ; write to port
    rts

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; write 8 bits
write8_lcd:				; write the 8-bit two character value in hex
    lda LCD_BUFF		; character or command stored at LCD_BUFF
    and #%11110000
    jsr write4_lcd
    lda LCD_BUFF        ; grab copy of value again
    asl
    asl
    asl
    asl
    jsr write4_lcd
    rts



lcd_print_display_mem:
    pha
    phx
    jsr lcd_goto_line0
    ldx #0
lcd_next_char1:
    lda LCD_MEMORY,X
    sta LCD_BUFF
    jsr write8_lcd
    inx
    cpx #16
    bne lcd_next_char1
    jsr lcd_goto_line1
lcd_next_char2:
    lda LCD_MEMORY,X
    sta LCD_BUFF
    jsr write8_lcd
    inx
    cpx #32
    bne lcd_next_char2
    plx
    pla
    rts


lcd_copy_string:
    pha
    phy
    phx
    ldy #0
    lda LCD_CURSOR_POS
    ldx LCD_LINE_POS
    beq do_lcd_str_copy         ; line 0, good to go
    clc                     ; line 1, add 16 to position
    adc #16
do_lcd_str_copy:
    tax
copy_next_char:
    lda (STRG_PTR),y
    beq exit_copy_str
    sta LCD_MEMORY,x
    iny					; increment to next char
    inx
    cpx #32                 ; if end of buffer, exit
    bne copy_next_char
exit_copy_str:
    plx
    ply
    pla  
    rts


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; copy the byte in LCD_BYTE in hex to LCD mem

lcd_copy_byte_hex:			; 
    phy
    phx
    pha                 
    lda LCD_BYTE
    lsr A
    lsr A
    lsr A
    lsr A				; high order nibble
    tax					; transfer to x reg
    lda LCD_CURSOR_POS
    ldy LCD_LINE_POS
    beq do_lcd_byte_hex_copy         ; line 0, good to go
    clc                     ; line 1, add 16 to position
    adc #16
do_lcd_byte_hex_copy:
    tay
    lda hexascii,x
    sta LCD_MEMORY,y
    iny
    cpy #32             ; check that we're not off screen
    beq exit_lcd_byte_hex_copy
    lda LCD_BYTE        ; reload value from mem
    and #%00001111		; low order nibble
    tax					; transfer to x reg
    lda hexascii,x
    sta LCD_MEMORY,y
exit_lcd_byte_hex_copy:
    pla
    plx
    ply
    rts

lcd_copy_byte:			; 
    phy
    phx
    pha                 

    lda LCD_CURSOR_POS
    ldy LCD_LINE_POS
    beq do_lcd_byte_copy         ; line 0, good to go
    clc                     ; line 1, add 16 to position
    adc #16
do_lcd_byte_copy:
    tay
    lda LCD_BYTE
    sta LCD_MEMORY,y

exit_lcd_byte_copy:
    pla
    plx
    ply
    rts



lcd_goto_line0:
    pha
    lda #LCD_COMMAND_MODE			; set to LCD command/instruction mode
    sta LCD_MODE
    lda #LCD_LINE0			; put cursor at start of line 1
    sta LCD_BUFF
    jsr write8_lcd
    lda #LCD_CHAR_MODE			; reset to LCD character mode
    sta LCD_MODE
    lda #0
    sta LCD_CURSOR_POS
    lda #LCD_LINE0
    sta LCD_LINE_POS
    pla
    rts
	
lcd_goto_line1:
    pha
    lda #LCD_COMMAND_MODE			; set to LCD command/instruction mode
    sta LCD_MODE
    lda #LCD_LINE1			; put cursor at start of line 1
    sta LCD_BUFF
    jsr write8_lcd
    lda #LCD_CHAR_MODE			; reset to LCD character mode
    sta LCD_MODE
    lda #0
    sta LCD_CURSOR_POS
    lda #LCD_LINE1
    sta LCD_LINE_POS
    pla
    rts 


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Initialize LCD
init_lcd:
    pha
    lda #LCD_COMMAND_MODE	; set to LCD command/instruction mode
    sta LCD_MODE
    
    lda #LCD_RESET  ; reset
    jsr write4_lcd
    lda #LCD_RESET  ; reset
    jsr write4_lcd
    lda #LCD_RESET  ; reset
    jsr write4_lcd
    
    lda #LCD_4BITMODE  	    ; 4-bit operation
    jsr write4_lcd
    
    lda #LCD_2LINES  		; 2 lines
    sta LCD_BUFF
    jsr write8_lcd
    
    lda #LCD_CLEAR	  	    ; clear display
    sta LCD_BUFF
    jsr write8_lcd

    lda #LCD_CURSOROFF  	; 00001100 Display on, cursor off, blink off
    sta LCD_BUFF
    jsr write8_lcd
	
    lda #LCD_ENTRYMODE		; 00000110  Entry mode, increment and shift cursor, no scroll
    sta LCD_BUFF
    jsr write8_lcd
	
    lda #LCD_LINE0			; put cursor at start of line 0
    sta LCD_BUFF
    jsr write8_lcd
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;;  LCD is in CHAR_MODE by default
    ;;  whenever COMMAND_MODE is used, set back to CHAR_MODE
    lda #LCD_CHAR_MODE		; set to LCD character mode
    sta LCD_MODE

    lda #0
    sta LCD_CURSOR_POS
    lda #LCD_LINE0
    sta LCD_LINE_POS
    pla
    rts



clear_lcd:
    pha
    phx
    lda #LCD_COMMAND_MODE		; set to LCD command/instruction mode
    sta LCD_MODE
    lda #LCD_CLEAR			; clear display
    sta LCD_BUFF
    jsr write8_lcd
    lda #LCD_CHAR_MODE			; reset to LCD character mode
    sta LCD_MODE
    ldx #0
clear_screen_mem:
    lda #$20
    sta LCD_MEMORY,X
    inx
    cpx #32
    bne clear_screen_mem
    plx
    pla
    rts




init_push_buttons:
    lda #$00
    sta PB_DDR		; set pushbuttons as inputs
    stz PB_KEY		; zero keypad buffer
    ldx #0
    lda #$FF
loop_clear:
    sta PB_BUFF,X
    inx
    cpx #8
    bne loop_clear
    rts



display_reg:  ; in the format "A X Y  PC   P SP"
    jsr clear_lcd	
    copy_lcd_string regdumplabel, 0, 0

    lda #1
    sta LCD_LINE_POS

    lda #0
    sta LCD_CURSOR_POS
    lda AREG
    sta LCD_BYTE
    jsr lcd_copy_byte_hex

    lda #2
    sta LCD_CURSOR_POS
    lda XREG
    sta LCD_BYTE
    jsr lcd_copy_byte_hex

    lda #4
    sta LCD_CURSOR_POS
    lda YREG
    sta LCD_BYTE
    jsr lcd_copy_byte_hex

    lda #7
    sta LCD_CURSOR_POS
    lda PCH
    sta LCD_BYTE
    jsr lcd_copy_byte_hex
    lda #9
    sta LCD_CURSOR_POS
    lda PCL
    sta LCD_BYTE
    jsr lcd_copy_byte_hex

    lda #12
    sta LCD_CURSOR_POS
    lda PREG
    sta LCD_BYTE
    jsr lcd_copy_byte_hex

    lda #14
    sta LCD_CURSOR_POS
    lda SPREG
    sta LCD_BYTE
    jsr lcd_copy_byte_hex

    jsr lcd_print_display_mem
    rts

nmi_service:
    cli
    jsr run_monitor
    rti

int_service:
    pha
    phx
    phy
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; grab the stack point and read the status register
    tsx 
    lda $0100+4,x	; read status register from stack
    sta PREG		; processor status
    and #$10		; brk bit set?
    beq acia_service	; no, check acia
	
brk_service:
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; brk occurred, save registers and display them on the LCD
    lda $100+6,x	
    sta PCH			; program counter high byte
    lda $100+5,x	
    sta PCL			; program counter low byte
    lda $100+3,x	
    sta AREG		; A register
    lda $100+2,x	
    sta XREG		; X register
    lda $100+1,x	
    sta YREG		; Y register
    tsx 
    stx SPREG		; Stack pointer
    jsr display_reg
do_nothing:
    jmp do_nothing
    jmp exit_int_service

acia_service:
	lda ACIA_STAT
	bpl timer1_service	; no, (bit 7 not set) must be T1 interrupt
	and #%00000111      ; check if error bits set
	bne brk_service     ; if error, dump the registers
	jsr write_to_acia_buffer
	jsr acia_buff_diff     ; returns num chars in buffer in A
	cmp #$F0               ; if the buffer has less than 240 bytes
	bcc exit_int_service   ; just exit
	; do stuff to turn off sending of data to port
	jmp brk_service ; shouldn't get here, RTS not implemented yet
	jmp exit_int_service
	
timer1_service:
    bit T1LSB1		; clear interrupt flag
    dec LCD_REFRESH
    inc TICKS
    ;lda TICKS
    ;sta LED_DATA
    bne timer1_checkpb
    inc TICKS+1
    bne timer1_checkpb
    inc TICKS+2
    bne timer1_checkpb
    inc TICKS+3

timer1_checkpb:
    lda PB_DATA		; read VIA port, PB keypad input
                  ; PB pulls low on press
    ;sta LED_DATA
    ldx #0
read_pb:
    clc
    ror A            ; rightmost bit in A rotated into carry
    ror PB_BUFF,X       ; rotate that carry into PB_BUFF+x
    inx
    cpx #8          ; done all of them?
    bne read_pb
 
exit_debounce:

exit_int_service:
    ply
    plx
    pla
    rti

    .segment "VECTORS"
    .word nmi_service       ; nmi
    .word reset
    .word int_service       ; irq

    

    
