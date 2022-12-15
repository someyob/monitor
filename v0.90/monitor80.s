;/usr/bin/cl65 --listing monitor.lst --cpu 65c02 --target none -C firmware.cfg monitor.s
.PC02
.include "via.inc"


.code 

; Monitor v 0.73
; VIA1 is at $7000, LED array on PORTA 
; VIA2 is at $7100, LCD on PORTA
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

LCD_IN 	     = %00001111  ; PA2-PA0 LCD control lines, PA3 NC
LCD_OUT 	   = %11111111  ; PA7-PA4 = 1 (write), = 0 (read)

LCD_CLEAR	      = %00000001
LCD_BUSY 	      = %10000000
LCD_RESET	      = %01100000
LCD_4BITMODE    = %00100000
LCD_2LINES 	    = %00101000
LCD_CURSOR_R    = %00010100
LCD_CURSOR_L    = %00010000
LCD_CURSORON	  = %00001110		; 00001110 Display on, cursor on, blink off
LCD_CURSOROFF 	= %00001100		; 00001100 Display on, cursor off, blink off
LCD_ENTRYMODE	  = %00000110
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
BTN_C		= 7
BTN_B		= 6
BTN_A		= 5

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
LCD_REFRESH		   = UMEM+20   ; 1 byte, refresh LCD countdown timer

LCD_MEMORY		= UMEM+21	; 32 bytes, LCD display memory
;					....
;				= UMEM+36
;				= UMEM+37
;					....
;				= UMEM+52

PB_KEY   = UMEM+53		; 1 byte, pb keypad from VIA2 Port B
PB_BUFF	 = UMEM+54		; 8 bytes, circ buffer for debouncing keypad
;          UMEM+61

MEMPTR      = UMEM+62       ; 2 bytes, MEM ptr for monitor
;             UMEM+63
MEMINC      = UMEM+64       ; 2 bytes, increment when scrolling through memory
;           = UMEM+65

welcomemsg: 
	.asciiz "Monitor v0.80"
releasemsg: 
	.asciiz "KJ 27 Dec 2021"
hexascii: 
	.asciiz "0123456789ABCDEF"
regdumplabel:
	.asciiz "A X Y  PC   P SP"
systemticklabel:
  .asciiz "System ticks:"
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
jump_table:
    .word run_monitor
    .word display_welcome
    .word blink_leds
    .word display_ticks
    
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
    jsr clear_lcd
    
    lda #0
menu_entry_point:
    tay
    asl
    tax
    tya
    
    
    jmp (jump_table,x)
return_from_jump:
    inc A
    cmp #4
    bne menu_entry_point
    lda #0
    jmp menu_entry_point
    

do_nothing1:
    jmp do_nothing1
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; Blink LEDs

blink_leds:
.scope BlinkLEDs
    pha
busy_loop:
    lda TICKS
    sta LED_DATA
    ;jsr delay
    
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
    jmp return_from_jump
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
    jmp return_from_jump
.endscope
    
    

    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; MONITOR
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

check_btn_C:    ; simulate a crash
    cmp #BTN_C
    bne check_other
    jmp end_run_monitor
check_other:



end_check:

    jmp busy_loop
end_run_monitor:
    pla
    jmp return_from_jump
.endscope


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

display_ticks:
.scope DisplayTicks
    pha
    jsr clear_lcd	
    copy_lcd_string systemticklabel, 0, 0

    lda #1
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
    jmp return_from_jump   
;    jsr lcd_goto_line0
 ;   lda TICKS+3
;    jsr lcd_print_byte_hex
;    lda TICKS+2
;    jsr lcd_print_byte_hex
;    lda TICKS+1
 ;   jsr lcd_print_byte_hex
 ;   lda TICKS
 ;   jsr lcd_print_byte_hex  
.endscope

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



lcd_print_string:
    pha
    phy
    ldy #0
print_next_char:
    lda (STRG_PTR),y
    sta LCD_BUFF
    beq exit_print_str
    jsr write8_lcd		; print the character that's in the A reg
    iny					; increment to next char
    jmp print_next_char
exit_print_str:
    ply
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
;; print the byte in the A Reg in hex
lcd_print_byte_hex:			; 
    phx
    pha                 ; save a copy of A
    lsr A
    lsr A
    lsr A
    lsr A				; high order nibble
    tax					; transfer to x reg
    lda hexascii,x
    sta LCD_BUFF
    jsr write8_lcd 		; print the character that's in the LCD_BUFF
    pla
    and #%00001111		; low order nibble
    tax					; transfer to x reg
    lda hexascii,x
    sta LCD_BUFF
    jsr write8_lcd
    plx
    rts

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




lcd_inc_cursor:
    pha
    lda LCD_CURSOR_POS
    cmp #15
    bne inc_pos
    lda #0
    sta LCD_CURSOR_POS
    lda LCD_LINE_POS
    cmp #LCD_LINE0
    beq set_line1
    lda #LCD_LINE0
    sta LCD_LINE_POS
    jmp exit_inc
set_line1:
    lda #LCD_LINE1
    sta LCD_LINE_POS
    jmp exit_inc

inc_pos:
    inc A
    sta LCD_CURSOR_POS
exit_inc:
    ;lda LCD_LINE_POS
    ;sta LED_DATA
    pla
    rts

lcd_dec_cursor:
    pha
    lda LCD_CURSOR_POS
    bne dec_pos
    lda #15
    sta LCD_CURSOR_POS
    lda LCD_LINE_POS
    cmp #LCD_LINE1
    beq set_line0
    lda #LCD_LINE1
    sta LCD_LINE_POS
    jmp exit_inc
set_line0:
    lda #LCD_LINE0
    sta LCD_LINE_POS
    jmp exit_inc

dec_pos:
    dec A
    sta LCD_CURSOR_POS
exit_dec:
    ;lda LCD_LINE_POS
    ;sta LED_DATA
    pla
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


lcd_cursor_on:
    pha
    lda #LCD_COMMAND_MODE	; set to LCD command/instruction mode
    sta LCD_MODE
    lda #LCD_CURSORON  	; 00001100 Display on, cursor ON, blink off
    sta LCD_BUFF
    jsr write8_lcd
    lda #LCD_CHAR_MODE		; set to LCD character mode
    sta LCD_MODE
    pla
    rts

lcd_cursor_off:
    pha
    lda #LCD_COMMAND_MODE	; set to LCD command/instruction mode
    sta LCD_MODE
    lda #LCD_CURSOROFF  	; 00001100 Display on, cursor ON, blink off
    sta LCD_BUFF
    jsr write8_lcd
    lda #LCD_CHAR_MODE		; set to LCD character mode
    sta LCD_MODE
    pla
    rts

lcd_cursor_right:
    pha
    lda #LCD_COMMAND_MODE	; set to LCD command/instruction mode
    sta LCD_MODE
    ;lda #LCD_CURSOR_R  	
    lda #%10000000
    ora LCD_CURSOR_POS
    sta LCD_BUFF
    jsr write8_lcd
    lda #LCD_CHAR_MODE		; set to LCD character mode
    sta LCD_MODE
    pla
    rts

lcd_cursor_left:
    pha
    lda #LCD_COMMAND_MODE	; set to LCD command/instruction mode
    sta LCD_MODE
    lda #LCD_CURSOR_L
    sta LCD_BUFF
    jsr write8_lcd
    lda #LCD_CHAR_MODE		; set to LCD character mode
    sta LCD_MODE
    pla
    rts

lcd_cursor_set_pos:
    pha
    lda #LCD_COMMAND_MODE	; set to LCD command/instruction mode
    sta LCD_MODE	
    lda LCD_LINE_POS
    ora LCD_CURSOR_POS
    ;sta LED_DATA
    sta LCD_BUFF
    jsr write8_lcd
    lda #LCD_CHAR_MODE		; set to LCD character mode
    sta LCD_MODE
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
    beq timer1_service	; no, 
	
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
    .word $0000
    .word reset
    .word int_service

    

    
