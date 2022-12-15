.include "via.inc"

T1LSB1 = VIA1_T1CL
T1MSB1 = VIA1_T1CH 
ACR1   = VIA1_ACR
IER1   = VIA1_IER

LED_DATA    = VIA1_PORTA
LED_DDR     = VIA1_DDRA

TIMER1_CONT	= %01000000		   ; TIMER1 continuous, PB7 toggle disabled (ACR)
ENABLE_T1	  = %11000000      ; enable interrupt TIMER1 (IER)

LCD_DATA    = VIA2_PORTA
LCD_DDR     = VIA2_DDRA
PB_DATA     = VIA2_PORTB
PB_DDR      = VIA2_DDRB

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

LCD_IN 	  = %00001111  ; PA2-PA0 LCD control lines, PA3 NC
LCD_OUT 	= %11111111  ; PA7-PA4 = 1 (write), = 0 (read)

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
