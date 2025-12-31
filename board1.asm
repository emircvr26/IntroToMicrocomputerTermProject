;============================================================================
; PROJECT: HOME AUTOMATION - BOARD #1
; AUTHOR: Emir Civir
; PROCCESSOR: PIC16F877A
; COMPILER: XC8 (MPLAB X) - pic-as driver
; OSCILATOR: 4MHz Kristal (HS Modu)
;============================================================================
;
; DONANIM BAGLANTILARI (PINOUT)
; -------------------------------------------
; 1. 7-SEGMENT DISPLAY (Ortak Katot - Multiplexed)
;    Segmentler (PORTD):
;      RD7 -> a
;      RD6 -> b
;      RD5 -> c
;      RD4 -> d
;      RD3 -> e
;      RD2 -> f
;      RD1 -> g
;      RD0 -> dp (Nokta)
;
;    Ortak Uclar (PORTA - Transistor ile surulmeli):
;      RA1 -> Digit 1 (Soldaki)
;      RA2 -> Digit 2
;      RA3 -> Digit 3
;      RA5 -> Digit 4 (Sembol/Sagdaki) - NOT: RA4 degil!
;
; -------------------------------------------
; 2. KEYPAD (4x4 Matrix)
;    Satirlar (Cikis - PORTB):
;      RB0 -> Satir 1 (1, 2, 3, A)
;      RB1 -> Satir 2 (4, 5, 6, B)
;      RB2 -> Satir 3 (7, 8, 9, C)
;      RB3 -> Satir 4 (*, 0, #, D)
;
;    Sutunlar (Giris - PORTB - Dahili Pull-up Aktif):
;      RB4 -> Sutun 1 (1, 4, 7, *)
;      RB5 -> Sutun 2 (2, 5, 8, 0)
;      RB6 -> Sutun 3 (3, 6, 9, #)
;      RB7 -> Sutun 4 (A, B, C, D)
;
; -------------------------------------------
; 3. SENSOR VE KONTROL
;      RA0 (AN0)   -> LM35 Sicaklik Sensoru (Vout)
;      RA4 (T0CKI) -> Fan Hiz Sayaci (Pulse Girisi)
;      RC0         -> Isitici Rolesi/LED (Isitmada HIGH)
;      RC1         -> Sogutucu Fan/LED (Sogutmada HIGH)
;
; -------------------------------------------
; 4. ILETISIM (UART - 9600 Baud)
;      RC6 (TX)    -> PC RX Pinine
;      RC7 (RX)    -> PC TX Pinine
;
;============================================================================

    PROCESSOR 16F877A
    #include <xc.inc>

    ; KONFIGURASYON
    
    CONFIG FOSC = HS      ; Osilator (4MHz Kristal)
    CONFIG WDTE = OFF     ; Watchdog Timer Kapali
    CONFIG PWRTE = ON     ; Power-up Timer Acik
    CONFIG BOREN = ON     ; Brown-out Reset Acik
    CONFIG LVP = OFF      ; Low Voltage Programming Kapali
    CONFIG CPD = OFF      ; Data Code Protection Kapali
    CONFIG CP = OFF       ; Code Protection Kapali

    
    
    PSECT udata_bank0
    ; Delay
d1:                 DS 1
d2:                 DS 1
d3:                 DS 1

    ; ADC ve Hesaplama
ADC_L:              DS 1
ADC_H:              DS 1
MATH_L:             DS 1
MATH_H:             DS 1

    ; System
DESIRED_TEMP_INT:   DS 1
DESIRED_TEMP_FRAC:  DS 1
AMBIENT_TEMP_INT:   DS 1
AMBIENT_TEMP_FRAC:  DS 1
FAN_SPEED:          DS 1

    ; Keypad
KEYPAD_STATE:       DS 1
KEY_BUFFER0:        DS 1
KEY_BUFFER1:        DS 1
KEY_BUFFER2:        DS 1
KEY_COUNT:          DS 1
DECIMAL_ENTERED:    DS 1
LAST_KEY:           DS 1
KEY_DEBOUNCE:       DS 1
KEY_PRESSED:        DS 1

    ; Display
DIGIT1_VAL:         DS 1
DIGIT2_VAL:         DS 1
DIGIT3_VAL:         DS 1
DIGIT4_VAL:         DS 1
DISPLAY_MODE:       DS 1

    ; Timer
TIMER_COUNTER:      DS 1

    ; UART
UART_RX_DATA:       DS 1
UART_TX_DATA:       DS 1

TEMP_VAR:           DS 1
FIRST_RUN:          DS 1

    
    PSECT udata_shr
W_TEMP:             DS 1
STATUS_TEMP:        DS 1
PCLATH_TEMP:        DS 1

    
    PSECT code, abs, delta=2
    
    ORG 0x0000
resetVec:
    pagesel Baslangic
    goto    Baslangic

    ORG 0x0004
isrVec:
    goto    ISR

    ;----------------------------------------------------------------------------
    ; ISR
    ;----------------------------------------------------------------------------
ISR:
    ; --- Context Saving ---
    movwf   W_TEMP
    swapf   STATUS, w
    clrf    STATUS          ; Bank 0
    movwf   STATUS_TEMP
    movf    PCLATH, w
    movwf   PCLATH_TEMP
    clrf    PCLATH

    ; --- PORTB Interrupt (Keypad) ---
    btfsc   INTCON, 0       ; RBIF
    call    Keypad_ISR_Handler

    ; --- UART RX Interrupt ---
    BANKSEL PIR1
    btfsc   PIR1, 5         ; RCIF
    call    UART_RX_Handler
    BANKSEL PORTA           

    ; --- Context Restore ---
    movf    PCLATH_TEMP, w
    movwf   PCLATH
    swapf   STATUS_TEMP, w
    movwf   STATUS
    swapf   W_TEMP, f
    swapf   W_TEMP, w
    retfie

    ;----------------------------------------------------------------------------
    ; KEYPAD ISR HANDLER
    ;----------------------------------------------------------------------------
Keypad_ISR_Handler:
    movf    KEY_PRESSED, w
    btfss   STATUS, 2       ; Z Flag
    goto    ISR_Keypad_Exit
    
    call    Scan_Keypad
    movwf   LAST_KEY
    
    movf    LAST_KEY, w
    btfsc   STATUS, 2       ; Z Flag
    goto    ISR_Keypad_Exit

    movlw   1
    movwf   KEY_PRESSED
    
    movf    LAST_KEY, w
    sublw   'A'
    btfss   STATUS, 2
    goto    ISR_Check_Input_Mode
    
    movlw   1
    movwf   KEYPAD_STATE
    clrf    KEY_COUNT
    clrf    DECIMAL_ENTERED
    clrf    KEY_BUFFER0
    clrf    KEY_BUFFER1
    clrf    KEY_BUFFER2
    
    movlw   3
    movwf   DISPLAY_MODE    ; Input Mode
    
    ; Display temizle
    clrf    DIGIT1_VAL
    clrf    DIGIT2_VAL
    clrf    DIGIT3_VAL
    movlw   12              ; Bos
    movwf   DIGIT4_VAL
    
    goto    ISR_Keypad_Exit
    
ISR_Check_Input_Mode:
    movf    KEYPAD_STATE, w
    btfsc   STATUS, 2       ; Z Flag
    goto    ISR_Keypad_Exit
    
    ; '#' -> Bitir
    movf    LAST_KEY, w
    sublw   '#'
    btfss   STATUS, 2
    goto    ISR_Check_Star
    
    call    Process_Temperature_Input
    
    ; Normal moda don
    clrf    KEYPAD_STATE
    clrf    DISPLAY_MODE
    goto    ISR_Keypad_Exit
    
ISR_Check_Star:
    ; '*' -> Ondalik
    movf    LAST_KEY, w
    sublw   '*'
    btfss   STATUS, 2
    goto    ISR_Check_Digit
    
    call    Set_Decimal_Flag
    call    Update_Input_Display
    goto    ISR_Keypad_Exit
    
ISR_Check_Digit:
    ; Rakam mi? (0-9)
    movf    LAST_KEY, w
    sublw   '0'
    btfss   STATUS, 0       ; C Flag
    goto    ISR_Keypad_Exit ; < '0'
    
    movf    LAST_KEY, w
    sublw   '9'
    btfss   STATUS, 0       ; C Flag
    goto    ISR_Keypad_Exit ; > '9'
    
    call    Add_Digit_To_Buffer
    call    Update_Input_Display

ISR_Keypad_Exit:
    movlw   50
    movwf   KEY_DEBOUNCE

    movf    PORTB, w        
    bcf     INTCON, 0       
    return

    ;----------------------------------------------------------------------------
    ; UART RX HANDLER
    ;----------------------------------------------------------------------------
UART_RX_Handler:
    BANKSEL RCREG
    movf    RCREG, w
    BANKSEL PORTA
    movwf   UART_RX_DATA
    call    Process_UART_Command
    return

    ;----------------------------------------------------------------------------
    ; UART PROCESS
    ;----------------------------------------------------------------------------
Process_UART_Command:
    movf    UART_RX_DATA, w
    movwf   TEMP_VAR
    
    btfsc   TEMP_VAR, 7
    goto    UART_Set_Cmd
    
    movf    UART_RX_DATA, w
    sublw   0x01
    btfsc   STATUS, 2
    goto    UART_Send_Desired_Frac
    
    movf    UART_RX_DATA, w
    sublw   0x02
    btfsc   STATUS, 2
    goto    UART_Send_Desired_Int
    
    movf    UART_RX_DATA, w
    sublw   0x03
    btfsc   STATUS, 2
    goto    UART_Send_Ambient_Frac
    
    movf    UART_RX_DATA, w
    sublw   0x04
    btfsc   STATUS, 2
    goto    UART_Send_Ambient_Int
    
    movf    UART_RX_DATA, w
    sublw   0x05
    btfsc   STATUS, 2
    goto    UART_Send_Fan_Speed
    return

UART_Send_Desired_Frac:
    movf    DESIRED_TEMP_FRAC, w
    call    UART_Send_Byte
    return

UART_Send_Desired_Int:
    movf    DESIRED_TEMP_INT, w
    call    UART_Send_Byte
    return

UART_Send_Ambient_Frac:
    movf    AMBIENT_TEMP_FRAC, w
    call    UART_Send_Byte
    return

UART_Send_Ambient_Int:
    movf    AMBIENT_TEMP_INT, w
    call    UART_Send_Byte
    return

UART_Send_Fan_Speed:
    movf    FAN_SPEED, w
    call    UART_Send_Byte
    return

UART_Set_Cmd:
    btfsc   TEMP_VAR, 6
    goto    UART_Set_Temp_Int
    
    movf    UART_RX_DATA, w
    andlw   0x3F
    movwf   DESIRED_TEMP_FRAC
    return

UART_Set_Temp_Int:
    movf    UART_RX_DATA, w
    andlw   0x3F
    movwf   DESIRED_TEMP_INT
    return

    ;----------------------------------------------------------------------------
    ; UART SEND
    ;----------------------------------------------------------------------------
UART_Send_Byte:
    movwf   UART_TX_DATA
    BANKSEL TXSTA
UART_Wait:
    btfss   TXSTA, 1        ; TRMT
    goto    UART_Wait
    
    BANKSEL TXREG
    movf    UART_TX_DATA, w
    movwf   TXREG
    BANKSEL PORTA
    return

    
    ; 7-SEGMENT TABLE
Get_7Seg_Code:
    addwf   PCL, f
    retlw   0xFC            ; 0
    retlw   0x60            ; 1
    retlw   0xDA            ; 2
    retlw   0xF2            ; 3
    retlw   0x66            ; 4
    retlw   0xB6            ; 5
    retlw   0xBE            ; 6
    retlw   0xE0            ; 7
    retlw   0xFE            ; 8
    retlw   0xF6            ; 9
    retlw   0x9C            ; C
    retlw   0x8E            ; F
    retlw   0x00            ; Bos

    ;----------------------------------------------------------------------------
    ; MAIN
    ;----------------------------------------------------------------------------
Baslangic:
    ; Komparator Kapat
    BANKSEL CMCON
    movlw   0x07
    movwf   CMCON
    
    ; TRIS
    BANKSEL TRISA
    movlw   0b00010001      ; RA0 Analog, RA4 Input
    movwf   TRISA
    
    movlw   0b11110000      ; RB0-3 Out, RB4-7 In
    movwf   TRISB
    
    movlw   0b10000000      ; RC7 RX
    movwf   TRISC
    
    clrf    TRISD
    
    ; ADC Setup (Bank 1)
    movlw   0b10001110      ; AN0 Analog, sag yasla
    movwf   ADCON1
    
    ; Timer0 Setup (Bank 1)
    movlw   0b00101000      ; External CLK (RA4), Prescaler TMR0
    movwf   OPTION_REG      ; NOT: RBPU Bit 7 = 0, yani Pull-up AKTIF
    
    ; UART Setup (Bank 1)
    movlw   25              ; 9600 Baud
    movwf   SPBRG
    movlw   0b00100100      ; TXEN=1, BRGH=1
    movwf   TXSTA
    bsf     PIE1, 5         ; RCIE Ac
    
    BANKSEL PORTA           ; Bank 0 Donus
    
    ; Port Init
    clrf    PORTA
    movlw   0xF0            ; Sutunlar High (Pull-up mantigi)
    movwf   PORTB
    clrf    PORTC
    clrf    PORTD
    
    ; ADC Ac
    movlw   0b11000001      ; ADC ON
    movwf   ADCON0
    
    ; UART RX Ac
    BANKSEL RCSTA
    movlw   0b10010000      ; SPEN=1, CREN=1
    movwf   RCSTA
    BANKSEL PORTA
    
    ; Interrupt Enable
    bsf     INTCON, 3       ; RBIE
    bsf     INTCON, 6       ; PEIE
    bsf     INTCON, 7       ; GIE
    
    ; Degiskenler
    movlw   35
    movwf   DESIRED_TEMP_INT
    clrf    DESIRED_TEMP_FRAC
    clrf    AMBIENT_TEMP_INT
    clrf    AMBIENT_TEMP_FRAC
    clrf    FAN_SPEED
    clrf    DISPLAY_MODE
    clrf    KEYPAD_STATE
    clrf    KEY_DEBOUNCE
    clrf    KEY_PRESSED
    clrf    TMR0
    movlw   1
    movwf   FIRST_RUN

    ;----------------------------------------------------------------------------
    ; MAIN LOOP
    ;----------------------------------------------------------------------------
Main_Loop:
    ; Ilk calistirma
    movf    FIRST_RUN, w
    btfss   STATUS, 2
    call    Stabilize_ADC
    
    ; Debounce
    movf    KEY_DEBOUNCE, w
    btfsc   STATUS, 2
    goto    Skip_Debounce
    decf    KEY_DEBOUNCE, f
Skip_Debounce:
    
    call    Read_Temperature
    call    Control_Temperature
    
    movf    KEYPAD_STATE, w
    btfss   STATUS, 2
    goto    Input_Mode_Display
    
    call    Prepare_Display
    
    ; Gosterim Dongusu
    movlw   200
    movwf   d3
    
Main_Display_Loop:
    movlw   10
    movwf   d2
    
Main_Display_Inner:
    call    Show_Display

    movf    KEYPAD_STATE, w
    btfss   STATUS, 2
    goto    Input_Mode_Display

    decfsz  d2, f
    goto    Main_Display_Inner
    
    decfsz  d3, f
    goto    Main_Display_Loop
    
    ; Fan Hizi
    movf    TMR0, w
    movwf   FAN_SPEED
    clrf    TMR0
    
    ; Mod Degistir
    incf    DISPLAY_MODE, f
    movf    DISPLAY_MODE, w
    sublw   3
    btfss   STATUS, 2
    goto    Main_Loop
    
    clrf    DISPLAY_MODE
    goto    Main_Loop

Input_Mode_Display:
    call    Show_Display
    clrf    KEY_PRESSED
    goto    Main_Loop

    ;---------------------------------------------------------------------------
    ; SUB PROGRAMS
    ;---------------------------------------------------------------------------
Stabilize_ADC:
    call    Read_Temperature
    call    Delay_Long
    call    Read_Temperature
    call    Delay_Long
    call    Read_Temperature
    clrf    FIRST_RUN
    return

Delay_Long:
    movlw   255
    movwf   d3
DL1:
    movlw   100
    movwf   d2
DL2:
    decfsz  d2, f
    goto    DL2
    decfsz  d3, f
    goto    DL1
    return

    ; KEYPAD SCAN
Scan_Keypad:
    movlw   0x0F
    movwf   PORTB
    nop
    nop
    
    ; ROW 1
    movlw   0x0E
    movwf   PORTB
    nop
    nop
    nop
    btfss   PORTB, 4
    retlw   '1'
    btfss   PORTB, 5
    retlw   '2'
    btfss   PORTB, 6
    retlw   '3'
    btfss   PORTB, 7
    retlw   'A'
    
    ; ROW 2
    movlw   0x0D
    movwf   PORTB
    nop
    nop
    nop
    btfss   PORTB, 4
    retlw   '4'
    btfss   PORTB, 5
    retlw   '5'
    btfss   PORTB, 6
    retlw   '6'
    btfss   PORTB, 7
    retlw   'B'
    
    ; ROW 3
    movlw   0x0B
    movwf   PORTB
    nop
    nop
    nop
    btfss   PORTB, 4
    retlw   '7'
    btfss   PORTB, 5
    retlw   '8'
    btfss   PORTB, 6
    retlw   '9'
    btfss   PORTB, 7
    retlw   'C'
    
    ; ROW 4
    movlw   0x07
    movwf   PORTB
    nop
    nop
    nop
    btfss   PORTB, 4
    retlw   '*'
    btfss   PORTB, 5
    retlw   '0'
    btfss   PORTB, 6
    retlw   '#'
    btfss   PORTB, 7
    retlw   'D'
    
    movlw   0x0F
    movwf   PORTB
    retlw   0x00

Add_Digit_To_Buffer:
    movf    LAST_KEY, w
    andlw   0x0F
    
    movf    DECIMAL_ENTERED, w
    btfss   STATUS, 2
    goto    Add_Frac_Digit
    
    movf    KEY_COUNT, w
    btfsc   STATUS, 2
    goto    Add_First
    sublw   1
    btfsc   STATUS, 2
    goto    Add_Second
    return

Add_First:
    movf    LAST_KEY, w
    andlw   0x0F
    movwf   KEY_BUFFER0
    incf    KEY_COUNT, f
    return

Add_Second:
    movf    LAST_KEY, w
    andlw   0x0F
    movwf   KEY_BUFFER1
    incf    KEY_COUNT, f
    return

Add_Frac_Digit:
    movf    LAST_KEY, w
    andlw   0x0F
    movwf   KEY_BUFFER2
    return

Set_Decimal_Flag:
    movlw   1
    movwf   DECIMAL_ENTERED
    return

Update_Input_Display:
    movf    DECIMAL_ENTERED, w
    btfss   STATUS, 2
    goto    Update_With_Decimal
    
    movf    KEY_COUNT, w
    btfsc   STATUS, 2
    goto    Update_Empty
    
    sublw   1
    btfsc   STATUS, 2
    goto    Update_One_Digit
    
    ; Iki Rakam
    movf    KEY_BUFFER0, w
    movwf   DIGIT1_VAL
    movf    KEY_BUFFER1, w
    movwf   DIGIT2_VAL
    clrf    DIGIT3_VAL
    movlw   12
    movwf   DIGIT4_VAL
    return

Update_One_Digit:
    clrf    DIGIT1_VAL
    movf    KEY_BUFFER0, w
    movwf   DIGIT2_VAL
    clrf    DIGIT3_VAL
    movlw   12
    movwf   DIGIT4_VAL
    return

Update_Empty:
    clrf    DIGIT1_VAL
    clrf    DIGIT2_VAL
    clrf    DIGIT3_VAL
    movlw   12
    movwf   DIGIT4_VAL
    return

Update_With_Decimal:
    movf    KEY_BUFFER0, w
    movwf   DIGIT1_VAL
    movf    KEY_BUFFER1, w
    movwf   DIGIT2_VAL
    movf    KEY_BUFFER2, w
    movwf   DIGIT3_VAL
    movlw   10
    movwf   DIGIT4_VAL
    return

Process_Temperature_Input:
    movf    KEY_BUFFER0, w
    movwf   d1
    movf    d1, w
    movwf   d2
    
    bcf     STATUS, 0
    rlf     d1, f
    rlf     d1, f
    rlf     d1, f       ; x8
    
    bcf     STATUS, 0
    rlf     d2, f       ; x2
    
    movf    d1, w
    addwf   d2, w
    addwf   KEY_BUFFER1, w
    movwf   TEMP_VAR
    
    movf    TEMP_VAR, w
    sublw   10
    btfsc   STATUS, 0   ; <= 10 ise
    goto    Invalid_Input
    
    movf    TEMP_VAR, w
    sublw   50
    btfss   STATUS, 0   ; > 50 ise
    goto    Invalid_Input
    
    movf    TEMP_VAR, w
    movwf   DESIRED_TEMP_INT
    movf    KEY_BUFFER2, w
    movwf   DESIRED_TEMP_FRAC
    return

Invalid_Input:
    return

Read_Temperature:
    call    Delay_Short
    bsf     ADCON0, 2
Wait_ADC:
    btfsc   ADCON0, 2
    goto    Wait_ADC
    
    BANKSEL ADRESL
    movf    ADRESL, w
    BANKSEL PORTA
    movwf   ADC_L
    movf    ADRESH, w
    movwf   ADC_H
    
    ; Hesaplama (ADC * 4.88)
    movf    ADC_L, w
    movwf   MATH_L
    movf    ADC_H, w
    movwf   MATH_H
    
    bcf     STATUS, 0
    rlf     MATH_L, f
    rlf     MATH_H, f
    bcf     STATUS, 0
    rlf     MATH_L, f
    rlf     MATH_H, f   ; x4
    
    movf    ADC_L, w
    addwf   MATH_L, f
    btfsc   STATUS, 0
    incf    MATH_H, f
    movf    ADC_H, w
    addwf   MATH_H, f   ; +ADC
    
    movf    ADC_L, w
    movwf   d1
    movf    ADC_H, w
    movwf   d2
    bcf     STATUS, 0
    rrf     d2, f
    rrf     d1, f
    bcf     STATUS, 0
    rrf     d2, f
    rrf     d1, f
    bcf     STATUS, 0
    rrf     d2, f
    rrf     d1, f       ; /8
    
    movf    d1, w
    subwf   MATH_L, f
    btfss   STATUS, 0
    decf    MATH_H, f
    movf    d2, w
    subwf   MATH_H, f   ; - (ADC/8)
    
    movlw   2
    addwf   MATH_L, f
    btfsc   STATUS, 0
    incf    MATH_H, f   ; +2
    
    clrf    DIGIT1_VAL
Calc_100:
    movf    MATH_H, w
    btfss   STATUS, 2
    goto    Sub_100
    movlw   100
    subwf   MATH_L, w
    btfss   STATUS, 0
    goto    Calc_10
Sub_100:
    movlw   100
    subwf   MATH_L, f
    btfss   STATUS, 0
    decf    MATH_H, f
    incf    DIGIT1_VAL, f
    goto    Calc_100
    
Calc_10:
    clrf    DIGIT2_VAL
Loop_10:
    movlw   10
    subwf   MATH_L, w
    btfss   STATUS, 0
    goto    Calc_1
    movwf   MATH_L
    incf    DIGIT2_VAL, f
    goto    Loop_10
    
Calc_1:
    movf    MATH_L, w
    movwf   DIGIT3_VAL
    
    movf    DIGIT1_VAL, w
    movwf   d1
    movf    d1, w
    movwf   d2
    bcf     STATUS, 0
    rlf     d1, f
    rlf     d1, f
    rlf     d1, f
    bcf     STATUS, 0
    rlf     d2, f
    movf    d1, w
    addwf   d2, w
    addwf   DIGIT2_VAL, w
    movwf   AMBIENT_TEMP_INT
    movf    DIGIT3_VAL, w
    movwf   AMBIENT_TEMP_FRAC
    return

Control_Temperature:
    ; DESIRED+1
    movf    DESIRED_TEMP_INT, w
    movwf   d1
    incf    d1, f
    movf    d1, w
    subwf   AMBIENT_TEMP_INT, w
    btfsc   STATUS, 0
    goto    Need_Cooling
    
    ; DESIRED-1
    movf    DESIRED_TEMP_INT, w
    movwf   d1
    decf    d1, f
    movf    AMBIENT_TEMP_INT, w
    subwf   d1, w
    btfsc   STATUS, 0
    goto    Need_Heating
    
In_Deadband:
    bcf     PORTC, 0
    bcf     PORTC, 1
    return
Need_Cooling:
    bcf     PORTC, 0
    bsf     PORTC, 1
    return
Need_Heating:
    bsf     PORTC, 0
    bcf     PORTC, 1
    return

Prepare_Display:
    movf    DISPLAY_MODE, w
    sublw   0
    btfsc   STATUS, 2
    goto    Prep_Ambient
    
    movf    DISPLAY_MODE, w
    sublw   1
    btfsc   STATUS, 2
    goto    Prep_Desired
    
    movf    DISPLAY_MODE, w
    sublw   2
    btfsc   STATUS, 2
    goto    Prep_Fan
    return

Prep_Fan:
    movf    FAN_SPEED, w
    movwf   d1
    clrf    DIGIT1_VAL
P_F100:
    movlw   100
    subwf   d1, w
    btfss   STATUS, 0
    goto    P_F10
    movwf   d1
    incf    DIGIT1_VAL, f
    goto    P_F100
P_F10:
    clrf    DIGIT2_VAL
P_F10L:
    movlw   10
    subwf   d1, w
    btfss   STATUS, 0
    goto    P_F1
    movwf   d1
    incf    DIGIT2_VAL, f
    goto    P_F10L
P_F1:
    movf    d1, w
    movwf   DIGIT3_VAL
    movlw   11
    movwf   DIGIT4_VAL
    return

Prep_Ambient:
    movlw   10
    movwf   DIGIT4_VAL
    return

Prep_Desired:
    movf    DESIRED_TEMP_INT, w
    movwf   d1
    clrf    DIGIT1_VAL
P_D10:
    movlw   10
    subwf   d1, w
    btfss   STATUS, 0
    goto    P_D1
    movwf   d1
    incf    DIGIT1_VAL, f
    goto    P_D10
P_D1:
    movf    d1, w
    movwf   DIGIT2_VAL
    movf    DESIRED_TEMP_FRAC, w
    movwf   DIGIT3_VAL
    movlw   10
    movwf   DIGIT4_VAL
    return

Show_Display:
    ; Digit 1
    bcf     PORTA, 1
    bcf     PORTA, 2
    bcf     PORTA, 3
    bcf     PORTA, 5
    movf    DIGIT1_VAL, w
    call    Get_7Seg_Code
    movwf   PORTD
    bsf     PORTA, 1
    call    Delay_Short
    
    ; Digit 2 
    bcf     PORTA, 1
    bcf     PORTA, 2
    bcf     PORTA, 3
    bcf     PORTA, 5
    movf    DIGIT2_VAL, w
    call    Get_7Seg_Code
    movwf   TEMP_VAR
    
    ; Nokta Kontrol
    movf    DIGIT4_VAL, w
    sublw   11
    btfsc   STATUS, 2
    goto    No_Dot
    movf    DIGIT4_VAL, w
    sublw   12
    btfsc   STATUS, 2
    goto    No_Dot
    
    movf    TEMP_VAR, w
    iorlw   0x01        
    movwf   TEMP_VAR
No_Dot:
    movf    TEMP_VAR, w
    movwf   PORTD
    bsf     PORTA, 2
    call    Delay_Short
    
    ; Digit 3
    bcf     PORTA, 1
    bcf     PORTA, 2
    bcf     PORTA, 3
    bcf     PORTA, 5
    movf    DIGIT3_VAL, w
    call    Get_7Seg_Code
    movwf   PORTD
    bsf     PORTA, 3
    call    Delay_Short
    
    ; Digit 4
    bcf     PORTA, 1
    bcf     PORTA, 2
    bcf     PORTA, 3
    bcf     PORTA, 5
    movf    DIGIT4_VAL, w
    call    Get_7Seg_Code
    movwf   PORTD
    bsf     PORTA, 5
    call    Delay_Short
    return

Delay_Short:
    movlw   50
    movwf   d1
DS_Loop:
    decfsz  d1, f
    goto    DS_Loop
    return

    END resetVec