;===============================================================================
; PROJECT: HOME AUTOMATION - BOARD #2 (CURTAIN CONTROL SYSTEM)
; AUTHORS: Ali Yilmaz, Zeynep Pazarözyurt, Nazli Polat
; PROCESSOR: PIC16F877A
; COMPILER: XC8 (MPLAB X) - pic-as driver
; OSCILLATOR: 4MHz Crystal (XT Mode)
;===============================================================================
;
; TASK DISTRIBUTION
; -------------------------------------------
; * Ali Yilmaz		: Coding LCD Display
; * Zeynep Pazarözyurt	: UART Communication & Potentiometer Handling
; * Nazli Polat		: Coding Stepper Motor Driver & LDR Sensor Logic
;
;===============================================================================
;
; HARDWARE PINOUT & CONNECTIONS
; -------------------------------------------
; 1. LCD DISPLAY (HD44780 16x2 - 8-bit Mode)
;    Data Bus (PORTD):
;      RD0 - RD7  -> LCD D0 - D7
;
;    Control Pins (PORTB):
;      RB4        -> LCD RS (Register Select)
;      RB5        -> LCD EN (Enable)
;      RB0        -> LCD RW (Read/Write)
;                    * IMPORTANT: Must be configured as OUTPUT LOW (GND).
;
; -------------------------------------------
; 2. STEPPER MOTOR (28BYJ-48 Driver)
;    Control Pins (PORTC):
;      RC0        -> IN1
;      RC1        -> IN2
;      RC2        -> IN3
;      RC5        -> IN4  (Note: RC3 & RC4 are skipped/reserved)
;
; -------------------------------------------
; 3. SENSORS (Analog Inputs)
;      RA0 (AN0)  -> LDR (Light Dependent Resistor)
;      RA1 (AN1)  -> Potentiometer (Manual Position Control)
;
; -------------------------------------------
; 4. SERIAL COMMUNICATION (UART - 9600 Baud)
;      RC6 (TX)   -> Connect to PC RX
;      RC7 (RX)   -> Connect to PC TX
;
;===============================================================================

PROCESSOR 16F877A
#include <xc.inc>

CONFIG FOSC = XT            ; High-speed crystal oscillator
CONFIG WDTE = OFF           ; Watchdog Timer disabled
CONFIG PWRTE = ON           ; Power-up Timer enabled
CONFIG BOREN = ON           ; Brown-out Reset enabled
CONFIG LVP = OFF            ; Low-Voltage Programming disabled
CONFIG CPD = OFF            ; Data EEPROM code protection off
CONFIG WRT = OFF            ; Flash Program Memory Write disabled
CONFIG CP = OFF             ; Flash Program Memory Code Protection off

LDR_THRESHOLD       EQU     80      ; Darkness threshold (0-255)
STEPS_PER_PERCENT   EQU     10      ; 10 motor steps = 1%

; LCD Commands
LCD_CLEAR           EQU     0x01
LCD_HOME            EQU     0x02
LCD_ENTRY_MODE      EQU     0x06
LCD_DISPLAY_ON      EQU     0x0C
LCD_FUNCTION_8B     EQU     0x38
LCD_LINE1           EQU     0x80
LCD_LINE2           EQU     0xC0

PSECT udata_bank0

; ISR Context Save
W_TEMP:             DS  1
STATUS_TEMP:        DS  1
PCLATH_TEMP:        DS  1

; Motor Control
Current_Percent:    DS  1       ; Current curtain position (0-100%)
Desired_Percent:    DS  1       ; Target curtain position (0-100%)
Step_Index:         DS  1       ; Current step sequence (0-3)
Motor_Dir:          DS  1       ; 0=CCW (open), 1=CW (close)
Current_Steps:      DS  1       ; Steps within current percent (0-9)

; ADC Variables
LDR_Value:          DS  1       ; Light sensor reading (0-255)
POT_Value:          DS  1       ; Potentiometer reading (0-255)

; UART Variables
UART_Received:      DS  1       ; Received byte buffer
UART_Flag:          DS  1       ; New data flag

; General Purpose
Temp1:              DS  1
Temp2:              DS  1
Temp3:              DS  1
LCD_Temp:           DS  1
Delay_Cnt1:         DS  1
Delay_Cnt2:         DS  1
Delay_Cnt3:         DS  1

; Control Flags
Update_Cnt:         DS  1       ; LCD update counter
Override_Flag:      DS  1       ; UART override active

;===============================================================================
; Reset Vector - Address 0x0000
;===============================================================================
PSECT resetVec,class=CODE,abs,delta=2
ORG 0x0000
    GOTO    Main_Init

;===============================================================================
; Interrupt Vector - Address 0x0004
;===============================================================================
PSECT isrVec,class=CODE,abs,delta=2
ORG 0x0004
    GOTO    ISR_Handler

;===============================================================================
; Main Code Section
;===============================================================================
PSECT code,class=CODE,delta=2

;-------------------------------------------------------------------------------
; Interrupt Service Routine - UART Receive Handler
;-------------------------------------------------------------------------------
ISR_Handler:
    ; Context Saving 
    MOVWF   W_TEMP
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP
    MOVF    PCLATH, W
    MOVWF   PCLATH_TEMP
    CLRF    PCLATH

    ; OVERRUN ERROR KONTROLÜ 
    BANKSEL RCSTA
    BTFSS   RCSTA, 1        
    GOTO    Check_RCIF      

    BCF     RCSTA, 4        ; CREN = 0 (RX Kapat)
    MOVF    RCREG, W        
    MOVF    RCREG, W
    BSF     RCSTA, 4        ; CREN = 1 (RX Tekrar Aç)
    GOTO    ISR_Exit        
    ; -----------------------------------------------

Check_RCIF:
    ; UART Veri Geldi mi?
    BANKSEL PIR1
    BTFSS   PIR1, 5         ; RCIF set edildi mi?
    GOTO    ISR_Exit

    ; Veriyi oku
    BANKSEL RCREG
    MOVF    RCREG, W
    BANKSEL UART_Received
    MOVWF   UART_Received

    ; Bayra?? kald?r (Ana döngü i?lesin diye)
    MOVLW   1
    MOVWF   UART_Flag

ISR_Exit:
    ; Context Restore (Yazmaçlar? geri yükle)
    MOVF    PCLATH_TEMP, W
    MOVWF   PCLATH
    SWAPF   STATUS_TEMP, W
    MOVWF   STATUS
    SWAPF   W_TEMP, F
    SWAPF   W_TEMP, W
    RETFIE
;-------------------------------------------------------------------------------
; Main Initialization Routine
;-------------------------------------------------------------------------------
Main_Init:
    ; Clear output ports first
    BANKSEL PORTA
    CLRF    PORTA
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTD

    ;---------------------------------------------------------------------------
    ; Configure I/O Port Directions (TRIS registers in Bank 1)
    ;---------------------------------------------------------------------------
    BANKSEL TRISA
    
    ; PORTA: All inputs for ADC (RA0=LDR, RA1=POT)
    MOVLW   0xFF
    MOVWF   TRISA

    ; PORTB: RB0=RW(out), RB4=RS(out), RB5=EN(out), others input
    ; Bit pattern: 0=output, 1=input
    ; RB7 RB6 RB5 RB4 RB3 RB2 RB1 RB0
    ;  1   1   0   0   1   1   1   0  = 0xCE
    MOVLW   0xCE
    MOVWF   TRISB

    ; PORTC: RC0,RC1,RC2,RC5=stepper(out), RC6=TX(out), RC7=RX(in)
    ; RC3,RC4 reserved for I2C - set as inputs
    ; Bit pattern:
    ; RC7 RC6 RC5 RC4 RC3 RC2 RC1 RC0
    ;  1   0   0   1   1   0   0   0  = 0x98
    MOVLW   0x98
    MOVWF   TRISC

    ; PORTD: All outputs for LCD 8-bit data bus
    MOVLW   0x00
    MOVWF   TRISD

    ;---------------------------------------------------------------------------
    ; Configure ADC - ADCON1 for analog inputs
    ;---------------------------------------------------------------------------
    BANKSEL ADCON1
    ; Left justified result, RA0-RA1 analog, Vdd/Vss references
    MOVLW   0b00000100         ; PCFG3:0 = 0100: AN0,AN1 analog
    MOVWF   ADCON1

    ;---------------------------------------------------------------------------
    ; CRITICAL: Set RB0 LOW for LCD RW pin (Write mode always)
    ;---------------------------------------------------------------------------
    BANKSEL PORTB
    BCF     PORTB, 0            ; RW = LOW immediately at startup

    ;---------------------------------------------------------------------------
    ; Initialize RAM Variables
    ;---------------------------------------------------------------------------
    BANKSEL Current_Percent
    CLRF    Current_Percent     ; Start at 0% (fully open)
    CLRF    Desired_Percent
    CLRF    Step_Index
    CLRF    Motor_Dir
    CLRF    Current_Steps
    CLRF    LDR_Value
    CLRF    POT_Value
    CLRF    UART_Flag
    CLRF    Update_Cnt
    CLRF    Override_Flag

    ;---------------------------------------------------------------------------
    ; UART Ayarlar? (4MHz @ 9600 Baud - High Speed)
    ;---------------------------------------------------------------------------
    BANKSEL SPBRG
    MOVLW   25              ; SPBRG = 25 (Hata oran? %0.16)
    MOVWF   SPBRG

    BANKSEL TXSTA
    ; TXEN=1, SYNC=0, BRGH=1 
    MOVLW   0b00100100      ; Bit 2 = 1
    MOVWF   TXSTA

    BANKSEL RCSTA
    ; SPEN=1, CREN=1
    MOVLW   0b10010000
    MOVWF   RCSTA

    ;---------------------------------------------------------------------------
    ; Initialize LCD (HD44780 8-bit mode)
    ;---------------------------------------------------------------------------
    CALL    LCD_Init

    ;---------------------------------------------------------------------------
    ; Enable UART Receive Interrupt
    ;---------------------------------------------------------------------------
    BANKSEL PIE1
    BSF     PIE1, 5             ; RCIE = 1 (Enable UART RX interrupt)

    BANKSEL INTCON
    BSF     INTCON, 7           ; GIE = 1 (Global Interrupt Enable)
    BSF     INTCON, 6           ; PEIE = 1 (Peripheral Interrupt Enable)

    ;---------------------------------------------------------------------------
    ; Initial LCD Display
    ;---------------------------------------------------------------------------
    CALL    LCD_Update


Main_Loop:
    ; Safety: Keep LCD RW pin LOW
    BANKSEL PORTB
    BCF     PORTB, 0

    ;---------------------------------------------------------------------------
    ; UART KOMUT KONTROLÜ
    ;---------------------------------------------------------------------------
    BANKSEL UART_Flag
    MOVF    UART_Flag, F
    BTFSC   STATUS, 2           
    GOTO    Read_Sensors        
    
    CALL    Process_UART        
    BANKSEL UART_Flag
    CLRF    UART_Flag

Read_Sensors:
    ;---------------------------------------------------------------------------
    ; SENSÖR OKUMA
    ;---------------------------------------------------------------------------
    MOVLW   0                   ; Channel 0 (AN0)
    CALL    ADC_Read
    BANKSEL LDR_Value
    MOVWF   LDR_Value

    ;---------------------------------------------------------------------------
    ; NIGHTMODE
    ;---------------------------------------------------------------------------
    
    MOVLW   LDR_THRESHOLD       ; 80
    SUBWF   LDR_Value, W        ; LDR - 80
    BTFSC   STATUS, 0           
    GOTO    Day_Mode_Check      

    ; --- NIGHT MODU ---
    ; (LDR < 80).
    ; Set %100
    BANKSEL Desired_Percent
    MOVLW   100
    MOVWF   Desired_Percent
    
    ; Override
    BANKSEL Override_Flag
    CLRF    Override_Flag
    
    GOTO    Motor_Update        

Day_Mode_Check:
    ;---------------------------------------------------------------------------
    ; DAYMODE
    ;---------------------------------------------------------------------------
    
    BANKSEL Override_Flag
    MOVF    Override_Flag, F
    BTFSS   STATUS, 2           
    GOTO    Motor_Update        

    ;---------------------------------------------------------------------------
    ; POT MODE
    ;---------------------------------------------------------------------------
    
    MOVLW   1                   ; Channel 1 (AN1)
    CALL    ADC_Read
    BANKSEL POT_Value
    MOVWF   POT_Value

    CALL    Map_POT             ; Pot de?erini %'ye çevir

Motor_Update:
    ;---------------------------------------------------------------------------
    ; Motor Hareket ve LCD Güncelleme
    ;---------------------------------------------------------------------------
    CALL    Motor_Control

    BANKSEL Update_Cnt
    INCF    Update_Cnt, F
    MOVLW   50
    SUBWF   Update_Cnt, W
    BTFSS   STATUS, 0
    GOTO    Main_Loop

    CLRF    Update_Cnt
    CALL    LCD_Update

    GOTO    Main_Loop

;-------------------------------------------------------------------------------
; ADC Read Subroutine
; Input:  W = Channel number (0-7)
; Output: W = 8-bit ADC result (high byte, left justified)
;-------------------------------------------------------------------------------
ADC_Read:
    BANKSEL Temp1
    MOVWF   Temp1               ; Save channel number

    ; Build ADCON0 value: Fosc/8, selected channel, ADC ON
    ; Bits: ADCS1:0=01 (Fosc/8), CHS2:0=channel, GO=0, ADON=1
    RLF     Temp1, F            ; Shift channel left 3 times
    RLF     Temp1, F
    RLF     Temp1, W
    ANDLW   0b00111000         ; Mask channel bits
    IORLW   0b01000001         ; Add Fosc/8 and ADON

    BANKSEL ADCON0
    MOVWF   ADCON0

    ; Acquisition delay (~20us)
    CALL    Delay_20us

    ; Start conversion
    BSF     ADCON0, 2           ; GO = 1

ADC_Wait:
    BTFSC   ADCON0, 2           ; Wait for GO = 0
    GOTO    ADC_Wait

    ; Return result (ADRESH contains 8 MSBs)
    MOVF    ADRESH, W
    RETURN

;-------------------------------------------------------------------------------
; Map Potentiometer (0-255) to Percent (0-100)
; Uses multiplication: (POT * 100) / 256 ? POT * 0.39
; Result stored in Desired_Percent
;-------------------------------------------------------------------------------
Map_POT:
    BANKSEL POT_Value
    MOVF    POT_Value, W
    MOVWF   Temp1

    ; Quick approximation: (value * 100) / 256
    ; = (value * 64 + value * 32 + value * 4) / 256
    ; We calculate the high byte of the 16-bit product

    ; Multiply by 100 using shifts and adds
    ; POT * 4
    CLRF    Temp2
    MOVF    Temp1, W
    MOVWF   Temp3
    BCF     STATUS, 0
    RLF     Temp3, F
    RLF     Temp2, F
    RLF     Temp3, F
    RLF     Temp2, F            ; Temp2:Temp3 = POT * 4

    ; Save POT * 4
    MOVF    Temp2, W
    MOVWF   Delay_Cnt1          ; Borrow delay var
    MOVF    Temp3, W
    MOVWF   Delay_Cnt2

    ; POT * 32
    MOVF    Temp1, W
    CLRF    Temp2
    MOVWF   Temp3
    BCF     STATUS, 0
    RLF     Temp3, F
    RLF     Temp2, F
    RLF     Temp3, F
    RLF     Temp2, F
    RLF     Temp3, F
    RLF     Temp2, F
    RLF     Temp3, F
    RLF     Temp2, F
    RLF     Temp3, F
    RLF     Temp2, F            ; Temp2:Temp3 = POT * 32

    ; Add POT*4
    MOVF    Delay_Cnt2, W
    ADDWF   Temp3, F
    BTFSC   STATUS, 0
    INCF    Temp2, F
    MOVF    Delay_Cnt1, W
    ADDWF   Temp2, F            ; Temp2:Temp3 = POT * 36

    ; POT * 64
    MOVF    Temp1, W
    CLRF    Delay_Cnt1
    MOVWF   Delay_Cnt2
    BCF     STATUS, 0
    RLF     Delay_Cnt2, F
    RLF     Delay_Cnt1, F
    RLF     Delay_Cnt2, F
    RLF     Delay_Cnt1, F
    RLF     Delay_Cnt2, F
    RLF     Delay_Cnt1, F
    RLF     Delay_Cnt2, F
    RLF     Delay_Cnt1, F
    RLF     Delay_Cnt2, F
    RLF     Delay_Cnt1, F
    RLF     Delay_Cnt2, F
    RLF     Delay_Cnt1, F       ; Delay_Cnt1:Delay_Cnt2 = POT * 64

    ; Add to get POT * 100
    MOVF    Delay_Cnt2, W
    ADDWF   Temp3, F
    BTFSC   STATUS, 0
    INCF    Temp2, F
    MOVF    Delay_Cnt1, W
    ADDWF   Temp2, F            ; Temp2 = high byte of (POT * 100)

    ; Temp2 is effectively (POT * 100) / 256
    MOVF    Temp2, W

    ; Clamp to 100 maximum
    MOVWF   Temp1
    MOVLW   100
    SUBWF   Temp1, W
    BTFSC   STATUS, 0           ; If Temp1 >= 100
    GOTO    Map_Clamp
    MOVF    Temp1, W
    GOTO    Map_Store
Map_Clamp:
    MOVLW   100
Map_Store:
    MOVWF   Desired_Percent
    RETURN

;-------------------------------------------------------------------------------
; Motor Control - Move toward Desired_Percent
;-------------------------------------------------------------------------------
Motor_Control:
    BANKSEL Current_Percent
    MOVF    Desired_Percent, W
    SUBWF   Current_Percent, W  ; W = Current - Desired

    BTFSC   STATUS, 2           ; Z=1: Equal, no movement
    RETURN

    BTFSC   STATUS, 0           ; C=1: Current >= Desired (open, CCW)
    GOTO    Motor_Open

    ; Current < Desired: Close curtain (CW)
Motor_Close:
    MOVLW   1
    MOVWF   Motor_Dir
    CALL    Step_Motor

    ; Increment step counter
    INCF    Current_Steps, F
    MOVLW   STEPS_PER_PERCENT
    SUBWF   Current_Steps, W
    BTFSS   STATUS, 2           ; 10 steps done?
    RETURN

    ; Completed 10 steps = 1%
    CLRF    Current_Steps
    INCF    Current_Percent, F
    RETURN

Motor_Open:
    ; Open curtain (CCW)
    CLRF    Motor_Dir
    CALL    Step_Motor

    ; Decrement step counter
    MOVF    Current_Steps, F
    BTFSS   STATUS, 2           ; If steps > 0
    GOTO    Dec_Step

    ; Steps at 0, need to decrement percent
    MOVF    Current_Percent, F
    BTFSC   STATUS, 2           ; If percent is also 0
    RETURN                      ; Can't go below 0

    MOVLW   STEPS_PER_PERCENT - 1
    MOVWF   Current_Steps
    DECF    Current_Percent, F
    RETURN

Dec_Step:
    DECF    Current_Steps, F
    RETURN

;-------------------------------------------------------------------------------
; Step Motor - Output next step sequence
; Uses Step_Index and Motor_Dir
; Outputs to: RC0 (IN1), RC1 (IN2), RC2 (IN3), RC5 (IN4)
;-------------------------------------------------------------------------------
Step_Motor:
    BANKSEL Motor_Dir
    MOVF    Motor_Dir, F
    BTFSS   STATUS, 2           ; Z=1 means CCW
    GOTO    Step_CW

    ; CCW - Decrement step index
    MOVF    Step_Index, F
    BTFSS   STATUS, 2
    GOTO    Step_Dec
    MOVLW   3                   ; Wrap 0 -> 3
    MOVWF   Step_Index
    GOTO    Step_Output

Step_Dec:
    DECF    Step_Index, F
    GOTO    Step_Output

Step_CW:
    ; CW - Increment step index
    INCF    Step_Index, F
    MOVLW   4
    SUBWF   Step_Index, W
    BTFSS   STATUS, 2           ; Wrap 4 -> 0
    GOTO    Step_Output
    CLRF    Step_Index

Step_Output:
    ; Computed GOTO with PCLATH handling for step patterns
    MOVF    Step_Index, W
    ANDLW   0x03                ; Ensure 0-3
    ADDLW   LOW Step_Table
    MOVWF   Temp1
    MOVLW   HIGH Step_Table
    BTFSC   STATUS, 0           ; Check for page crossing
    ADDLW   1
    MOVWF   PCLATH
    MOVF    Temp1, W
    MOVWF   PCL

Step_Table:
    GOTO    Step_Pattern0
    GOTO    Step_Pattern1
    GOTO    Step_Pattern2
    GOTO    Step_Pattern3

; Full-step sequence patterns for stepper motor
; IN1=RC0, IN2=RC1, IN3=RC2, IN4=RC5

Step_Pattern0:                  ; Step 1: IN4 active
    BANKSEL PORTC
    BCF     PORTC, 0
    BCF     PORTC, 1
    BCF     PORTC, 2
    BSF     PORTC, 5
    GOTO    Step_Done

Step_Pattern1:                  ; Step 2: IN3 active
    BANKSEL PORTC
    BCF     PORTC, 0
    BCF     PORTC, 1
    BSF     PORTC, 2
    BCF     PORTC, 5
    GOTO    Step_Done

Step_Pattern2:                  ; Step 3: IN2 active
    BANKSEL PORTC
    BCF     PORTC, 0
    BSF     PORTC, 1
    BCF     PORTC, 2
    BCF     PORTC, 5
    GOTO    Step_Done

Step_Pattern3:                  ; Step 4: IN1 active
    BANKSEL PORTC
    BSF     PORTC, 0
    BCF     PORTC, 1
    BCF     PORTC, 2
    BCF     PORTC, 5
    ; Fall through to Step_Done

Step_Done:
    CALL    Delay_2ms           ; Inter-step delay
    RETURN

;-------------------------------------------------------------------------------
; Process UART Command
;-------------------------------------------------------------------------------
Process_UART:
    BANKSEL UART_Received
    MOVF    UART_Received, W
    MOVWF   Temp1

    ; Check for Set command (0x80 | value)
    BTFSC   Temp1, 7
    GOTO    UART_Set

    ; Command 0x01: Send Desired % Low byte (fractional = 0)
    MOVLW   0x01
    XORWF   Temp1, W
    BTFSS   STATUS, 2
    GOTO    Check_02
    MOVLW   0
    CALL    UART_Send
    RETURN

Check_02:
    ; Command 0x02: Send Desired % High byte
    MOVLW   0x02
    XORWF   Temp1, W
    BTFSS   STATUS, 2
    GOTO    Check_03
    BANKSEL Desired_Percent
    MOVF    Desired_Percent, W
    CALL    UART_Send
    RETURN

Check_03:
    ; Command 0x03: Send Temp Low 
    MOVLW   0x03
    XORWF   Temp1, W
    BTFSS   STATUS, 2
    GOTO    Check_04
    MOVLW   0xFF
    CALL    UART_Send
    RETURN

Check_04:
    ; Command 0x04: Send Temp High 
    MOVLW   0x04
    XORWF   Temp1, W
    BTFSS   STATUS, 2
    GOTO    Check_05
    MOVLW   0x00
    CALL    UART_Send
    RETURN

Check_05:
    ; Command 0x05: Send Pressure Low Byte
    MOVLW   0x05
    XORWF   Temp1, W
    BTFSS   STATUS, 2
    GOTO    Check_06
    
    MOVLW   3           
    
    CALL    UART_Send
    RETURN

Check_06:
    ; Command 0x06: Send Pressure High Byte 
    MOVLW   0x06
    XORWF   Temp1, W
    BTFSS   STATUS, 2
    GOTO    Check_07
    MOVLW   101         ; Ekranda 101 yazacak
    CALL    UART_Send
    RETURN

Check_07:
    ; Command 0x07: Send LDR Low Byte 
    MOVLW   0x07
    XORWF   Temp1, W
    BTFSS   STATUS, 2
    GOTO    Check_08
    
    MOVLW   0           
    CALL    UART_Send
    RETURN

Check_08:
    ; Command 0x08: Send LDR High Byte
    
    MOVLW   0x08
    XORWF   Temp1, W
    BTFSS   STATUS, 2
    RETURN              
    
    BANKSEL LDR_Value
    MOVF    LDR_Value, W 
    CALL    UART_Send
    RETURN

UART_Set:
    
    MOVF    Temp1, W
    ANDLW   0x7F                       
    MOVWF   Temp2

    ;---------------------------------------------------------------------------
    ; 100 SINIR KONTROLÜ
    ;---------------------------------------------------------------------------
    MOVLW   100
    SUBWF   Temp2, W
    BTFSC   STATUS, 0   
    GOTO    Set_Max     
    
    MOVF    Temp2, W    
    GOTO    Set_Apply

Set_Max:
    MOVLW   100

Set_Apply:
    BANKSEL Desired_Percent
    MOVWF   Desired_Percent
    
    ; PC kontrolünü aktif et (Override)
    MOVLW   1
    MOVWF   Override_Flag
    RETURN

;-------------------------------------------------------------------------------
; UART Send Byte
; Input: W = byte to send
;-------------------------------------------------------------------------------
UART_Send:
    BANKSEL Temp3
    MOVWF   Temp3               ; Save byte to send

    BANKSEL TXSTA
UART_Wait:
    BTFSS   TXSTA, 1            ; Wait for TRMT = 1
    GOTO    UART_Wait

    BANKSEL Temp3
    MOVF    Temp3, W
    BANKSEL TXREG
    MOVWF   TXREG
    RETURN

;-------------------------------------------------------------------------------
; LCD Initialization (HD44780 8-bit mode)
;-------------------------------------------------------------------------------
LCD_Init:
    ; Wait for LCD power-up (>40ms)
    CALL    Delay_50ms

    ; Ensure RW is LOW
    BANKSEL PORTB
    BCF     PORTB, 0

    ; Function set: 8-bit, 2 lines, 5x8 font (send 3 times per datasheet)
    MOVLW   LCD_FUNCTION_8B
    CALL    LCD_Cmd
    CALL    Delay_5ms

    MOVLW   LCD_FUNCTION_8B
    CALL    LCD_Cmd
    CALL    Delay_1ms

    MOVLW   LCD_FUNCTION_8B
    CALL    LCD_Cmd
    CALL    Delay_1ms

    ; Display ON, Cursor OFF
    MOVLW   LCD_DISPLAY_ON
    CALL    LCD_Cmd
    CALL    Delay_1ms

    ; Clear display
    MOVLW   LCD_CLEAR
    CALL    LCD_Cmd
    CALL    Delay_5ms

    ; Entry mode: Increment, no shift
    MOVLW   LCD_ENTRY_MODE
    CALL    LCD_Cmd
    CALL    Delay_1ms

    RETURN

;-------------------------------------------------------------------------------
; LCD Send Command
; Input: W = command byte
;-------------------------------------------------------------------------------
LCD_Cmd:
    BANKSEL LCD_Temp
    MOVWF   LCD_Temp

    BANKSEL PORTB
    BCF     PORTB, 0            ; RW = 0 (Write)
    BCF     PORTB, 4            ; RS = 0 (Command)

    BANKSEL LCD_Temp
    MOVF    LCD_Temp, W
    BANKSEL PORTD
    MOVWF   PORTD               ; Output data

    ; Pulse Enable
    BANKSEL PORTB
    BSF     PORTB, 5            ; EN = 1
    NOP
    NOP
    BCF     PORTB, 5            ; EN = 0

    CALL    Delay_1ms
    RETURN

;-------------------------------------------------------------------------------
; LCD Send Data (Character)
; Input: W = ASCII character
;-------------------------------------------------------------------------------
LCD_Data:
    BANKSEL LCD_Temp
    MOVWF   LCD_Temp

    BANKSEL PORTB
    BCF     PORTB, 0            ; RW = 0 (Write)
    BSF     PORTB, 4            ; RS = 1 (Data)

    BANKSEL LCD_Temp
    MOVF    LCD_Temp, W
    BANKSEL PORTD
    MOVWF   PORTD               ; Output character

    ; Pulse Enable
    BANKSEL PORTB
    BSF     PORTB, 5            ; EN = 1
    NOP
    NOP
    BCF     PORTB, 5            ; EN = 0

    CALL    Delay_1ms
    RETURN

;-------------------------------------------------------------------------------
; LCD Update Display
; Line 1: "+25.5C 1013hPa"
; Line 2: "L:XXX P:YYY%"
;-------------------------------------------------------------------------------
LCD_Update:
    ; Position cursor at Line 1
    MOVLW   LCD_LINE1
    CALL    LCD_Cmd

    ; Display simulated BMP180 values: "+25.5C 1013hPa"
    MOVLW   '+'
    CALL    LCD_Data
    MOVLW   '2'
    CALL    LCD_Data
    MOVLW   '5'
    CALL    LCD_Data
    MOVLW   '.'
    CALL    LCD_Data
    MOVLW   '5'
    CALL    LCD_Data
    MOVLW   'C'
    CALL    LCD_Data
    MOVLW   ' '
    CALL    LCD_Data
    MOVLW   '1'
    CALL    LCD_Data
    MOVLW   '0'
    CALL    LCD_Data
    MOVLW   '1'
    CALL    LCD_Data
    MOVLW   '3'
    CALL    LCD_Data
    MOVLW   'h'
    CALL    LCD_Data
    MOVLW   'P'
    CALL    LCD_Data
    MOVLW   'a'
    CALL    LCD_Data
    MOVLW   ' '
    CALL    LCD_Data
    MOVLW   ' '
    CALL    LCD_Data

    ; Position cursor at Line 2
    MOVLW   LCD_LINE2
    CALL    LCD_Cmd

    ; Display "L:"
    MOVLW   'L'
    CALL    LCD_Data
    MOVLW   ':'
    CALL    LCD_Data

    ; Display LDR value (3 digits)
    BANKSEL LDR_Value
    MOVF    LDR_Value, W
    CALL    Display_Num

    ; Display " P:"
    MOVLW   ' '
    CALL    LCD_Data
    MOVLW   'P'
    CALL    LCD_Data
    MOVLW   ':'
    CALL    LCD_Data

    ; Display Current Percent (3 digits)
    BANKSEL Current_Percent
    MOVF    Current_Percent, W
    CALL    Display_Num

    ; Display "%"
    MOVLW   '%'
    CALL    LCD_Data

    ; Pad line
    MOVLW   ' '
    CALL    LCD_Data
    MOVLW   ' '
    CALL    LCD_Data
    MOVLW   ' '
    CALL    LCD_Data

    RETURN

;-------------------------------------------------------------------------------
; Display 3-Digit Number
; Input: W = number (0-255)
;-------------------------------------------------------------------------------
Display_Num:
    BANKSEL Temp1
    MOVWF   Temp1               ; Number to convert

    ; Extract hundreds digit
    CLRF    Temp2
Hundreds:
    MOVLW   100
    SUBWF   Temp1, W
    BTFSS   STATUS, 0           ; Borrow?
    GOTO    Show_H
    MOVWF   Temp1
    INCF    Temp2, F
    GOTO    Hundreds

Show_H:
    MOVF    Temp2, W
    ADDLW   '0'
    CALL    LCD_Data

    ; Extract tens digit
    CLRF    Temp2
Tens:
    MOVLW   10
    SUBWF   Temp1, W
    BTFSS   STATUS, 0
    GOTO    Show_T
    MOVWF   Temp1
    INCF    Temp2, F
    GOTO    Tens

Show_T:
    MOVF    Temp2, W
    ADDLW   '0'
    CALL    LCD_Data

    ; Units digit
    MOVF    Temp1, W
    ADDLW   '0'
    CALL    LCD_Data

    RETURN

;-------------------------------------------------------------------------------
; Delay Subroutines (4MHz clock = 1us/instruction)
;-------------------------------------------------------------------------------

; ~20us delay
Delay_20us:
    BANKSEL Delay_Cnt1
    MOVLW   6
    MOVWF   Delay_Cnt1
D20_Loop:
    NOP
    DECFSZ  Delay_Cnt1, F
    GOTO    D20_Loop
    RETURN

; ~1ms delay
Delay_1ms:
    BANKSEL Delay_Cnt1
    MOVLW   250
    MOVWF   Delay_Cnt1
D1ms_Loop:
    NOP
    NOP
    DECFSZ  Delay_Cnt1, F
    GOTO    D1ms_Loop
    RETURN

; ~2ms delay
Delay_2ms:
    CALL    Delay_1ms
    CALL    Delay_1ms
    RETURN

; ~5ms delay
Delay_5ms:
    BANKSEL Delay_Cnt2
    MOVLW   5
    MOVWF   Delay_Cnt2
D5ms_Loop:
    CALL    Delay_1ms
    BANKSEL Delay_Cnt2
    DECFSZ  Delay_Cnt2, F
    GOTO    D5ms_Loop
    RETURN

; ~50ms delay
Delay_50ms:
    BANKSEL Delay_Cnt2
    MOVLW   50
    MOVWF   Delay_Cnt2
D50ms_Loop:
    CALL    Delay_1ms
    BANKSEL Delay_Cnt2
    DECFSZ  Delay_Cnt2, F
    GOTO    D50ms_Loop
    RETURN

END