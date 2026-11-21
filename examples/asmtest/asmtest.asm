; nanoemu self-test in plain AVR assembly (build: avra asmtest.asm)
; - USART 115200 (U2X), RX interrupt echo
; - Timer0 fast PWM /64 on OC0A (D6, 25%) + overflow ISR tick counter
; - Timer1 CTC toggling OC1A (D9) at 1 kHz
; - ADC channel 0 read, MUL/branch arithmetic check, D13 blink
.include "m328Pdef.inc"

.def tmp  = r16
.def arg  = r17
.def tick = r24          ; incremented by TIMER0_OVF

.cseg
.org 0x0000
    rjmp reset
.org OVF0addr
    rjmp t0_ovf
.org URXCaddr
    rjmp usart_rx
.org INT_VECTORS_SIZE

reset:
    ldi tmp, high(RAMEND)
    out SPH, tmp
    ldi tmp, low(RAMEND)
    out SPL, tmp
    ; USART 115200 @16MHz with U2X -> UBRR = 16
    ldi tmp, (1<<U2X0)
    sts UCSR0A, tmp
    ldi tmp, 16
    sts UBRR0L, tmp
    ldi tmp, (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)
    sts UCSR0B, tmp
    ; pins: D13 (PB5), D9 (PB1) out; D6 (PD6) out
    ldi tmp, (1<<PB5)|(1<<PB1)
    out DDRB, tmp
    sbi DDRD, 6
    ; Timer0 fast PWM, non-inverting OC0A, /64, OCR0A=63
    ldi tmp, (1<<COM0A1)|(1<<WGM01)|(1<<WGM00)
    out TCCR0A, tmp
    ldi tmp, 63
    out OCR0A, tmp
    ldi tmp, (1<<CS01)|(1<<CS00)
    out TCCR0B, tmp
    ldi tmp, (1<<TOIE0)
    sts TIMSK0, tmp
    ; Timer1 CTC toggle OC1A, /8, OCR1A=999 -> 1 kHz square wave
    ldi tmp, high(999)
    sts OCR1AH, tmp
    ldi tmp, low(999)
    sts OCR1AL, tmp
    ldi tmp, (1<<COM1A0)
    sts TCCR1A, tmp
    ldi tmp, (1<<WGM12)|(1<<CS11)
    sts TCCR1B, tmp
    ; ADC: AVCC ref, channel 0, /128
    ldi tmp, (1<<REFS0)
    sts ADMUX, tmp
    ldi tmp, (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
    sts ADCSRA, tmp
    clr tick
    sei

    ldi ZL, low(hello*2)
    ldi ZH, high(hello*2)
    rcall puts
    ; arithmetic check: 0x37 * 0x5A = 0x1356
    ldi tmp, 0x37
    ldi arg, 0x5A
    mul tmp, arg
    mov arg, r1
    rcall puthex
    mov arg, r0
    rcall puthex
    rcall crlf

main:
    cpi tick, 250             ; ~256 ms
    brlo main
    clr tick
    sbi PINB, PB5             ; toggle D13
    ; ADC conversion
    lds tmp, ADCSRA
    ori tmp, (1<<ADSC)
    sts ADCSRA, tmp
adc_wait:
    lds tmp, ADCSRA
    sbrc tmp, ADSC
    rjmp adc_wait
    ldi ZL, low(adcmsg*2)
    ldi ZH, high(adcmsg*2)
    rcall puts
    lds r20, ADCL
    lds arg, ADCH
    rcall puthex
    mov arg, r20
    rcall puthex
    rcall crlf
    rjmp main

; ---- helpers ----
putc:                         ; arg -> UART
    lds tmp, UCSR0A
    sbrs tmp, UDRE0
    rjmp putc
    sts UDR0, arg
    ret
puts:                         ; Z -> zero-terminated flash string
    lpm arg, Z+
    tst arg
    breq puts_end
    rcall putc
    rjmp puts
puts_end:
    ret
crlf:
    ldi arg, 13
    rcall putc
    ldi arg, 10
    rjmp putc
puthex:                       ; arg as two hex digits
    push arg
    swap arg
    rcall nib
    pop arg
nib:
    andi arg, 0x0F
    cpi arg, 10
    brlo nib_d
    subi arg, -('A'-10)
    rjmp putc
nib_d:
    subi arg, -'0'
    rjmp putc

t0_ovf:
    push tmp
    in tmp, SREG
    inc tick
    out SREG, tmp
    pop tmp
    reti

usart_rx:                     ; echo received byte
    push tmp
    push arg
    in tmp, SREG
    push tmp
    lds arg, UDR0
    rcall putc
    pop tmp
    out SREG, tmp
    pop arg
    pop tmp
    reti

hello:  .db "nanoemu asm self-test", 13, 10, "mul=", 0
adcmsg: .db "ADC0=", 0
