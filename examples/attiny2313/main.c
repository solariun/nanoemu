#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

static void uart_init(unsigned int ubrr) {
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)ubrr;
    UCSRB = (1 << TXEN) | (1 << RXEN);
    UCSRC = (1 << UCSZ1) | (1 << UCSZ0); // 8N1
}

static void uart_putc(char c) {
    while (!(UCSRA & (1 << UDRE)))
        ;
    UDR = c;
}

static void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

int main(void) {
    // PB0..PB7 as outputs
    DDRB = 0xFF;
    PORTB = 0xAA; // 10101010 pattern

    // PD2..PD6 as outputs
    DDRD = (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6);
    PORTD = (1 << PD2) | (1 << PD4) | (1 << PD6);

    // PA0, PA1 as outputs
    DDRA = (1 << PA0) | (1 << PA1);
    PORTA = (1 << PA0);

    // Initialize UART at 9600 baud (UBRR = 103 for 16MHz)
    uart_init(103);

    uart_puts("Hello from ATtiny2313!\r\n");

    while (1) {
        _delay_ms(100);
        PORTB ^= 0xFF;
        uart_puts("Tick\r\n");
    }

    return 0;
}
