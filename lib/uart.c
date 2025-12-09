// src/uart.c
#include <avr/io.h>
#include <stdarg.h>
#include <stdio.h>
#include "uart.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

static inline void uart_set_baud(uint32_t baud) {
    uint16_t ubrr = (F_CPU / 16 / baud) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);
}

void uart_init(uint32_t baud) {
    uart_set_baud(baud);
    UCSR0B = (1 << TXEN0);                       // 
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);      // 8N1
}

void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {;}
    UDR0 = c;
}

void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

void uart_print(const char *fmt, ...) {
    char buf[64];                     // 
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart_puts(buf);
}