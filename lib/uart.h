// include/uart.h
#ifndef UART_H
#define UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uart_init(uint32_t baud);
void uart_putc(char c);
void uart_puts(const char *s);

void uart_print(const char *fmt, ...);

#ifdef __cplusplus
}
#endif
#endif