#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>

#include "i2c.h"    // I2C driver from WS3 (implemented in lib_i2c_imu.a)

/* ----------------------------------------------------------
   UART at 9600 baud (8N1)
   Baud = F_CPU / (16 * (UBRR+1))
   UBRR = 103 for 9600 baud @ 16 MHz
   ---------------------------------------------------------- */
static void uart_init(void) {
    uint16_t ubrr = 103;   // 9600 baud @ 16 MHz (normal speed)

    UCSR0A = 0;                  // normal UART speed (U2X0 = 0)
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;

    UCSR0B = _BV(TXEN0);         // enable UART TX
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);   // 8 data bits, no parity, 1 stop bit
}

static void uart_putc(char c) {
    while (!(UCSR0A & _BV(UDRE0)));  // wait for transmit buffer
    UDR0 = c;
}

static void uart_print(const char *s) {
    while (*s) uart_putc(*s++);
}

static void uart_print_dec32(uint32_t v) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)v);
    uart_print(buf);
}

static void uart_print_int16(int16_t v) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", v);
    uart_print(buf);
}

/* ----------------------------------------------------------
   LSM6DSO IMU Defines
   ---------------------------------------------------------- */
#define IMU_ADDR 0x6B    // using 0x6B based on your wiring (SA0/SDO high)

#define LSM6DSO_WHO_AM_I   0x0F
#define LSM6DSO_CTRL1_XL   0x10
#define LSM6DSO_CTRL2_G    0x11
#define LSM6DSO_CTRL3_C    0x12

#define LSM6DSO_OUTX_L_G   0x22
#define LSM6DSO_OUTX_L_A   0x28

#define CTRL1_XL_104HZ     0x40  // 104 Hz, ±2g
#define CTRL2_G_104HZ      0x40  // 104 Hz, 245 dps
#define CTRL3_C_SETTINGS   0x44  // BDU + auto-increment

/* ----------------------------------------------------------
   IMU wrapper using WS3 I2C library
   ---------------------------------------------------------- */
static void imu_write_reg(uint8_t reg, uint8_t data) {
    I2C_writeRegister(IMU_ADDR, data, reg);
}

static void imu_read_regs(uint8_t start_reg, uint8_t *buf, uint8_t len) {
    I2C_readCompleteStream(buf, IMU_ADDR, start_reg, len);
}

static void imu_init(void) {
    uint8_t who = 0;
    I2C_readRegister(IMU_ADDR, &who, LSM6DSO_WHO_AM_I);

    imu_write_reg(LSM6DSO_CTRL3_C, CTRL3_C_SETTINGS);
    imu_write_reg(LSM6DSO_CTRL1_XL, CTRL1_XL_104HZ);
    imu_write_reg(LSM6DSO_CTRL2_G,  CTRL2_G_104HZ);

    _delay_ms(10);
}

/* Read accel + gyro blocks */
static void imu_read_axes(
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    // Gyro
    imu_read_regs(LSM6DSO_OUTX_L_G, buf, 6);
    *gx = (int16_t)(buf[1]<<8 | buf[0]);
    *gy = (int16_t)(buf[3]<<8 | buf[2]);
    *gz = (int16_t)(buf[5]<<8 | buf[4]);

    // Accel
    imu_read_regs(LSM6DSO_OUTX_L_A, buf, 6);
    *ax = (int16_t)(buf[1]<<8 | buf[0]);
    *ay = (int16_t)(buf[3]<<8 | buf[2]);
    *az = (int16_t)(buf[5]<<8 | buf[4]);
}

/* ----------------------------------------------------------
   MAIN
   ---------------------------------------------------------- */
int main(void) {
    uart_init();
    I2C_init(1);
    _delay_ms(50);

    uart_print("Starting IMU Logger at 9600 baud...\r\n");
    uart_print("Initializing IMU...\r\n");

    imu_init();

    uart_print("IMU Ready!\r\n");
    uart_print("timestamp_ms,ax,ay,az,gx,gy,gz\r\n");

    uint32_t t_ms = 0;

    // Log indefinitely at ~100 Hz
    while (1) {
        int16_t ax, ay, az, gx, gy, gz;
        imu_read_axes(&ax, &ay, &az, &gx, &gy, &gz);

        uart_print_dec32(t_ms); uart_print(",");
        uart_print_int16(ax); uart_print(",");
        uart_print_int16(ay); uart_print(",");
        uart_print_int16(az); uart_print(",");
        uart_print_int16(gx); uart_print(",");
        uart_print_int16(gy); uart_print(",");
        uart_print_int16(gz); uart_print("\r\n");
//        uart_print("t=");   uart_print_dec32(t_ms); uart_print(",");
//        uart_print("ax=");  uart_print_int16(ax);   uart_print(",");
//        uart_print("ay=");  uart_print_int16(ay);   uart_print(",");
//        uart_print("az=");  uart_print_int16(az);   uart_print(",");
//        uart_print("gx=");  uart_print_int16(gx);   uart_print(",");
//        uart_print("gy=");  uart_print_int16(gy);   uart_print(",");
//        uart_print("gz=");  uart_print_int16(gz);   uart_print("\r\n");


        _delay_ms(10);
        t_ms += 10;   // rolls over after ~49 days; fine for your use
    }

    return 0;
}
