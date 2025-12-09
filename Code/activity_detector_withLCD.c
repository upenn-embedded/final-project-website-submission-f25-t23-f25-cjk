#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "i2c.h"                 // I2C driver from WS3 (lib_i2c_imu.a)
#include "activity_model.h"  // generated decision tree: predict_activity()

#include "lib/ST7735.h"
#include "lib/LCD_GFX.h"


/* ----------------------------------------------------------
   UART at 9600 baud (8N1)
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
   LSM6DSO IMU Defines (same as your working logger)
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
   Stats accumulator: only what we need:
   - std_ay  -> needs sum_ay, sumsq_ay, count
   - ptp_ax  -> needs min_ax, max_ax
   ---------------------------------------------------------- */
typedef struct {
    float    sum_ay;
    float    sumsq_ay;
    uint16_t count;

    int16_t  min_ax;
    int16_t  max_ax;
    uint8_t  initialized;   // whether min/max have been set yet
} Stats;

static Stats stats;

static void stats_reset(Stats *s) {
    memset(s, 0, sizeof(Stats));
}

static void stats_update(Stats *s,
                         int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz)
{
    (void)az; (void)gx; (void)gy; (void)gz; // unused in this minimal model

    // Update count and ay accumulators
    s->count++;

    float fay = (float)ay;
    s->sum_ay   += fay;
    s->sumsq_ay += fay * fay;

    // Update ptp_ax (min/max ax)
    if (!s->initialized) {
        s->min_ax     = ax;
        s->max_ax     = ax;
        s->initialized = 1;
    } else {
        if (ax < s->min_ax) s->min_ax = ax;
        if (ax > s->max_ax) s->max_ax = ax;
    }
}

/*
   compute_features:
   - Builds a 32-element feature vector, but only fills:
       features[7]  = std_ay
       features[16] = ptp_ax
     All other entries are 0.0f, which is fine since the tree
     only uses indices 7 and 16.
*/
static void compute_features(const Stats *s, float out[32]) {
    // Zero all features by default
    for (uint8_t i = 0; i < 32; i++) {
        out[i] = 0.0f;
    }

    if (s->count == 0 || !s->initialized) {
        return;
    }

    float n = (float)s->count;

    // std_ay
    float mean_ay = s->sum_ay / n;
    float var_ay  = s->sumsq_ay / n - mean_ay * mean_ay;
    if (var_ay < 0) var_ay = 0;
    float std_ay  = sqrtf(var_ay);

    // ptp_ax (peak-to-peak of ax)
    int32_t diff_ax = (int32_t)s->max_ax - (int32_t)s->min_ax;
    float ptp_ax = (float)diff_ax;

    // Map into indices used by the decision tree:
    //   features[7]  -> std_ay
    //   features[16] -> ptp_ax
    out[7]  = std_ay;
    out[16] = ptp_ax;
}

/* ----------------------------------------------------------
   LCD
   ---------------------------------------------------------- */

static void lcd_init_screen(void)
{
    // Initialize LCD (SPI + GPIO + ST7735 settings)   
    lcd_init();              // function from ST7735.c sets up SPI and LCD commands  
    LCD_brightness(200);     // Set LCD backlight brightness (range 0–255)    
    LCD_rotate(1);           // Rotate display orientation (0–3 depending on physical LCD orientation) 
    LCD_setScreen(rgb565(0,0,0));  
    
    LCD_drawString(10, 10, "IMU + Activity",
                   rgb565(255,255,0),  // yellow
                   rgb565(0,0,0));     // black

    // show “WAITING” first
    LCD_drawString(10, 20, "Activity:",
                   rgb565(255,255,255), // white
                   rgb565(0,0,0));
    LCD_drawString(10, 50, "WAITING",
                   rgb565(128,128,128), // gray
                   rgb565(0,0,0));

    _delay_ms(500);
}

/* ---------------- Display activity result on LCD ---------------- */
static void lcd_show_activity(int cls)
{
    const char *label;
    uint16_t color;

    // class → Text + Color
    switch(cls) {
        case ACTIVITY_SITTING:
            label = "SITTING";
            color = rgb565(0,255,0);      // green
            break;

        case ACTIVITY_RUNNING:
            label = "RUNNING";
            color = rgb565(0,128,255);    // blue-ish
            break;

        case ACTIVITY_JUMPINGJACKS:
            label = "JUMPING";
            color = rgb565(255,0,0);      // red
            break;

        default:
            label = "UNKNOWN";
            color = rgb565(255,255,255);  // white
            break;
    }

    // clear screen and show activity
    LCD_setScreen(rgb565(0,0,0)); // black

    LCD_drawString(10, 20, "Activity:",
                   rgb565(255,255,255), // white
                   rgb565(0,0,0));  

    LCD_drawString(10, 50, (char*)label,
                   color,
                   rgb565(0,0,0));
}


/* ----------------------------------------------------------
   MAIN
   ---------------------------------------------------------- */
int main(void) {
    //uart_init();
    I2C_init(1);
    _delay_ms(50);

    //uart_print("Starting IMU Logger + 3-class Classifier at 9600 baud...\r\n");
    //uart_print("Initializing IMU...\r\n");

    imu_init();

    //uart_print("IMU Ready!\r\n");
    //uart_print("timestamp_ms,ax,ay,az,gx,gy,gz\r\n");

    lcd_init_screen();

    stats_reset(&stats);
    int last_cls = -1;   // The last class displayed, avoid reloading if no changes occur.
    
    //uint32_t t_ms = 0;
    //stats_reset(&stats);

    // Loop at ~100 Hz
    while (1) {
        int16_t ax, ay, az, gx, gy, gz;
        imu_read_axes(&ax, &ay, &az, &gx, &gy, &gz);

        // Update stats for classification window
        stats_update(&stats, ax, ay, az, gx, gy, gz);

        // Print raw CSV line so you can still log if desired
//        uart_print_dec32(t_ms); uart_print(",");
//        uart_print_int16(ax);   uart_print(",");
//        uart_print_int16(ay);   uart_print(",");
//        uart_print_int16(az);   uart_print(",");
//        uart_print_int16(gx);   uart_print(",");
//        uart_print_int16(gy);   uart_print(",");
//        uart_print_int16(gz);   uart_print("\r\n");

        // Every 100 samples (~1 second), compute features and classify
        if (stats.count >= 100) {
            float features[32];
            compute_features(&stats, features);

            int cls = predict_activity(features);

            /*uart_print("Activity: ");
            if (cls == ACTIVITY_SITTING) {
                uart_print("sitting\r\n");
            } else if (cls == ACTIVITY_RUNNING) {
                uart_print("running\r\n");
            } else if (cls == ACTIVITY_JUMPINGJACKS) {
                uart_print("jumpingjacks\r\n");
            } else {
                uart_print("unknown\r\n");
            }*/

             if (cls != last_cls) {
                lcd_show_activity(cls);
                last_cls = cls;
            }
            
            stats_reset(&stats);
        }

        _delay_ms(10);
        //t_ms += 10;
    }

    return 0;
}
