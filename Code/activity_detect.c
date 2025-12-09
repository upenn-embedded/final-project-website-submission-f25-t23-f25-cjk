#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "i2c.h"
#include "activity_model.h"

#include "../lib/ST7735.h"
#include "../lib/LCD_GFX.h"

// ==== State Tracking ====
static int last_cls = -1;          // Previous predicted class
static uint32_t sitting_secs = 0;  // Sitting duration counter (approx seconds)
static uint32_t running_secs = 0;  // Running duration counter (approx seconds)
static uint16_t jumping_jacks_secs = 0; // Jumping jacks duration counter (approx seconds)
static uint8_t sitting_warning_shown = 0; // Flag for sitting warning message
static uint16_t break_count = 0;   // Number of breaks taken (exercise -> sitting)

/* ---------------------------------------------------------
   IMU / LSM6DSO
--------------------------------------------------------- */
#define IMU_ADDR 0x6B

#define LSM6DSO_WHO_AM_I   0x0F
#define LSM6DSO_CTRL1_XL   0x10
#define LSM6DSO_CTRL2_G    0x11
#define LSM6DSO_CTRL3_C    0x12
#define LSM6DSO_OUTX_L_G   0x22
#define LSM6DSO_OUTX_L_A   0x28

#define CTRL1_XL_104HZ     0x40
#define CTRL2_G_104HZ      0x40
#define CTRL3_C_SETTINGS   0x44

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

static void imu_read_axes(
    int16_t *ax, int16_t *ay, int16_t *az,
    int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    imu_read_regs(LSM6DSO_OUTX_L_G, buf, 6);
    *gx = (int16_t)(buf[1]<<8 | buf[0]);
    *gy = (int16_t)(buf[3]<<8 | buf[2]);
    *gz = (int16_t)(buf[5]<<8 | buf[4]);

    imu_read_regs(LSM6DSO_OUTX_L_A, buf, 6);
    *ax = (int16_t)(buf[1]<<8 | buf[0]);
    *ay = (int16_t)(buf[3]<<8 | buf[2]);
    *az = (int16_t)(buf[5]<<8 | buf[4]);
}

/* ---------------------------------------------------------
   Statistics
--------------------------------------------------------- */
typedef struct {
    float    sum_ax;
    float    sum_acc_mag;
    float    sumsq_acc_mag;
    uint16_t count;
} Stats;

static Stats stats;

static void stats_reset(Stats *s) {
    memset(s, 0, sizeof(Stats));
}

static void stats_update(Stats *s,
                         int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz)
{
    (void)gx; (void)gy; (void)gz;

    s->count++;
    float fax = (float)ax;
    float fay = (float)ay;
    float faz = (float)az;

    s->sum_ax += fax;

    // Compute accelerometer magnitude
    float acc_mag = sqrtf(fax*fax + fay*fay + faz*faz);
    s->sum_acc_mag += acc_mag;
    s->sumsq_acc_mag += acc_mag * acc_mag;
}

static void compute_features(const Stats *s, float out[32]) {
    for (uint8_t i = 0; i < 32; i++) out[i] = 0.0f;
    if (s->count == 0) return;

    float n = (float)s->count;

    // Feature 0: mean_ax
    out[0] = s->sum_ax / n;

    // Feature 13: acc_mag_std
    float mean_acc_mag = s->sum_acc_mag / n;
    float var_acc_mag = s->sumsq_acc_mag / n - mean_acc_mag * mean_acc_mag;
    if (var_acc_mag < 0) var_acc_mag = 0;
    out[13] = sqrtf(var_acc_mag);
}

/* ---------------------------------------------------------
   LCD Display
--------------------------------------------------------- */
static void lcd_init_screen(void)
{
    lcd_init();
    LCD_brightness(200);
    LCD_rotate(1);
    LCD_setScreen(rgb565(0,0,0));

    LCD_drawString(10, 10, "IMU + Activity",
                   rgb565(255,255,0), rgb565(0,0,0));
    LCD_drawString(10, 20, "Activity:",
                   rgb565(255,255,255), rgb565(0,0,0));
    LCD_drawString(10, 50, "WAITING",
                   rgb565(128,128,128), rgb565(0,0,0));
    _delay_ms(500);
}

static void lcd_show_activity(int cls)
{
    const char *label;
    uint16_t bg;
    uint16_t fg;

    switch (cls) {
        case ACTIVITY_SITTING:
            label = "SITTING";
            bg = rgb565(0, 40, 0);        // dark green
            fg = rgb565(0, 255, 0);      // bright green
            break;

        case ACTIVITY_RUNNING:
            label = "RUNNING";
            bg = rgb565(0, 0, 80);       // dark blue
            fg = rgb565(0, 128, 255);    // bright blue
            break;

        case ACTIVITY_JUMPINGJACKS:
            label = "JUMPING";
            bg = rgb565(80, 0, 0);       // dark red
            fg = rgb565(255, 0, 0);      // bright red
            break;

        default:
            label = "UNKNOWN";
            bg = rgb565(0, 0, 0);        // black
            fg = rgb565(255, 255, 255);  // white
            break;
    }

    LCD_setScreen(bg);

    LCD_drawString(10, 20, "Activity:",
                   rgb565(255,255,255), bg);

    LCD_drawString(10, 50, (char*)label,
                   fg, bg);

    // Display break count
    char break_str[20];
    snprintf(break_str, sizeof(break_str), "Breaks Taken: %u", break_count);
    LCD_drawString(10, 70, break_str,
                   rgb565(200,200,200), bg);
}

static void lcd_show_sitting_warning(void)
{
    uint16_t bg = rgb565(0, 40, 0);  // dark green (sitting background)
    LCD_drawString(10, 90, "You've been",
                   rgb565(255,255,0), bg);
    LCD_drawString(10, 100, "sitting too long!",
                   rgb565(255,255,0), bg);
}

static void lcd_clear_sitting_warning(void)
{
    uint16_t bg = rgb565(0, 40, 0);  // dark green (sitting background)
    LCD_drawBlock(10, 90, 150, 110, bg);
}

/* ---------------------------------------------------------
   Beeper (Normal GPIO toggling)
--------------------------------------------------------- */
#define SPK_DDR   DDRD
#define SPK_PORT  PORTD
#define SPK_PIN   PD5

static void beep_init(void)
{
    TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0)); // Disable Timer0 PWM on PD5
    SPK_DDR  |=  (1 << SPK_PIN);
    SPK_PORT &= ~(1 << SPK_PIN);               // Silence
}

static void delay_us_runtime(uint16_t us)
{
    while (us--) _delay_us(1);
}

static void beep_tone(uint16_t freq_hz, uint16_t duration_ms)
{
    if (freq_hz == 0) return;

    uint32_t period_us = 1000000UL / freq_hz;
    uint16_t half_us   = (uint16_t)(period_us / 2);
    uint32_t cycles    = ((uint32_t)duration_ms * 1000UL) / period_us;

    for (uint32_t i = 0; i < cycles; i++) {
        SPK_PORT |=  (1 << SPK_PIN);
        delay_us_runtime(half_us);
        SPK_PORT &= ~(1 << SPK_PIN);
        delay_us_runtime(half_us);
    }
    SPK_PORT &= ~(1 << SPK_PIN);
}

/* Activity Transition Beep */
static void beep_transition(int cls)
{
    (void)cls;
    beep_tone(1000, 80);
}

/* Sitting Reminder - three lower long beeps */
static void beep_reminder_sitting(void)
{
    beep_tone(700, 200);
    _delay_ms(150);
    beep_tone(700, 200);
    _delay_ms(150);
    beep_tone(700, 200);
}

/* Running Reminder - one long and short beep */
static void beep_reminder_running(void)
{
    beep_tone(1200, 280);
    _delay_ms(120);
    beep_tone(1200, 80);
}

/* Jumping Reminder - two short high beeps */
static void beep_reminder_jumping(void)
{
    beep_tone(2000, 80);
    _delay_ms(80);
    beep_tone(2000, 80);
}

/* ---------------------------------------------------------
   MAIN LOOP
--------------------------------------------------------- */
int main(void)
{
    I2C_init(1);
    _delay_ms(50);
    imu_init();

    lcd_init_screen();
    beep_init();

    stats_reset(&stats);

    last_cls = -1;
    sitting_secs = 0;
    running_secs = 0;
    jumping_jacks_secs = 0;
    sitting_warning_shown = 0;
    break_count = 0;

    while (1) {

        int16_t ax, ay, az, gx, gy, gz;
        imu_read_axes(&ax, &ay, &az, &gx, &gy, &gz);
        stats_update(&stats, ax, ay, az, gx, gy, gz);

        if (stats.count >= 100) {
            float features[32];
            compute_features(&stats, features);
            int cls = predict_activity(features);

            // 1) Activity Transition
            if (cls != last_cls) {
                beep_transition(cls);

                // Increment break counter when switching from exercise to sitting
                if (cls == ACTIVITY_SITTING &&
                    (last_cls == ACTIVITY_RUNNING || last_cls == ACTIVITY_JUMPINGJACKS)) {
                    break_count++;
                }

                lcd_show_activity(cls);

                if (cls == ACTIVITY_SITTING) sitting_secs = 0;
                if (cls == ACTIVITY_RUNNING) running_secs = 0;
                if (cls == ACTIVITY_JUMPINGJACKS) jumping_jacks_secs = 0;

                // Clear sitting warning when switching activities
                if (sitting_warning_shown) {
                    lcd_clear_sitting_warning();
                    sitting_warning_shown = 0;
                }

                last_cls = cls;
            }

            // Each 100 samples ? 1 second
            if (cls == ACTIVITY_SITTING) {
                sitting_secs++;
                if (sitting_secs >= 10) { // 10s
                    beep_reminder_sitting();

                    // Show warning message on LCD
                    if (!sitting_warning_shown) {
                        lcd_show_sitting_warning();
                        sitting_warning_shown = 1;
                    }

                    sitting_secs = 0;
                }
                running_secs = 0;
                jumping_jacks_secs = 0;
            }
            else if (cls == ACTIVITY_RUNNING) {
                running_secs++;
                if (running_secs >= 5) { // 5s
                    beep_reminder_running();
                    running_secs = 0;
                }
                sitting_secs = 0;
                jumping_jacks_secs = 0;
            }
            else if (cls == ACTIVITY_JUMPINGJACKS) {
                jumping_jacks_secs++;
                if (jumping_jacks_secs >= 10) { // 10 jumps approx
                    beep_reminder_jumping();
                    jumping_jacks_secs = 0;
                }
                sitting_secs = 0;
                running_secs = 0;
            }
            else {
                sitting_secs = running_secs = jumping_jacks_secs = 0;
            }

            stats_reset(&stats);
        }

        _delay_ms(10);
    }

    return 0;
}
