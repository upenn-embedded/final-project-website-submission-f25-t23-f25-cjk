#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "running_audio.h"
#include "sitting_audio.h"
#include "jumping_audio.h"

volatile uint32_t audio_index = 0;

/* -------- Timer1: 16 kHz sample playback ISR (updates PWM duty) -------- */
ISR(TIMER1_COMPA_vect)
{
    // Playback using jumping_audio (change index if testing other clips)
    if (audio_index < audio_jumping_len)
    {
        uint8_t sample = pgm_read_byte(&audio_jumping[audio_index++]);
        OCR0B = sample;   // push sample to PWM duty cycle (0–255)
    }
    else
    {
        // End of audio → stop Timer1 interrupt (uncomment below to loop playback)
        TIMSK1 &= ~(1 << OCIE1A);
        // audio_index = 0;   // enable this to loop continuously
    }
}

/* -------- Timer0: high-frequency PWM output on PD5 (OC0B) -------- */
void pwm_init(void)
{
    // Ensure Timer0/Timer1 are not in power-down mode
    PRR0 &= ~(1 << PRTIM0);
    PRR0 &= ~(1 << PRTIM1);

    // PD5 set as output (OC0B → speaker)
    DDRD |= (1 << PD5);

    // Timer0 Fast PWM, 8-bit, TOP = 0xFF
    // Mode 3: WGM02=0, WGM01=1, WGM00=1
    TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0B1);
    TCCR0B = (1 << CS00);  // No prescaling → ~62.5 kHz PWM carrier

    OCR0B = 128; // Mid-level duty when idle (silence)
}

/* -------- Timer1: 16 kHz sample playback timing -------- */
void sample_timer_init(void)
{
    TCCR1A = 0;
    TCCR1B = 0;

    // CTC mode, TOP = OCR1A
    // Playback rate Fs = 16 kHz:
    // F_CPU / (prescaler * (OCR1A + 1)) = 16000
    // 16 MHz / 8 = 2 MHz → 2 MHz / 16000 = 125 → OCR1A = 124
    OCR1A = 124;

    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS11);    // prescaler = 8

    // Enable Timer1 Compare A interrupt
    TIMSK1 |= (1 << OCIE1A);
}

int main(void)
{
    pwm_init();          // Initialize high-frequency PWM engine
    sample_timer_init(); // Initialize 16 kHz sample clock

    sei();               // Enable global interrupts

    while (1)
    {
        // Nothing required here; ISR handles audio playback
    }
}
