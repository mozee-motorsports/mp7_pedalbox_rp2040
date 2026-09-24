#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "can.hpp"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include <cstdint>

#define POT0 26
#define POT1 27

#define RTD_BUTTON 21
#define RTD_SRC 22

#define TAPS_BUFFER_SIZE 3

static bool toggle = true;

// DEFAULT
// NOTE: these need tuned experimentally
const static uint16_t TUNED_MIN = 1192;
const static uint16_t TUNED_MAX = 1455;

// EMA filter variables
static float filtered_val = 0;
float alpha = 0.08f; // Adjust between 0.1 (heavy filter) and 0.9 (light filter)

static uint16_t msg_num = 0;

#define NUM_SAMPLES 10
uint16_t samples[NUM_SAMPLES];
uint8_t idx = 0;
long total = 0;

int running_average(uint16_t new_value)
{
    total -= samples[idx];
    samples[idx] = new_value;
    total += samples[idx];

    idx = (idx + 1) % NUM_SAMPLES;

    return total / NUM_SAMPLES;
}

bool can_tx_timer_callback(__unused struct repeating_timer *t)
{
    uint16_t pct = 0;
    ready_timer_cb();
    gpio_xor_mask(1 << 25);

    /* 1. Sample pots (Both normalized to go UP) */
    adc_select_input(0);

    uint32_t sum = 0;
    uint8_t num_of_samples = 100;
    for (int i = 0; i < num_of_samples; i++)
    {
        sum += adc_read();
    }

    uint16_t pot0_raw = sum / num_of_samples;

    adc_select_input(1);

    sum = 0;
    for (int i = 0; i < num_of_samples; i++)
    {
        sum += adc_read();
    }

    uint16_t pot1_raw = sum / num_of_samples;

    /* 2. Safety: Check for sensor divergence */
    // uint16_t diff = (pot0_raw > pot1_raw) ? (pot0_raw - pot1_raw) : (pot1_raw - pot0_raw);

    // If pots disagree by more than ~10%, you might want to flag an error
    // For now, we'll just use the primary pot (pot0) for the calculation
    uint16_t current_sample = pot0_raw;

    /* 3. Normalization (Mapping raw ADC to 0-65535) */
    uint16_t mapped_pos = 0;
    if (current_sample <= TUNED_MIN)
    {
        mapped_pos = 0;
        pct = 0;
    }
    else if (current_sample >= TUNED_MAX)
    {
        mapped_pos = 4095;
        pct = 100;
    }
    else
    {
        // Linear interpolation: ((val - min) / (max - min)) * 65535 (16-bit range)
        // We use double to prevent integer truncation errors
        mapped_pos = (uint16_t)(((double)(current_sample - TUNED_MIN) / (TUNED_MAX - TUNED_MIN)) * 4095.0);
        pct = (uint16_t)(((double)(current_sample - TUNED_MIN) / (TUNED_MAX - TUNED_MIN)) * 100.0);
    }

    // Average out the samples to filter out noise
    uint16_t sample_average = running_average(mapped_pos);

    // Exponential Moving Average (EMA) filter
    filtered_val = (alpha * (float)sample_average) + ((1.0f - alpha) * filtered_val);

    uint16_t pos_taps = (uint16_t)filtered_val;
    if (pos_taps > 4095)
        pos_taps = 4095;

    if (!can_tx_adc_taps(pos_taps, msg_num))
    {
        // printf("Transmit error!\n");
    }

    if (msg_num % 10 == 0)
    {
        printf("%d | Final=%u (Raw0=%u Raw1=%u) Pct=%u\n", msg_num, pos_taps, pot0_raw, pot1_raw, pct);
    }

    msg_num++;
    return true;
}

void pedal_init(void)
{
    adc_init();
    adc_gpio_init(POT0);  // pot 0 is low at startup
    gpio_pull_down(POT0); // pull down pot0 on float
    adc_gpio_init(POT1);  // pot1 is high at startup
    gpio_pull_up(POT1);   // pull up pot 1 on float
}

void pedal_enable_callback(struct repeating_timer *t)
{
    add_repeating_timer_ms(10, can_tx_timer_callback, NULL, t);
}

// void tune_throttle(uint16_t *min, uint16_t *max, bool ready_pressed)
void tune_throttle(bool ready_pressed)
{
    static uint16_t min_t = 4095;
    static uint16_t max_t = 0;

    if (!ready_pressed)
    {
        adc_select_input(0);
        uint16_t pot_value = adc_read();

        // Removed tuning
        // uint16_t median_tuning_value = running_average(pot_value);

        // if (median_tuning_value < min_t)
        //     min_t = pot_value;
        // if (median_tuning_value > max_t)
        //     max_t = pot_value;

        printf("Tuning: value=%u min=%u max=%u\n", pot_value, TUNED_MIN, TUNED_MAX);
    }

    // *min = min_t;
    // *max = max_t - 25;
}

int main()
{
    stdio_init_all();

    gpio_init(RTD_BUTTON);
    gpio_init(RTD_SRC);
    gpio_set_dir(RTD_BUTTON, GPIO_IN);
    gpio_set_dir(RTD_SRC, GPIO_OUT);
    gpio_put(RTD_SRC, 1);

    gpio_init(19);
    gpio_set_dir(19, GPIO_OUT);
    gpio_put(19, 0);
    can_init();
    pedal_init();

    // bool pressed = false;
    // while (!pressed)
    // {
    //     tune_throttle(pressed);
    //     if (gpio_get(RTD_BUTTON))
    //     {
    //         sleep_ms(5);
    //         if (gpio_get(RTD_BUTTON))
    //         {
    //             pressed = true;
    //         }
    //     }
    // }
    // printf("done tuning\n");

    // Reset samples for averaging
    for (uint8_t i = 0; i < NUM_SAMPLES; i++)
    {
        samples[i] = 0;
    }

    throttle_watchdog_set();
    rtd_enable_heartbeat();

    struct repeating_timer t;
    pedal_enable_callback(&t);

    // gpio_set_irq_enabled_with_callback(SSOK, GPIO_IRQ_EDGE_FALL, true, ssok_off_callback);

    sleep_ms(500);

    while (true)
    {
        sleep_ms(100);
        //gpio_put(19, 0);
        // sleep_ms(1000);
        // gpio_put(25, 1);
        // sleep_ms(1000);
    }
}
