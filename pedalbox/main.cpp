#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "can.hpp"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include <cstdint>

#define POT0 26
#define POT1 27


static bool toggle = true;

// DEFAULT
// NOTE: these need tuned experimentally
static uint16_t min = 670;
static uint16_t max = 880;

bool can_tx_timer_callback(__unused struct repeating_timer *t) {
    gpio_xor_mask(1 << 25);

    /* sample pots */
    adc_select_input(0);
    uint16_t pot0_taps = adc_read();
    adc_select_input(1);
    uint16_t pot1_taps = 4095 - adc_read();

    uint16_t diff = (pot0_taps > pot1_taps ? pot0_taps - pot1_taps : pot1_taps - pot0_taps);

    uint16_t pot_avg = diff < 200 ? (pot0_taps + pot1_taps)/2 : 0;
    printf("pot_avg: %d\n", pot_avg);
    double pot_avg_pct = ((static_cast<double>(pot0_taps) - min)/(max - min));
    uint16_t pos_taps = static_cast<uint16_t>(pot_avg_pct*4095.0);

    can_tx_adc_taps(pos_taps);
    return true;
}



void pedal_init(void) {
    adc_init();
    adc_gpio_init(POT0); // pot 0 is low at startup
    gpio_pull_down(POT0); // pull down pot0 on float
    adc_gpio_init(POT1); // pot1 is high at startup
    gpio_pull_up(POT1); // pull up pot 1 on float
}

void pedal_enable_callback(struct repeating_timer *t) {
    add_repeating_timer_ms(10, can_tx_timer_callback, NULL, t);
}

void tune_throttle(uint16_t *min, uint16_t *max) {
    
    uint16_t min_t = 4095;
    uint16_t max_t = 0;
    while (!bready_to_drive()) {
        adc_select_input(0);
        uint16_t pot0_taps = adc_read();

        if (pot0_taps < min_t) {
            min_t = pot0_taps;
        }

        if (max_t < pot0_taps) {
            max_t = pot0_taps;
        }
        printf("min: %d, max: %d\n", min_t, max_t);
    }
    *min = min_t;
    *max = max_t;
}

int main() {
    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    can_init();
    pedal_init();

    tune_throttle(&min, &max);

    struct repeating_timer t; 
    pedal_enable_callback(&t);


    while (true) {
        sleep_ms(100);
        // gpio_put(25, 0);
        // sleep_ms(1000);
        // gpio_put(25, 1);
        // sleep_ms(1000);
    }
}
