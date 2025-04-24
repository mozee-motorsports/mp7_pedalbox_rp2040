#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "can.hpp"
#include "hardware/timer.h"
#include "hardware/adc.h"
#include <cstdint>

#define STB 19
#define POT0 26
#define POT1 27

void stb_init(void) {
    gpio_init(STB);
    gpio_set_dir(STB, GPIO_OUT);
    gpio_put(STB, 0);
}



static bool toggle = true;
bool can_tx_timer_callback(__unused struct repeating_timer *t) {
    gpio_xor_mask(1 << 25);
    adc_select_input(0);
    uint16_t pot0_taps = adc_read();
    adc_select_input(1);
    uint16_t pot1_taps = adc_read();
    // printf("pot0: %d, pot1: %d\n", pot0_taps, pot1_taps);
    uint16_t pot_avg = (pot0_taps + pot1_taps)/2;



    can_tx_adc_taps(4095*toggle);
    toggle = !toggle;

    // can_tx_adc_taps(pot_avg);
    return true;
}


void pedal_init(struct repeating_timer *t) {
    adc_init();
    adc_gpio_init(POT0);
    adc_gpio_init(POT1);
    add_repeating_timer_ms(1000, can_tx_timer_callback, NULL, t);
}

int main() {
    stdio_init_all();

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    can_init();
    stb_init();

    struct repeating_timer t; 
    pedal_init(&t);

    // adc_init();
    // adc_gpio_init(POT0);
    // adc_gpio_init(POT1);

    while (true) {
        sleep_ms(100);
        // gpio_put(25, 0);
        // sleep_ms(1000);
        // gpio_put(25, 1);
        // sleep_ms(1000);
    }
}
