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


static bool toggle = true;

// DEFAULT
// NOTE: these need tuned experimentally
static uint16_t min = 670;
static uint16_t max = 880;

bool can_tx_timer_callback(__unused struct repeating_timer *t) {
    ready_timer_cb();
    gpio_xor_mask(1 << 25);

    /* sample pots */
    adc_select_input(0);
    uint16_t pot0_taps = adc_read();
    adc_select_input(1);
    uint16_t pot1_taps = 4095 - adc_read();

    uint16_t diff = (pot0_taps > pot1_taps ? pot0_taps - pot1_taps : pot1_taps - pot0_taps);

    uint16_t pot_avg = diff < 200 ? (pot0_taps + pot1_taps)/2 : 0;

    double pot_avg_pct = ((static_cast<double>(pot0_taps) - min)/(max - min));
    uint16_t pos_taps = static_cast<uint16_t>(pot_avg_pct*4095.0);

    printf("pos_taps=%u bytes=%02X %02X\n", pos_taps, pos_taps & 0xFF, (pos_taps >> 8) & 0xFF);
    can_tx_adc_taps(pos_taps);
    //throttle_watchdog_reset();
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

void tune_throttle(uint16_t *min, uint16_t *max, bool ready_pressed) {
    static uint16_t min_t = 4095;
    static uint16_t max_t = 0;

    if (!ready_pressed) {
        adc_select_input(0);
        uint16_t pot_value = adc_read();

        if (pot_value < min_t) min_t = pot_value;
        if (pot_value > max_t) max_t = pot_value;

        printf("Tuning: value=%u min=%u max=%u\n", pot_value, min_t, max_t);
    }

    *min = min_t;
    *max = max_t;
}



int main() {
    stdio_init_all();

    gpio_init(RTD_BUTTON);
    gpio_init(RTD_SRC);
    gpio_set_dir(RTD_BUTTON, GPIO_IN);
    gpio_set_dir(RTD_SRC, GPIO_OUT);
    gpio_put(RTD_SRC, 1);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    can_init();
    pedal_init();



    bool pressed = false;
    while(!pressed) {
        tune_throttle(&min, &max, pressed);
        if(gpio_get(RTD_BUTTON)) {
            sleep_ms(5);
            if(gpio_get(RTD_BUTTON)) {
                pressed = true;
            }
        }
    }
    printf("done tuning\n");

    throttle_watchdog_set();
    rtd_enable_heartbeat();

    struct repeating_timer t; 
    pedal_enable_callback(&t);


    // gpio_set_irq_enabled_with_callback(SSOK, GPIO_IRQ_EDGE_FALL, true, ssok_off_callback);

    sleep_ms(500);
    
    while (true) {
        sleep_ms(100);
        // gpio_put(25, 0);
        // sleep_ms(1000);
        // gpio_put(25, 1);
        // sleep_ms(1000);
    }
}
