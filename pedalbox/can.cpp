#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "can.hpp"

extern "C" {
    #include "can2040.h"
#include <cstdio>
}

#define STB 19

sCAN_Header parse_id(uint32_t id) {
    return sCAN_Header {
        .priority = (uint8_t)((id >> 8) & 0b0111),
        .module  = (eModule)((id >> 5) & 0b0111),
        .direction = (eDirection) ((id >> 4) & 0b01),
        .command = (uint8_t)(id & 0xF),
    };
}

uint32_t header2id(sCAN_Header header) {
    uint32_t id = 0;
    id |= (header.priority << 8);
    id |= (header.module << 5);
    id |= (header.direction << 4);
    id |= header.command;
    return id;
}

static struct can2040 cbus;
static struct can2040_msg msg;

static volatile bool samplefresh = false;
static double sample = 0.0;

static bool ready_to_drive = false;

bool bready_to_drive(void) {
    return ready_to_drive;
}

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    switch (notify) {
        case CAN2040_NOTIFY_TX: 
            break;
        case CAN2040_NOTIFY_RX: 
            if(!ready_to_drive) {
                sCAN_Header header = parse_id(msg->id);
                if(gpio_get(21) == 1) {
                    ready_to_drive = true;
                }
            }

            throttle_watchdog_reset();

            break;
    }
    return;
}

void ready_timer_cb() {
    if (!ready_to_drive && gpio_get(21)) {
        ready_to_drive = true;
    }
    return; // keep repeating
}

static void PIOx_IRQHandler(void) {
    can2040_pio_irq_handler(&cbus);
}

void can_init(void) {

    uint32_t pio_num = 0;
    uint32_t sys_clock = RP2040_SYS_CLK;
    uint32_t bitrate = ONE_MEG;
    uint32_t gpio_tx = CAN_TX;
    uint32_t gpio_rx = CAN_RX;
     
    gpio_init(STB);
    gpio_set_dir(STB, GPIO_OUT);
    gpio_put(STB, 0);

    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);

    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

bool can_tx_adc_taps(uint16_t taps, uint16_t msg_num) {
    
    sCAN_Header header = {
        .priority = 0,
        .module = PEDAL_BOX,
        .direction = FROM,
        .command = 3, // throttle command
    };
    uint32_t id = header2id(header);
    msg.id = id;
    msg.dlc = 4;
    msg.data[0] = taps & 0xFF;
    msg.data[1] = (taps >> 8) & 0xFF;
    // TODO: remove after troubleshooting
    //transmit message number for log alignment
    msg.data[2] = msg_num & 0xFF;
    msg.data[3] = (msg_num >> 8) & 0xFF;
    
    printf("TX: %d %d %d %d\n", msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
    return can2040_transmit(&cbus, &msg);
}

struct repeating_timer rtd_timer;
struct repeating_timer throttle_watchdog;

bool throttle_watchdog_callback() {
    //printf("throttle watchdog tick\n");
    // keep the timer running
    return true;
}

void throttle_watchdog_reset() {
    //printf("throttle watchdog reset\n");
    cancel_repeating_timer(&throttle_watchdog); // early stop
    add_repeating_timer_ms(500, (repeating_timer_callback_t)throttle_watchdog_callback, NULL, &throttle_watchdog); 
}

void throttle_watchdog_set() {
    cancel_repeating_timer(&throttle_watchdog);
    add_repeating_timer_ms(500, (repeating_timer_callback_t)throttle_watchdog_callback, NULL, &throttle_watchdog);
}

static const sCAN_Header rtd_header = {
    .priority = 0,
    .module = BROADCAST,
    .direction = FROM,
    .command = 0, // heartbeat?
};

static bool rtd_heartbeat(__unused struct repeating_timer *rtd_timer) {
    //printf("RTD heartbeat\n");
    gpio_xor_mask(1 << 25);
    msg.id = header2id(rtd_header);
    msg.dlc = 0;
    //printf("%d\n", can2040_transmit(&cbus, &msg));
    return 1;
}

void rtd_enable_heartbeat() {
    add_repeating_timer_ms(500, (repeating_timer_callback_t)rtd_heartbeat, NULL, &rtd_timer);
}

void rtd_disable_heartbeat() {
    cancel_repeating_timer(&rtd_timer);
}