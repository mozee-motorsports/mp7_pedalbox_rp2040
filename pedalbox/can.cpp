#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "can.hpp"

extern "C" {
    #include "can2040.h"
}

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


static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    switch (notify) {
        case CAN2040_NOTIFY_TX: 
            break;
        case CAN2040_NOTIFY_RX: 
            gpio_xor_mask(1 << 25);
            break;
    }
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

    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    irq_set_exclusive_handler(PIO0_IRQ_0, PIOx_IRQHandler);
    irq_set_priority(PIO0_IRQ_0, 1);
    irq_set_enabled(PIO0_IRQ_0, true);

    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

bool can_tx_adc_taps(uint16_t taps) {
    sCAN_Header header = {
        .priority = 0,
        .module = PEDAL_BOX,
        .direction = FROM,
        .command = 3, // throttle command
    };
    uint32_t id = header2id(header);
    msg.id = id;
    msg.dlc = 2;
    msg.data[0] = taps & 0xFF;
    msg.data[1] = (taps >> 8) & 0xFF;
    return can2040_transmit(&cbus, &msg);
}