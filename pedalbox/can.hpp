#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <cstdint>

#define CAN_TX 3
#define CAN_RX 4
#define ONE_MEG 1000000
#define RP2350_SYS_CLK 150000000
#define RP2040_SYS_CLK 125000000

bool bready_to_drive(void);
void can_init(void);
bool can_tx_adc_taps(uint16_t taps, uint16_t msg_num);
void ready_timer_cb();

void rtd_enable_heartbeat(void);
void rtd_disable_heartbeat(void);

void throttle_watchdog_set();
void throttle_watchdog_reset();

#endif
