#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <cstdint>

#define CAN_TX 3
#define CAN_RX 4
#define ONE_MEG 1000000
#define RP2350_SYS_CLK 150000000
#define RP2040_SYS_CLK 125000000

typedef enum {
    SAFETY_SYSTEM = 0,
    BROADCAST = 1,
    THROTTLE_CONTROL_BOARD = 2,
    PEDAL_BOX = 3,
    STEERING_WHEEL = 4,
    THERMO_CONTROL_BOARD = 5,
    ISOLATION_EXPANSION_DEVICE = 6,
} eModule;

typedef enum {
    TO = 0,
    FROM = 1,
} eDirection;

typedef struct {
    uint8_t priority; 
    eModule module; 
    eDirection direction;
    uint8_t command;
} sCAN_Header; 


sCAN_Header parse_id(uint32_t id);
bool bready_to_drive(void);
void can_init(void);
bool can_tx_adc_taps(uint16_t taps);
void ready_timer_cb();

void rtd_enable_heartbeat(void);
void rtd_disable_heartbeat(void);

void throttle_watchdog_set();
void throttle_watchdog_reset();

#endif
