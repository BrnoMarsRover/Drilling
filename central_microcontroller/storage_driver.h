#ifndef kriz_246877
#define kriz_246877

#include <stdio.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"


struct storage{
    uint8_t command;
    uint8_t old_command;
    uint8_t demand_pos;
    uint16_t old_weight;

    uint8_t active_slot;
    uint8_t errors;
    uint16_t weight;
    bool meas_ready;
    bool busy;
    uint16_t samples[4];
    uint8_t raw;
};


int storage_read(struct storage* storage);
int storage_write(struct storage* storage);
void storage_init(struct storage* storage);

void storage_goto(struct storage *storage);
void storage_get_weight(struct storage *storage);
void storage_hold(struct storage *storage);
void storage_wreset(struct storage *storage);
bool is_storage_ok(struct storage *storage);



#endif
