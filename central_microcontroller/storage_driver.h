#ifndef kriz_246877
#define kriz_246877

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define STORAGE_ADDR 0x08

#define STORE_SLOTS 4
#define DEF_POS 0


struct storage{
    uint8_t command;
    uint8_t demand_pos;
    bool weight_recieved;
    bool weighting;

    uint8_t active_slot;
    uint8_t error;
    uint16_t weight;
    bool scaleTared;
    bool active;
    uint16_t samples[STORE_SLOTS];
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
void storage_reset(struct storage *storage);
void storage_get_tared(struct storage *storage);





#endif
