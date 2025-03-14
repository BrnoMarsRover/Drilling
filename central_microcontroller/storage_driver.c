#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "storage_driver.h"

#define I2C_PORT i2c0
#define STORAGE_ADDR 0x08

#define DEF_POS 0


int storage_read(struct storage* storage)
{
    uint8_t buffer[3];
    if (i2c_read_blocking(I2C_PORT, STORAGE_ADDR, buffer, 3, false) != 3)
    {
        return -1;
    }
    uint16_t *ptr = (uint16_t*) buffer;
    storage->weight = *ptr;
    
    storage->raw = buffer[2];
    uint8_t tmp = 192 & buffer[2]; //192 = B'1100 0000'
    storage->active_slot = tmp >> 6;


    tmp = 32 & buffer[2]; //32 = B'0010 0000'
    storage->meas_ready = tmp >> 5;

    tmp = 16 & buffer[2]; //16 = B'0001 0000'
    storage->busy = tmp >> 4;

    storage->errors = 15 & buffer[2]; //15 = B'0000 1111'
    return 0;
}

int storage_write(struct storage* storage)
{
    uint8_t buffer = storage->command;
    if (i2c_write_blocking(I2C_PORT, STORAGE_ADDR, &buffer, 1, false) != 1);
    {
        return -1;
    }
    return 0;
}

void storage_init(struct storage *storage)
{
    storage->command = 40;
    storage->demand_pos = 0;
    
    storage_write(storage);
    storage->old_command = 40;

    
    storage_read(storage);
    
    storage->samples[0] = 0;
    storage->samples[1] = 0;
    storage->samples[2] = 0;
    storage->samples[3] = 0;
}

void storage_goto(struct storage *storage)
{
    storage->old_command = storage->command;
    
    switch (storage->demand_pos)
    {
        case 0:
            storage->command = 30;
            break;
        
        case 1:
            storage->command = 31;
            break;

        case 2:
            storage->command = 32;
            break;

        default:
            storage->command = 33;
            break;
    }

   return; 
}

void storage_get_weight(struct storage *storage)
{
    if (storage->old_command != 20)
    {
        storage->old_weight = storage->weight;
        storage->old_command = storage->command;
        storage->command = 20;
    }

    if (storage->old_weight != storage->weight)
        storage->samples[storage->demand_pos] = storage->weight;

    return;
}

void storage_hold(struct storage *storage)
{
    storage->old_command = storage->command;
    storage->command = 40;
    return;
}

void storage_wreset(struct storage *storage)
{
    storage->samples[0] = 0;
    storage->samples[1] = 0;
    storage->samples[2] = 0;
    storage->samples[3] = 0;
    return;
}

bool is_storage_ok(struct storage *storage)
{
    if (storage->active_slot == DEF_POS)
        return true;
    return false;
}



