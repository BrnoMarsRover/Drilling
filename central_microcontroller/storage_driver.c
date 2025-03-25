#include "storage_driver.h"

int storage_read(struct storage* storage)
{
    if (!storage)
        return -2;
    uint8_t buffer[3];
    if (i2c_read_timeout_us(I2C_PORT, STORAGE_ADDR, buffer, 3, false, 1000) != 3)
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
    if (!storage)
        return -2;
    uint8_t buffer = storage->command;
    if (i2c_write_timeout_us(I2C_PORT, STORAGE_ADDR, &buffer, 1, false, 1000) != 1);
    {
        return -1;
    }
    return 0;
}

void storage_init(struct storage *storage)
{
    if (!storage)
        return;
    storage->command = 40;
    storage->demand_pos = 0;
    
    storage_write(storage);
    storage->old_command = 40;

    
    storage_read(storage);
    
    for (int i = 0; i < STORE_SLOTS; ++i) 
    {
        storage->samples[i] = 0;
    }
}

void storage_goto(struct storage *storage)
{
    if (!storage)
        return;
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

        case 3:
            storage->command = 33;
            break;

        default:
            storage->command = 34;
            break;
    }

   return; 
}

void storage_get_weight(struct storage *storage)
{
    if (!storage)
        return;
    if (storage->old_command != 20)
    {
        storage->old_weight = storage->weight;
        storage->old_command = storage->command;
        storage->command = 20;
    }

    if (storage->old_weight != storage->weight)
        if (storage->demand_pos <= STORE_SLOTS && storage->demand_pos != 0)
            storage->samples[storage->demand_pos - 1] = storage->weight;

    return;
}

void storage_hold(struct storage *storage)
{
    if (!storage)
        return;
    storage->old_command = storage->command;
    storage->command = 40;
    return;
}

void storage_wreset(struct storage *storage)
{
    if (!storage)
        return;
    for (int i = 0; i < STORE_SLOTS; ++i) 
    {
        storage->samples[i] = 0;
    }
    return;
}

bool is_storage_ok(struct storage *storage)
{
    if (!storage)
        return false;
    if (storage->active_slot == DEF_POS)
        return true;
    return false;
}



