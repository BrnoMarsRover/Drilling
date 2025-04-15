#include "storage_driver.h"

int storage_read(struct storage* storage)
{
    if (!storage)
        return -2;
    uint8_t buffer[3];
    if (i2c_read_timeout_us(I2C_PORT, STORAGE_ADDR, buffer, 3, false, 8000) != 3)
    {
        return -1;
    }
    uint16_t *ptr = (uint16_t*) buffer;
    storage->weight = *ptr;
    
    storage->raw = buffer[2];
    uint8_t tmp = 224 & buffer[2]; //224 = B'1110 0000'
    tmp = tmp >> 5;
    if (tmp == 0)
        storage->active_slot = 44;
    else
        storage->active_slot = tmp - 1;

    tmp = 16 & buffer[2]; //16 = B'0001 0000'
    storage->active = tmp >> 4;

    if (storage->active)    // if the storage is active no command will be send
        storage->command = 0;

    tmp = 8 & buffer[2]; //8 = B'0000 1000'
    storage->scaleTared = tmp >> 3;

    storage->error = 7 & buffer[2]; //7 = B'0000 0111'
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
    storage->command = 0;
    storage->demand_pos = 0;
    
    storage_write(storage);
    storage_read(storage);
    
    for (int i = 0; i < STORE_SLOTS; ++i) 
    {
        storage->samples[i] = 0;
    }
}

void storage_goto(struct storage *storage, uint8_t pos)
{
    if (!storage)
        return;
    
    storage->weight_recieved = false;
    if (pos == storage->active_slot)
        return;

    switch (pos)
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
    
    if (!storage->weighting)
        storage->command = 20;

    if (storage->active)
        storage->weighting = true;

    if (storage->weighting && !storage->active)
    {
        if (storage->demand_pos <= STORE_SLOTS && storage->demand_pos != 0)
            storage->samples[storage->demand_pos - 1] = storage->weight;
        storage->weight_recieved = true;
        storage->weighting = false;
    }

    return;
}

void storage_hold(struct storage *storage)
{
    if (!storage)
        return;
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

void storage_get_tared(struct storage *storage)
{
    if (!storage)
        return;
    if (!storage->scaleTared)
        storage->command = 10;
    return;
}

void storage_reset(struct storage *storage)
{
    if (!storage)
        return;

    storage->command = 50;
    return;

}
    



