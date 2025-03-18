#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "linear_driver.h"

#define I2C_PORT i2c0
#define LINEAR_ADDR 0x09

#define SAFE_POS 50 // [mm]

int linear_read(struct linear* linear)
{
    if (!linear)
        return -2;
    uint8_t buffer[5];
    if (i2c_read_blocking(I2C_PORT, LINEAR_ADDR, buffer, 5, false) != 5)
    {
        return -1;
    }

    uint8_t tmp = 7 & buffer[0]; //7 = B'0000 0111'
    linear->state = tmp;

    tmp = 248 & buffer[0]; //248 = B'1111 1000'
    linear->error = tmp >> 3;

    uint16_t *ptr = (uint16_t*)(buffer + 1);
    linear->height = *ptr;

    ptr = (uint16_t*)(buffer + 3);
    linear->toGround = *ptr;
    
    return 0;
}

int linear_write(struct linear* linear)
{
    if (!linear)
        return -2;
    uint8_t buffer[2];
    buffer[0] = linear->command;
    buffer[1] = linear->speed;
    if (i2c_write_blocking(I2C_PORT, LINEAR_ADDR, buffer, 2, false) != 2);
    {
        return -1;
    }
    return 0;
}

void linear_init(struct linear* linear)
{    
    if (!linear)
        return;
    linear->command = 1;
    linear->speed = 0;
    linear_write(linear);
    linear_read(linear);
    linear->goalHeight = 0;
}

void linear_stop(struct linear* linear)
{
    if (!linear)
        return;
    linear->command = 4;
}

void linear_go_down(struct linear* linear)
{
    if (!linear)
        return;
    linear->command = 2;
}

void linear_go_up(struct linear* linear)
{
    if (!linear)
        return;
    linear->command = 1;
}

bool is_linear_stucked(struct linear* linear)
{
    if (!linear)
        return true;
    if(linear->error > 0)
        return true;
    return false;
}

bool linear_reached_goal(struct linear* linear)
{
    if (!linear)
        return true;
    if(linear->goalHeight == linear->height)
        return true;
    return false;
}

bool is_linear_home(struct linear* linear) 
{
    if (!linear)
        return true;
    if(linear->state == 0)
        return true;
    return false;
}

bool is_linear_safe(struct linear* linear) 
{
    if (!linear)
        return false;
    if(linear->height < SAFE_POS)
        return true;
    return false;
}