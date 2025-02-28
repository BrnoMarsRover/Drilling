#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "linear_driver.h"

#define I2C_PORT i2c0
#define LINEAR_ADDR 0x09

int linear_read(struct linear* linear)
{
    uint8_t buffer[3];
    if (i2c_read_blocking(I2C_PORT, LINEAR_ADDR, buffer, 3, false) != 3)
    {
        return -1;
    }

    linear->states = buffer[0];
    uint16_t *ptr = (uint16_t*)(buffer + 1);
    linear->height = *ptr;
    
    return 0;
}

int linear_write(struct linear* linear)
{
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
    linear->command = 0;
    linear->speed = 0;
    linear->states = 0;
    linear->height = 0;
    linear->goal_height = 0;
}

void linear_stop(struct linear* linear)
{
    linear->command = 4;
}

void linear_go_down(struct linear* linear)
{
    linear->command = 2;
}

void linear_go_up(struct linear* linear)
{
    linear->command = 1;
}

bool is_linear_stucked(struct linear* linear)
{
    if(linear->states == 1)
        return true;
    return false;
}

bool linear_reached_goal(struct linear* linear)
{
    if(linear->goal_height == linear->height)
        return true;
    return false;
}

bool is_linear_home(struct linear* linear) 
{
    if(linear->states == 4) //maybe it will be different number
        return true;
    return false;
}