#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "motor_driver.h"

#define I2C_PORT i2c0
#define MOTOR_ADDR 0x0A

int motor_read(struct motor* motor)
{
    if (!motor)
        return -2;
    uint8_t buffer[4];
    if (i2c_read_blocking(I2C_PORT, MOTOR_ADDR, buffer, 4, false) != 4)
    {
        return -1;
    }

    uint8_t tmp = 3 & buffer[0]; //3 = B'0000 0011'
    motor->state = tmp;

    tmp = 4 & buffer[0]; //4 = B'0000 0100'
    motor->error = tmp >> 2;

	motor->rpsMeas = buffer[1];
	motor->torqueMeas = buffer[2];
    motor->temperature = buffer[3];
    return 0;
}

int motor_write(struct motor* motor)
{
    if (!motor)
        return -2;

    uint8_t buffer = motor->rps;
    if (i2c_write_blocking(I2C_PORT, MOTOR_ADDR, buffer, 1, false) != 1);
    {
        return -1;
    }
    return 0;
}

void motor_init(struct motor* motor)
{
    if (!motor)
        return;
    motor->rps = 0;
    motor->rpsGoal = 0;
    motor_write(motor);
    motor_read(motor);
    motor->stucked = false;
}

void motor_stop(struct motor* motor)
{
    if (!motor)
        return;
    motor->rps = 0;
}

void motor_left(struct motor* motor)
{
    if (!motor)
        return;
    motor_unblock(motor);
    motor->rps = motor->rpsGoal;
}

void motor_right(struct motor* motor)
{
    if (!motor)
        return;
    if (!motor->stucked)
        motor->rps = - motor->rpsGoal;
}

bool is_motor_stucked(struct motor* motor)
{
    if (!motor)
        return true;
    if(motor->state > 1)
    {
        motor->stucked = true;
        return true;
    }
    return false;
}

void motor_unblock(struct motor* motor)
{
    if (!motor)
        return true;
    motor->stucked = false;
}