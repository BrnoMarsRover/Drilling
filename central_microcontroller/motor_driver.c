/******************************************************************************
 * @file    motor_driver.c
 * @author  Martin Kriz
 * @brief   Functions for handling the DC motor subsystem.
 * @date    2025-04-26
 ******************************************************************************/

#include "motor_driver.h"

int motor_read(struct motor* motor)
{
    if (!motor)
        return -2;
    uint8_t buffer[4];
    
    if (i2c_read_timeout_us(I2C_PORT, MOTOR_ADDR, buffer, 4, false, 1000) != 4)
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

    if (i2c_write_timeout_us(I2C_PORT, MOTOR_ADDR, &buffer, 1, false, 1000) != 1);
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
    motor->deadTicks = 0;
    motor->running = false;
    motor->stucked = false;
}

void motor_stop(struct motor* motor)
{
    if (!motor)
        return;
    motor->rps = 0;
    motor->running = false;
}

void motor_right(struct motor* motor)
{
    if (!motor)
        return;
    motor_unblock(motor);
    motor->rps = - motor->rpsGoal;
}

void motor_left(struct motor* motor)
{
    if (!motor)
        return;
    if (!motor->stucked)
        motor->rps = motor->rpsGoal;
}

bool is_motor_stucked(struct motor* motor)
{
    if (!motor)
        return true;

    if (motor->state == 1)
    {
        motor->running = true;
        motor->deadTicks = 0;
    }

    if (motor->stucked)
        return true;
    
    if(motor->state > 1)
    {
        if (motor->running)
        {
            motor->stucked = true;
            return true;
        }
        else if(motor->deadTicks > MAX_DEAD_TICKS)
        {
            motor->stucked = true;
            return true;
        }
        ++motor->deadTicks;
    }
    return false;
}

void motor_unblock(struct motor* motor)
{
    if (!motor)
        return;
    motor->deadTicks = 0;
    motor->stucked = false;
}