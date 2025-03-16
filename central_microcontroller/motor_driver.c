#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "motor_driver.h"

#define I2C_PORT i2c0
#define MOTOR_ADDR 0x0A
#define MAX_TORQUE 2.5
#define RANGE 1000

int motor_read(struct motor* motor)
{
    if (!motor)
        return -2;
    uint8_t buffer[9];
    if (i2c_read_blocking(I2C_PORT, MOTOR_ADDR, buffer, 9, false) != 9)
    {
        return -1;
    }

    motor->state = buffer[0];
	float* floatPtr = (float*)(buffer + 1);
    motor->rps_meas = *floatPtr;

    floatPtr = (float*)(buffer + 5);
	motor->torque_meas = *floatPtr;
    return 0;
}

int motor_write(struct motor* motor)
{
    if (!motor)
        return -2;
    uint8_t buffer[4];
    //buffer[0] = motor->direction;
	*(float*)(buffer) = motor->rps;

    if (i2c_write_blocking(I2C_PORT, MOTOR_ADDR, buffer, 4, false) != 4);
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
    motor->rps_goal = 0;
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
    motor->rps = motor->rps_goal;
}

void motor_right(struct motor* motor)
{
    if (!motor)
        return;
    if (!motor->stucked)
        motor->rps = - motor->rps_goal;
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

float float_decode(uint16_t aNum)
{
    float constant = MAX_TORQUE/RANGE;
    float result =  constant * (float)aNum;
    return result;
}

uint16_t float_code(float aNum)
{
    float constant = RANGE/MAX_TORQUE;
    uint16_t result =  (uint16_t)(constant * aNum);
    return result;
}