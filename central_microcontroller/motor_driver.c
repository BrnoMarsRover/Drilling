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
    uint8_t buffer[5];
    if (i2c_read_blocking(I2C_PORT, MOTOR_ADDR, buffer, 5, false) != 5)
    {
        return -1;
    }

    motor->state = buffer[0];
	float* floatPtr = (float*)(buffer + 1);
	motor->torque_meas = *floatPtr;
    return 0;
}

int motor_write(struct motor* motor)
{
    uint8_t buffer[4];
    //buffer[0] = motor->direction;
	*(float*)(buffer) = motor->torque;

    if (i2c_write_blocking(I2C_PORT, MOTOR_ADDR, buffer, 4, false) != 4);
    {
        return -1;
    }
    return 0;
}

void motor_init(struct motor* motor)
{
    motor->torque = 0;
    motor->torque_goal = 0;
	motor->torque_meas = 0;
	motor->state = 0;
}

void motor_stop(struct motor* motor)
{
    motor->torque = 0;
}

void motor_left(struct motor* motor)
{
    motor->torque = motor->torque_goal;
}

void motor_right(struct motor* motor)
{
    motor->torque = - motor->torque_goal;
}

bool is_motor_stucked(struct motor* motor)
{
    if(motor->state == 2)
        return true;
    return false;
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