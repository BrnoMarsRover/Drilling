/******************************************************************************
 * @file    linear_driver.c
 * @author  Martin Kriz
 * @brief   Functions for handling the linear actuator subsystem.
 * @date    2025-04-26
 ******************************************************************************/

#include "linear_driver.h"

int linear_read(struct linear* linear)
{
    if (!linear)
    return -2;
    uint8_t buffer[5];
        
    if (i2c_read_timeout_us(I2C_PORT, LINEAR_ADDR, buffer, 5, false, 1000) != 5)
    {
        return -1;
    }

    uint8_t tmp = 7 & buffer[0]; //7 = B'0000 0111'
    linear->state = tmp;

    tmp = 248 & buffer[0]; //248 = B'1111 1000'
    linear->error = tmp >> 3;

    uint16_t *ptr = (uint16_t*)(buffer + 1);
    linear->height = *ptr;

    if (linear->height == 0 && linear->state != 1) linear->height = 1;
    else if (linear->height > 1000) linear->height = 1;

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


    if (i2c_write_timeout_us(I2C_PORT, LINEAR_ADDR, buffer, 2, false, 1000) != 2);
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
    linear->command = 1;
}

void linear_goto(struct linear* linear, float dt)
{
    if (!linear)
    return;
    
    if (linear->goalHeight != 0)
    {
        float speed = linear_step(linear, dt);

        if (speed == 0) {linear->command = 1;}
        else if (speed < 0) {linear->command = 3;}
        else {linear->command = 2;}
    
        linear->speed = (uint8_t) fabs(speed);
    }
    else
    {
        linear->command = 0;
    }
}

void set_drilling_speed(struct linear* linear, float error, float dt)
{
    if (!linear)
    return;

    float speed = linear_step_drilling(linear, error, dt);

    if (speed == 0) {linear->command = 1;}
    else if (speed < 0) {linear->command = 3;}
    else {linear->command = 2;}

    linear->speed = (uint8_t) fabs(speed);
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
    if(linear->goalHeight == linear->height || linear_reached_max(linear))
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

bool can_storage_move(struct linear* linear) 
{
    if (!linear)
        return false;
    if(linear->height < SAFE_POS + 1)
        return true;
    return false;
}

bool can_linear_goDown(struct linear* linear) 
{
    if (!linear)
        return false;
    
    if (linear_reached_max(linear))
        return false;
        
    if(linear->height < STORE_POS || linear->height > linear->goalHeight)
        return true;
    return false;
}

bool linear_reached_max(struct linear* linear)
{
    if (!linear)
        return true;
    if(linear->height >= LOWEST_POS)
        return true;
    return false;
}

float linear_step(struct linear* linear, float dt) {
    float error = (float)linear->goalHeight - (float)linear->height;
    
    if (error == 0)
        return 0;

    // P-regulator
    float output = LIN_Kp * error;

    // Output saturation
    if (output > MAX_OUTPUT) {
        output = MAX_OUTPUT;
    } else if (output < MIN_OUTPUT) {
        output = MIN_OUTPUT;
    }

    // Setting minimal speed
    if (output < MIN_SPEED && output > 0)
        output = MIN_SPEED;
    if (output > -MIN_SPEED && output < 0)
        output = -MIN_SPEED;

    return output;
}

float linear_step_drilling(struct linear* linear, float error, float dt) { 
    // Step only if the drill goes down
    if (linear->goalHeight < linear->height)
        return 0;

    // P-regulator
    float output = LIN_Kp_drilling * error;

    // Output saturation
    if (output > MAX_DRILLING_SPEED) output = MAX_DRILLING_SPEED;
    else if (output < 0) output = 0;
    
    // Output inversion
    output = MAX_DRILLING_SPEED - output;

    return output;
}


