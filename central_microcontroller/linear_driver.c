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
    linear->pid_integral = 0;
    linear->pid_prevError = 0;
}

void linear_stop(struct linear* linear)
{
    if (!linear)
        return;
    linear->pid_integral = 0;
    linear->pid_prevError = 0;
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
    
        speed = speed / 2 * 255.0f; 
        speed = fabs(speed);
        if (speed > 255.0f) speed = 255.0f;
    
        linear->speed = (uint8_t) speed;
    }
    else
    {
        linear->command = 0;
    }
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
    if(linear->goalHeight == linear->height-10 || linear->goalHeight == linear->height+10)
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

float linear_step(struct linear* linear, float dt) {
    float error = ((float)linear->goalHeight)/1000.0f - ((float)linear->height)/1000.0f;
    linear->pid_integral += error * dt;
    float derivative = (error - linear->pid_prevError) / dt;
    float output = LIN_Kp * error + LIN_Ki * linear->pid_integral + LIN_Kd * derivative;
    linear->pid_prevError = error;
    return output;
}

